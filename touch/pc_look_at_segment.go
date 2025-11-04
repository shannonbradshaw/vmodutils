package touch

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/robot"
	"go.viam.com/rdk/spatialmath"

	"github.com/erh/vmodutils"
)

var LookAtCameraModel = vmodutils.NamespaceFamily.WithModel("pc-look-at-crop-camera")

func init() {
	resource.RegisterComponent(
		camera.API,
		LookAtCameraModel,
		resource.Registration[camera.Camera, *LookAtCameraConfig]{
			Constructor: newLookAtCamera,
		})
}

type LookAtCameraConfig struct {
	Src string
}

func (ccc *LookAtCameraConfig) Validate(path string) ([]string, []string, error) {
	if ccc.Src == "" {
		return nil, nil, fmt.Errorf("need a src camera")
	}
	return []string{ccc.Src}, nil, nil
}

func newLookAtCamera(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (camera.Camera, error) {
	newConf, err := resource.NativeConfig[*LookAtCameraConfig](config)
	if err != nil {
		return nil, err
	}

	cc := &lookAtCamera{
		name:   config.ResourceName(),
		cfg:    newConf,
		logger: logger,
	}

	cc.src, err = camera.FromProvider(deps, newConf.Src)
	if err != nil {
		return nil, err
	}

	cc.client, err = vmodutils.ConnectToMachineFromEnv(ctx, logger)
	if err != nil {
		return nil, err
	}

	return cc, nil
}

type lookAtCamera struct {
	resource.AlwaysRebuild

	name   resource.Name
	cfg    *LookAtCameraConfig
	logger logging.Logger

	src    camera.Camera
	client robot.Robot

	lock               sync.Mutex
	active             bool
	lastPointCloud     pointcloud.PointCloud
	lastPointCloudTime time.Time
	lastPointCloudErr  error
}

func (cc *lookAtCamera) Name() resource.Name {
	return cc.name
}

func (cc *lookAtCamera) Image(ctx context.Context, mimeType string, extra map[string]interface{}) ([]byte, camera.ImageMetadata, error) {
	pc, err := cc.NextPointCloud(ctx, extra)
	if err != nil {
		return nil, camera.ImageMetadata{}, err
	}
	img := PCToImage(pc)

	data, err := rimage.EncodeImage(ctx, img, mimeType)
	if err != nil {
		return nil, camera.ImageMetadata{}, err
	}

	return data, camera.ImageMetadata{mimeType}, err
}

func (cc *lookAtCamera) Images(ctx context.Context, filterSourceNames []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	pc, err := cc.NextPointCloud(ctx, extra)
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	start := time.Now()
	img := PCToImage(pc)
	elapsed := time.Since(start)
	if elapsed > (time.Millisecond * 100) {
		cc.logger.Infof("PCToImage took %v", elapsed)
	}
	ni, err := camera.NamedImageFromImage(img, "cropped", "image/png")
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	return []camera.NamedImage{ni}, resource.ResponseMetadata{time.Now()}, nil
}

func (cc *lookAtCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

func (cc *lookAtCamera) NextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {

	start := time.Now()
	cc.lock.Lock()
	if cc.active {
		cc.lock.Unlock()
		return cc.waitForPointCloudAfter(ctx, start)
	}

	cc.active = true
	cc.lock.Unlock()

	pc, err := cc.doNextPointCloud(ctx, extra)

	cc.lock.Lock()
	cc.active = false
	cc.lastPointCloud = pc
	cc.lastPointCloudErr = err
	cc.lastPointCloudTime = time.Now()
	cc.lock.Unlock()

	return pc, err
}

func (cc *lookAtCamera) waitForPointCloudAfter(ctx context.Context, when time.Time) (pointcloud.PointCloud, error) {
	for {
		if ctx.Err() != nil {
			return nil, ctx.Err()
		}

		if time.Since(when) > time.Minute {
			return nil, fmt.Errorf("waitForPointCloudAfter timed out after %v", time.Since(when))
		}

		cc.lock.Lock()
		if cc.lastPointCloudTime.After(when) {
			pc := cc.lastPointCloud
			err := cc.lastPointCloudErr
			cc.lock.Unlock()
			return pc, err
		}
		cc.lock.Unlock()

		time.Sleep(time.Millisecond * 50)
	}
}

func (cc *lookAtCamera) doNextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
	pc, err := cc.src.NextPointCloud(ctx, extra)
	if err != nil {
		return nil, err
	}

	return PCLookAtSegment(pc)
}

func (cc *lookAtCamera) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{
		SupportsPCD: true,
	}, nil
}

func (cc *lookAtCamera) Close(ctx context.Context) error {
	return cc.client.Close(ctx)
}

func (cc *lookAtCamera) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
}

func addRecursive(t *pointcloud.BasicOctree, start r3.Vector, good pointcloud.PointCloud, radius float64) error {
	close, err := t.PointsWithinRadius(start, 5)
	if err != nil {
		return err
	}
	for _, c := range close {
		_, got := good.At(c.X, c.Y, c.Z)
		if got {
			continue
		}
		d, got := t.At(c.X, c.Y, c.Z)
		if !got {
			fmt.Printf("how is this possible\n")
			continue
		}
		good.Set(c, d)
		err = addRecursive(t, c, good, radius)
		if err != nil {
			return err
		}
	}
	return nil
}

func PCLookAtSegment(pc pointcloud.PointCloud) (pointcloud.PointCloud, error) {

	t, err := pointcloud.ToBasicOctree(pc, 1)
	if err != nil {
		return nil, err
	}

	var best r3.Vector
	distanceFromCenter := 100000.0
	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if p.Z < 20 {
			return true
		}
		myD := math.Pow((p.X*p.X)+(p.Y*p.Y), .5)
		if myD < distanceFromCenter {
			distanceFromCenter = myD
			best = p
		}
		return true
	})

	good := pointcloud.NewBasicEmpty()

	err = addRecursive(t, best, good, 5)
	if err != nil {
		return nil, err
	}

	return good, nil
}
