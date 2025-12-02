package touch

import (
	"image"
	"image/color"
	"image/png"
	"math"
	"os"
	"testing"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestPCD1(t *testing.T) {
	logger := logging.NewTestLogger(t)

	in, err := pointcloud.NewFromFile("data/test.pcd", "")
	test.That(t, err, test.ShouldBeNil)

	cleaner, err := pointcloud.StatisticalOutlierFilter(50, 1)
	test.That(t, err, test.ShouldBeNil)

	out := pointcloud.NewBasicEmpty()
	err = cleaner(in, out)
	test.That(t, err, test.ShouldBeNil)

	b1 := PrepBoundingRectForSearch()
	out.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if math.Abs(p.Z) < 10 && d.HasColor() {
			BoundingRectMinMax(b1, p)
		}
		return true
	})

	logger.Infof("b1 %v -> %v", b1, b1.Size())

	b2 := PrepBoundingRectForSearch()

	img := image.NewRGBA(image.Rectangle{Max: b1.Size()})
	out.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if math.Abs(p.Z) < 25 && d.HasColor() {
			x := int(p.X)
			y := int(p.Y)

			x = x - b1.Min.X
			y = y - b1.Min.Y

			car := math.Pow((p.X*p.X)+(p.Y*p.Y), .5)

			dis := rimage.White.Distance(rimage.NewColorFromColor(d.Color()))

			if dis < 3 && car > 100 {
				if x > 400 {
					logger.Infof("%v,%v -> %v,%v", int(p.X), int(p.Y), x, y)
				}
				img.Set(x, y, d.Color())
				BoundingRectMinMax(b2, p)

				//img.Set(x,y,rimage.White)
			}

		}

		return true
	})

	logger.Infof("b2 %v %v", b2, b2.Size())

	file, err := os.Create("test.png")
	test.That(t, err, test.ShouldBeNil)
	defer file.Close()

	err = png.Encode(file, img)
	test.That(t, err, test.ShouldBeNil)

	h := PCFindHighestInRegion(out, *b2)
	logger.Infof("hi %v", h)
}

func TestPCCrop(t *testing.T) {
	a := pointcloud.NewBasicEmpty()
	a.Set(r3.Vector{1, 1, 1}, pointcloud.NewBasicData())
	a.Set(r3.Vector{5, 5, 5}, pointcloud.NewBasicData())
	a.Set(r3.Vector{9, 9, 9}, pointcloud.NewBasicData())
	a.Set(r3.Vector{5, 0, 5}, pointcloud.NewBasicData())

	test.That(t, a.Size(), test.ShouldEqual, 4)
	_, got := a.At(1, 1, 1)
	test.That(t, got, test.ShouldBeTrue)
	_, got = a.At(5, 5, 5)
	test.That(t, got, test.ShouldBeTrue)
	_, got = a.At(5, 0, 5)
	test.That(t, got, test.ShouldBeTrue)

	b := PCCrop(a, r3.Vector{2, 2, 2}, r3.Vector{7, 7, 7})
	test.That(t, b.Size(), test.ShouldEqual, 1)
	_, got = b.At(1, 1, 1)
	test.That(t, got, test.ShouldBeFalse)
	_, got = b.At(5, 5, 5)
	test.That(t, got, test.ShouldBeTrue)

}

func TestPCProject1(t *testing.T) {
	in, err := pointcloud.NewFromFile("data/cup1.pcd", "")
	test.That(t, err, test.ShouldBeNil)

	img := PCToImage(in)

	file, err := os.Create("projecttest1.png")
	test.That(t, err, test.ShouldBeNil)
	defer file.Close()

	err = png.Encode(file, img)
	test.That(t, err, test.ShouldBeNil)

}

func BenchmarkPCProject1(t *testing.B) {
	in, err := pointcloud.NewFromFile("data/cup1.pcd", "")
	test.That(t, err, test.ShouldBeNil)

	PCToImage(in)

	t.ResetTimer()

	for range t.N {
		PCToImage(in)
	}
}

func TestPCStats(t *testing.T) {
	in, err := pointcloud.NewFromFile("data/cup2onlycup.pcd", "")
	test.That(t, err, test.ShouldBeNil)

	md := in.MetaData()
	test.That(t, md.MaxSideLength(), test.ShouldBeGreaterThan, 100)
	test.That(t, md.MaxSideLength(), test.ShouldBeLessThan, 110)

}

func TestGetApproachPoint(t *testing.T) {
	c := r3.Vector{233.8, 202.05, 85.0}

	p := GetApproachPoint(c, 100, &spatialmath.OrientationVectorDegrees{OX: 1})
	test.That(t, p.X, test.ShouldAlmostEqual, c.X-100, 1)
	test.That(t, p.Y, test.ShouldAlmostEqual, c.Y, 1)
	test.That(t, p.Z, test.ShouldAlmostEqual, c.Z, 1)

	p = GetApproachPoint(c, 100, &spatialmath.OrientationVectorDegrees{OX: -1})
	test.That(t, p.X, test.ShouldAlmostEqual, c.X+100, 1)
	test.That(t, p.Y, test.ShouldAlmostEqual, c.Y, 1)
	test.That(t, p.Z, test.ShouldAlmostEqual, c.Z, 1)

	p = GetApproachPoint(c, 100, &spatialmath.OrientationVectorDegrees{OY: 1})
	test.That(t, p.X, test.ShouldAlmostEqual, c.X, 1)
	test.That(t, p.Y, test.ShouldAlmostEqual, c.Y-100, 1)
	test.That(t, p.Z, test.ShouldAlmostEqual, c.Z, 1)

	p = GetApproachPoint(c, 100, &spatialmath.OrientationVectorDegrees{OY: -1})
	test.That(t, p.X, test.ShouldAlmostEqual, c.X, 1)
	test.That(t, p.Y, test.ShouldAlmostEqual, c.Y+100, 1)
	test.That(t, p.Z, test.ShouldAlmostEqual, c.Z, 1)

	p = GetApproachPoint(c, 100, &spatialmath.OrientationVectorDegrees{OZ: -1})
	test.That(t, p.X, test.ShouldAlmostEqual, c.X, 1)
	test.That(t, p.Y, test.ShouldAlmostEqual, c.Y, 1)
	test.That(t, p.Z, test.ShouldAlmostEqual, c.Z+100, 1)

	p = GetApproachPoint(c, 100, &spatialmath.OrientationVectorDegrees{OX: -1, OY: -1})
	test.That(t, p.X, test.ShouldAlmostEqual, c.X+70.7, 1)
	test.That(t, p.Y, test.ShouldAlmostEqual, c.Y+70.7, 1)
	test.That(t, p.Z, test.ShouldAlmostEqual, c.Z, 1)

	p = GetApproachPoint(c, 100, &spatialmath.OrientationVectorDegrees{OX: -1, OZ: -1})
	test.That(t, p.X, test.ShouldAlmostEqual, c.X+70.7, 1)
	test.That(t, p.Y, test.ShouldAlmostEqual, c.Y, 1)
	test.That(t, p.Z, test.ShouldAlmostEqual, c.Z+70.7, 1)

	p = GetApproachPoint(c, 100, &spatialmath.OrientationVectorDegrees{OX: -1, OY: -1, OZ: -1})
	test.That(t, p.X, test.ShouldAlmostEqual, c.X+57.7, 1)
	test.That(t, p.Y, test.ShouldAlmostEqual, c.Y+57.7, 1)
	test.That(t, p.Z, test.ShouldAlmostEqual, c.Z+57.7, 1)
}

func TestPCCropColor(t *testing.T) {
	in, err := pointcloud.NewFromFile("data/test.pcd", "")
	test.That(t, err, test.ShouldBeNil)

	min := r3.Vector{-5000, -5000, -5000}
	max := r3.Vector{5000, 5000, 5000}

	filtered := PCCropWithColor(in, min, max, nil)
	test.That(t, filtered.Size(), test.ShouldEqual, in.Size())

	filtered = PCCropWithColor(in, min, max, []ColorFilter{
		{color.RGBA{0, 0, 0, 0}, 100},
	})

	test.That(t, filtered.Size(), test.ShouldBeLessThan, in.Size())

}
