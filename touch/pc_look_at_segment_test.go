package touch

import (
	"os"
	"testing"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/test"
)

func TestPCLookAtSegment(t *testing.T) {
	in, err := pointcloud.NewFromFile("data/spray1.pcd", "")
	test.That(t, err, test.ShouldBeNil)
	in, err = PCLookAtSegment(in)
	test.That(t, err, test.ShouldBeNil)

	f, err := os.OpenFile("foo.pcd", os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0o644)
	test.That(t, err, test.ShouldBeNil)
	defer f.Close()
	err = pointcloud.ToPCD(in, f, pointcloud.PCDBinary)
	test.That(t, err, test.ShouldBeNil)
}
