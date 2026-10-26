# lidar_od API reference {#mainpage}

`lidar_od` detects and tracks objects in LiDAR point clouds with classical
geometry: no learned models. lidar_od::Pipeline runs the stages below on each
frame; every stage is an interface whose implementation is chosen by name in
the lidar_od::Config.

| Stage | Interface | Implementations |
|---|---|---|
| Range-image layout | lidar_od::computeScanLayout() | ring recovery from point order, elevation binning |
| Region of interest | lidar_od::cropToRoi() | |
| Ground segmentation | lidar_od::GroundSegmenter | lidar_od::RegionGroundSegmenter, lidar_od::RansacGroundSegmenter, lidar_od::RayGroundSegmenter, lidar_od::GridGroundSegmenter |
| Clustering | lidar_od::Clusterer | lidar_od::RangeImageClusterer, lidar_od::EuclideanClusterer |
| Boxes | lidar_od::BoxFitter | lidar_od::LShapeFitter, lidar_od::PcaFitter, lidar_od::AabbFitter |
| Ego-motion | lidar_od::EgoMotionEstimator | NDT, GICP, ICP |
| Tracking | lidar_od::MultiObjectTracker | lidar_od::ConstantVelocityKalman, lidar_od::solveAssignment() |

Minimal use:

```cpp
#include "lidar_od/io.h"
#include "lidar_od/pipeline.h"

lidar_od::Pipeline pipeline(lidar_od::loadConfig("config/default.yaml"));
for (const auto& frame : lidar_od::listSequence("data/sequence_02", 0.1)) {
    const lidar_od::FrameResult r = pipeline.process(lidar_od::loadPcd(frame.path), frame.timestamp);
    for (const auto& track : r.tracks)
        if (track.confirmed) { /* track.id, track.box_world, track.velocity, ... */ }
}
```

All indices in the results refer to the input cloud of the frame
(lidar_od::FrameResult::cloud). The algorithms, their trade-offs and measured
results are described in `docs/ALGORITHMS.md` in the repository.
