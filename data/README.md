# Sample data

| Directory | Frames | Content |
|---|---|---|
| `sequence_01/` | 22 | city street, ego vehicle driving |
| `sequence_02/` | 154 | city street with a cyclist riding ahead of the ego vehicle; used for all figures in `docs/` |

Each frame is a binary PCD file with `x y z intensity` fields, about 120 000 points,
recorded at 10 Hz by a roof-mounted Velodyne HDL-64E. Points are stored laser by
laser, which lets the pipeline recover the exact ring of every point.

## Source and licence

The scans come from the [KITTI raw dataset](https://www.cvlibs.net/datasets/kitti/raw_data.php)
(Geiger, Lenz, Stiller and Urtasun, *Vision meets Robotics: The KITTI Dataset*,
IJRR 2013), converted to PCD. KITTI is published under the
[Creative Commons Attribution-NonCommercial-ShareAlike 3.0](https://creativecommons.org/licenses/by-nc-sa/3.0/)
licence, which also applies to these files: credit the KITTI authors and do not
use the data commercially.

## Using your own data

Any directory of PCD files works:

```bash
./build/lidar_od_run --data /path/to/pcds --out output/mine
```

Frames are processed in lexicographic order. Add a `timestamps.txt` with one time
in seconds per frame (same order) when frames are not evenly spaced at
`sensor.frame_period`. For other sensors, adjust the `sensor` section of the
config (height, ring count, vertical field of view).
