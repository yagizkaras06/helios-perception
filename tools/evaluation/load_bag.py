"""Extracts /rslidar_points from a rosbag2 (mcap) into a compressed npz (stamps + xyz per frame)."""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
bag, out = sys.argv[1], sys.argv[2]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
stamps, clouds = [], []
while r.has_next():
    topic, data, _ = r.read_next()
    if topic != '/rslidar_points':
        continue
    m = deserialize_message(data, PointCloud2)
    off = {f.name: f.offset for f in m.fields}
    raw = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
    xyz = np.stack([raw[:, off[c]:off[c] + 4].copy().view(np.float32)[:, 0] for c in 'xyz'], 1)
    xyz = xyz[np.isfinite(xyz).all(1)]
    stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    clouds.append(xyz.astype(np.float16))
lens = np.array([len(c) for c in clouds])
np.savez_compressed(out, stamps=np.array(stamps), lens=lens, xyz=np.concatenate(clouds))
print(len(stamps), 'frames', lens.mean(), 'pts/frame',
      [(f.name, f.offset, f.datatype) for f in m.fields])
