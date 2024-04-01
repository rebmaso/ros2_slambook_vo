from rosbags import convert as bagconv
from pathlib import Path

src_path = "/home/tommaso/Datasets/uhumans/uHumans2_apartment_s1_00h.bag"
dst_path = "/home/tommaso/Datasets/uhumans/uHumans2_apartment_s1_00h_ros2.bag"

src_path = Path(src_path)
dst_path = Path(dst_path)

bagconv.convert(src=src_path, dst=dst_path)

