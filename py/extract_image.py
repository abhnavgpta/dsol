import cv2
from pathlib import Path
from tqdm import tqdm

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

if __name__ == '__main__':
    bag_file = "/media/psf/robo/bagfiles/car_loop_onboard.bag"
    bag_name = "car_loop_onboard"
    bag = rosbag.Bag(bag_file, "r")
    bridge = CvBridge()
    count = 0
    # topics = [
        # "/device_0/sensor_0/Infrared_1/image/data",
        # "/device_0/sensor_0/Infrared_2/image/data"
    # ]
    topics = [
        "/camera/infra1/image_rect_raw",
        "/camera/infra2/image_rect_raw"
    ]

    data_dir = Path("/media/psf/robo/bagfiles/realsense") / bag_name
    left_dir = data_dir / "infra1"
    right_dir = data_dir / "infra2"
    left_dir.mkdir(parents=True, exist_ok=True)
    right_dir.mkdir(parents=True, exist_ok=True)

    for topic, msg, t in tqdm(bag.read_messages(topics=topics)):
        count = t.to_nsec() // 1000000

        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if topic == topics[0]:
            cv2.imwrite(str(left_dir / f"image{count:08d}.png"), cv_img)
        elif topic == topics[1]:
            cv2.imwrite(str(right_dir / f"image{count:08d}.png"), cv_img)
    bag.close()
