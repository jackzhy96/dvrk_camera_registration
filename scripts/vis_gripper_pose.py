#!/usr/bin/env python3

# Author: Juan Antonio Barragan  
# Date: 2024-04-19

# (C) Copyright 2022 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))

from dataclasses import dataclass, field
import time
import click
from pathlib import Path
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import crtk
from dvrk_camera_registration import psm
from tf_conversions.posemath import toMatrix
import json


@dataclass
class ImageSubscriber:
    rect_image_topic: str
    camera_info_topic: str
    current_frame: np.ndarray = field(default=None, init=False)
    camera_matrix: np.ndarray = field(default=None, init=False)

    def __post_init__(self):
        # rospy.init_node("image_subscriber", anonymous=True)
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(
            self.rect_image_topic, Image, self._img_callback
        )
        self.info_subscriber = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._info_callback
        )

    def _img_callback(self, data):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def _info_callback(self, info_msg):
        projection_matrix = np.array(info_msg.P).reshape((3, 4))
        self.camera_matrix = projection_matrix[0:3, 0:3]
        self.camera_frame = info_msg.header.frame_id

    def wait_until_first_frame(self):
        print("Waiting for image topic...")
        timeout = 10
        start = time.time()
        while self.current_frame is None:
            if time.time() - start > timeout:
                raise TimeoutError("Timeout waiting for first frame")

            time.sleep(0.2)


@dataclass
class PoseAnnotator:
    camera_matrix: np.ndarray
    cam_T_base: np.ndarray
    dist_coeffs: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0, 0.0, 0.0]).reshape((-1, 1)),
        init=False,
    )

    def __post_init__(self):
        print(self.dist_coeffs.shape)

    def draw_pose_on_img(self, img: np.ndarray, local_measured_cp: np.ndarray):
        pose = self.cam_T_base @ local_measured_cp

        tvec = pose[:3, 3]
        rvec = cv2.Rodrigues(pose[:3, :3])[0]

        points_3d = np.array([[[0, 0, 0]]], np.float32)
        points_2d, _ = cv2.projectPoints(
            points_3d, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )

        # print(points_2d)
        # print(pose_data[idx])
        # print(img.shape)

        points_2d = tuple(points_2d.astype(np.int32)[0, 0])

        # print(points_2d)

        img = cv2.circle(img, points_2d, 10, (0, 0, 255), -1)
        img = self.draw_axis(img, self.camera_matrix, self.dist_coeffs, pose, size=0.01)

        return img

    def draw_axis(
        self,
        img: np.ndarray,
        mtx: np.ndarray,
        dist: np.ndarray,
        pose: np.ndarray,
        size: int = 10,
    ):

        s = size
        thickness = 2
        R, t = pose[:3, :3], pose[:3, 3]
        K = mtx

        rotV, _ = cv2.Rodrigues(R)
        points = np.float32([[s, 0, 0], [0, s, 0], [0, 0, s], [0, 0, 0]]).reshape(-1, 3)
        axisPoints, _ = cv2.projectPoints(points, rotV, t, K, dist)
        axisPoints = axisPoints.astype(int)

        img = cv2.line(
            img,
            tuple(axisPoints[3].ravel()),
            tuple(axisPoints[0].ravel()),
            (255, 0, 0),
            thickness,
        )
        img = cv2.line(
            img,
            tuple(axisPoints[3].ravel()),
            tuple(axisPoints[1].ravel()),
            (0, 255, 0),
            thickness,
        )

        img = cv2.line(
            img,
            tuple(axisPoints[3].ravel()),
            tuple(axisPoints[2].ravel()),
            (0, 0, 255),
            thickness,
        )
        return img


def run_pose_visualizer(
    arm_handle: psm.PSM,
    img_topic: str,
    camera_info_topic: str,
    cam_T_robot_base: np.ndarray,
):

    img_subscriber = ImageSubscriber(img_topic, camera_info_topic)
    img_subscriber.wait_until_first_frame()

    pose_annotator = PoseAnnotator(img_subscriber.camera_matrix, cam_T_robot_base)

    window_name = img_topic
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 640, 480)

    while not rospy.is_shutdown():

        img = img_subscriber.current_frame
        local_measured_cp = toMatrix(arm_handle.local.measured_cp())

        img = pose_annotator.draw_pose_on_img(img, local_measured_cp)
        cv2.imshow(window_name, img)
        k = cv2.waitKey(1)

        if k == 27 or k == ord("q"):
            break

    cv2.destroyAllWindows()


def load_hand_eye_calibration(json_file: Path) -> np.ndarray:
    with open(json_file, "r") as f:
        data = json.load(f)

    cam_T_robot_base = np.array(data['base-frame']['transform']).reshape(4, 4)
    return cam_T_robot_base


@click.command()
@click.option(
    "-c",
    "--img_topic",
    default="/jhu_daVinci/decklink/left/image_rect_color",
    help="Rectified camera image topic",
)
@click.option(
    "-t",
    "--camera_info_topic",
    default="/jhu_daVinci/decklink/left/camera_info",
    help="Camera info topic",
)
@click.option(
    "-h",
    "--hand_eye_json",
    default="sample_registrations/PSM2_registration_small_grid.json",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    help="Hand eye calibration json file",
)
def main(img_topic: str, camera_info_topic: str, hand_eye_json: Path):
    """
    Use hand-eye calibration to visualize the gripper's pose in the camera frame
    """

    ral = crtk.ral("dvrk_test")
    arm_handle = psm.PSM(ral, arm_name="PSM2", expected_interval=0.1)
    ral.check_connections()
    cv2.setNumThreads(2)
    cam_T_robot_base = load_hand_eye_calibration(hand_eye_json)

    print(f"measured_jp {arm_handle.local.measured_cp()}")
    run_pose_visualizer(arm_handle, img_topic, camera_info_topic, cam_T_robot_base)


if __name__ == "__main__":
    main()
