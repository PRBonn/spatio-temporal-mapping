# MIT License
#
# Copyright (c) 2024 Luca Lobefaro, Meher V.R. Malladi, Tiziano Guadagnino, Cyrill Stachniss
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import os
from abc import ABC
import polyscope as ps
import polyscope.imgui as psim
import cv2

import numpy as np

# OLD PARAMETERS: TODO remove them
YELLOW = np.array([1, 0.706, 0])
RED = [1.0, 0.0, 0.0]
BLACK = np.array([0, 0, 0])
WHITE = np.array([1.0, 1.0, 1.0])
BLUE = np.array([0.4, 0.5, 0.9])
ARB = np.array([0.1, 0.5, 0.6])
SPHERE_SIZE = 0.20

BACKGROUND_COLOR = [0.8470, 0.8588, 0.8863]


class StubVisualizer(ABC):
    def __init__(self):
        pass

    def update(self, pose, rgb_img, frame_pcd):
        pass

    def quit(self):
        pass


def start_pause_callback():
    button_name = "PAUSE" if PangolinoVisualizer.play_mode else "START"
    if psim.Button(button_name) or psim.IsKeyPressed(psim.ImGuiKey_Space):
        PangolinoVisualizer.play_mode = not PangolinoVisualizer.play_mode


def next_frame_callback():
    if psim.Button("NEXT FRAME") or psim.IsKeyPressed(psim.ImGuiKey_N):
        PangolinoVisualizer.block_execution = not PangolinoVisualizer.block_execution


def quit_callback():
    if (
        psim.Button("QUIT")
        or psim.IsKeyPressed(psim.ImGuiKey_Escape)
        or psim.IsKeyPressed(psim.ImGuiKey_Q)
    ):
        PangolinoVisualizer.quit()
        os._exit(0)


def main_gui_callback():
    start_pause_callback()
    if not PangolinoVisualizer.play_mode:
        psim.SameLine()
        next_frame_callback()
    psim.Separator()
    quit_callback()


class PangolinoVisualizer(StubVisualizer):
    # Static parameteters
    play_mode: bool = False
    block_execution: bool = True

    def __init__(self):
        # Initialize visualizer
        self._initialize_visualizer()
        self._img_window = cv2.namedWindow("Image Stream")

        # Initilize attributes
        self._camera_poses = []

    def update(self, pose, rgb_img, frame_pcd):
        # Visualize image
        cv2.imshow("Image Stream", rgb_img)
        cv2.waitKey(1)

        # Visualize camera pose
        self._update_camera_pose(pose)

        # Visualize frame point cloud
        self._visualize_point_cloud(frame_pcd)

        # Visualization loop
        self._update_visualizer()

    def _initialize_visualizer(self):
        ps.set_program_name("Spatio Temporal Mapping Visualizer")
        ps.init()
        ps.set_background_color(BACKGROUND_COLOR)
        ps.set_verbosity(0)
        ps.set_ground_plane_mode("none")
        ps.set_user_callback(main_gui_callback)
        ps.set_build_default_gui_panels(False)

    def _update_camera_pose(self, pose):
        pass

    # Create new camera pose
    # camera_params = ps.CameraParameters(
    #     ps.CameraIntrinsics(fov_vertical_deg=30, aspect=2),
    #     ps.CameraExtrinsics(mat=np.linalg.inv(pose)),
    # )
    # new_cam = ps.register_camera_view(f"cam {self._next_cam_number}", camera_params)
    # new_cam.set_widget_color(RED)
    # self._next_cam_number += 1
    # self._cameras.append(new_cam)

    # # Resize and re-color previous camera
    # if len(self._cameras) > 1:
    #     self._cameras[-2].set_widget_color(BLACK)
    #     self._cameras[-2].set_widget_focal_length(0.02)

    def _visualize_point_cloud(self, frame_pcd):
        points, colors = frame_pcd.get_points_and_colors()
        cloud = ps.register_point_cloud("frame_pcd", points, point_render_mode="quad")
        cloud.add_color_quantity("colors", colors, enabled=True)
        cloud.set_radius(0.01, relative=False)

    def _update_visualizer(self):
        while PangolinoVisualizer.block_execution:
            ps.frame_tick()
            if PangolinoVisualizer.play_mode:
                break
        PangolinoVisualizer.block_execution = not PangolinoVisualizer.block_execution

    @staticmethod
    def quit():
        ps.unshow()
        cv2.destroyAllWindows()
        print("Visualizer bye bye!")
