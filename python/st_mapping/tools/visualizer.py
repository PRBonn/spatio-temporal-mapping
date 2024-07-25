# ----------------------------------------------------------------------------
# NOTE: This file has been adapted from the Kiss-ICP project, but copyright
# still belongs to Kiss-ICP. All rights reserved
# ----------------------------------------------------------------------------
# -              Kiss-ICP: https://github.com/PRBonn/kiss-icp                -
# ----------------------------------------------------------------------------
# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
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
import importlib
import os
from abc import ABC
from functools import partial
from typing import Callable, List
import polyscope as ps
import polyscope.imgui as psim
import cv2

import numpy as np

YELLOW = np.array([1, 0.706, 0])
RED = [1.0, 0.0, 0.0]
BLACK = np.array([0, 0, 0])
WHITE = np.array([1.0, 1.0, 1.0])
BLUE = np.array([0.4, 0.5, 0.9])
ARB = np.array([0.1, 0.5, 0.6])
SPHERE_SIZE = 0.20


class StubVisualizer(ABC):
    def __init__(self):
        pass

    def update(self, pose, rgb_img):
        pass

    def quit(self):
        pass


class PangolinoVisualizer(StubVisualizer):
    def __init__(self):
        # Initialize visualizer
        self._initialize_visualizer()
        self._img_window = cv2.namedWindow("Image Stream")

        # Initialize GUI controls
        ps._block_execution = True
        ps._play_mode = False

        # Initilize attributes
        self._next_cam_number = 0
        self._cameras = []

    def update(self, pose, rgb_img):
        # Visualize image
        cv2.imshow("Image Stream", rgb_img)
        cv2.waitKey(1)

        # Visualize camera pose
        self._update_camera_pose(pose)

        # Visualization loop
        self._update_visualizer()

    def quit(self):
        ps.unshow()
        cv2.destroyAllWindows()
        pass

    def _initialize_visualizer(self):
        ps.init()
        ps.set_ground_plane_mode("none")
        ps.set_user_callback(PangolinoVisualizer._gui_callback)

    def _update_camera_pose(self, pose):
        # Create new camera pose
        camera_params = ps.CameraParameters(
            ps.CameraIntrinsics(fov_vertical_deg=60, aspect=2),
            ps.CameraExtrinsics(mat=pose),
        )
        new_cam = ps.register_camera_view(f"cam {self._next_cam_number}", camera_params)
        new_cam.set_widget_color(RED)
        self._next_cam_number += 1
        self._cameras.append(new_cam)

        # Resize and re-color previous camera
        if len(self._cameras) > 1:
            self._cameras[-2].set_widget_color(BLACK)
            self._cameras[-2].set_widget_focal_length(0.02)

    def _update_visualizer(self):
        while ps._block_execution:
            ps.frame_tick()
            if ps._play_mode:
                break
        ps._block_execution = not ps._block_execution

    @staticmethod
    def _gui_callback():
        # START/STOP
        if psim.Button("START/STOP"):
            ps._play_mode = not ps._play_mode

        # NEXT FRAME
        if not ps._play_mode:
            if psim.Button("NEXT FRAME"):
                ps._block_execution = not ps._block_execution

        # QUIT
        if psim.Button("QUIT"):
            print("Destroying Visualizer")
            ps.unshow()
            cv2.destroyAllWindows()
            os._exit(0)


class PosesVisualizer(StubVisualizer):
    def __init__(self):
        try:
            self._o3d = importlib.import_module("open3d")
        except ModuleNotFoundError as err:
            print(f'open3d is not installed on your system, run "pip install open3d"')
            exit(1)

        # Initialize GUI controls
        self._block_vis = True
        self._play_crun = False

        # Create data
        self._frames = []

        # Initialize visualizer
        self._vis = self._o3d.visualization.VisualizerWithKeyCallback()
        self._register_key_callbacks()
        self._initialize_visualizer()
        self._img_window = cv2.namedWindow("Image Stream")

    def quit(self):
        cv2.destroyAllWindows()
        self._vis.destroy_window()

    def update(self, pose, rgb_img):
        cv2.imshow("Image Stream", rgb_img)
        cv2.waitKey(1)

        new_frame = self._o3d.geometry.TriangleMesh.create_coordinate_frame(SPHERE_SIZE)
        new_frame.transform(pose)
        self._frames.append(new_frame)

        self._vis.add_geometry(self._frames[-1], reset_bounding_box=False)
        self._vis.reset_view_point(True)

        while self._block_vis:
            self._vis.poll_events()
            self._vis.update_renderer()
            if self._play_crun:
                break
        self._block_vis = not self._block_vis

    def _initialize_visualizer(self):
        w_name = self.__class__.__name__
        self._vis.create_window(window_name=w_name, width=1920, height=1080)
        self._set_black_background(self._vis)
        self._vis.reset_view_point(True)
        self._vis.get_render_option().point_size = 1
        print(
            f"{w_name} initialized. Press:\n"
            "\t[SPACE] to pause/start\n"
            "\t  [ESC] to exit\n"
            "\t    [N] to step\n"
            "\t    [C] to center the viewpoint\n"
            "\t    [W] to toggle a white background\n"
            "\t    [B] to toggle a black background\n"
        )

    def _register_key_callback(self, keys: List, callback: Callable):
        for key in keys:
            self._vis.register_key_callback(ord(str(key)), partial(callback))

    def _register_key_callbacks(self):
        self._register_key_callback(["Ā", "Q", "\x1b"], self._quit)
        self._register_key_callback([" "], self._start_stop)
        self._register_key_callback(["N"], self._next_frame)
        self._register_key_callback(["C"], self._center_viewpoint)
        self._register_key_callback(["B"], self._set_black_background)
        self._register_key_callback(["W"], self._set_white_background)

    def _set_black_background(self, vis):
        vis.get_render_option().background_color = np.array(BLACK)

    def _set_white_background(self, vis):
        vis.get_render_option().background_color = WHITE

    def _quit(self, vis):
        print("Destroying Visualizer")
        vis.destroy_window()
        os._exit(0)

    def _next_frame(self, vis):
        self._block_vis = not self._block_vis

    def _start_stop(self, vis):
        self._play_crun = not self._play_crun

    def _center_viewpoint(self, vis):
        vis.reset_view_point(True)
