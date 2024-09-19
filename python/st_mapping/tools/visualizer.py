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
import polyscope.imgui as gui
import cv2

import numpy as np

# Buttons names
START_BUTTON = "START [SPACE]"
PAUSE_BUTTON = "PAUSE [SPACE]"
NEXT_FRAME_BUTTON = "NEXT FRAME [N]"
QUIT_BUTTON = "QUIT [Q]"

# Colors
BACKGROUND_COLOR = [0.8470, 0.8588, 0.8863]
CAMERA_COLOR = [1.0, 0.0, 0.0]

# Size constants
CLOUD_POINT_SIZE = 0.004
POINTS_SIZE_STEP = 0.001
POINTS_SIZE_MIN = 0.001
POINTS_SIZE_MAX = 0.01
INSTANCES_CENTERS_SIZE = 0.01
LINES_SIZE = 0.003


class StubVisualizer(ABC):
    def __init__(self):
        pass

    def update(self, robot_pose, camera_pose, rgb_img, frame_pcd, map):
        pass

    def register_reference_map(self, ref_map):
        pcd

    def keep_running(self):
        pass

    def quit(self):
        pass


class MappingVisualizer(StubVisualizer):
    # --- PUBLIC INTERFACE ---------------------------------------------------------------
    def __init__(self):
        # Initialize visualizer
        self._initialize_visualizer()

        # Initilize attributes
        self._camera_poses = []

        # Initilize GUI controls and attributes
        self._play_mode: bool = False
        self._block_execution: bool = True
        self._frame_points_size: float = CLOUD_POINT_SIZE + POINTS_SIZE_STEP
        self._map_points_size: float = CLOUD_POINT_SIZE - POINTS_SIZE_STEP
        self._frame_transparency: float = 1.0
        self._map_transparency: float = 0.9

        # Initialize visualizer
        self._initialize_visualizer()

    def update(self, robot_pose, camera_pose, rgb_img, frame_pcd, map):
        # Visualize image
        self._visualize_image(rgb_img)

        # Visualize camera pose
        self._update_camera_pose(camera_pose)

        # Visualize point clouds
        self._visualize_current_frame(frame_pcd, robot_pose)
        self._visualize_map(map)

        # Visualization loop
        self._update_visualizer()

    def register_reference_map(self, ref_map):
        points, colors = ref_map.get_points_and_colors()
        cloud = ps.register_point_cloud(
            "ref_map_pcd",
            points,
            point_render_mode="quad",
            transparency=self._map_transparency,
        )
        cloud.add_color_quantity("colors", colors, enabled=True)
        cloud.set_radius(self._map_points_size, relative=False)
        cloud.set_transparency(0.2)

    def keep_running(self):
        cv2.destroyAllWindows()
        self._play_mode = False
        self._block_execution = True
        ps.get_camera_view("camera").set_enabled(False)
        ps.get_point_cloud("frame_pcd").set_enabled(False)
        ps.set_user_callback(self._keep_running_callback)
        ps.show()

    # --- PRIVATE INTERFACE --------------------------------------------------------------
    def _initialize_visualizer(self):
        ps.set_program_name("Spatio-Temporal Mapping Visualizer")
        ps.init()
        ps.set_background_color(BACKGROUND_COLOR)
        ps.set_verbosity(0)
        ps.set_ground_plane_mode("none")
        ps.set_user_callback(self._main_gui_callback)
        ps.set_build_default_gui_panels(False)
        cv2.namedWindow("Camera Stream")

    def _visualize_image(self, img):
        rgb_img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        cv2.imshow("Camera Stream", rgb_img)
        cv2.setWindowProperty("Camera Stream", cv2.WND_PROP_TOPMOST, 1)
        cv2.waitKey(1)

    def _update_camera_pose(self, pose):
        self._camera_poses.append(pose)
        camera_params = ps.CameraParameters(
            ps.CameraIntrinsics(fov_vertical_deg=30, aspect=2),
            ps.CameraExtrinsics(mat=np.linalg.inv(pose)),
        )
        new_cam = ps.register_camera_view("camera", camera_params)
        new_cam.set_widget_color(CAMERA_COLOR)

    def _visualize_current_frame(self, frame_pcd, pose):
        points, colors = frame_pcd.get_points_and_colors()
        cloud = ps.register_point_cloud(
            "frame_pcd",
            points,
            point_render_mode="quad",
            transparency=self._frame_transparency,
        )
        cloud.add_color_quantity("colors", colors, enabled=True)
        cloud.set_radius(self._frame_points_size, relative=False)
        cloud.set_transparency(self._frame_transparency)
        cloud.set_transform(pose)

    def _visualize_map(self, map):
        points, colors = map.get_points_and_colors()
        cloud = ps.register_point_cloud(
            "map_pcd",
            points,
            point_render_mode="quad",
            transparency=self._map_transparency,
        )
        cloud.add_color_quantity("colors", colors, enabled=True)
        cloud.set_radius(self._map_points_size, relative=False)
        cloud.set_transparency(self._map_transparency)

    def _update_visualizer(self):
        while self._block_execution:
            ps.frame_tick()
            if self._play_mode:
                break
        self._block_execution = not self._block_execution

    # --- GUI Callbacks ------------------------------------------------------------------
    def _start_pause_callback(self):
        button_name = PAUSE_BUTTON if self._play_mode else START_BUTTON
        if gui.Button(button_name) or gui.IsKeyPressed(gui.ImGuiKey_Space):
            self._play_mode = not self._play_mode

    def _next_frame_callback(self):
        if gui.Button(NEXT_FRAME_BUTTON) or gui.IsKeyPressed(gui.ImGuiKey_N):
            self._block_execution = not self._block_execution

    def _frame_points_size_callback(self):
        key_changed = False
        if gui.IsKeyPressed(gui.ImGuiKey_Minus):
            self._frame_points_size = max(
                POINTS_SIZE_MIN, self._frame_points_size - POINTS_SIZE_STEP
            )
            key_changed = True
        if gui.IsKeyPressed(gui.ImGuiKey_Equal):
            self._frame_points_size = min(
                POINTS_SIZE_MAX, self._frame_points_size + POINTS_SIZE_STEP
            )
            key_changed = True
        changed, self._frame_points_size = gui.SliderFloat(
            "Frame points Size",
            self._frame_points_size,
            v_min=POINTS_SIZE_MIN,
            v_max=POINTS_SIZE_MAX,
        )
        if changed or key_changed:
            ps.get_point_cloud("frame_pcd").set_radius(
                self._frame_points_size, relative=False
            )

    def _map_points_size_callback(self):
        key_changed = False
        if gui.IsKeyPressed(gui.ImGuiKey_Minus):
            self._map_points_size = max(
                POINTS_SIZE_MIN, self._map_points_size - POINTS_SIZE_STEP
            )
            key_changed = True
        if gui.IsKeyPressed(gui.ImGuiKey_Equal):
            self._map_points_size = min(
                POINTS_SIZE_MAX, self._map_points_size + POINTS_SIZE_STEP
            )
            key_changed = True
        changed, self._map_points_size = gui.SliderFloat(
            "Map points Size",
            self._map_points_size,
            v_min=POINTS_SIZE_MIN,
            v_max=POINTS_SIZE_MAX,
        )
        if changed or key_changed:
            ps.get_point_cloud("map_pcd").set_radius(
                self._map_points_size, relative=False
            )

    def _frame_transparency_callback(self):
        changed, self._frame_transparency = gui.SliderFloat(
            "Frame Transparency",
            self._frame_transparency,
            v_min=0,
            v_max=1.0,
        )
        if changed:
            ps.get_point_cloud("frame_pcd").set_transparency(self._frame_transparency)

    def _map_transparency_callback(self):
        changed, self._map_transparency = gui.SliderFloat(
            "Map Transparency",
            self._map_transparency,
            v_min=0,
            v_max=1.0,
        )
        if changed:
            ps.get_point_cloud("map_pcd").set_transparency(self._map_transparency)

    def _quit_callback(self):
        posX = (
            gui.GetCursorPosX()
            + gui.GetColumnWidth()
            - gui.CalcTextSize(QUIT_BUTTON)[0]
            - gui.GetScrollX()
            - gui.ImGuiStyleVar_ItemSpacing
        )
        gui.SetCursorPosX(posX)
        if (
            gui.Button(QUIT_BUTTON)
            or gui.IsKeyPressed(gui.ImGuiKey_Escape)
            or gui.IsKeyPressed(gui.ImGuiKey_Q)
        ):
            print("Visualizer Bye Bye!")
            ps.unshow()
            cv2.destroyAllWindows()
            os._exit(0)

    def _main_gui_callback(self):
        self._start_pause_callback()
        if not self._play_mode:
            gui.SameLine()
            self._next_frame_callback()
        gui.Separator()
        gui.TextUnformatted("Scene Options:")
        self._frame_points_size_callback()
        self._frame_transparency_callback()
        self._map_points_size_callback()
        self._map_transparency_callback()
        gui.Separator()
        self._quit_callback()

    def _keep_running_callback(self):
        gui.TextUnformatted("Scene Options:")
        self._map_points_size_callback()
        self._map_transparency_callback()
        gui.Separator()
        self._quit_callback()
