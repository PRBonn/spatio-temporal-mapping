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
from pathlib import Path
import numpy as np
import yaml
import open3d as o3d
from pyquaternion import Quaternion


def load_camera_parameters(filename: Path):
    with open(str(filename), "r") as file:
        yaml_file = yaml.load(file, Loader=yaml.FullLoader)
    img_width = yaml_file["width"]
    img_height = yaml_file["height"]
    intrinsics = yaml_file["intrinsics"]
    extrinsics = yaml_file["extrinsics"]

    intrinsic_matrix = o3d.camera.PinholeCameraIntrinsic(
        img_width,
        img_height,
        intrinsics[0][0],
        intrinsics[1][1],
        intrinsics[0][2],
        intrinsics[1][2],
    ).intrinsic_matrix

    extrinsic_matrix = np.eye(4)
    extrinsic_matrix[0:3, 0:3] = Quaternion(
        x=extrinsics[1][0],
        y=extrinsics[1][1],
        z=extrinsics[1][2],
        w=extrinsics[1][3],
    ).rotation_matrix
    extrinsic_matrix[0:3, 3] = [extrinsics[0][0], extrinsics[0][1], extrinsics[0][2]]

    return (
        intrinsic_matrix,
        extrinsic_matrix,
        img_width,
        img_height,
    )


def load_kitti_poses(filename: Path) -> np.ndarray:
    poses = np.loadtxt(str(filename), delimiter=" ")
    n = poses.shape[0]
    poses = np.concatenate(
        (poses, np.zeros((n, 3), dtype=np.float32), np.ones((n, 1), dtype=np.float32)),
        axis=1,
    )
    poses = poses.reshape((n, 4, 4))  # [N, 4, 4]
    return poses


def save_kitti_poses(filename: Path, poses: np.ndarray):
    def _to_kitti_format(poses: np.ndarray) -> np.ndarray:
        return np.array([np.concatenate((pose[0], pose[1], pose[2])) for pose in poses])

    np.savetxt(fname=f"{filename}", X=_to_kitti_format(poses))


def save_point_cloud(filename: Path, points: np.ndarray, colors: np.ndarray):
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud(str(filename), pcd)
