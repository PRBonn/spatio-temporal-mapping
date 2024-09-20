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
import numpy as np
import cv2
import open3d as o3d


def visualize_point_cloud(points: np.ndarray, colors: np.ndarray):
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd])


def visualize_aligned_point_clouds(
    points: np.ndarray,
    colors: np.ndarray,
    ref_points: np.ndarray,
    ref_colors: np.ndarray,
):
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    pcd.colors = o3d.utility.Vector3dVector(colors)
    ref_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(ref_points))
    ref_pcd.colors = o3d.utility.Vector3dVector([[255, 0, 0] for _ in ref_colors])
    o3d.visualization.draw_geometries([pcd, ref_pcd])


def generate_visual_matches_image(
    img1: np.ndarray,
    img2: np.ndarray,
    pts1: np.ndarray,
    pts2: np.ndarray,
    space_between_images: int = 100,
):
    # Create a new output image that concatenates the two images together
    # (a.k.a) a montage
    rows1 = img1.shape[0]
    cols1 = img1.shape[1]
    rows2 = img2.shape[0]
    cols2 = img2.shape[1]
    out = (
        np.ones(
            (rows1 + rows2 + space_between_images, max([cols1, cols2]), 3),
            dtype="uint8",
        )
        * 255
    )

    # Place the second image to the left
    out[:rows2, :cols2, :] = img2

    # Place the next image to the right of it
    out[
        rows2 + space_between_images : rows2 + rows1 + space_between_images, :cols1, :
    ] = img1

    # For each pair of matches draw a circle and a line connecting them
    for pt_idx in range(pts1.shape[0]):
        coordinates_pt1 = (
            pts1[pt_idx][0],
            pts1[pt_idx][1] + rows2 + space_between_images,
        )
        coordinates_pt2 = (pts2[pt_idx][0], pts2[pt_idx][1])
        out = cv2.circle(
            out,
            coordinates_pt1,
            2,
            (255, 0, 0),
            2,
        )
        out = cv2.circle(out, coordinates_pt2, 2, (255, 0, 0), 2)
        out = cv2.line(out, coordinates_pt1, coordinates_pt2, (0, 255, 0), 1)

    return out


def visualize_3dmatches(
    points: np.ndarray, colors: np.ndarray, pts1: np.ndarray, pts2: np.ndarray
):
    # Initialization
    n_points = pts1.shape[0]

    # Build pcd for the points
    pcd1 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts1))
    pcd1.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(0, n_points)])
    pcd2 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts2))
    pcd2.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(0, n_points)])

    # Build pcd for the ref map
    pcd_map = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    pcd_map.colors = o3d.utility.Vector3dVector(colors)

    # Build the lines
    lines_points = np.concatenate((pts1, pts2), axis=0)
    lines_edges = [[i, i + n_points] for i in range(0, n_points)]
    lines_colors = o3d.utility.Vector3dVector([[1, 0, 0] for el in lines_edges])
    lines = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(lines_points),
        lines=o3d.utility.Vector2iVector(lines_edges),
    )
    lines.colors = o3d.utility.Vector3dVector(lines_colors)

    o3d.visualization.draw_geometries([pcd1, pcd2, pcd_map, lines])
