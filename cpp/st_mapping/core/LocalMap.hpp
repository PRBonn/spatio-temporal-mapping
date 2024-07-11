// MIT License
//
// Copyright (c) 2024 Luca Lobefaro, Meher V.R. Malladi, Tiziano Guadagnino, Cyrill Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <tsl/robin_map.h>

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <vector>

#include "Mapping.hpp"

namespace st_mapping {

class LocalMap {
public:
    // Constructors
    LocalMap(const Sophus::SE3d &camera_dislacement,
             const double voxel_size,
             const double map_size,
             const int max_points_per_voxel = 1)
        : _camera_displacement(camera_dislacement),
          _voxel_size(voxel_size),
          _map_size(map_size),
          _max_points_per_voxel(max_points_per_voxel){};
    LocalMap(const LocalMap &other) = delete;
    LocalMap(LocalMap &&other) = default;
    LocalMap &operator=(const LocalMap &other) = delete;
    LocalMap &operator=(LocalMap &&other) = default;

    using Voxel = Eigen::Vector3i;
    struct VoxelBlock {
        PointCloud _pcd;
        int _max_points;
        inline bool AddPoint(const PointWithColor &point) {
            if (_pcd.size() < static_cast<size_t>(_max_points)) {
                _pcd.push_back(point);
                return true;
            }
            return false;
        }
    };
    struct VoxelHash {
        size_t operator()(const Voxel &voxel) const {
            const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
            return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
        }
    };

    // Methods to augment point cloud
    PointCloud IntegratePointCloud(const PointCloud &pcd,
                                   const Sophus::SE3d &T,
                                   const bool pose_only_for_resize = false);

    // Methods to access
    inline bool Empty() const { return this->_voxel_map.empty(); }
    std::vector<Eigen::Vector3d> GetPoints(const std::vector<Voxel> &query_voxels) const;
    PointsAndColor GetPointsAndColors() const;
    inline Voxel PointToVoxel(const Eigen::Vector3d &point) const {
        return Voxel(static_cast<int>(std::floor(point.x() / _voxel_size)),
                     static_cast<int>(std::floor(point.y() / _voxel_size)),
                     static_cast<int>(std::floor(point.z() / _voxel_size)));
    }

private:
    void _Resize(const Eigen::Vector3d &center);
    const Sophus::SE3d _camera_displacement;
    double _voxel_size;
    double _map_size;
    int _max_points_per_voxel;
    tsl::robin_map<Voxel, VoxelBlock, VoxelHash> _voxel_map;
};

}  // namespace st_mapping
