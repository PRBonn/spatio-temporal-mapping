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
#include "LocalMap.hpp"

#include <tsl/robin_map.h>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "Mapping.hpp"

namespace st_mapping {

PointCloud LocalMap::IntegratePointCloud(const PointCloud &pcd,
                                         const Sophus::SE3d &T,
                                         const bool pose_only_for_resize) {
    Sophus::SE3d T_transform = T;
    if (pose_only_for_resize) T_transform = Sophus::SE3d();
    PointCloud integrated_pcd;
    integrated_pcd.reserve(pcd.size());
    std::for_each(pcd.cbegin(), pcd.cend(), [&](const auto &colored_point) {
        const auto &[point, color] = colored_point;
        Eigen::Vector3d transformed_point = T_transform * point;
        auto voxel = PointToVoxel(transformed_point);
        auto search = this->_voxel_map.find(voxel);
        if (search != this->_voxel_map.end()) {
            auto &voxel_block = search.value();
            if (voxel_block.AddPoint({transformed_point, color}))
                integrated_pcd.emplace_back(colored_point);
        } else {
            this->_voxel_map.insert(
                {voxel, VoxelBlock{{{transformed_point, color}}, this->_max_points_per_voxel}});
            integrated_pcd.emplace_back(colored_point);
        }
    });
    _Resize((T * _camera_displacement).translation());
    return integrated_pcd;
}

std::vector<Eigen::Vector3d> LocalMap::GetPoints(const std::vector<Voxel> &query_voxels) const {
    std::vector<Eigen::Vector3d> points;
    points.reserve(query_voxels.size() * static_cast<size_t>(this->_max_points_per_voxel));
    std::for_each(query_voxels.cbegin(), query_voxels.cend(), [&](const auto &query) {
        auto search = this->_voxel_map.find(query);
        if (search != this->_voxel_map.end()) {
            for (const auto &[pt, color] : search.value()._pcd) {
                points.emplace_back(pt);
            }
        }
    });
    points.shrink_to_fit();
    return points;
}

PointsAndColor LocalMap::GetPointsAndColors() const {
    std::vector<Eigen::Vector3d> points, colors;
    points.reserve(_max_points_per_voxel * _voxel_map.size());
    colors.reserve(_max_points_per_voxel * _voxel_map.size());
    std::for_each(_voxel_map.cbegin(), _voxel_map.cend(), [&](const auto &voxel_and_block) {
        const auto &[voxel, voxel_block] = voxel_and_block;
        (void)voxel;
        for (const auto &[pt, color] : voxel_block._pcd) {
            points.emplace_back(pt);
            colors.emplace_back(color);
        }
    });
    points.shrink_to_fit();
    colors.shrink_to_fit();
    return std::make_pair(points, colors);
}

void LocalMap::_Resize(const Eigen::Vector3d &center) {
    const auto dimension2 = _map_size * _map_size;
    for (auto it = this->_voxel_map.begin(); it != this->_voxel_map.end();) {
        const auto &[voxel, voxel_block] = *it;
        const auto &[pt, color] = voxel_block._pcd.front();
        if ((pt - center).squaredNorm() >= (dimension2)) {
            it = this->_voxel_map.erase(it);
        } else {
            ++it;
        }
    }
}

}  // namespace st_mapping
