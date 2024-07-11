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
#include "GlobalMap.hpp"

#include <algorithm>

#include "Mapping.hpp"

namespace st_mapping {

void GlobalMap::IntegratePointCloud(const PointCloud &pcd, const Sophus::SE3d &T) {
    std::for_each(pcd.cbegin(), pcd.cend(), [&](const PointWithColor &colored_point) {
        const auto &[point, color] = colored_point;
        Eigen::Vector3d transformed_point = T * point;
        _points.push_front({transformed_point, color});
        _n_points++;
    });
}

// TODO: can this be parallelized
PointsAndColor GlobalMap::GetPointsAndColors() const {
    std::vector<Eigen::Vector3d> points, colors;
    points.reserve(_n_points);
    colors.reserve(_n_points);
    std::for_each(_points.cbegin(), _points.cend(), [&](const PointWithColor &colored_point) {
        const auto &[point, color] = colored_point;
        points.emplace_back(point);
        colors.emplace_back(color);
    });
    return std::make_pair(points, colors);
}

}  // namespace st_mapping
