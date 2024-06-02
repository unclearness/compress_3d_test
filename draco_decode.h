#pragma once

#include "Eigen/Geometry"

namespace draco_decode {

bool DracoDecode(const std::vector<char>& bytes,
                 std::vector<Eigen::Vector3f>& verts,
                 std::vector<Eigen::Vector2f>& uvs,
                 std::vector<Eigen::Vector3i>& indices,
                 std::vector<Eigen::Vector3i>& uv_indices,
                 std::vector<Eigen::Vector<uint8_t, 3>>& colors,
                 std::vector<Eigen::Vector3f>& normals);

}  // namespace draco_decode
