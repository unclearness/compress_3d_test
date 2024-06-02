#pragma once

#include "Eigen/Geometry"

namespace draco_decode {

bool DracoDecodeMesh(const std::vector<char>& bytes,
                     std::vector<Eigen::Vector3f>& verts,
                     std::vector<Eigen::Vector2f>& uvs,
                     std::vector<Eigen::Vector3i>& indices,
                     std::vector<Eigen::Vector3i>& uv_indices);

}  // namespace draco_decode
