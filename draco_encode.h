#pragma once

#include "Eigen/Geometry"

namespace draco_encode {

struct DracoEncodeOptions {
  DracoEncodeOptions();
  int pos_quantization_bits;
  int tex_coords_quantization_bits;
  int compression_level;
};

bool DracoEncodeMesh(const std::vector<Eigen::Vector3f>& verts,
                     const std::vector<Eigen::Vector2f>& uvs,
                     const std::vector<Eigen::Vector3i>& indices,
                     const std::vector<Eigen::Vector3i>& uv_indices,
                     const DracoEncodeOptions& options,
                     std::vector<char>& bytes);

}  // namespace draco_encode
