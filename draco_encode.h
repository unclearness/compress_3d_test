#include "Eigen/Geometry"

namespace draco_encode {

struct Options {
  Options();

  // bool is_point_cloud;
  int pos_quantization_bits;
  int tex_coords_quantization_bits;
  // bool tex_coords_deleted;
  // int normals_quantization_bits;
  // bool normals_deleted;
  // int generic_quantization_bits;
  // bool generic_deleted;
  int compression_level;
  // bool preserve_polygons;
  // bool use_metadata;
  // std::string input;
  // std::string output;
};

bool EncodeMesh(const std::vector<Eigen::Vector3f>& verts,
                const std::vector<Eigen::Vector2f>& uvs,
                const std::vector<Eigen::Vector3i>& indices,
                const std::vector<Eigen::Vector3i>& uv_indices,
                Options& options, std::vector<char>& bytes);

}  // namespace draco_encode
