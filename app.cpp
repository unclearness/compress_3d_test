#include "draco_decode.h"
#include "draco_encode.h"
#include "ugu/mesh.h"
#include "ugu/timer.h"

int main() {
  ugu::MeshPtr mesh = ugu::Mesh::Create();
  mesh->LoadObj("../third_party/ugu/data/bunny/bunny.obj");
  ugu::Timer timer;

  std::vector<char> bytes;
  draco_encode::DracoEncodeOptions options;
  options.pos_quantization_bits = 11;
  options.tex_coords_quantization_bits = 10;
  options.compression_level = 7;
  timer.Start();
  draco_encode::DracoEncodeMesh(mesh->vertices(), mesh->uv(),
                                mesh->vertex_indices(), mesh->uv_indices(),
                                options, bytes);
  timer.End();
  std::cout << "Encode time: " << timer.elapsed_msec() << " ms" << std::endl;

  size_t org_size = mesh->vertices().size() * sizeof(float) * 3 +
                    mesh->uv().size() * sizeof(float) * 2 +
                    mesh->vertex_indices().size() * sizeof(int) * 3 +
                    mesh->uv_indices().size() * sizeof(int) * 3;

  std::cout << "Original size: " << org_size / 1024 << " kb" << std::endl;
  std::cout << "Draco size: " << bytes.size() / 1024 << " kb" << std::endl;

  std::string drc_path = "encode.drc";

  std::ofstream ofs(drc_path, std::ios::binary);
  for (const char& b : bytes) {
    ofs << b;
  }
  std::vector<Eigen::Vector3f> verts;
  std::vector<Eigen::Vector2f> uvs;
  std::vector<Eigen::Vector3i> indices;
  std::vector<Eigen::Vector3i> uv_indices;
  timer.Start();
  draco_decode::DracoDecodeMesh(bytes, verts, uvs, indices, uv_indices);
  timer.End();
  std::cout << "Decode time: " << timer.elapsed_msec() << " ms" << std::endl;

  ugu::MeshPtr decoded = ugu::Mesh::Create();
  decoded->set_vertices(verts);
  decoded->set_uv(uvs);
  decoded->set_vertex_indices(indices);
  decoded->set_uv_indices(uv_indices);
  decoded->set_default_material();
  auto mats = decoded->materials();
  mats[0].diffuse_tex = mesh->materials()[0].diffuse_tex;  // Copy texture
  decoded->set_materials(mats);

  decoded->WriteObj("from_draco.obj");

  return 0;
}
