#include "draco_encode.h"
#include "ugu/mesh.h"
#include "ugu/timer.h"

int main() {
  ugu::MeshPtr mesh = ugu::Mesh::Create();
  mesh->LoadObj("../third_party/ugu/data/bunny/bunny.obj");
  ugu::Timer timer;

  std::vector<char> bytes;
  draco_encode::Options options;
  options.pos_quantization_bits = 11;
  options.tex_coords_quantization_bits = 10;
  options.compression_level = 7;
  timer.Start();
  draco_encode::EncodeMesh(mesh->vertices(), mesh->uv(), mesh->vertex_indices(),
                           mesh->uv_indices(), options, bytes);
  timer.End();
  std::cout << "Encode time: " << timer.elapsed_msec() << " ms" << std::endl;

  size_t org_size = mesh->vertices().size() * sizeof(float) * 3 +
                    mesh->uv().size() * sizeof(float) * 2 +
                    mesh->vertex_indices().size() * sizeof(int) * 3 +
                    mesh->uv_indices().size() * sizeof(int) * 3;

  std::cout << "Original size: " << org_size / 1024 << " kb" << std::endl;
  std::cout << "Draco size: " << bytes.size() / 1024 << " kb" << std::endl;

  std::ofstream ofs("encode.drc", std::ios::binary);
  for (const char& b : bytes) {
    ofs << b;
  }

  return 0;
}
