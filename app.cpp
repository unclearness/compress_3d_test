#include "draco_decode.h"
#include "draco_encode.h"
#include "ugu/mesh.h"
#include "ugu/timer.h"
#include "ugu/util/path_util.h"

void TestObjMesh() {
  std::cout << "wavefront obj with multiple uvs per vertex" << std::endl;

  ugu::MeshPtr mesh = ugu::Mesh::Create();
  mesh->LoadObj("../data/bunny.obj");
  ugu::Timer timer;

  std::vector<char> bytes;
  draco_encode::DracoEncodeOptions options;
  options.pos_quantization_bits = 11;
  options.tex_coords_quantization_bits = 10;
  options.compression_level = 7;
  timer.Start();
  draco_encode::DracoEncode(mesh->vertices(), mesh->uv(),
                            mesh->vertex_indices(), mesh->uv_indices(), {}, {},
                            options, bytes);
  timer.End();
  std::cout << "Encode time: " << timer.elapsed_msec() << " ms" << std::endl;

  size_t org_size = mesh->vertices().size() * sizeof(float) * 3 +
                    mesh->uv().size() * sizeof(float) * 2 +
                    mesh->vertex_indices().size() * sizeof(int) * 3 +
                    mesh->uv_indices().size() * sizeof(int) * 3;

  std::cout << "Original size: " << org_size / 1024 << " kb" << std::endl;
  std::cout << "Draco size: " << bytes.size() / 1024 << " kb" << std::endl;

  ugu::EnsureDirExists("../data_out");
  std::string drc_path = "../data_out/bunny.drc";

  std::ofstream ofs(drc_path, std::ios::binary);
  for (const char& b : bytes) {
    ofs << b;
  }
  std::vector<Eigen::Vector3f> verts;
  std::vector<Eigen::Vector2f> uvs;
  std::vector<Eigen::Vector3i> indices;
  std::vector<Eigen::Vector3i> uv_indices;
  std::vector<Eigen::Vector<uint8_t, 3>> colors;
  std::vector<Eigen::Vector3f> normals;
  timer.Start();
  draco_decode::DracoDecode(bytes, verts, uvs, indices, uv_indices, colors,
                            normals);
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

  decoded->WriteObj("../data_out/bunny_from_draco.obj");
}

void TestPlyPc() {
  std::cout << "Colored point cloud with normals" << std::endl;

  ugu::MeshPtr mesh = ugu::Mesh::Create();
  mesh->LoadPly("../data/longdress_viewdep_vox12_sampled.ply");
  std::vector<Eigen::Vector<uint8_t, 3>> mesh_colors_8;
  std::transform(mesh->vertex_colors().begin(), mesh->vertex_colors().end(),
                 std::back_inserter(mesh_colors_8),
                 [&](const Eigen::Vector3f& c) {
                   return Eigen::Vector<uint8_t, 3>(static_cast<uint8_t>(c[0]),
                                                    static_cast<uint8_t>(c[1]),
                                                    static_cast<uint8_t>(c[2]));
                 });
  ugu::Timer timer;

  std::vector<char> bytes;
  draco_encode::DracoEncodeOptions options;
  options.pos_quantization_bits = 11;
  options.tex_coords_quantization_bits = 10;
  options.normals_quantization_bits = 8;
  options.compression_level = 7;
  timer.Start();
  draco_encode::DracoEncode(mesh->vertices(), mesh->uv(),
                            mesh->vertex_indices(), mesh->uv_indices(),
                            mesh_colors_8, mesh->normals(), options, bytes);
  timer.End();
  std::cout << "Encode time: " << timer.elapsed_msec() << " ms" << std::endl;

  size_t org_size = mesh->vertices().size() * sizeof(float) * 3 +
                    mesh->uv().size() * sizeof(float) * 2 +
                    mesh->vertex_indices().size() * sizeof(int) * 3 +
                    mesh->uv_indices().size() * sizeof(int) * 3 +
                    mesh_colors_8.size() * sizeof(uint8_t) * 3 +
                    mesh->normals().size() * sizeof(float) * 3;

  std::cout << "Original size: " << org_size / 1024 << " kb" << std::endl;
  std::cout << "Draco size: " << bytes.size() / 1024 << " kb" << std::endl;

  ugu::EnsureDirExists("../data_out");
  std::string drc_path = "../data_out/longdress_viewdep_vox12_sampled.drc";

  std::ofstream ofs(drc_path, std::ios::binary);
  for (const char& b : bytes) {
    ofs << b;
  }
  std::vector<Eigen::Vector3f> verts;
  std::vector<Eigen::Vector2f> uvs;
  std::vector<Eigen::Vector3i> indices;
  std::vector<Eigen::Vector3i> uv_indices;
  std::vector<Eigen::Vector<uint8_t, 3>> colors;
  std::vector<Eigen::Vector3f> normals;
  timer.Start();
  draco_decode::DracoDecode(bytes, verts, uvs, indices, uv_indices, colors,
                            normals);
  timer.End();
  std::cout << "Decode time: " << timer.elapsed_msec() << " ms" << std::endl;

  ugu::MeshPtr decoded = ugu::Mesh::Create();
  decoded->set_vertices(verts);
  decoded->set_uv(uvs);
  decoded->set_vertex_indices(indices);
  decoded->set_uv_indices(uv_indices);
  decoded->set_normals(normals);
  std::vector<Eigen::Vector3f> colors_f;
  std::transform(colors.begin(), colors.end(), std::back_inserter(colors_f),
                 [&](const Eigen::Vector<uint8_t, 3>& c) {
                   return Eigen::Vector3f(c[0], c[1], c[2]);
                 });
  decoded->set_vertex_colors(colors_f);

  decoded->WritePly(
      "../data_out/longdress_viewdep_vox12_sampled_from_draco.ply");
}

int main() {
  TestObjMesh();
  std::cout << std::endl;
  TestPlyPc();

  return 0;
}
