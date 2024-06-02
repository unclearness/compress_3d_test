#include "draco_decode.h"

#include <cinttypes>

#include "draco/compression/decode.h"
#include "draco/core/cycle_timer.h"
#include "draco/io/file_utils.h"

namespace {

std::vector<Eigen::Vector3f> DecodePositions(draco::PointCloud* pc) {
  std::vector<Eigen::Vector3f> verts;

  const draco::PointAttribute* const att =
      pc->GetNamedAttribute(draco::GeometryAttribute::POSITION);
  if (att == nullptr || att->size() == 0) {
    return verts;  // Position attribute must be valid.
  }
  verts.resize(att->size());
  for (draco::AttributeValueIndex i(0); i < static_cast<uint32_t>(att->size());
       ++i) {
    if (!att->ConvertValue<float, 3>(i, verts[i.value()].data())) {
      return verts;
    }
  }
  return verts;
}

std::vector<Eigen::Vector2f> DecodeUvs(draco::PointCloud* pc) {
  std::vector<Eigen::Vector2f> uvs;

  const draco::PointAttribute* const att =
      pc->GetNamedAttribute(draco::GeometryAttribute::TEX_COORD);
  if (att == nullptr || att->size() == 0) {
    return uvs;
  }
  uvs.resize(att->size());
  for (draco::AttributeValueIndex i(0); i < static_cast<uint32_t>(att->size());
       ++i) {
    if (!att->ConvertValue<float, 2>(i, uvs[i.value()].data())) {
      return uvs;
    }
  }
  return uvs;
}

#if 0
std::vector<Eigen::Vector3i> DecodeFaces(const draco::PointAttribute* const att,
                                         uint32_t num_faces) {
  std::vector<Eigen::Vector3i> face(num_faces);
  std::array<uint32_t, 3> f;
  for (uint32_t i = 0; i < num_faces; i++) {
    att->mapped_index() att->GetMappedValue(draco::PointIndex(i), &f);
    face[i][0] = static_cast<int32_t>(f[0]);
    face[i][1] = static_cast<int32_t>(f[1]);
    face[i][2] = static_cast<int32_t>(f[2]);
  }
  return face;
}
#endif

std::pair<std::vector<Eigen::Vector3i>, std::vector<Eigen::Vector3i>>
DecodeFaces(draco::Mesh* mesh) {
  uint32_t num_faces = mesh->num_faces();

  const draco::PointAttribute* const verts =
      mesh->GetNamedAttribute(draco::GeometryAttribute::POSITION);
  if (verts == nullptr || verts->size() == 0) {
    return std::make_pair(std::vector<Eigen::Vector3i>(),
                          std::vector<Eigen::Vector3i>());
  }

  const draco::PointAttribute* const uvs =
      mesh->GetNamedAttribute(draco::GeometryAttribute::TEX_COORD);

  std::vector<Eigen::Vector3i> indices(num_faces);
  std::vector<Eigen::Vector3i> uv_indices(num_faces);
  const bool with_uv = uvs != nullptr && uvs->size() > 0;
  for (uint32_t i = 0; i < num_faces; i++) {
    for (uint32_t j = 0; j < 3; j++) {
      const draco::PointIndex vert_index = mesh->face(draco::FaceIndex(i))[j];
      indices[i][j] =
          static_cast<int32_t>(verts->mapped_index(vert_index).value());
      if (with_uv) {
        uv_indices[i][j] =
            static_cast<int32_t>(uvs->mapped_index(vert_index).value());
      }
    }
  }

  return std::make_pair(indices, uv_indices);
}

#if 0

bool EncodeFaces() {
  for (FaceIndex i(0); i < in_mesh_->num_faces(); ++i) {
    for (int j = 0; j < 3; ++j) {
      if (!EncodeFaceCorner(i, j)) {
        return false;
      }
    }
  }
  return true;
}
#endif

}  // namespace

namespace draco_decode {

bool DracoDecodeMesh(const std::vector<char>& data,
                     std::vector<Eigen::Vector3f>& verts,
                     std::vector<Eigen::Vector2f>& uvs,
                     std::vector<Eigen::Vector3i>& indices,
                     std::vector<Eigen::Vector3i>& uv_indices) {
  // Create a draco decoding buffer. Note that no data is copied in this step.
  draco::DecoderBuffer buffer;
  buffer.Init(data.data(), data.size());

  draco::CycleTimer timer;
  // Decode the input data into a geometry.
  std::unique_ptr<draco::PointCloud> pc;
  draco::Mesh* mesh = nullptr;
  auto type_statusor = draco::Decoder::GetEncodedGeometryType(&buffer);
  if (!type_statusor.ok()) {
    // return ReturnError(type_statusor.status());
    return false;
  }
  const draco::EncodedGeometryType geom_type = type_statusor.value();
  if (geom_type == draco::TRIANGULAR_MESH) {
    timer.Start();
    draco::Decoder decoder;
    auto statusor = decoder.DecodeMeshFromBuffer(&buffer);
    if (!statusor.ok()) {
      // return ReturnError(statusor.status());
      return false;
    }
    std::unique_ptr<draco::Mesh> in_mesh = std::move(statusor).value();
    timer.Stop();
    if (in_mesh) {
      mesh = in_mesh.get();
      pc = std::move(in_mesh);
    }
  } else if (geom_type == draco::POINT_CLOUD) {
    // Failed to decode it as mesh, so let's try to decode it as a point
    // cloud.
    timer.Start();
    draco::Decoder decoder;
    auto statusor = decoder.DecodePointCloudFromBuffer(&buffer);
    if (!statusor.ok()) {
      // return ReturnError(statusor.status());
      return false;
    }
    pc = std::move(statusor).value();
    timer.Stop();
  }

  if (pc == nullptr) {
    printf("Failed to decode the input file.\n");
    return false;
  }

  // Decode positions
  verts = DecodePositions(mesh);

  // Decode UVs
  uvs = DecodeUvs(mesh);

  // Decode faces
  std::tie(indices, uv_indices) = DecodeFaces(mesh);

  return true;
}

}  // namespace draco_decode