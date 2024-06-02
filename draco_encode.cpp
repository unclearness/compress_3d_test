#include "draco_encode.h"

#include <cinttypes>
#include <cstdlib>

#include "draco/compression/encode.h"
#include "draco/compression/expert_encode.h"
#include "draco/core/cycle_timer.h"
#include "draco/mesh/mesh.h"

namespace draco_encode {

DracoEncodeOptions::DracoEncodeOptions()
    : pos_quantization_bits(11),
      tex_coords_quantization_bits(10),
      tex_coords_deleted(false),
      normals_quantization_bits(8),
      normals_deleted(false),
      compression_level(7) {}

std::unique_ptr<draco::Mesh> MakeMesh(
    const std::vector<Eigen::Vector3f>& positions,
    const std::vector<Eigen::Vector2f>& uvs,
    const std::vector<Eigen::Vector3i>& indices,
    const std::vector<Eigen::Vector3i>& uv_indices,
    const std::vector<Eigen::Vector<uint8_t, 3>>& colors,
    const std::vector<Eigen::Vector3f>& normals) {
  std::unique_ptr<draco::Mesh> mesh(new draco::Mesh());

  if (positions.empty()) {
    return mesh;
  }

  const bool is_mesh = indices.size() > 0;

  if (is_mesh) {
    // Set face size
    mesh->SetNumFaces(indices.size());

    // Set splitted vertices size
    mesh->set_num_points(indices.size() * 3);
  } else {
    mesh->set_num_points(positions.size());
  }
  const bool identity_mapping = !is_mesh;

  // Init vertex positions
  draco::GeometryAttribute pos_att;
  pos_att.Init(draco::GeometryAttribute::POSITION, nullptr, 3,
               draco::DT_FLOAT32, false, sizeof(float) * 3, 0);
  int pos_att_id =
      mesh->AddAttribute(pos_att, identity_mapping, positions.size());
  draco::PointAttribute* pos_attribute = mesh->attribute(pos_att_id);

  // Init UVs
  const bool with_uvs = uvs.size() > 0 && uv_indices.size() == indices.size();
  draco::GeometryAttribute uv_att;
  int uv_att_id = -1;
  draco::PointAttribute* uv_attribute = nullptr;
  if (with_uvs) {
    uv_att.Init(draco::GeometryAttribute::TEX_COORD, nullptr, 2,
                draco::DT_FLOAT32, false, sizeof(float) * 2, 0);
    uv_att_id = mesh->AddAttribute(uv_att, identity_mapping, uvs.size());
    uv_attribute = mesh->attribute(uv_att_id);
  }

  // Init colors
  const bool with_colors = colors.size() == positions.size();
  draco::GeometryAttribute col_att;
  int col_att_id = -1;
  draco::PointAttribute* col_attribute = nullptr;
  if (with_colors) {
    col_att.Init(draco::GeometryAttribute::COLOR, nullptr, 3, draco::DT_UINT8,
                 true, sizeof(uint8_t) * 3, 0);
    col_att_id = mesh->AddAttribute(col_att, identity_mapping, colors.size());
    col_attribute = mesh->attribute(col_att_id);
  }

  // Init normals
  const bool with_normals = normals.size() == positions.size();
  draco::GeometryAttribute nor_att;
  int nor_att_id = -1;
  draco::PointAttribute* nor_attribute = nullptr;
  if (with_normals) {
    nor_att.Init(draco::GeometryAttribute::NORMAL, nullptr, 3,
                 draco::DT_FLOAT32, false, sizeof(float) * 3, 0);
    nor_att_id = mesh->AddAttribute(nor_att, identity_mapping, normals.size());
    nor_attribute = mesh->attribute(nor_att_id);
  }

  // Set vertex positions
  for (uint32_t i = 0; i < static_cast<uint32_t>(positions.size()); ++i) {
    pos_attribute->SetAttributeValue(draco::AttributeValueIndex(i),
                                     &positions[i]);
  }

  // Set UVs
  if (with_uvs) {
    for (uint32_t i = 0; i < static_cast<uint32_t>(uvs.size()); ++i) {
      uv_attribute->SetAttributeValue(draco::AttributeValueIndex(i), &(uvs[i]));
    }
  }

  // Set colors
  if (with_colors) {
    for (uint32_t i = 0; i < static_cast<uint32_t>(colors.size()); ++i) {
      col_attribute->SetAttributeValue(draco::AttributeValueIndex(i),
                                       &(colors[i]));
    }
  }

  // Set normals
  if (with_normals) {
    for (uint32_t i = 0; i < static_cast<uint32_t>(normals.size()); ++i) {
      nor_attribute->SetAttributeValue(draco::AttributeValueIndex(i),
                                       &(normals[i]));
    }
  }

  if (is_mesh) {
    // Set splitted faces
    for (uint32_t i = 0; i < static_cast<uint32_t>(indices.size()); ++i) {
      draco::Mesh::Face face;
      for (uint32_t j = 0; j < 3; ++j) {
        face[j] = 3 * i + j;
      }
      mesh->SetFace(draco::FaceIndex(i), face);
    }

    // Set mapping from splitted vertex id to original vertex id
    for (uint32_t i = 0; i < static_cast<uint32_t>(uv_indices.size()); ++i) {
      for (uint32_t j = 0; j < 3; ++j) {
        pos_attribute->SetPointMapEntry(
            draco::PointIndex(i * 3 + j),
            draco::AttributeValueIndex(indices[i][j]));

        if (with_uvs) {
          uv_attribute->SetPointMapEntry(
              draco::PointIndex(i * 3 + j),
              draco::AttributeValueIndex(uv_indices[i][j]));
        }

        if (with_colors) {
          col_attribute->SetPointMapEntry(
              draco::PointIndex(i * 3 + j),
              draco::AttributeValueIndex(indices[i][j]));
        }

        if (with_normals) {
          nor_attribute->SetPointMapEntry(
              draco::PointIndex(i * 3 + j),
              draco::AttributeValueIndex(indices[i][j]));
        }
      }
    }
  }

  return mesh;
}

bool DracoEncode(const std::vector<Eigen::Vector3f>& verts,
                 const std::vector<Eigen::Vector2f>& uvs,
                 const std::vector<Eigen::Vector3i>& indices,
                 const std::vector<Eigen::Vector3i>& uv_indices,
                 const std::vector<Eigen::Vector<uint8_t, 3>>& colors,
                 const std::vector<Eigen::Vector3f>& normals,
                 const DracoEncodeOptions& options_, std::vector<char>& bytes) {
  DracoEncodeOptions options = options_;

  std::unique_ptr<draco::PointCloud> pc;
  draco::Mesh* mesh = nullptr;

  std::unique_ptr<draco::Mesh> maybe_mesh =
      MakeMesh(verts, uvs, indices, uv_indices, colors, normals);

  mesh = maybe_mesh.get();
  pc = std::move(maybe_mesh);

  if (options.pos_quantization_bits < 0) {
    printf("Error: Position attribute cannot be skipped.\n");
    return false;
  }

  // Delete attributes if needed. This needs to happen before we set any
  // quantization settings.
  if (options.tex_coords_quantization_bits < 0) {
    if (pc->NumNamedAttributes(draco::GeometryAttribute::TEX_COORD) > 0) {
      options.tex_coords_deleted = true;
    }
    while (pc->NumNamedAttributes(draco::GeometryAttribute::TEX_COORD) > 0) {
      pc->DeleteAttribute(
          pc->GetNamedAttributeId(draco::GeometryAttribute::TEX_COORD, 0));
    }
  }
  if (options.normals_quantization_bits < 0) {
    if (pc->NumNamedAttributes(draco::GeometryAttribute::NORMAL) > 0) {
      options.normals_deleted = true;
    }
    while (pc->NumNamedAttributes(draco::GeometryAttribute::NORMAL) > 0) {
      pc->DeleteAttribute(
          pc->GetNamedAttributeId(draco::GeometryAttribute::NORMAL, 0));
    }
  }

  // Convert compression level to speed (that 0 = slowest, 10 = fastest).
  const int speed = 10 - options.compression_level;

  draco::Encoder encoder;

  // Setup encoder options.
  if (options.pos_quantization_bits > 0) {
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION,
                                     options.pos_quantization_bits);
  }
  if (options.tex_coords_quantization_bits > 0) {
    encoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD,
                                     options.tex_coords_quantization_bits);
  }
  if (options.normals_quantization_bits > 0) {
    encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL,
                                     options.normals_quantization_bits);
  }

  encoder.SetSpeedOptions(speed, speed);

  const bool input_is_mesh = mesh && mesh->num_faces() > 0;

  // Convert to ExpertEncoder that allows us to set per-attribute options.
  std::unique_ptr<draco::ExpertEncoder> expert_encoder;
  if (input_is_mesh) {
    expert_encoder.reset(new draco::ExpertEncoder(*mesh));
  } else {
    expert_encoder.reset(new draco::ExpertEncoder(*pc));
  }
  expert_encoder->Reset(encoder.CreateExpertEncoderOptions(*pc));

  bool ret = false;
  draco::EncoderBuffer buffer;
  draco::CycleTimer timer;
  timer.Start();
  const draco::Status status = expert_encoder.get()->EncodeToBuffer(&buffer);
  timer.Stop();
  // printf("encode: %i\n", timer.GetInMs());
  if (!status.ok()) {
    printf("Failed to encode the mesh.\n");
    printf("%s\n", status.error_msg());
    ret = false;
  } else {
    ret = true;
  }

  bytes = *buffer.buffer();

  return ret;
}

}  // namespace draco_encode
