// Copyright 2016 The Draco Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
#include "draco_encode.h"

#include <cinttypes>
#include <cstdlib>

#include "draco/compression/encode.h"
#include "draco/compression/expert_encode.h"
#include "draco/mesh/mesh.h"

namespace draco_encode {

Options::Options()
    :  // is_point_cloud(false),
      pos_quantization_bits(11),
      tex_coords_quantization_bits(10),
      // tex_coords_deleted(false),
      // normals_quantization_bits(8),
      // normals_deleted(false),
      // generic_quantization_bits(8),
      // generic_deleted(false),
      compression_level(7)
// preserve_polygons(false),
// use_metadata(false)
{}

#if 0
void PrintOptions(const draco::PointCloud& pc, const Options& options) {
  printf("Encoder options:\n");
  printf("  Compression level = %d\n", options.compression_level);
  if (options.pos_quantization_bits == 0) {
    printf("  Positions: No quantization\n");
  } else {
    printf("  Positions: Quantization = %d bits\n",
           options.pos_quantization_bits);
  }

  if (pc.GetNamedAttributeId(draco::GeometryAttribute::TEX_COORD) >= 0) {
    if (options.tex_coords_quantization_bits == 0) {
      printf("  Texture coordinates: No quantization\n");
    } else {
      printf("  Texture coordinates: Quantization = %d bits\n",
             options.tex_coords_quantization_bits);
    }
  } else if (options.tex_coords_deleted) {
    printf("  Texture coordinates: Skipped\n");
  }

  if (pc.GetNamedAttributeId(draco::GeometryAttribute::NORMAL) >= 0) {
    if (options.normals_quantization_bits == 0) {
      printf("  Normals: No quantization\n");
    } else {
      printf("  Normals: Quantization = %d bits\n",
             options.normals_quantization_bits);
    }
  } else if (options.normals_deleted) {
    printf("  Normals: Skipped\n");
  }

  if (pc.GetNamedAttributeId(draco::GeometryAttribute::GENERIC) >= 0) {
    if (options.generic_quantization_bits == 0) {
      printf("  Generic: No quantization\n");
    } else {
      printf("  Generic: Quantization = %d bits\n",
             options.generic_quantization_bits);
    }
  } else if (options.generic_deleted) {
    printf("  Generic: Skipped\n");
  }
  printf("\n");
}
#endif

std::unique_ptr<draco::Mesh> MakeMesh(
    const std::vector<Eigen::Vector3f>& positions,
    const std::vector<Eigen::Vector2f>& uvs,
    const std::vector<Eigen::Vector3i>& indices,
    const std::vector<Eigen::Vector3i>& uv_indices) {
  using namespace draco;

  std::unique_ptr<draco::Mesh> mesh(new draco::Mesh());

  // Set face size
  mesh->SetNumFaces(indices.size());

  // Set splitted vertices size
  mesh->set_num_points(indices.size() * 3);

  // Init vertex positions
  draco::GeometryAttribute pos_att;
  pos_att.Init(draco::GeometryAttribute::POSITION, nullptr, 3,
               draco::DT_FLOAT32, false, sizeof(float) * 3, 0);
  int pos_att_id = mesh->AddAttribute(pos_att, false, positions.size());
  auto pos_attribute = mesh->attribute(pos_att_id);

  // Init UVs
  draco::GeometryAttribute uv_att;
  uv_att.Init(draco::GeometryAttribute::TEX_COORD, nullptr, 2,
              draco::DT_FLOAT32, false, sizeof(float) * 2, 0);
  int uv_att_id = mesh->AddAttribute(uv_att, false, uvs.size());
  auto uv_attribute = mesh->attribute(uv_att_id);

  // Set vertex positions
  for (size_t i = 0; i < positions.size(); ++i) {
    pos_attribute->SetAttributeValue(AttributeValueIndex(i), &positions[i]);
  }

  // Set UVs
  for (uint32_t i = 0; i < uvs.size(); ++i) {
    uv_attribute->SetAttributeValue(draco::AttributeValueIndex(i), &(uvs[i]));
  }

  // Set splitted faces
  for (draco::FaceIndex i(0); i < indices.size(); ++i) {
    draco::Mesh::Face face;
    for (int j = 0; j < 3; ++j) {
      face[j] = 3 * i.value() + j;
    }
    mesh->SetFace(i, face);
  }

  // Set mapping from splitted vertex id to original vertex id
  for (size_t i = 0; i < uv_indices.size(); ++i) {
    for (int j = 0; j < 3; ++j) {
      pos_attribute->SetPointMapEntry(
          draco::PointIndex(i * 3 + j),
          draco::AttributeValueIndex(indices[i][j]));
      uv_attribute->SetPointMapEntry(
          draco::PointIndex(i * 3 + j),
          draco::AttributeValueIndex(uv_indices[i][j]));
    }
  }

  return mesh;
}

bool EncodeMesh(const std::vector<Eigen::Vector3f>& verts,
                const std::vector<Eigen::Vector2f>& uvs,
                const std::vector<Eigen::Vector3i>& indices,
                const std::vector<Eigen::Vector3i>& uv_indices,
                Options& options, std::vector<char>& bytes) {
  std::unique_ptr<draco::PointCloud> pc;
  draco::Mesh* mesh = nullptr;

  std::unique_ptr<draco::Mesh> maybe_mesh =
      MakeMesh(verts, uvs, indices, uv_indices);
  mesh = maybe_mesh.get();
  pc = std::move(maybe_mesh);

  if (options.pos_quantization_bits < 0) {
    printf("Error: Position attribute cannot be skipped.\n");
    return false;
  }

#if 0
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
  if (options.generic_quantization_bits < 0) {
    if (pc->NumNamedAttributes(draco::GeometryAttribute::GENERIC) > 0) {
      options.generic_deleted = true;
    }
    while (pc->NumNamedAttributes(draco::GeometryAttribute::GENERIC) > 0) {
      pc->DeleteAttribute(
          pc->GetNamedAttributeId(draco::GeometryAttribute::GENERIC, 0));
    }
  }
#ifdef DRACO_ATTRIBUTE_INDICES_DEDUPLICATION_SUPPORTED
  // If any attribute has been deleted, run deduplication of point indices again
  // as some points can be possibly combined.
  if (options.tex_coords_deleted || options.normals_deleted ||
      options.generic_deleted) {
    pc->DeduplicatePointIds();
  }
#endif
#endif
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
#if 0
  if (options.normals_quantization_bits > 0) {
    encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL,
                                     options.normals_quantization_bits);
  }
  if (options.generic_quantization_bits > 0) {
    encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC,
                                     options.generic_quantization_bits);
  }
#endif
  encoder.SetSpeedOptions(speed, speed);

#if 0
  if (options.output.empty()) {
    // Create a default output file by attaching .drc to the input file name.
    options.output = options.input + ".drc";
  }
#endif

  // PrintOptions(*pc, options);

  const bool input_is_mesh = mesh && mesh->num_faces() > 0;

  // Convert to ExpertEncoder that allows us to set per-attribute options.
  std::unique_ptr<draco::ExpertEncoder> expert_encoder;
  if (input_is_mesh) {
    expert_encoder.reset(new draco::ExpertEncoder(*mesh));
  } else {
    expert_encoder.reset(new draco::ExpertEncoder(*pc));
  }
  expert_encoder->Reset(encoder.CreateExpertEncoderOptions(*pc));

  // Check if there is an attribute that stores polygon edges. If so, we disable
  // the default prediction scheme for the attribute as it actually makes the
  // compression worse.
  const int poly_att_id =
      pc->GetAttributeIdByMetadataEntry("name", "added_edges");
  if (poly_att_id != -1) {
    expert_encoder->SetAttributePredictionScheme(
        poly_att_id, draco::PredictionSchemeMethod::PREDICTION_NONE);
  }

  bool ret = false;
  draco::EncoderBuffer buffer;
  const draco::Status status = expert_encoder.get()->EncodeToBuffer(&buffer);
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
