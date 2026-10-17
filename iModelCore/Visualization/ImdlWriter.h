/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
#pragma once

#include <DgnPlatform/DgnPlatformApi.h>
#include <unordered_map>

BEGIN_BENTLEY_DGN_NAMESPACE

// Major version 37, matching CurrentImdlVersion in @itwin/core-common.
constexpr uint32_t IMDL_FORMAT_VERSION = 37 << 16;

// Bits of the iMdl header's flags (ImdlFlags in @itwin/core-common).
enum class ImdlFlags : uint32_t {
  None = 0,
  ContainsCurves = 1 << 0,
  Incomplete = 1 << 2,
  DisallowMagnification = 1 << 3,
};

// ImdlEdgeVisibility in core-frontend: two bits per triangle edge.
enum class ImdlEdgeVisibility : uint8_t {
  Hidden = 0,
  Silhouette = 1,
  Visible = 2,
};

// Triangles sharing one color and texture, in tile tree coordinates.
struct ImdlMesh {
  ColorDef m_color;
  DgnTextureId m_textureId;
  uint32_t m_fillFlags;
  DRange3d m_range = DRange3d::NullRange();
  bvector<DPoint3d> m_points;
  bvector<uint16_t> m_normals; // oct-encoded
  bvector<uint32_t> m_featureIndices; // one per point
  bvector<FPoint2d> m_params; // one per point if textured
  bvector<uint32_t> m_indices;
  bvector<ImdlEdgeVisibility> m_edges; // one per index: the edge from that index to the next in its triangle
  bvector<uint32_t> m_silhouetteNormals; // one per silhouette edge, in index order

  ImdlMesh(ColorDef color, DgnTextureId textureId, uint32_t fillFlags) : m_color(color), m_textureId(textureId), m_fillFlags(fillFlags) { }
};

// Line strings sharing one set of line symbology, in tile tree coordinates.
struct ImdlPolyline {
  ColorDef m_color;
  uint32_t m_width;
  uint32_t m_linePixels;
  DRange3d m_range = DRange3d::NullRange();
  bvector<DPoint3d> m_points;
  bvector<uint32_t> m_featureIndices; // one per point
  bvector<bvector<uint32_t>> m_lines;

  ImdlPolyline(ColorDef color, uint32_t width, uint32_t linePixels) : m_color(color), m_width(width), m_linePixels(linePixels) { }
};

// Everything needed to write one tile's content.
struct ImdlTile {
  DRange3d m_contentRange = DRange3d::NullRange();
  double m_tolerance = 0.0;
  uint32_t m_numElementsIncluded = 0;
  uint32_t m_numElementsExcluded = 0;
  uint32_t m_flags = 0;
  uint32_t m_emptySubRanges = 0;
  bool m_is2d = false;
  bool m_wantEdges = false;
  bvector<ImdlMesh> m_meshes;
  bvector<ImdlPolyline> m_polylines;

  // Returns the index of the feature, adding it if necessary.
  uint32_t GetFeatureIndex(DgnElementId elementId, DgnSubCategoryId subCategoryId, Render::DgnGeometryClass geometryClass);
  // Returns a mesh of the given color and texture with room for vertexCount more vertices.
  ImdlMesh& GetMesh(ColorDef color, DgnTextureId textureId, uint32_t fillFlags, size_t vertexCount);
  ImdlPolyline& GetPolyline(ColorDef color, uint32_t width, uint32_t linePixels, size_t vertexCount);
  // Adds a texture image to embed in the tile, if not already present.
  void AddTexture(DgnDbR db, DgnTextureId textureId);

  ByteStream Write() const;

private:
  struct Feature {
    DgnElementId m_elementId;
    uint32_t m_subCategoryIndex;
    Render::DgnGeometryClass m_geometryClass;
  };

  struct Texture {
    uint32_t m_format;
    bvector<uint8_t> m_data;
  };

  bvector<Feature> m_features;
  std::map<std::tuple<uint64_t, uint64_t, uint8_t>, uint32_t> m_featureIndices;
  bvector<DgnSubCategoryId> m_subCategories;
  std::unordered_map<uint64_t, uint32_t> m_subCategoryIndices;
  std::map<uint64_t, Texture> m_textures;
};

uint16_t OctEncodeNormal(double x, double y, double z);

END_BENTLEY_DGN_NAMESPACE
