/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
#pragma once

#include <DgnPlatform/DgnPlatformApi.h>

BEGIN_BENTLEY_DGN_NAMESPACE

inline FPoint2d MakeFPoint2d(double x, double y) {
  FPoint2d pt;
  pt.x = (float)x;
  pt.y = (float)y;
  return pt;
}

// Triangles for one element sharing a single set of display properties, in world coordinates.
struct ElementMesh {
  ColorDef m_color;
  bool m_isTwoSided = false;
  DgnSubCategoryId m_subCategoryId;
  Render::DgnGeometryClass m_geometryClass = Render::DgnGeometryClass::Primary;
  DgnTextureId m_textureId;
  uint32_t m_fillFlags = 0; // FillFlags in core-common
  bvector<uint32_t> m_indices;
  bvector<double> m_points; // xyz
  bvector<float> m_normals; // xyz, one per point
  bvector<float> m_params; // uv, one per point; only if textured

  ElementMesh(ColorDef color, bool isTwoSided, DgnSubCategoryId subCategoryId, Render::DgnGeometryClass geometryClass, DgnTextureId textureId, uint32_t fillFlags)
    : m_color(color), m_isTwoSided(isTwoSided), m_subCategoryId(subCategoryId), m_geometryClass(geometryClass), m_textureId(textureId), m_fillFlags(fillFlags) { }

  size_t GetPointCount() const { return m_points.size() / 3; }
  bool IsTextured() const { return m_textureId.IsValid(); }
};

// Line strings for one element sharing a single set of display properties, in world coordinates.
struct ElementPolyline {
  ColorDef m_color;
  uint32_t m_width = 1;
  uint32_t m_linePixels = 0;
  DgnSubCategoryId m_subCategoryId;
  Render::DgnGeometryClass m_geometryClass = Render::DgnGeometryClass::Primary;
  bvector<DPoint3d> m_points;
  bvector<bvector<uint32_t>> m_lines; // indices into m_points

  ElementPolyline(ColorDef color, uint32_t width, uint32_t linePixels, DgnSubCategoryId subCategoryId, Render::DgnGeometryClass geometryClass)
    : m_color(color), m_width(width), m_linePixels(linePixels), m_subCategoryId(subCategoryId), m_geometryClass(geometryClass) { }
};

struct ElementGraphics {
  bvector<ElementMesh> m_meshes;
  bvector<ElementPolyline> m_polylines;
};

// Produces an element's graphics the way exportGraphics does: surfaces, solids, b-reps, filled
// regions and text are faceted; open curves and unfilled regions are stroked.
struct ElementMesher {
  static bool Mesh(ElementGraphics& graphics, GeometrySourceCR source, double chordTolerance);
};

END_BENTLEY_DGN_NAMESPACE
