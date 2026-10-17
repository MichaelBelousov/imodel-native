/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
// Adapted from iModelJsNodeAddon/JsInteropExportGraphics.cpp.
#include "ElementMesher.h"
#include <DgnPlatform/SimplifyGraphic.h>

USING_NAMESPACE_BENTLEY_DGN

namespace {

constexpr float UV_TOLERANCE = 0.00025f;

// Indices are zero-based triangles with no blocking; normals are indexed separately from points.
struct IntermediateMesh {
  uint32_t m_indexCount;
  uint32_t m_pointCount;
  uint32_t m_normalCount;
  std::unique_ptr<DPoint3d[]> m_points;
  std::unique_ptr<int32_t[]> m_pointIndices;
  std::unique_ptr<FPoint3d[]> m_normals;
  std::unique_ptr<int32_t[]> m_normalIndices;
  std::unique_ptr<FPoint2d[]> m_params; // one per index, or null

  IntermediateMesh(PolyfaceQueryCR pf, TransformCR transform, std::unique_ptr<FPoint2d[]>&& params)
    : m_indexCount((uint32_t)(pf.GetPointIndexCount() - (pf.GetPointIndexCount() / 4))),
      m_pointCount((uint32_t)pf.GetPointCount()),
      m_normalCount((uint32_t)pf.GetNormalCount()),
      m_points(new DPoint3d[m_pointCount]),
      m_pointIndices(new int32_t[m_indexCount]),
      m_normals(new FPoint3d[m_normalCount]),
      m_normalIndices(new int32_t[m_indexCount]),
      m_params(std::move(params)) {
    bool isIdentity = transform.IsIdentity();
    bool isMirrored = !isIdentity && transform.Determinant() < 0;
    uint32_t indexOffset1 = isMirrored ? 2 : 1;
    uint32_t indexOffset2 = isMirrored ? 1 : 2;

    uint32_t srcIndexCount = (uint32_t)pf.GetPointIndexCount();
    int32_t const* srcPointIndices = pf.GetPointIndexCP();
    for (uint32_t dst = 0, src = 0; src < srcIndexCount; src += 4, dst += 3) {
      m_pointIndices[dst] = abs(srcPointIndices[src]) - 1;
      m_pointIndices[dst + 1] = abs(srcPointIndices[src + indexOffset1]) - 1;
      m_pointIndices[dst + 2] = abs(srcPointIndices[src + indexOffset2]) - 1;
    }

    int32_t const* srcNormalIndices = pf.GetNormalIndexCP();
    for (uint32_t dst = 0, src = 0; src < srcIndexCount; src += 4, dst += 3) {
      m_normalIndices[dst] = srcNormalIndices[src] - 1;
      m_normalIndices[dst + 1] = srcNormalIndices[src + indexOffset1] - 1;
      m_normalIndices[dst + 2] = srcNormalIndices[src + indexOffset2] - 1;
    }

    memcpy(m_points.get(), pf.GetPointCP(), sizeof(DPoint3d) * m_pointCount);
    if (!isIdentity)
      transform.Multiply(m_points.get(), (int)m_pointCount);

    RotMatrix normalMatrix = RotMatrix::From(transform);
    isIdentity = normalMatrix.IsIdentity();
    if (!isIdentity)
      normalMatrix.Invert();

    DPoint3dCP srcNormals = pf.GetNormalCP();
    for (uint32_t i = 0; i < m_normalCount; ++i) {
      DPoint3d n = srcNormals[i];
      if (!isIdentity) {
        normalMatrix.MultiplyTranspose(n);
        n.Normalize();
      }
      m_normals[i] = FPoint3d::From(n);
    }
  }
};

// Unify point, normal and param indices so each output vertex has one of each.
void unifyIndices(IntermediateMesh const& in, ElementMesh& out) {
  struct NodeEntry { int32_t newIndex; int32_t normal; FPoint2d param; };
  constexpr static uint32_t NODE_ENTRY_COUNT = 8;
  struct Node { NodeEntry entries[NODE_ENTRY_COUNT]; };

  std::unique_ptr<Node[]> remapper(new Node[in.m_pointCount]);
  memset(remapper.get(), 0, sizeof(Node) * in.m_pointCount);
  bool hasParams = nullptr != in.m_params;

  auto addVertex = [&](int32_t pointIndex, int32_t normalIndex, FPoint2dCR param) {
    out.m_indices.push_back((uint32_t)out.GetPointCount());
    DPoint3dCR p = in.m_points[pointIndex];
    FPoint3dCR n = in.m_normals[normalIndex];
    out.m_points.insert(out.m_points.end(), {p.x, p.y, p.z});
    out.m_normals.insert(out.m_normals.end(), {n.x, n.y, n.z});
    if (hasParams)
      out.m_params.insert(out.m_params.end(), {param.x, param.y});
  };

  auto addIndex = [&](uint32_t i) {
    int32_t pointIndex = in.m_pointIndices[i];
    int32_t normalIndex = in.m_normalIndices[i];
    FPoint2d param = hasParams ? in.m_params[i] : MakeFPoint2d(0, 0);
    for (uint32_t j = 0; j < NODE_ENTRY_COUNT; ++j) {
      NodeEntry& entry = remapper[pointIndex].entries[j];
      if (0 == entry.newIndex) {
        entry.normal = normalIndex;
        entry.param = param;
        entry.newIndex = (int32_t)out.GetPointCount() + 1; // one-based so zero means unused
        addVertex(pointIndex, normalIndex, param);
        return;
      }

      if (entry.normal == normalIndex && std::abs(entry.param.x - param.x) < UV_TOLERANCE && std::abs(entry.param.y - param.y) < UV_TOLERANCE) {
        out.m_indices.push_back((uint32_t)(entry.newIndex - 1));
        return;
      }
    }

    addVertex(pointIndex, normalIndex, param);
  };

  for (uint32_t i = 0; i < in.m_indexCount; i += 3) {
    int32_t i0 = in.m_pointIndices[i], i1 = in.m_pointIndices[i + 1], i2 = in.m_pointIndices[i + 2];
    if (i0 == i1 || i0 == i2 || i1 == i2)
      continue;

    addIndex(i);
    addIndex(i + 1);
    addIndex(i + 2);
  }
}

bool isTriangulatedWithZeroBlocking(PolyfaceQueryCR pf) {
  int32_t const* indices = pf.GetPointIndexCP();
  for (uint32_t i = 0, n = (uint32_t)pf.GetPointIndexCount(); i < n; i += 4) {
    for (uint32_t j = 0; j < 3; ++j)
      if (0 == indices[i + j])
        return false;

    if (0 != indices[i + 3])
      return false;
  }

  return true;
}

// Calls process with a polyface of zero-blocked triangles with normals, or not at all if there is no usable data.
void processPolyface(PolyfaceQueryCR pf, std::function<void(PolyfaceQueryCR)> const& process) {
  if (pf.GetPointIndexCount() < 3 || pf.GetPointCount() < 3)
    return;

  bool hasNormals = nullptr != pf.GetNormalIndexCP() && pf.GetNormalCount() > 0;
  if (MESH_ELM_STYLE_INDEXED_FACE_LOOPS != pf.GetMeshStyle() || !hasNormals)
    return;

  if (3 == pf.GetNumPerFace() && !pf.IsVariableSizeIndexed()) {
    PolyfaceHeaderPtr variableSizeIndexed = PolyfaceHeader::CreateVariableSizeIndexed();
    variableSizeIndexed->CopyFrom(pf);
    variableSizeIndexed->ConvertToVariableSizeSignedOneBasedIndexedFaceLoops();
    process(*variableSizeIndexed);
    return;
  }

  if (isTriangulatedWithZeroBlocking(pf))
    process(pf);
}

// FillFlags in core-common.
uint32_t fillFlagsFrom(Render::FillDisplay fill) {
  switch (fill) {
    case Render::FillDisplay::ByView: return 1;
    case Render::FillDisplay::Always: return 2;
    case Render::FillDisplay::Blanking: return 2 | 4;
    default: return 0;
  }
}

struct GraphicsProcessor : IGeometryProcessor {
  DgnDbR m_db;
  IFacetOptionsPtr m_facetOptions;
  ElementGraphics& m_graphics;
  bool m_is3d;
  bool m_inText = false;

  GraphicsProcessor(DgnDbR db, IFacetOptionsPtr facetOptions, ElementGraphics& graphics, bool is3d)
    : m_db(db), m_facetOptions(facetOptions), m_graphics(graphics), m_is3d(is3d) { }

  IFacetOptionsP _GetFacetOptionsP() override { return m_facetOptions.get(); }

  UnhandledPreference _GetUnhandledPreference(CurveVectorCR, SimplifyGraphic&) const override { return UnhandledPreference::Facet; }
  UnhandledPreference _GetUnhandledPreference(ISolidPrimitiveCR, SimplifyGraphic&) const override { return UnhandledPreference::Facet; }
  UnhandledPreference _GetUnhandledPreference(MSBsplineSurfaceCR, SimplifyGraphic&) const override { return UnhandledPreference::Facet; }
  UnhandledPreference _GetUnhandledPreference(PolyfaceQueryCR, SimplifyGraphic&) const override { return UnhandledPreference::Ignore; }
  UnhandledPreference _GetUnhandledPreference(IBRepEntityCR, SimplifyGraphic&) const override { return UnhandledPreference::Facet; }
  UnhandledPreference _GetUnhandledPreference(TextStringCR, SimplifyGraphic&) const override { return UnhandledPreference::Curve; }

  // Text is drawn as glyph regions, which are always filled.
  void _OnNewGeometry() override { m_inText = false; }
  bool _ProcessTextString(TextStringCR, SimplifyGraphic&) override {
    m_inText = true;
    return false;
  }

  bool _ProcessCurveVector(CurveVectorCR curve, bool filled, SimplifyGraphic& sg) override {
    // Regions are thrown back to be faceted, except unfilled ones in 2d which draw as outlines.
    if (curve.IsAnyRegionType() && (filled || m_is3d))
      return false;

    bvector<DPoint3d> strokes;
    curve.AddStrokePoints(strokes, *m_facetOptions);
    if (strokes.size() < 2)
      return true;

    TransformCR transform = sg.GetLocalToWorldTransform();
    Render::GraphicParamsCR graphicParams = sg.GetCurrentGraphicParams();
    Render::GeometryParamsCR geomParams = sg.GetCurrentGeometryParams();
    ColorDef color = graphicParams.GetLineColor();
    uint32_t width = std::max(1u, graphicParams.GetWidth());
    uint32_t linePixels = graphicParams.GetLinePixels();
    DgnSubCategoryId subCategoryId = geomParams.GetSubCategoryId();
    Render::DgnGeometryClass geometryClass = geomParams.GetGeometryClass();

    ElementPolyline* polyline = nullptr;
    for (auto& entry : m_graphics.m_polylines) {
      if (entry.m_color == color && entry.m_width == width && entry.m_linePixels == linePixels && entry.m_subCategoryId == subCategoryId && entry.m_geometryClass == geometryClass) {
        polyline = &entry;
        break;
      }
    }

    if (nullptr == polyline) {
      m_graphics.m_polylines.emplace_back(color, width, linePixels, subCategoryId, geometryClass);
      polyline = &m_graphics.m_polylines.back();
    }

    bvector<uint32_t> line;
    auto flush = [&]() {
      if (line.size() >= 2)
        polyline->m_lines.push_back(std::move(line));

      line.clear();
    };

    for (DPoint3d pt : strokes) {
      if (pt.IsDisconnect()) {
        flush();
        continue;
      }

      transform.Multiply(pt);
      line.push_back((uint32_t)polyline->m_points.size());
      polyline->m_points.push_back(pt);
    }

    flush();
    return true;
  }

  ColorDef ResolveColor(SimplifyGraphic& sg, std::unique_ptr<BeJsDocument>& patternMap) {
    ColorDef color = sg.GetCurrentGraphicParams().GetFillColor();
    RenderMaterialId materialId = sg.GetCurrentGeometryParams().GetMaterialId();
    if (!materialId.IsValid())
      return color;

    RenderMaterialCPtr material = RenderMaterial::Get(m_db, materialId);
    if (!material.IsValid())
      return color;

    RenderingAssetCR asset = material->GetRenderingAsset();
    if (asset.GetBool(RENDER_MATERIAL_FlagHasBaseColor, false)) {
      RgbFactor rgb = asset.GetColor(RENDER_MATERIAL_Color);
      color.SetRed((Byte)(rgb.red * 255.0));
      color.SetGreen((Byte)(rgb.green * 255.0));
      color.SetBlue((Byte)(rgb.blue * 255.0));
    }

    if (asset.GetBool(RENDER_MATERIAL_FlagHasTransmit, false))
      color.SetAlpha((Byte)(asset.GetDouble(RENDER_MATERIAL_Transmit, 0.0) * 255.0));

    auto map = asset.GetPatternMap();
    if (map.IsValid()) {
      patternMap = std::make_unique<BeJsDocument>();
      map.m_value.SaveTo(*patternMap);
    }

    return color;
  }

  // Texture coordinates for each index of pf, or null if the texture map yields none.
  std::unique_ptr<FPoint2d[]> ComputeParams(PolyfaceQueryCR pf, BeJsDocument const& patternMap, TransformCR localToWorld, DgnTextureId& textureId) {
    RenderingAsset::TextureMap textureMap(patternMap, RenderingAsset::TextureMap::Type::Pattern);
    textureId = textureMap.GetTextureId();
    if (!textureId.IsValid())
      return nullptr;

    Render::TextureMapping::Params mapParams = textureMap.GetTextureMapParams();
    DPoint3d globalOrigin = m_db.GeoLocation().GetGlobalOrigin();
    globalOrigin.Negate();
    ElevationDrapeParams drapeParams(Transform::FromProduct(localToWorld, Transform::From(globalOrigin)));
    bool isMirrored = localToWorld.Determinant() < 0;

    uint32_t indexCount = (uint32_t)(pf.GetPointIndexCount() - pf.GetPointIndexCount() / 4);
    std::unique_ptr<FPoint2d[]> params(new FPoint2d[indexCount]);
    bvector<DPoint2d> faceParams(3, DPoint2d::FromZero());
    uint32_t count = 0;
    for (auto visitor = PolyfaceVisitor::Attach(pf); visitor->AdvanceToNextFace() && count + 3 <= indexCount; ) {
      mapParams.ComputeUVParams(faceParams, *visitor, &drapeParams);
      for (int i = 0; i < 3; ++i)
        params[count + i] = MakeFPoint2d((float)faceParams[i].x, (float)faceParams[i].y);

      if (isMirrored)
        std::swap(params[count + 1], params[count + 2]);

      count += 3;
    }

    return params;
  }

  bool _ProcessPolyface(PolyfaceQueryCR pfIn, bool, SimplifyGraphic& sg) override {
    processPolyface(pfIn, [&](PolyfaceQueryCR pf) {
      std::unique_ptr<BeJsDocument> patternMap;
      ColorDef color = ResolveColor(sg, patternMap);
      TransformCR localToWorld = sg.GetLocalToWorldTransform();

      DgnTextureId textureId;
      std::unique_ptr<FPoint2d[]> params;
      if (patternMap)
        params = ComputeParams(pf, *patternMap, localToWorld, textureId);

      bool isTwoSided = pf.GetTwoSided();
      Render::GeometryParamsCR geomParams = sg.GetCurrentGeometryParams();
      DgnSubCategoryId subCategoryId = geomParams.GetSubCategoryId();
      Render::DgnGeometryClass geometryClass = geomParams.GetGeometryClass();
      uint32_t fillFlags = m_inText ? fillFlagsFrom(Render::FillDisplay::Always) : fillFlagsFrom(geomParams.GetFillDisplay());

      ElementMesh* mesh = nullptr;
      for (auto& entry : m_graphics.m_meshes) {
        if (entry.m_color == color && entry.m_isTwoSided == isTwoSided && entry.m_subCategoryId == subCategoryId && entry.m_geometryClass == geometryClass && entry.m_textureId == textureId && entry.m_fillFlags == fillFlags) {
          mesh = &entry;
          break;
        }
      }

      if (nullptr == mesh) {
        m_graphics.m_meshes.emplace_back(color, isTwoSided, subCategoryId, geometryClass, textureId, fillFlags);
        mesh = &m_graphics.m_meshes.back();
      }

      unifyIndices(IntermediateMesh(pf, localToWorld, std::move(params)), *mesh);
    });

    return true;
  }
};

struct GraphicsBuilder : SimplifyGraphic {
  DEFINE_T_SUPER(SimplifyGraphic);

  GraphicsBuilder(Render::GraphicBuilder::CreateParams const& params, IGeometryProcessorR processor, ViewContextR context)
    : T_Super(params, processor, context) { }

  void AddImage(ImageGraphicCR) override { }
  void AddImage2d(ImageGraphicCR, double) override { }
  bool _WantStrokePattern(PatternParamsCR) override { return false; }
  bool _WantStrokeLineStyle(Render::LineStyleSymbCR, IFacetOptionsPtr&) override { return false; }
  Render::GraphicPtr _Finish() override { m_isOpen = false; return nullptr; }

  Render::GraphicBuilderPtr _CreateSubGraphic(TransformCR subToGraphic, ClipVectorCP clip) const override {
    auto result = new GraphicsBuilder(Render::GraphicBuilder::CreateParams::Scene(GetDgnDb(),
      Transform::FromProduct(GetLocalToWorldTransform(), subToGraphic)), m_processor, m_context);

    result->m_currGraphicParams = m_currGraphicParams;
    result->m_currGeometryParams = m_currGeometryParams;
    result->m_currGeomEntryId = m_currGeomEntryId;
    result->m_currClip = nullptr != clip ? clip->Clone(&GetLocalToWorldTransform()) : nullptr;
    return result;
  }
};

struct GraphicsContext : NullContext {
  IGeometryProcessorR m_processor;

  explicit GraphicsContext(IGeometryProcessorR processor) : m_processor(processor) {
    m_purpose = processor._GetProcessPurpose();
    m_viewflags.SetShowConstructions(true);
  }

  Render::GraphicBuilderPtr _CreateGraphic(Render::GraphicBuilder::CreateParams const& params) override {
    return new GraphicsBuilder(params, m_processor, *this);
  }
};

bool hasPatternMap(DgnDbR db, Render::GeometryParamsCR params) {
  Render::GeometryParams resolved(params);
  resolved.Resolve(db);
  RenderMaterialId materialId = resolved.GetMaterialId();
  if (!materialId.IsValid())
    return false;

  RenderMaterialCPtr material = RenderMaterial::Get(db, materialId);
  return material.IsValid() && material->GetRenderingAsset().GetPatternMap().IsValid();
}

// Texture coordinates are expensive to compute for large meshes; only ask for them if a material needs them.
bool usesTexture(GeometrySourceCR source) {
  DgnDbR db = source.GetSourceDgnDb();
  for (auto iter : GeometryCollection(source)) {
    if (hasPatternMap(db, iter.GetGeometryParams()))
      return true;

    if (!iter.GetGeometryPartId().IsValid())
      continue;

    DgnGeometryPartCPtr part = iter.GetGeometryPartCPtr();
    if (part.IsNull())
      continue;

    for (auto partIter : GeometryCollection(part->GetGeometryStream(), db, &iter.GetGeometryParams()))
      if (hasPatternMap(db, partIter.GetGeometryParams()))
        return true;
  }

  return false;
}

} // namespace

bool ElementMesher::Mesh(ElementGraphics& graphics, GeometrySourceCR source, double chordTolerance) {
  auto facetOptions = IFacetOptions::CreateForSurfaces(chordTolerance, msGeomConst_piOver4, 0.0, true, true, usesTexture(source));
  facetOptions->SetIgnoreHiddenBRepEntities(true);
  facetOptions->SetBRepConcurrentFacetting(false);
  facetOptions->SetBRepIgnoredFeatureSize(chordTolerance);

  DgnDbR db = source.GetSourceDgnDb();
  GraphicsProcessor processor(db, facetOptions, graphics, source.Is3d());
  GraphicsContext context(processor);
  context.SetDgnDb(db);
  try {
    source.Draw(context, 0);
  } catch (...) {
    return false;
  }

  return true;
}
