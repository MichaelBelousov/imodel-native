/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
// Writes tiles in the iMdl format read by @itwin/core-frontend (see ImdlHeader in core-common and
// ParseImdlDocument.ts, VertexTableBuilder.ts, PolylineParams.ts and CompactEdges.ts in core-frontend).
#include "ImdlWriter.h"

USING_NAMESPACE_BENTLEY_DGN

namespace {

constexpr uint32_t IMDL_MAGIC = 0x6c644d69; // "iMdl"
constexpr uint32_t GLTF_MAGIC = 0x46546c67; // "glTF"
constexpr uint32_t IMDL_HEADER_LENGTH = 88;
constexpr uint32_t FEATURE_TABLE_HEADER_LENGTH = 12;
constexpr uint32_t GLTF_HEADER_LENGTH = 20;

// Mesh vertices are 4 RGBA: quantized xyz (3 x u16), then for lit meshes color index (u16), feature
// index (u32), oct normal (u16), unused (u16); for textured meshes oct normal (u16), feature index
// (u32), quantized uv (2 x u16). Polyline vertices are 3 RGBA: xyz, color index, feature index.
constexpr uint32_t MESH_RGBA_PER_VERTEX = 4;
constexpr uint32_t POLYLINE_RGBA_PER_VERTEX = 3;
// WebGL2 guarantees textures of at least this size.
constexpr uint32_t MAX_TEXTURE_DIMENSION = 2048;
constexpr size_t MAX_VERTICES_PER_PRIMITIVE = (MAX_TEXTURE_DIMENSION * MAX_TEXTURE_DIMENSION) / MESH_RGBA_PER_VERTEX - MAX_TEXTURE_DIMENSION;

// Enum values from core-common / core-frontend.
constexpr int FEATURE_INDEX_UNIFORM = 1;
constexpr int FEATURE_INDEX_NON_UNIFORM = 2;
constexpr int SURFACE_TYPE_LIT = 1;
constexpr int SURFACE_TYPE_TEXTURED_LIT = 3;
constexpr int PRIMITIVE_TYPE_MESH = 0;
constexpr int PRIMITIVE_TYPE_POLYLINE = 1;
constexpr int DISPLAY_PARAMS_MESH = 0;
constexpr int DISPLAY_PARAMS_LINEAR = 1;

struct Writer {
  bvector<uint8_t> m_bytes;

  size_t Size() const { return m_bytes.size(); }
  void Append(void const* data, size_t size) {
    auto p = static_cast<uint8_t const*>(data);
    m_bytes.insert(m_bytes.end(), p, p + size);
  }
  void U8(uint8_t v) { m_bytes.push_back(v); }
  void U16(uint16_t v) { Append(&v, 2); }
  void U24(uint32_t v) { U8(v & 0xff); U8((v >> 8) & 0xff); U8((v >> 16) & 0xff); }
  void U32(uint32_t v) { Append(&v, 4); }
  void F64(double v) { Append(&v, 8); }
  void PadTo4() { while (0 != m_bytes.size() % 4) m_bytes.push_back(0); }
  void SetU32(size_t pos, uint32_t v) { memcpy(&m_bytes[pos], &v, 4); }
};

void setXyz(BeJsValue json, DPoint3dCR pt) {
  json.SetEmptyArray();
  json.appendValue() = pt.x;
  json.appendValue() = pt.y;
  json.appendValue() = pt.z;
}

void setXy(BeJsValue json, double x, double y) {
  json.SetEmptyArray();
  json.appendValue() = x;
  json.appendValue() = y;
}

void appendString(BeJsValue json, Utf8CP value) {
  json.appendValue() = value;
}

double quantizationScale(double extent) {
  return 0.0 == extent ? 0.0 : 0xffff / extent;
}

uint16_t quantize(double pos, double origin, double scale) {
  return (uint16_t)std::floor(std::max(0.0, std::min(65535.0, 0.5 + (pos - origin) * scale)));
}

struct Quantizer {
  DPoint3d m_origin;
  DPoint3d m_scale;

  explicit Quantizer(DRange3dCR range) : m_origin(range.low) {
    m_scale = DPoint3d::From(quantizationScale(range.high.x - range.low.x), quantizationScale(range.high.y - range.low.y), quantizationScale(range.high.z - range.low.z));
  }

  void Append(Writer& out, DPoint3dCR pt) const {
    out.U16(quantize(pt.x, m_origin.x, m_scale.x));
    out.U16(quantize(pt.y, m_origin.y, m_scale.y));
    out.U16(quantize(pt.z, m_origin.z, m_scale.z));
  }
};

uint8_t octComponent(double v) {
  return (uint8_t)(uint16_t)(0.5 + (std::max(-1.0, std::min(1.0, v)) * 0.5 + 0.5) * 255);
}

// Matches computeDimensions in core-frontend's VertexTable.ts.
void computeDimensions(uint32_t& width, uint32_t& height, size_t numVertices, uint32_t rgbaPerVertex) {
  size_t nRgba = numVertices * rgbaPerVertex;
  if (nRgba <= MAX_TEXTURE_DIMENSION) {
    width = (uint32_t)std::max<size_t>(nRgba, rgbaPerVertex);
    height = 1;
    return;
  }

  width = (uint32_t)std::ceil(std::sqrt((double)nRgba));
  if (0 != width % rgbaPerVertex)
    width += rgbaPerVertex - width % rgbaPerVertex;

  height = (uint32_t)((nRgba + width - 1) / width);
}

bool isUniform(bvector<uint32_t> const& featureIndices) {
  for (size_t i = 1; i < featureIndices.size(); ++i)
    if (featureIndices[i] != featureIndices[0])
      return false;

  return true;
}

// Port of PolylineTesselator in core-frontend's PolylineParams.ts.
struct PolylineTesselator {
  enum Param : uint32_t {
    kSquare = 1 * 3,
    kMiter = 2 * 3,
    kMiterInsideOnly = 3 * 3,
    kJointBase = 4 * 3,
    kNegatePerp = 8 * 3,
    kNegateAlong = 16 * 3,
    kNoneAdjustWeight = 32 * 3,
  };

  struct Vertex {
    bool m_isSegmentStart = false;
    bool m_isPolylineStartOrEnd = false;
    uint32_t m_vertexIndex = 0;
    uint32_t m_prevIndex = 0;
    uint32_t m_nextIndex = 0;

    void Init(bool isSegmentStart, bool isPolylineStartOrEnd, uint32_t vertexIndex, uint32_t prevIndex, uint32_t nextIndex) {
      m_isSegmentStart = isSegmentStart;
      m_isPolylineStartOrEnd = isPolylineStartOrEnd;
      m_vertexIndex = vertexIndex;
      m_prevIndex = prevIndex;
      m_nextIndex = nextIndex;
    }

    uint32_t ComputeParam(bool negatePerp, bool adjacentToJoint = false, bool joint = false, bool noDisplacement = false) const {
      if (joint)
        return kJointBase;

      uint32_t param;
      if (noDisplacement)
        param = kNoneAdjustWeight;
      else if (adjacentToJoint)
        param = kMiterInsideOnly;
      else
        param = m_isPolylineStartOrEnd ? kSquare : kMiter;

      uint32_t adjust = 0;
      if (negatePerp)
        adjust = kNegatePerp;
      if (!m_isSegmentStart)
        adjust += kNegateAlong;

      return param + adjust;
    }
  };

  bvector<DPoint3d> const& m_points;
  bool m_doJoints;
  Writer m_indices;
  Writer m_prevIndices;
  Writer m_nextIndicesAndParams;
  uint32_t m_count = 0;

  PolylineTesselator(bvector<DPoint3d> const& points, bool doJoints) : m_points(points), m_doJoints(doJoints) { }

  void Add(Vertex const& v, uint32_t param) {
    m_indices.U24(v.m_vertexIndex);
    m_prevIndices.U24(v.m_prevIndex);
    m_nextIndicesAndParams.U24(v.m_nextIndex);
    m_nextIndicesAndParams.U8((uint8_t)(param & 0xff));
    ++m_count;
  }

  double DotProduct(Vertex const& v) const {
    DPoint3dCR pos = m_points[v.m_vertexIndex];
    DVec3d prevDir = DVec3d::FromStartEnd(m_points[v.m_prevIndex], pos);
    DVec3d nextDir = DVec3d::FromStartEnd(m_points[v.m_nextIndex], pos);
    return prevDir.DotProduct(nextDir);
  }

  void AddJointTriangles(Vertex const& v0, uint32_t p0, Vertex const& v1) {
    uint32_t param = v1.ComputeParam(false, false, true);
    for (uint32_t i = 0; i < 3; i++) {
      Add(v0, p0);
      Add(v1, param + i + 1);
      Add(v1, param + i);
    }
  }

  void Tesselate(bvector<bvector<uint32_t>> const& lines) {
    Vertex v0, v1;
    const double maxJointDot = -0.7;
    for (auto const& line : lines) {
      if (line.size() < 2)
        continue;

      size_t last = line.size() - 1;
      bool isClosed = line[0] == line[last];
      for (size_t i = 0; i < last; ++i) {
        uint32_t idx0 = line[i];
        uint32_t idx1 = line[i + 1];
        bool isStart = 0 == i;
        bool isEnd = last - 1 == i;
        uint32_t prevIdx0 = isStart ? (isClosed ? line[last - 1] : idx0) : line[i - 1];
        uint32_t nextIdx1 = isEnd ? (isClosed ? line[1] : idx1) : line[i + 2];

        v0.Init(true, isStart && !isClosed, idx0, prevIdx0, idx1);
        v1.Init(false, isEnd && !isClosed, idx1, nextIdx1, idx0);

        bool jointAt0 = m_doJoints && (isClosed || !isStart) && DotProduct(v0) > maxJointDot;
        bool jointAt1 = m_doJoints && (isClosed || !isEnd) && DotProduct(v1) > maxJointDot;
        if (jointAt0 || jointAt1) {
          Add(v0, v0.ComputeParam(true, jointAt0, false, false));
          Add(v1, v1.ComputeParam(false, jointAt1, false, false));
          Add(v0, v0.ComputeParam(false, jointAt0, false, true));
          Add(v0, v0.ComputeParam(false, jointAt0, false, true));
          Add(v1, v1.ComputeParam(false, jointAt1, false, false));
          Add(v1, v1.ComputeParam(false, jointAt1, false, true));
          Add(v0, v0.ComputeParam(false, jointAt0, false, true));
          Add(v1, v1.ComputeParam(false, jointAt1, false, true));
          Add(v0, v0.ComputeParam(false, jointAt0, false, false));
          Add(v0, v0.ComputeParam(false, jointAt0, false, false));
          Add(v1, v1.ComputeParam(false, jointAt1, false, true));
          Add(v1, v1.ComputeParam(true, jointAt1, false, false));

          if (jointAt0)
            AddJointTriangles(v0, v0.ComputeParam(false, true, false, true), v0);

          if (jointAt1)
            AddJointTriangles(v1, v1.ComputeParam(false, true, false, true), v1);
        } else {
          Add(v0, v0.ComputeParam(true));
          Add(v1, v1.ComputeParam(false));
          Add(v0, v0.ComputeParam(false));
          Add(v0, v0.ComputeParam(false));
          Add(v1, v1.ComputeParam(false));
          Add(v1, v1.ComputeParam(true));
        }
      }
    }
  }
};

struct DocumentWriter {
  Writer m_bin;
  BeJsDocument m_scene;
  uint32_t m_nextId = 0;

  DocumentWriter() {
    m_scene.SetEmptyObject();
    m_scene["bufferViews"].SetEmptyObject();
    m_scene["materials"].SetEmptyObject();
    m_scene["meshes"]["Mesh_Root"]["primitives"].SetEmptyArray();
  }

  BeJsValue BufferViews() { return m_scene["bufferViews"]; }
  BeJsValue Materials() { return m_scene["materials"]; }
  BeJsValue Primitives() { return m_scene["meshes"]["Mesh_Root"]["primitives"]; }

  Utf8String NextName(Utf8CP prefix) { return Utf8PrintfString("%s%u", prefix, m_nextId++); }

  Utf8String AddBufferView(Writer const& data) {
    Utf8String name = NextName("bv");
    m_bin.PadTo4();
    auto view = BufferViews()[name.c_str()];
    view["buffer"] = "binary_glTF";
    view["byteOffset"] = (uint32_t)m_bin.Size();
    view["byteLength"] = (uint32_t)data.Size();
    m_bin.Append(data.m_bytes.data(), data.Size());
    return name;
  }

  void WriteVertexTableJson(BeJsValue json, Utf8StringCR bufferView, DRange3dCR range, bvector<uint32_t> const& featureIndices, ColorDef color, size_t count, uint32_t width, uint32_t height, uint32_t rgbaPerVertex) {
    json["bufferView"] = bufferView.c_str();
    auto params = json["params"];
    setXyz(params["decodedMin"], range.low);
    setXyz(params["decodedMax"], range.high);
    bool uniform = isUniform(featureIndices);
    json["featureIndexType"] = uniform ? FEATURE_INDEX_UNIFORM : FEATURE_INDEX_NON_UNIFORM;
    if (uniform)
      json["featureID"] = featureIndices[0];
    json["hasTranslucency"] = 0 != color.GetAlpha();
    json["numColors"] = 0;
    json["uniformColor"] = color.GetValue();
    json["count"] = (uint32_t)count;
    json["width"] = width;
    json["height"] = height;
    json["numRgbaPerVertex"] = rgbaPerVertex;
  }

  Utf8String AddMaterial(int type, ColorDef color, uint32_t width, uint32_t linePixels, uint32_t fillFlags, DgnTextureId textureId) {
    Utf8String name = NextName("Material");
    auto material = Materials()[name.c_str()];
    material["type"] = type;
    material["ignoreLighting"] = false;
    material["fillColor"] = color.GetValue();
    material["fillFlags"] = fillFlags;
    material["lineColor"] = color.GetValue();
    material["lineWidth"] = width;
    material["linePixels"] = linePixels;
    if (textureId.IsValid()) {
      auto texture = material["texture"];
      texture["name"] = textureId.ToHexStr().c_str();
      auto params = texture["params"];
      auto transform = params["transform"];
      transform.SetEmptyArray();
      setXyz(transform.appendValue(), DPoint3d::From(1, 0, 0));
      setXyz(transform.appendValue(), DPoint3d::From(0, 1, 0));
      params["weight"] = 1.0;
      params["mode"] = 0; // TextureMapping.Mode.Parametric
      params["worldMapping"] = false;
    }

    return name;
  }

  void AddMesh(ImdlMesh const& mesh, bool isPlanar, bool wantEdges) {
    bool textured = mesh.m_textureId.IsValid() && mesh.m_params.size() == mesh.m_points.size();
    size_t numVertices = mesh.m_points.size();
    uint32_t width, height;
    computeDimensions(width, height, numVertices, MESH_RGBA_PER_VERTEX);

    Quantizer quantizer(mesh.m_range);
    DRange2d uvRange = DRange2d::NullRange();
    if (textured)
      for (auto const& uv : mesh.m_params)
        uvRange.Extend(DPoint2d::From(uv.x, uv.y));

    double su = textured ? quantizationScale(uvRange.high.x - uvRange.low.x) : 0;
    double sv = textured ? quantizationScale(uvRange.high.y - uvRange.low.y) : 0;

    Writer vertices;
    vertices.m_bytes.reserve((size_t)width * height * 4);
    for (size_t i = 0; i < numVertices; ++i) {
      quantizer.Append(vertices, mesh.m_points[i]);
      if (textured) {
        vertices.U16(mesh.m_normals[i]);
        vertices.U32(mesh.m_featureIndices[i]);
        vertices.U16(quantize(mesh.m_params[i].x, uvRange.low.x, su));
        vertices.U16(quantize(mesh.m_params[i].y, uvRange.low.y, sv));
      } else {
        vertices.U16(0);
        vertices.U32(mesh.m_featureIndices[i]);
        vertices.U16(mesh.m_normals[i]);
        vertices.U16(0);
      }
    }
    vertices.m_bytes.resize((size_t)width * height * 4, 0);

    Writer indices;
    for (uint32_t index : mesh.m_indices)
      indices.U24(index);

    Utf8String vertexView = AddBufferView(vertices);
    Utf8String indexView = AddBufferView(indices);
    Utf8String material = AddMaterial(DISPLAY_PARAMS_MESH, mesh.m_color, 1, 0, mesh.m_fillFlags, textured ? mesh.m_textureId : DgnTextureId());

    auto json = Primitives().appendObject();
    json["type"] = PRIMITIVE_TYPE_MESH;
    json["isPlanar"] = isPlanar;
    json["material"] = material.c_str();
    WriteVertexTableJson(json["vertices"], vertexView, mesh.m_range, mesh.m_featureIndices, mesh.m_color, numVertices, width, height, MESH_RGBA_PER_VERTEX);
    auto surface = json["surface"];
    surface["indices"] = indexView.c_str();
    surface["type"] = textured ? SURFACE_TYPE_TEXTURED_LIT : SURFACE_TYPE_LIT;
    if (textured) {
      auto uvParams = surface["uvParams"];
      setXy(uvParams["decodedMin"], uvRange.low.x, uvRange.low.y);
      setXy(uvParams["decodedMax"], uvRange.high.x, uvRange.high.y);
    }

    if (!wantEdges || mesh.m_edges.size() != mesh.m_indices.size())
      return;

    Writer visibility;
    uint32_t numVisible = 0;
    uint8_t bits = 0;
    for (size_t i = 0; i < mesh.m_edges.size(); ++i) {
      auto edge = mesh.m_edges[i];
      if (ImdlEdgeVisibility::Visible == edge)
        ++numVisible;

      bits |= (uint8_t)edge << ((i % 4) * 2);
      if (3 == i % 4) {
        visibility.U8(bits);
        bits = 0;
      }
    }
    if (0 != mesh.m_edges.size() % 4)
      visibility.U8(bits);

    if (0 == numVisible && mesh.m_silhouetteNormals.empty())
      return;

    auto compact = json["edges"]["compact"];
    compact["visibility"] = AddBufferView(visibility).c_str();
    compact["numVisible"] = numVisible;
    if (!mesh.m_silhouetteNormals.empty()) {
      Writer normals;
      for (uint32_t pair : mesh.m_silhouetteNormals)
        normals.U32(pair);

      compact["normalPairs"] = AddBufferView(normals).c_str();
    }
  }

  void AddPolyline(ImdlPolyline const& polyline, bool isPlanar, bool is2d) {
    PolylineTesselator tesselator(polyline.m_points, is2d || polyline.m_width >= 3);
    tesselator.Tesselate(polyline.m_lines);
    if (0 == tesselator.m_count)
      return;

    size_t numVertices = polyline.m_points.size();
    uint32_t width, height;
    computeDimensions(width, height, numVertices, POLYLINE_RGBA_PER_VERTEX);

    Quantizer quantizer(polyline.m_range);
    Writer vertices;
    vertices.m_bytes.reserve((size_t)width * height * 4);
    for (size_t i = 0; i < numVertices; ++i) {
      quantizer.Append(vertices, polyline.m_points[i]);
      vertices.U16(0);
      vertices.U32(polyline.m_featureIndices[i]);
    }
    vertices.m_bytes.resize((size_t)width * height * 4, 0);

    Utf8String vertexView = AddBufferView(vertices);
    Utf8String material = AddMaterial(DISPLAY_PARAMS_LINEAR, polyline.m_color, polyline.m_width, polyline.m_linePixels, 0, DgnTextureId());

    auto json = Primitives().appendObject();
    json["type"] = PRIMITIVE_TYPE_POLYLINE;
    json["isPlanar"] = isPlanar;
    json["material"] = material.c_str();
    WriteVertexTableJson(json["vertices"], vertexView, polyline.m_range, polyline.m_featureIndices, polyline.m_color, numVertices, width, height, POLYLINE_RGBA_PER_VERTEX);
    json["indices"] = AddBufferView(tesselator.m_indices).c_str();
    json["prevIndices"] = AddBufferView(tesselator.m_prevIndices).c_str();
    json["nextIndicesAndParams"] = AddBufferView(tesselator.m_nextIndicesAndParams).c_str();
  }
};

} // namespace

uint16_t Dgn::OctEncodeNormal(double nx, double ny, double nz) {
  double denom = std::abs(nx) + std::abs(ny) + std::abs(nz);
  if (0.0 == denom)
    return 0;

  double rx = nx / denom;
  double ry = ny / denom;
  if (nz < 0) {
    double x = rx, y = ry;
    rx = (1 - std::abs(y)) * (x < 0 ? -1.0 : 1.0);
    ry = (1 - std::abs(x)) * (y < 0 ? -1.0 : 1.0);
  }

  return (uint16_t)((octComponent(ry) << 8) | octComponent(rx));
}

uint32_t ImdlTile::GetFeatureIndex(DgnElementId elementId, DgnSubCategoryId subCategoryId, Render::DgnGeometryClass geometryClass) {
  auto key = std::make_tuple(elementId.GetValueUnchecked(), subCategoryId.GetValueUnchecked(), (uint8_t)geometryClass);
  auto found = m_featureIndices.find(key);
  if (m_featureIndices.end() != found)
    return found->second;

  uint32_t subCategoryIndex;
  auto foundSubCategory = m_subCategoryIndices.find(subCategoryId.GetValueUnchecked());
  if (m_subCategoryIndices.end() != foundSubCategory) {
    subCategoryIndex = foundSubCategory->second;
  } else {
    subCategoryIndex = (uint32_t)m_subCategories.size();
    m_subCategories.push_back(subCategoryId);
    m_subCategoryIndices[subCategoryId.GetValueUnchecked()] = subCategoryIndex;
  }

  uint32_t index = (uint32_t)m_features.size();
  m_features.push_back({elementId, subCategoryIndex, geometryClass});
  m_featureIndices[key] = index;
  return index;
}

ImdlMesh& ImdlTile::GetMesh(ColorDef color, DgnTextureId textureId, uint32_t fillFlags, size_t vertexCount) {
  for (auto& mesh : m_meshes)
    if (mesh.m_color == color && mesh.m_textureId == textureId && mesh.m_fillFlags == fillFlags && mesh.m_points.size() + vertexCount <= MAX_VERTICES_PER_PRIMITIVE)
      return mesh;

  m_meshes.emplace_back(color, textureId, fillFlags);
  return m_meshes.back();
}

ImdlPolyline& ImdlTile::GetPolyline(ColorDef color, uint32_t width, uint32_t linePixels, size_t vertexCount) {
  for (auto& polyline : m_polylines)
    if (polyline.m_color == color && polyline.m_width == width && polyline.m_linePixels == linePixels && polyline.m_points.size() + vertexCount <= MAX_VERTICES_PER_PRIMITIVE)
      return polyline;

  m_polylines.emplace_back(color, width, linePixels);
  return m_polylines.back();
}

void ImdlTile::AddTexture(DgnDbR db, DgnTextureId textureId) {
  if (!textureId.IsValid() || m_textures.end() != m_textures.find(textureId.GetValueUnchecked()))
    return;

  Texture& texture = m_textures[textureId.GetValueUnchecked()];
  DgnTextureCPtr element = DgnTexture::Get(db, textureId);
  if (element.IsNull())
    return;

  Render::ImageSourceCR image = element->GetImageSource();
  texture.m_format = (uint32_t)image.GetFormat();
  ByteStream const& bytes = image.GetByteStream();
  texture.m_data.assign(bytes.GetData(), bytes.GetData() + bytes.GetSize());
}

ByteStream ImdlTile::Write() const {
  Writer out;

  // Header; tileLength is filled in at the end.
  out.U32(IMDL_MAGIC);
  out.U32(IMDL_FORMAT_VERSION);
  out.U32(IMDL_HEADER_LENGTH);
  out.U32(m_flags);
  DRange3d range = m_contentRange.IsNull() ? DRange3d::NullRange() : m_contentRange;
  out.F64(range.low.x); out.F64(range.low.y); out.F64(range.low.z);
  out.F64(range.high.x); out.F64(range.high.y); out.F64(range.high.z);
  out.F64(m_tolerance);
  out.U32(m_numElementsIncluded);
  out.U32(m_numElementsExcluded);
  size_t tileLengthPos = out.Size();
  out.U32(0);
  out.U32(m_emptySubRanges);
  BeAssert(IMDL_HEADER_LENGTH == out.Size());

  // Feature table: 3 u32 per feature, then 2 u32 per subcategory (PackedFeatureTable in core-common).
  uint32_t featureTableLength = FEATURE_TABLE_HEADER_LENGTH + 4 * (3 * (uint32_t)m_features.size() + 2 * (uint32_t)m_subCategories.size());
  out.U32(featureTableLength);
  out.U32((uint32_t)m_subCategories.size());
  out.U32((uint32_t)m_features.size());
  for (auto const& feature : m_features) {
    uint64_t id = feature.m_elementId.GetValueUnchecked();
    out.U32((uint32_t)(id & 0xffffffff));
    out.U32((uint32_t)(id >> 32));
    out.U32(feature.m_subCategoryIndex | ((uint32_t)feature.m_geometryClass << 24));
  }
  for (auto const& subCategoryId : m_subCategories) {
    uint64_t id = subCategoryId.GetValueUnchecked();
    out.U32((uint32_t)(id & 0xffffffff));
    out.U32((uint32_t)(id >> 32));
  }

  DocumentWriter doc;
  for (auto const& mesh : m_meshes)
    if (!mesh.m_indices.empty())
      doc.AddMesh(mesh, m_is2d, m_wantEdges);

  for (auto const& polyline : m_polylines)
    if (!polyline.m_lines.empty())
      doc.AddPolyline(polyline, m_is2d, m_is2d);

  if (!m_textures.empty()) {
    auto namedTextures = doc.m_scene["namedTextures"];
    namedTextures.SetEmptyObject();
    for (auto const& entry : m_textures) {
      if (entry.second.m_data.empty())
        continue;

      Writer data;
      data.Append(entry.second.m_data.data(), entry.second.m_data.size());
      auto texture = namedTextures[DgnTextureId(entry.first).ToHexStr().c_str()];
      texture["bufferView"] = doc.AddBufferView(data).c_str();
      texture["format"] = entry.second.m_format;
    }
  }

  doc.m_bin.PadTo4();
  BeJsDocument& scene = doc.m_scene;
  scene["nodes"]["Node_Root"] = "Mesh_Root";
  appendString(scene["extensionsUsed"], "KHR_binary_glTF");
  appendString(scene["extensionsUsed"], "WEB3D_quantized_attributes");
  appendString(scene["glExtensionsUsed"], "OES_element_index_uint");
  scene["scene"] = "defaultScene";
  appendString(scene["scenes"]["defaultScene"]["nodes"], "rootNode");
  scene["buffers"]["binary_glTF"]["byteLength"] = (uint32_t)doc.m_bin.Size();

  Utf8String sceneStr = doc.m_scene.Stringify();
  while (0 != (out.Size() + GLTF_HEADER_LENGTH + sceneStr.size()) % 4)
    sceneStr.push_back(' ');

  size_t gltfStart = out.Size();
  out.U32(GLTF_MAGIC);
  out.U32(1);
  size_t gltfLengthPos = out.Size();
  out.U32(0);
  out.U32((uint32_t)sceneStr.size());
  out.U32(0); // scene format: JSON
  out.Append(sceneStr.data(), sceneStr.size());
  out.Append(doc.m_bin.m_bytes.data(), doc.m_bin.Size());

  out.SetU32(gltfLengthPos, (uint32_t)(out.Size() - gltfStart));
  out.SetU32(tileLengthPos, (uint32_t)out.Size());

  ByteStream result;
  result.SaveData(out.m_bytes.data(), (uint32_t)out.Size());
  return result;
}
