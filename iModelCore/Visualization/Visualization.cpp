/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
// Open-source stand-in for Bentley's private Visualization module. Produces iMdl tiles for 2d and 3d
// models: elements are faceted at each tile's chord tolerance (as exportGraphics does), simplified
// with meshoptimizer to the same tolerance, and clipped to the tile. Element graphics requests are not
// supported.
#include <Visualization/Visualization.h>
#include "ElementMesher.h"
#include "ImdlWriter.h"
#include <meshoptimizer.h>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <unordered_map>

USING_NAMESPACE_BENTLEY_DGN
USING_NAMESPACE_BENTLEY_SQLITE

namespace {

// ElementGraphicsStatus in iModelJsNodeAddon/api_package/ts/src/NativeLibrary.ts.
enum class ElementGraphicsStatus { Success = 0, Canceled = 1, NoGeometry = 2, InvalidJson = 3, UnknownMajorFormatVersion = 4, ElementNotFound = 5 };

// TreeFlags in @itwin/core-common.
constexpr uint32_t TreeFlags_UseProjectExtents = 1 << 0;
constexpr uint32_t TreeFlags_UseLargerTiles = 1 << 3;

// Elements whose range diagonal is smaller than this many chord tolerances (about 2 pixels) are left to child tiles.
constexpr double MIN_ELEMENT_SIZE_IN_TOLERANCES = 4.0;
// Meshes with fewer triangles than this are not worth simplifying.
constexpr size_t MIN_TRIANGLES_TO_SIMPLIFY = 64;
// Adjacent triangles whose normals differ by less than this are coplanar; their shared edge is never drawn.
constexpr double COPLANAR_DOT = 0.99999;
// Vertex normals closer than this on either side of an edge mean the edge is smooth rather than a crease.
constexpr float SMOOTH_NORMAL_DOT = 0.9999f;
// Faces meeting at less than this angle (cos 35 degrees) share smoothed normals after simplification.
constexpr double CREASE_DOT = 0.82;
// Once a tile holds this many vertices, smaller elements are left to child tiles.
constexpr size_t MAX_TILE_VERTICES = 1 << 20;

// A tile tree id is "<version>_<flags>-<options><modelId>", e.g. "25_1d-E:0_0x41" (iModelTileTreeIdToString in core-common).
struct TreeId {
  DgnModelId m_modelId;
  uint32_t m_flags = 0;
  bool m_wantEdges = true;

  static bool Parse(TreeId& out, Utf8StringCR id) {
    // Volume classifiers need closed volumes, which are not produced. Planar classifiers use ordinary graphics.
    size_t classifier = id.find("C:");
    if (Utf8String::npos != classifier && (0 == classifier || 'P' != id[classifier - 1]))
      return false;

    size_t modelPos = id.rfind("0x");
    if (Utf8String::npos == modelPos || (modelPos > 0 && id[modelPos - 1] != '_' && id[modelPos - 1] != '-'))
      return false;

    out.m_modelId = DgnModelId(BeInt64Id::FromString(id.c_str() + modelPos).GetValueUnchecked());
    if (!out.m_modelId.IsValid())
      return false;

    size_t dash = id.find('-');
    size_t underscore = id.find('_');
    if (Utf8String::npos != dash && Utf8String::npos != underscore && underscore < dash)
      out.m_flags = (uint32_t)strtoul(id.substr(underscore + 1, dash - underscore - 1).c_str(), nullptr, 16);

    out.m_wantEdges = Utf8String::npos == id.find("E:0_");
    return true;
  }
};

// The last five components of a content id are depth, i, j, k and size multiplier, in hex (ContentIdProvider in core-common).
struct ContentId {
  uint32_t m_depth = 0, m_i = 0, m_j = 0, m_k = 0, m_multiplier = 1;

  static bool Parse(ContentId& out, Utf8StringCR id) {
    bvector<Utf8String> parts;
    Utf8String part;
    for (char c : id) {
      if ('-' == c || '/' == c || '_' == c) {
        parts.push_back(part);
        part.clear();
      } else {
        part.push_back(c);
      }
    }
    parts.push_back(part);
    if (parts.size() < 5)
      return false;

    uint32_t values[5];
    for (size_t n = 0; n < 5; ++n) {
      Utf8StringCR str = parts[parts.size() - 5 + n];
      if (str.empty())
        return false;

      values[n] = (uint32_t)strtoul(str.c_str(), nullptr, 16);
    }

    out.m_depth = values[0];
    out.m_i = values[1];
    out.m_j = values[2];
    out.m_k = values[3];
    out.m_multiplier = std::max(values[4], 1u);
    return true;
  }
};

// Matches bisectTileRange3d and bisectTileRange2d in core-common.
void bisectRange(DRange3dR range, bool takeLow, bool is2d) {
  DVec3d diag = range.DiagonalVector();
  DPoint3dR pt = takeLow ? range.high : range.low;
  if (is2d) {
    if (diag.x > diag.y)
      pt.x = (range.low.x + range.high.x) / 2.0;
    else
      pt.y = (range.low.y + range.high.y) / 2.0;
  } else if (diag.x > diag.y && diag.x > diag.z) {
    pt.x = (range.low.x + range.high.x) / 2.0;
  } else if (diag.y > diag.z) {
    pt.y = (range.low.y + range.high.y) / 2.0;
  } else {
    pt.z = (range.low.z + range.high.z) / 2.0;
  }
}

DRange3d translated(DRange3dCR range, DPoint3dCR offset) {
  DRange3d result = range;
  result.low.Add(offset);
  result.high.Add(offset);
  return result;
}

// Tile tree coordinates are the model's coordinates relative to the center of the tree's range.
struct TreeInfo {
  DgnModelId m_modelId;
  bool m_is2d = false;
  bool m_isEmpty = true;
  bool m_wantEdges = true;
  DPoint3d m_center = DPoint3d::FromZero();
  DRange3d m_range = DRange3d::NullRange(); // root tile range, tree coordinates
  DRange3d m_contentRange = DRange3d::NullRange(); // model range, tree coordinates
  uint32_t m_tileScreenSize = 512;

  DRange3d ComputeTileRange(ContentId const& id) const {
    DRange3d range = m_range;
    for (uint32_t level = 1; level <= id.m_depth; ++level) {
      uint32_t shift = id.m_depth - level;
      bisectRange(range, 0 == ((id.m_i >> shift) & 1), m_is2d);
      bisectRange(range, 0 == ((id.m_j >> shift) & 1), m_is2d);
      if (!m_is2d)
        bisectRange(range, 0 == ((id.m_k >> shift) & 1), m_is2d);
    }

    return range;
  }

  double Diagonal(DRange3dCR range) const {
    DVec3d diag = range.DiagonalVector();
    return m_is2d ? diag.MagnitudeXY() : diag.Magnitude();
  }

  // Matches computeTileChordTolerance in core-common.
  double ComputeTolerance(DRange3dCR tileRange, uint32_t multiplier) const {
    return Diagonal(tileRange) / (m_tileScreenSize * 2.0 * std::max(multiplier, 1u));
  }

  static DgnDbStatus Compute(TreeInfo& info, DgnDbR db, Utf8StringCR treeIdStr) {
    TreeId treeId;
    if (!TreeId::Parse(treeId, treeIdStr))
      return DgnDbStatus::NotFound;

    auto model = db.Models().Get<GeometricModel>(treeId.m_modelId);
    if (model.IsNull())
      return DgnDbStatus::NotFound;

    info.m_modelId = treeId.m_modelId;
    info.m_is2d = nullptr == model->ToGeometricModel3d();
    info.m_wantEdges = treeId.m_wantEdges;
    info.m_tileScreenSize = 0 != (treeId.m_flags & TreeFlags_UseLargerTiles) ? 2048 : 512;

    AxisAlignedBox3d modelRange = model->QueryElementsRange();
    if (modelRange.IsNull())
      return DgnDbStatus::Success;

    info.m_isEmpty = false;
    DRange3d treeRange = modelRange;
    if (!info.m_is2d && 0 != (treeId.m_flags & TreeFlags_UseProjectExtents))
      treeRange.Extend(db.GeoLocation().GetProjectExtents());

    if (info.m_is2d) {
      // 2d graphics are offset in z by display priority.
      treeRange.low.z = std::min(treeRange.low.z, -1.0);
      treeRange.high.z = std::max(treeRange.high.z, 1.0);
    }

    info.m_center = DPoint3d::FromInterpolate(treeRange.low, 0.5, treeRange.high);
    DPoint3d offset = DPoint3d::From(-info.m_center.x, -info.m_center.y, -info.m_center.z);
    info.m_contentRange = translated(modelRange, offset);

    // Pad so planar models still have volume.
    double pad = std::max(1.0e-3, treeRange.DiagonalDistance() * 1.0e-6);
    info.m_range = translated(treeRange, offset);
    info.m_range.Extend(pad);
    return DgnDbStatus::Success;
  }
};

void writeRange(BeJsValue json, DRange3dCR range) {
  auto low = json["low"];
  low.SetEmptyArray();
  low.appendValue() = range.low.x;
  low.appendValue() = range.low.y;
  low.appendValue() = range.low.z;
  auto high = json["high"];
  high.SetEmptyArray();
  high.appendValue() = range.high.x;
  high.appendValue() = range.high.y;
  high.appendValue() = range.high.z;
}

// An element mesh in tree coordinates, with per-vertex flags for the tile faces each vertex lies on.
struct TileMesh {
  bvector<DPoint3d> m_points;
  bvector<FPoint3d> m_normals;
  bvector<FPoint2d> m_params;
  bvector<uint8_t> m_clipFaces;
  bvector<uint32_t> m_indices;
  bool m_hasParams = false;

  TileMesh(ElementMesh const& mesh, DPoint3dCR center) {
    size_t count = mesh.GetPointCount();
    m_hasParams = mesh.IsTextured() && mesh.m_params.size() == count * 2;
    m_points.resize(count);
    m_normals.resize(count);
    m_clipFaces.assign(count, 0);
    for (size_t i = 0; i < count; ++i) {
      m_points[i] = DPoint3d::From(mesh.m_points[i * 3] - center.x, mesh.m_points[i * 3 + 1] - center.y, mesh.m_points[i * 3 + 2] - center.z);
      m_normals[i] = FPoint3d::From(mesh.m_normals[i * 3], mesh.m_normals[i * 3 + 1], mesh.m_normals[i * 3 + 2]);
    }

    if (m_hasParams) {
      m_params.resize(count);
      for (size_t i = 0; i < count; ++i)
        m_params[i] = MakeFPoint2d(mesh.m_params[i * 2], mesh.m_params[i * 2 + 1]);
    }

    m_indices = mesh.m_indices;
  }

  // Weld vertices by position (and texture coordinates), simplify to within tolerance, then rebuild vertex
  // normals, splitting them where adjacent faces meet at more than the crease angle. Returns true if any
  // detail was removed.
  bool Simplify(double tolerance) {
    size_t indexCount = m_indices.size();
    if (indexCount < MIN_TRIANGLES_TO_SIMPLIFY * 3)
      return false;

    struct WeldVertex { float x, y, z, u, v; };
    size_t vertexCount = m_points.size();
    bvector<WeldVertex> vertices(vertexCount);
    for (size_t i = 0; i < vertexCount; ++i) {
      vertices[i] = {(float)m_points[i].x, (float)m_points[i].y, (float)m_points[i].z, 0, 0};
      if (m_hasParams) {
        vertices[i].u = m_params[i].x;
        vertices[i].v = m_params[i].y;
      }
    }

    bvector<uint32_t> remap(vertexCount);
    size_t weldedCount = meshopt_generateVertexRemap(remap.data(), m_indices.data(), indexCount, vertices.data(), vertexCount, sizeof(WeldVertex));
    bvector<WeldVertex> welded(weldedCount);
    meshopt_remapVertexBuffer(welded.data(), vertices.data(), vertexCount, sizeof(WeldVertex), remap.data());
    bvector<uint32_t> weldedIndices(indexCount);
    meshopt_remapIndexBuffer(weldedIndices.data(), m_indices.data(), indexCount, remap.data());

    bvector<uint32_t> simplified(indexCount);
    float resultError = 0;
    size_t count = meshopt_simplify(simplified.data(), weldedIndices.data(), indexCount, &welded[0].x, weldedCount,
      sizeof(WeldVertex), 0, (float)tolerance, meshopt_SimplifyErrorAbsolute | meshopt_SimplifyPrune, &resultError);

    // Nothing left means every part is within tolerance of nothing, e.g. many thin members.
    if (0 == count) {
      m_points.clear();
      m_normals.clear();
      m_params.clear();
      m_clipFaces.clear();
      m_indices.clear();
      return true;
    }

    // Keep the original if simplification gains little.
    if (count > indexCount * 9 / 10)
      return false;

    simplified.resize(count);

    // The original point (in double precision) and params for each welded vertex.
    bvector<uint32_t> source(weldedCount);
    for (size_t i = 0; i < vertexCount; ++i)
      source[remap[i]] = (uint32_t)i;

    size_t numTriangles = count / 3;
    bvector<DVec3d> faceNormals(numTriangles);
    for (size_t t = 0; t < numTriangles; ++t) {
      DPoint3dCR p0 = m_points[source[simplified[t * 3]]];
      faceNormals[t] = DVec3d::FromCrossProductToPoints(p0, m_points[source[simplified[t * 3 + 1]]], m_points[source[simplified[t * 3 + 2]]]);
    }

    // Group the corners at each welded vertex into smoothing clusters.
    struct Cluster { uint32_t m_welded; DVec3d m_seed; DVec3d m_sum; uint32_t m_output; };
    bvector<Cluster> clusters;
    bvector<bvector<uint32_t>> clustersAt(weldedCount);
    bvector<uint32_t> cornerCluster(count);
    for (size_t c = 0; c < count; ++c) {
      uint32_t w = simplified[c];
      DVec3dCR n = faceNormals[c / 3];
      DVec3d unit = n;
      unit.Normalize();
      uint32_t found = ~0u;
      for (uint32_t index : clustersAt[w]) {
        if (clusters[index].m_seed.DotProduct(unit) >= CREASE_DOT) {
          found = index;
          break;
        }
      }

      if (~0u == found) {
        found = (uint32_t)clusters.size();
        clusters.push_back({w, unit, DVec3d::FromZero(), 0});
        clustersAt[w].push_back(found);
      }

      clusters[found].m_sum.Add(n);
      cornerCluster[c] = found;
    }

    TileMesh rebuilt(*this, clusters.size());
    for (size_t i = 0; i < clusters.size(); ++i) {
      auto& cluster = clusters[i];
      uint32_t from = source[cluster.m_welded];
      DVec3d normal = cluster.m_sum;
      if (normal.Normalize() == 0.0)
        normal = cluster.m_seed;

      rebuilt.m_points[i] = m_points[from];
      rebuilt.m_normals[i] = FPoint3d::From(normal);
      rebuilt.m_clipFaces[i] = m_clipFaces[from];
      if (m_hasParams)
        rebuilt.m_params[i] = m_params[from];
    }

    rebuilt.m_indices.resize(count);
    for (size_t c = 0; c < count; ++c)
      rebuilt.m_indices[c] = cornerCluster[c];

    *this = std::move(rebuilt);
    return true;
  }

  // Clip triangles to the range, interpolating normals and params at the cuts.
  void Clip(DRange3dCR range) {
    struct Vertex { DPoint3d p; FPoint3d n; FPoint2d uv; uint8_t faces; };

    TileMesh clipped(*this, 0);
    bvector<Vertex> poly, next;
    for (size_t t = 0; t + 2 < m_indices.size(); t += 3) {
      poly.clear();
      for (size_t k = 0; k < 3; ++k) {
        uint32_t i = m_indices[t + k];
        poly.push_back({m_points[i], m_normals[i], m_hasParams ? m_params[i] : MakeFPoint2d(0, 0), m_clipFaces[i]});
      }

      for (int face = 0; face < 6 && poly.size() >= 3; ++face) {
        int axis = face / 2;
        bool keepAbove = 0 == face % 2;
        double limit = keepAbove ? range.low.GetComponent(axis) : range.high.GetComponent(axis);
        auto inside = [&](Vertex const& v) { return keepAbove ? v.p.GetComponent(axis) >= limit : v.p.GetComponent(axis) <= limit; };

        next.clear();
        for (size_t k = 0; k < poly.size(); ++k) {
          Vertex const& a = poly[k];
          Vertex const& b = poly[(k + 1) % poly.size()];
          bool aIn = inside(a), bIn = inside(b);
          if (aIn)
            next.push_back(a);

          if (aIn != bIn) {
            double f = (limit - a.p.GetComponent(axis)) / (b.p.GetComponent(axis) - a.p.GetComponent(axis));
            Vertex v;
            v.p = DPoint3d::FromInterpolate(a.p, f, b.p);
            v.p.SetComponent(limit, axis);
            DVec3d n = DVec3d::From(a.n.x + f * (b.n.x - a.n.x), a.n.y + f * (b.n.y - a.n.y), a.n.z + f * (b.n.z - a.n.z));
            n.Normalize();
            v.n = FPoint3d::From(n);
            v.uv = MakeFPoint2d((float)(a.uv.x + f * (b.uv.x - a.uv.x)), (float)(a.uv.y + f * (b.uv.y - a.uv.y)));
            v.faces = (uint8_t)((a.faces & b.faces) | (1 << face));
            next.push_back(v);
          }
        }

        std::swap(poly, next);
      }

      if (poly.size() < 3)
        continue;

      uint32_t base = (uint32_t)clipped.m_points.size();
      for (auto const& v : poly) {
        clipped.m_points.push_back(v.p);
        clipped.m_normals.push_back(v.n);
        if (m_hasParams)
          clipped.m_params.push_back(v.uv);
        clipped.m_clipFaces.push_back(v.faces);
      }

      for (uint32_t k = 1; k + 1 < (uint32_t)poly.size(); ++k)
        clipped.m_indices.insert(clipped.m_indices.end(), {base, base + k, base + k + 1});
    }

    *this = std::move(clipped);
  }

  // Classify each triangle edge for display: creases (normals differ across the edge) and boundaries are
  // visible, smooth curved edges are silhouettes, and coplanar edges and edges on the tile's faces are hidden.
  void ComputeEdges(bvector<ImdlEdgeVisibility>& edges, bvector<uint32_t>& silhouettes) const {
    size_t indexCount = m_indices.size();
    edges.assign(indexCount, ImdlEdgeVisibility::Hidden);
    bvector<uint32_t> pairs(indexCount, 0);

    struct PointKey {
      double x, y, z;
      bool operator==(PointKey const& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct PointHash {
      size_t operator()(PointKey const& k) const {
        std::hash<double> h;
        return h(k.x) ^ (h(k.y) * 31) ^ (h(k.z) * 131);
      }
    };

    std::unordered_map<PointKey, uint32_t, PointHash> welded;
    bvector<uint32_t> weldIds(m_points.size());
    for (size_t i = 0; i < m_points.size(); ++i) {
      auto inserted = welded.insert({{m_points[i].x, m_points[i].y, m_points[i].z}, (uint32_t)welded.size()});
      weldIds[i] = inserted.first->second;
    }

    size_t numTriangles = indexCount / 3;
    bvector<DVec3d> faceNormals(numTriangles);
    for (size_t t = 0; t < numTriangles; ++t) {
      DPoint3dCR p0 = m_points[m_indices[t * 3]];
      faceNormals[t] = DVec3d::FromCrossProductToPoints(p0, m_points[m_indices[t * 3 + 1]], m_points[m_indices[t * 3 + 2]]);
      faceNormals[t].Normalize();
    }

    std::unordered_map<uint64_t, bvector<uint32_t>> edgeMap; // welded edge -> index positions of the edge's start
    for (uint32_t i = 0; i < indexCount; ++i) {
      uint32_t a = weldIds[m_indices[i]];
      uint32_t b = weldIds[m_indices[0 == (i + 1) % 3 ? i - 2 : i + 1]];
      if (a == b)
        continue;

      uint64_t key = a < b ? ((uint64_t)a << 32) | b : ((uint64_t)b << 32) | a;
      edgeMap[key].push_back(i);
    }

    auto endIndex = [&](uint32_t i) { return m_indices[0 == (i + 1) % 3 ? i - 2 : i + 1]; };
    auto normalAt = [&](uint32_t vertex) { return m_normals[vertex]; };
    auto sameNormal = [](FPoint3dCR a, FPoint3dCR b) { return a.x * b.x + a.y * b.y + a.z * b.z >= SMOOTH_NORMAL_DOT; };

    for (auto const& entry : edgeMap) {
      auto const& uses = entry.second;
      uint32_t first = uses[0];
      uint32_t v0 = m_indices[first], v1 = endIndex(first);
      if (1 == uses.size()) {
        bool onTileFace = 0 != (m_clipFaces[v0] & m_clipFaces[v1]);
        edges[first] = onTileFace ? ImdlEdgeVisibility::Hidden : ImdlEdgeVisibility::Visible;
        continue;
      }

      if (uses.size() > 2) {
        edges[first] = ImdlEdgeVisibility::Visible;
        continue;
      }

      uint32_t second = uses[1];
      DVec3dCR n0 = faceNormals[first / 3];
      DVec3dCR n1 = faceNormals[second / 3];
      if (n0.DotProduct(n1) >= COPLANAR_DOT)
        continue;

      // Match each end of the first triangle's edge with the vertex at the same position in the second.
      uint32_t w0 = m_indices[second], w1 = endIndex(second);
      if (weldIds[w0] != weldIds[v0])
        std::swap(w0, w1);

      bool smooth = sameNormal(normalAt(v0), normalAt(w0)) && sameNormal(normalAt(v1), normalAt(w1));
      if (!smooth) {
        edges[first] = ImdlEdgeVisibility::Visible;
      } else {
        edges[first] = ImdlEdgeVisibility::Silhouette;
        pairs[first] = (uint32_t)OctEncodeNormal(n0.x, n0.y, n0.z) | ((uint32_t)OctEncodeNormal(n1.x, n1.y, n1.z) << 16);
      }
    }

    silhouettes.clear();
    for (size_t i = 0; i < indexCount; ++i)
      if (ImdlEdgeVisibility::Silhouette == edges[i])
        silhouettes.push_back(pairs[i]);
  }

private:
  TileMesh(TileMesh const& source, size_t vertexCount) : m_hasParams(source.m_hasParams) {
    m_points.resize(vertexCount);
    m_normals.resize(vertexCount);
    m_clipFaces.assign(vertexCount, 0);
    if (m_hasParams)
      m_params.resize(vertexCount);
  }

};

void addMesh(ImdlTile& tile, DgnDbR db, ElementMesh const& elementMesh, DgnElementId elementId, TreeInfo const& tree, DRange3dCR tileRange, bool clip, double tolerance) {
  if (0 == elementMesh.GetPointCount() || elementMesh.m_indices.empty())
    return;

  TileMesh mesh(elementMesh, tree.m_center);
  // Simplified tiles must be refined rather than magnified when zoomed in.
  if (mesh.Simplify(tolerance))
    tile.m_flags |= (uint32_t)ImdlFlags::DisallowMagnification;

  if (clip)
    mesh.Clip(tileRange);

  if (mesh.m_indices.empty())
    return;

  bvector<ImdlEdgeVisibility> edges;
  bvector<uint32_t> silhouettes;
  if (tree.m_wantEdges)
    mesh.ComputeEdges(edges, silhouettes);

  DgnTextureId textureId = mesh.m_hasParams ? elementMesh.m_textureId : DgnTextureId();
  if (textureId.IsValid())
    tile.AddTexture(db, textureId);

  uint32_t featureIndex = tile.GetFeatureIndex(elementId, elementMesh.m_subCategoryId, elementMesh.m_geometryClass);
  ImdlMesh& out = tile.GetMesh(elementMesh.m_color, textureId, elementMesh.m_fillFlags, mesh.m_points.size());
  uint32_t baseIndex = (uint32_t)out.m_points.size();
  for (size_t i = 0; i < mesh.m_points.size(); ++i) {
    out.m_points.push_back(mesh.m_points[i]);
    out.m_range.Extend(mesh.m_points[i]);
    out.m_normals.push_back(OctEncodeNormal(mesh.m_normals[i].x, mesh.m_normals[i].y, mesh.m_normals[i].z));
    out.m_featureIndices.push_back(featureIndex);
    if (textureId.IsValid())
      out.m_params.push_back(mesh.m_params[i]);
  }

  for (uint32_t index : mesh.m_indices)
    out.m_indices.push_back(baseIndex + index);

  if (tree.m_wantEdges) {
    out.m_edges.insert(out.m_edges.end(), edges.begin(), edges.end());
    out.m_silhouetteNormals.insert(out.m_silhouetteNormals.end(), silhouettes.begin(), silhouettes.end());
  }

  tile.m_contentRange.Extend(out.m_range);
}

// Clip the segment to the range (Liang-Barsky). Returns false if nothing remains.
bool clipSegment(DPoint3dR p0, DPoint3dR p1, DRange3dCR range) {
  double t0 = 0, t1 = 1;
  DVec3d d = DVec3d::FromStartEnd(p0, p1);
  for (int axis = 0; axis < 3; ++axis) {
    double start = p0.GetComponent(axis), delta = d.GetComponent(axis);
    double low = range.low.GetComponent(axis), high = range.high.GetComponent(axis);
    if (0.0 == delta) {
      if (start < low || start > high)
        return false;

      continue;
    }

    double a = (low - start) / delta, b = (high - start) / delta;
    if (a > b)
      std::swap(a, b);

    t0 = std::max(t0, a);
    t1 = std::min(t1, b);
    if (t0 > t1)
      return false;
  }

  DPoint3d start = p0;
  p0 = DPoint3d::FromInterpolate(start, t0, p1);
  p1 = DPoint3d::FromInterpolate(start, t1, p1);
  return true;
}

// Douglas-Peucker: the subset of the line's points within tolerance of the original.
bvector<uint32_t> simplifyLine(bvector<DPoint3d> const& points, bvector<uint32_t> const& line, double tolerance) {
  if (line.size() < 3)
    return line;

  bvector<bool> keep(line.size(), false);
  keep.front() = keep.back() = true;
  bvector<std::pair<size_t, size_t>> stack{{0, line.size() - 1}};
  while (!stack.empty()) {
    auto [first, last] = stack.back();
    stack.pop_back();
    if (last <= first + 1)
      continue;

    DSegment3d segment = DSegment3d::From(points[line[first]], points[line[last]]);
    double maxDistance = -1;
    size_t farthest = first;
    for (size_t i = first + 1; i < last; ++i) {
      DPoint3d closest;
      double fraction;
      segment.ProjectPointBounded(closest, fraction, points[line[i]]);
      double distance = closest.Distance(points[line[i]]);
      if (distance > maxDistance) {
        maxDistance = distance;
        farthest = i;
      }
    }

    if (maxDistance > tolerance) {
      keep[farthest] = true;
      stack.push_back({first, farthest});
      stack.push_back({farthest, last});
    }
  }

  bvector<uint32_t> result;
  for (size_t i = 0; i < line.size(); ++i)
    if (keep[i])
      result.push_back(line[i]);

  return result;
}

void addPolyline(ImdlTile& tile, ElementPolyline const& elementPolyline, DgnElementId elementId, TreeInfo const& tree, DRange3dCR tileRange, bool clip, double tolerance) {
  bvector<DPoint3d> points;
  bvector<bvector<uint32_t>> lines;
  for (auto const& line : elementPolyline.m_lines) {
    bvector<uint32_t> current;
    auto flush = [&]() {
      if (current.size() >= 2)
        lines.push_back(std::move(current));

      current.clear();
    };

    for (size_t i = 0; i + 1 < line.size(); ++i) {
      DPoint3d p0 = elementPolyline.m_points[line[i]];
      DPoint3d p1 = elementPolyline.m_points[line[i + 1]];
      p0.Subtract(tree.m_center);
      p1.Subtract(tree.m_center);
      DPoint3d c0 = p0, c1 = p1;
      if (clip && !clipSegment(c0, c1, tileRange)) {
        flush();
        continue;
      }

      bool continues = !current.empty() && c0.IsEqual(p0) && points[current.back()].IsEqual(c0);
      if (!continues) {
        flush();
        current.push_back((uint32_t)points.size());
        points.push_back(c0);
      }

      current.push_back((uint32_t)points.size());
      points.push_back(c1);
      if (!c1.IsEqual(p1))
        flush();
    }

    // Keep closed loops closed so the tesselator joins them.
    if (!clip && line.size() > 2 && line.front() == line.back() && current.size() == line.size())
      current.back() = current.front();

    flush();
  }

  bvector<uint32_t> remap(points.size(), ~0u);
  bvector<DPoint3d> kept;
  for (auto& line : lines) {
    line = simplifyLine(points, line, tolerance);
    for (auto& index : line) {
      if (~0u == remap[index]) {
        remap[index] = (uint32_t)kept.size();
        kept.push_back(points[index]);
      }
      index = remap[index];
    }
  }

  points = std::move(kept);
  if (lines.empty())
    return;

  uint32_t featureIndex = tile.GetFeatureIndex(elementId, elementPolyline.m_subCategoryId, elementPolyline.m_geometryClass);
  ImdlPolyline& out = tile.GetPolyline(elementPolyline.m_color, elementPolyline.m_width, elementPolyline.m_linePixels, points.size());
  uint32_t baseIndex = (uint32_t)out.m_points.size();
  for (auto const& pt : points) {
    out.m_points.push_back(pt);
    out.m_range.Extend(pt);
    out.m_featureIndices.push_back(featureIndex);
  }

  for (auto& line : lines) {
    for (auto& index : line)
      index += baseIndex;

    out.m_lines.push_back(std::move(line));
  }

  tile.m_contentRange.Extend(out.m_range);
  tile.m_flags |= (uint32_t)ImdlFlags::ContainsCurves;
}

struct GeometrySource3dData : GeometrySource3d {
  DgnDbR m_db;
  DgnCategoryId m_categoryId;
  Placement3d m_placement;
  GeometryStream m_geom;

  explicit GeometrySource3dData(DgnDbR db) : m_db(db) { }
  DgnDbR _GetSourceDgnDb() const override { return m_db; }
  DgnElementCP _ToElement() const override { return nullptr; }
  GeometrySource3dCP _GetAsGeometrySource3d() const override { return this; }
  DgnCategoryId _GetCategoryId() const override { return m_categoryId; }
  GeometryStreamCR _GetGeometryStream() const override { return m_geom; }
  Placement3dCR _GetPlacement() const override { return m_placement; }
  DgnDbStatus _SetCategoryId(DgnCategoryId categoryId) override { m_categoryId = categoryId; return DgnDbStatus::Success; }
  DgnDbStatus _SetPlacement(Placement3dCR placement) override { m_placement = placement; return DgnDbStatus::Success; }
};

struct GeometrySource2dData : GeometrySource2d {
  DgnDbR m_db;
  DgnCategoryId m_categoryId;
  Placement2d m_placement;
  GeometryStream m_geom;

  explicit GeometrySource2dData(DgnDbR db) : m_db(db) { }
  DgnDbR _GetSourceDgnDb() const override { return m_db; }
  DgnElementCP _ToElement() const override { return nullptr; }
  GeometrySource2dCP _GetAsGeometrySource2d() const override { return this; }
  DgnCategoryId _GetCategoryId() const override { return m_categoryId; }
  GeometryStreamCR _GetGeometryStream() const override { return m_geom; }
  Placement2dCR _GetPlacement() const override { return m_placement; }
  DgnDbStatus _SetCategoryId(DgnCategoryId categoryId) override { m_categoryId = categoryId; return DgnDbStatus::Success; }
  DgnDbStatus _SetPlacement(Placement2dCR placement) override { m_placement = placement; return DgnDbStatus::Success; }
};

// The elements of a tile's model whose ranges intersect it, in world coordinates.
struct ElementCandidate {
  DgnElementId m_id;
  DRange3d m_range;
};

void findElements3d(bvector<ElementCandidate>& out, DgnDbR db, DgnModelId modelId, DRange3dCR range) {
  Statement stmt(db, "SELECT s.ElementId,s.MinX,s.MinY,s.MinZ,s.MaxX,s.MaxY,s.MaxZ FROM " DGN_VTABLE_SpatialIndex " s, " BIS_TABLE(BIS_CLASS_Element) " e"
    " WHERE s.MinX<=? AND s.MaxX>=? AND s.MinY<=? AND s.MaxY>=? AND s.MinZ<=? AND s.MaxZ>=? AND e.Id=s.ElementId AND e.ModelId=?");
  if (!stmt.IsPrepared())
    return;

  stmt.BindDouble(1, range.high.x);
  stmt.BindDouble(2, range.low.x);
  stmt.BindDouble(3, range.high.y);
  stmt.BindDouble(4, range.low.y);
  stmt.BindDouble(5, range.high.z);
  stmt.BindDouble(6, range.low.z);
  stmt.BindId(7, modelId);
  while (BE_SQLITE_ROW == stmt.Step()) {
    out.push_back({stmt.GetValueId<DgnElementId>(0), DRange3d::From(stmt.GetValueDouble(1), stmt.GetValueDouble(2), stmt.GetValueDouble(3),
      stmt.GetValueDouble(4), stmt.GetValueDouble(5), stmt.GetValueDouble(6))});
  }
}

Placement2d placement2dFrom(Statement& stmt, int col) {
  return Placement2d(DPoint2d::From(stmt.GetValueDouble(col), stmt.GetValueDouble(col + 1)), AngleInDegrees::FromDegrees(stmt.GetValueDouble(col + 2)),
    ElementAlignedBox2d(stmt.GetValueDouble(col + 3), stmt.GetValueDouble(col + 4), stmt.GetValueDouble(col + 5), stmt.GetValueDouble(col + 6)));
}

void findElements2d(bvector<ElementCandidate>& out, DgnDbR db, DgnModelId modelId, DRange3dCR range) {
  Statement stmt(db, "SELECT g.ElementId,g.Origin_X,g.Origin_Y,g.Rotation,g.BBoxLow_X,g.BBoxLow_Y,g.BBoxHigh_X,g.BBoxHigh_Y FROM "
    BIS_TABLE(BIS_CLASS_GeometricElement2d) " g, " BIS_TABLE(BIS_CLASS_Element) " e WHERE e.Id=g.ElementId AND e.ModelId=? AND g.GeometryStream IS NOT NULL");
  if (!stmt.IsPrepared())
    return;

  stmt.BindId(1, modelId);
  while (BE_SQLITE_ROW == stmt.Step()) {
    DRange3d elementRange = placement2dFrom(stmt, 1).CalculateRange();
    if (elementRange.low.x <= range.high.x && elementRange.high.x >= range.low.x && elementRange.low.y <= range.high.y && elementRange.high.y >= range.low.y)
      out.push_back({stmt.GetValueId<DgnElementId>(0), elementRange});
  }
}

TileContentCPtr generateTile(DgnDbR db, TreeInfo const& tree, ContentId const& contentId) {
  ImdlTile tile;
  tile.m_is2d = tree.m_is2d;
  tile.m_wantEdges = tree.m_wantEdges;
  if (tree.m_isEmpty)
    return new TileContent(tile.Write());

  DRange3d tileRange = tree.ComputeTileRange(contentId);
  double tolerance = tree.ComputeTolerance(tileRange, contentId.m_multiplier);
  tile.m_tolerance = tolerance;
  DRange3d worldRange = translated(tileRange, tree.m_center);

  bvector<ElementCandidate> candidates;
  if (tree.m_is2d)
    findElements2d(candidates, db, tree.m_modelId, worldRange);
  else
    findElements3d(candidates, db, tree.m_modelId, worldRange);

  Statement geometry3d(db, "SELECT CategoryId,GeometryStream,Yaw,Pitch,Roll,Origin_X,Origin_Y,Origin_Z,"
    "BBoxLow_X,BBoxLow_Y,BBoxLow_Z,BBoxHigh_X,BBoxHigh_Y,BBoxHigh_Z FROM " BIS_TABLE(BIS_CLASS_GeometricElement3d) " WHERE ElementId=?");
  Statement geometry2d(db, "SELECT CategoryId,GeometryStream,Origin_X,Origin_Y,Rotation,BBoxLow_X,BBoxLow_Y,BBoxHigh_X,BBoxHigh_Y FROM "
    BIS_TABLE(BIS_CLASS_GeometricElement2d) " WHERE ElementId=?");
  Statement& geometry = tree.m_is2d ? geometry2d : geometry3d;
  if (!geometry.IsPrepared())
    return nullptr;

  // Largest first, so a tile that runs out of room leaves only the smaller elements to its children.
  bvector<std::pair<double, size_t>> order;
  order.reserve(candidates.size());
  for (size_t i = 0; i < candidates.size(); ++i)
    order.push_back({tree.Diagonal(candidates[i].m_range), i});

  std::sort(order.begin(), order.end(), [](auto const& a, auto const& b) { return a.first > b.first; });

  double minElementSize = tolerance * MIN_ELEMENT_SIZE_IN_TOLERANCES;
  size_t vertexCount = 0;
  for (auto const& entry : order) {
    ElementCandidate const& candidate = candidates[entry.second];
    if (entry.first < minElementSize) {
      ++tile.m_numElementsExcluded;
      continue;
    }

    if (vertexCount >= MAX_TILE_VERTICES) {
      ++tile.m_numElementsExcluded;
      tile.m_flags |= (uint32_t)ImdlFlags::Incomplete;
      continue;
    }

    geometry.Reset();
    geometry.BindId(1, candidate.m_id);
    if (BE_SQLITE_ROW != geometry.Step())
      continue;

    GeometryStream geom;
    if (DgnDbStatus::Success != db.Elements().LoadGeometryStream(geom, geometry.GetValueBlob(1), geometry.GetColumnBytes(1)))
      continue;

    ElementGraphics graphics;
    bool meshed;
    if (tree.m_is2d) {
      GeometrySource2dData source(db);
      source.m_categoryId = geometry.GetValueId<DgnCategoryId>(0);
      source.m_placement = placement2dFrom(geometry, 2);
      source.m_geom = std::move(geom);
      meshed = ElementMesher::Mesh(graphics, source, tolerance);
    } else {
      GeometrySource3dData source(db);
      source.m_categoryId = geometry.GetValueId<DgnCategoryId>(0);
      YawPitchRollAngles angles(Angle::FromDegrees(geometry.GetValueDouble(2)), Angle::FromDegrees(geometry.GetValueDouble(3)), Angle::FromDegrees(geometry.GetValueDouble(4)));
      DPoint3d origin = DPoint3d::From(geometry.GetValueDouble(5), geometry.GetValueDouble(6), geometry.GetValueDouble(7));
      ElementAlignedBox3d box(geometry.GetValueDouble(8), geometry.GetValueDouble(9), geometry.GetValueDouble(10),
        geometry.GetValueDouble(11), geometry.GetValueDouble(12), geometry.GetValueDouble(13));
      source.m_placement = Placement3d(origin, angles, box);
      source.m_geom = std::move(geom);
      meshed = ElementMesher::Mesh(graphics, source, tolerance);
    }

    if (!meshed)
      continue;

    ++tile.m_numElementsIncluded;
    bool clip = !candidate.m_range.IsContained(worldRange);
    for (auto const& mesh : graphics.m_meshes)
      addMesh(tile, db, mesh, candidate.m_id, tree, tileRange, clip, tolerance);

    for (auto const& polyline : graphics.m_polylines)
      addPolyline(tile, polyline, candidate.m_id, tree, tileRange, clip, tolerance);

    vertexCount = 0;
    for (auto const& mesh : tile.m_meshes)
      vertexCount += mesh.m_points.size();
    for (auto const& polyline : tile.m_polylines)
      vertexCount += polyline.m_points.size();
  }

  return new TileContent(tile.Write());
}

// Moves graphics from world coordinates into the coordinates of location.
void transformGraphics(ElementGraphics& graphics, TransformCR worldToLocal) {
  RotMatrix normalMatrix = RotMatrix::From(worldToLocal);
  normalMatrix.Invert();
  normalMatrix.Transpose();
  for (auto& mesh : graphics.m_meshes) {
    for (size_t i = 0; i < mesh.GetPointCount(); ++i) {
      DPoint3d pt = DPoint3d::From(mesh.m_points[i * 3], mesh.m_points[i * 3 + 1], mesh.m_points[i * 3 + 2]);
      worldToLocal.Multiply(pt);
      mesh.m_points[i * 3] = pt.x;
      mesh.m_points[i * 3 + 1] = pt.y;
      mesh.m_points[i * 3 + 2] = pt.z;
      DVec3d n = DVec3d::From(mesh.m_normals[i * 3], mesh.m_normals[i * 3 + 1], mesh.m_normals[i * 3 + 2]);
      normalMatrix.Multiply(n);
      n.Normalize();
      mesh.m_normals[i * 3] = (float)n.x;
      mesh.m_normals[i * 3 + 1] = (float)n.y;
      mesh.m_normals[i * 3 + 2] = (float)n.z;
    }
  }

  for (auto& polyline : graphics.m_polylines)
    for (auto& pt : polyline.m_points)
      worldToLocal.Multiply(pt);
}

// Graphics for one persistent element, or for geometry supplied by the frontend (GraphicsRequestProps in core-common).
ElementGraphicsStatus generateElementGraphics(ByteStream& out, DgnDbR db, BeJsConst props) {
  if ((props["formatVersion"].asUInt(IMDL_FORMAT_VERSION >> 16)) > (IMDL_FORMAT_VERSION >> 16))
    return ElementGraphicsStatus::UnknownMajorFormatVersion;

  double tolerance = std::pow(10.0, props["toleranceLog10"].asDouble(-2));
  ElementGraphics graphics;
  bool is2d = false;
  bool meshed = false;
  DgnElementId elementId;
  if (props["elementId"].isString())
    elementId = DgnElementId(BeInt64Id::FromString(props["elementId"].asCString()).GetValueUnchecked());

  if (props.isMember("geometry")) {
    BeJsConst geometry = props["geometry"];
    BeJsConst data = geometry["data"];
    bool isFlatBuffer = 0 == strcmp("flatbuffer", geometry["format"].asCString(""));
    DgnCategoryId categoryId(BeInt64Id::FromString(props["categoryId"].asCString("0")).GetValueUnchecked());
    is2d = 0 == strcmp("2d", props["type"].asCString("3d"));
    auto build = [&](GeometrySourceR source) {
      return isFlatBuffer ? GeometryBuilder::UpdateFromJsonEntryArray(source, data) : GeometryBuilder::UpdateFromJson(source, data);
    };
    if (is2d) {
      GeometrySource2dData source(db);
      source.m_categoryId = categoryId;
      DPoint2d origin;
      BeJsGeomUtils::DPoint2dFromJson(origin, props["placement"]["origin"]);
      source.m_placement = Placement2d(origin, BeJsGeomUtils::AngleInDegreesFromJson(props["placement"]["angle"]));
      if (!build(source))
        return ElementGraphicsStatus::InvalidJson;
      meshed = ElementMesher::Mesh(graphics, source, tolerance);
    } else {
      GeometrySource3dData source(db);
      source.m_categoryId = categoryId;
      DPoint3d origin;
      BeJsGeomUtils::DPoint3dFromJson(origin, props["placement"]["origin"]);
      source.m_placement = Placement3d(origin, BeJsGeomUtils::YawPitchRollFromJson(props["placement"]["angles"]));
      if (!build(source))
        return ElementGraphicsStatus::InvalidJson;
      meshed = ElementMesher::Mesh(graphics, source, tolerance);
    }
  } else {
    DgnElementCPtr element = db.Elements().GetElement(elementId);
    GeometrySourceCP source = element.IsValid() ? element->ToGeometrySource() : nullptr;
    if (nullptr == source)
      return ElementGraphicsStatus::ElementNotFound;

    is2d = !source->Is3d();
    meshed = ElementMesher::Mesh(graphics, *source, tolerance);
  }

  if (!meshed || (graphics.m_meshes.empty() && graphics.m_polylines.empty()))
    return ElementGraphicsStatus::NoGeometry;

  if (props.isMember("location")) {
    Transform location;
    BeJsGeomUtils::TransformFromJson(location, props["location"]);
    Transform worldToLocal;
    worldToLocal.InverseOf(location);
    transformGraphics(graphics, worldToLocal);
  }

  TreeInfo tree;
  tree.m_is2d = is2d;
  tree.m_wantEdges = !props["omitEdges"].asBool(false);
  ImdlTile tile;
  tile.m_is2d = is2d;
  tile.m_wantEdges = tree.m_wantEdges;
  tile.m_tolerance = tolerance;
  tile.m_numElementsIncluded = 1;
  DRange3d unused = DRange3d::NullRange();
  for (auto const& mesh : graphics.m_meshes)
    addMesh(tile, db, mesh, elementId, tree, unused, false, tolerance);

  for (auto const& polyline : graphics.m_polylines)
    addPolyline(tile, polyline, elementId, tree, unused, false, tolerance);

  out = tile.Write();
  return ElementGraphicsStatus::Success;
}

struct TileElementGraphicsRequests : ElementGraphicsRequests {
  DgnDbR m_db;

  explicit TileElementGraphicsRequests(DgnDbR db) : m_db(db) { }

  void Terminate() override { }
  void Cancel(bvector<Utf8String> const&) override { }

  Napi::Value Enqueue(BeJsConst props, Napi::Env env) override {
    auto deferred = Napi::Promise::Deferred::New(env);
    auto result = Napi::Object::New(env);
    ByteStream content;
    ElementGraphicsStatus status;
    try {
      status = generateElementGraphics(content, m_db, props);
    } catch (...) {
      status = ElementGraphicsStatus::NoGeometry;
    }

    result.Set("status", Napi::Number::New(env, (int)status));
    if (ElementGraphicsStatus::Success == status) {
      auto bytes = Napi::Uint8Array::New(env, content.GetSize());
      memcpy(bytes.Data(), content.GetData(), content.GetSize());
      result.Set("content", bytes);
    }

    deferred.Resolve(result);
    return deferred.Promise();
  }
};

struct ContentRequest {
  DgnDbPtr m_db;
  Utf8String m_treeId;
  Utf8String m_contentId;
  Visualization::PollState m_state = Visualization::PollState::Pending;
  bool m_done = false;
  TileContentCPtr m_content;
  DgnDbStatus m_status = DgnDbStatus::Success;
  std::chrono::steady_clock::time_point m_start = std::chrono::steady_clock::now();
  double m_elapsedSeconds = 0;
};

using ContentRequestPtr = std::shared_ptr<ContentRequest>;

struct TileVisualization : Visualization {
private:
  std::mutex m_mutex;
  std::condition_variable m_cv;
  std::deque<ContentRequestPtr> m_queue;
  std::map<Utf8String, ContentRequestPtr> m_requests;
  std::map<Utf8String, TreeInfo> m_trees;
  bvector<std::thread> m_workers;
  bool m_stop = false;

  static Utf8String MakeKey(DgnDbR db, Utf8StringCR treeId, Utf8StringCR contentId = "") {
    return Utf8PrintfString("%p|%s|%s", (void*)&db, treeId.c_str(), contentId.c_str());
  }

  DgnDbStatus GetTreeInfo(TreeInfo& info, DgnDbR db, Utf8StringCR treeId) {
    Utf8String key = MakeKey(db, treeId);
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      auto found = m_trees.find(key);
      if (m_trees.end() != found) {
        info = found->second;
        return DgnDbStatus::Success;
      }
    }

    DgnDbStatus status = TreeInfo::Compute(info, db, treeId);
    if (DgnDbStatus::Success == status) {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_trees[key] = info;
    }

    return status;
  }

  Visualization::TileContentResult Generate(DgnDbR db, Utf8StringCR treeId, Utf8StringCR contentIdStr) {
    TreeInfo tree;
    DgnDbStatus status = GetTreeInfo(tree, db, treeId);
    if (DgnDbStatus::Success != status)
      return status;

    ContentId contentId;
    if (!ContentId::Parse(contentId, contentIdStr))
      return DgnDbStatus::InvalidId;

    TileContentCPtr content;
    try {
      content = generateTile(db, tree, contentId);
    } catch (...) {
    }

    if (content.IsNull())
      return DgnDbStatus::BadRequest;

    return content;
  }

  void Work() {
    while (true) {
      ContentRequestPtr request;
      {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cv.wait(lock, [&] { return m_stop || !m_queue.empty(); });
        if (m_stop)
          return;

        request = m_queue.front();
        m_queue.pop_front();
        request->m_state = PollState::Loading;
      }

      TileContentCPtr content;
      DgnDbStatus status = DgnDbStatus::NotOpen;
      if (request->m_db->IsDbOpen()) {
        auto result = Generate(*request->m_db, request->m_treeId, request->m_contentId);
#ifdef BENTLEY_HAVE_VARIANT
        if (auto ptr = std::get_if<TileContentCPtr>(&result))
          content = *ptr;
        else
          status = std::get<DgnDbStatus>(result);
#else
        content = result.m_content;
        status = result.m_status;
#endif
      }

      std::lock_guard<std::mutex> lock(m_mutex);
      request->m_content = content;
      request->m_status = content.IsValid() ? DgnDbStatus::Success : status;
      request->m_elapsedSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - request->m_start).count();
      request->m_done = true;
      request->m_db = nullptr;
    }
  }

public:
  explicit TileVisualization(Render::System* renderSystem) : Visualization(renderSystem) { }

  ~TileVisualization() {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_stop = true;
    }
    m_cv.notify_all();
    for (auto& worker : m_workers)
      worker.join();
  }

  uint32_t GetCurrentFormatVersion() override { return IMDL_FORMAT_VERSION; }

  DgnDbStatus GetTileTreeJson(BeJsValue json, DgnDbR db, Utf8StringCR treeId, bool) override {
    TreeInfo tree;
    DgnDbStatus status = GetTreeInfo(tree, db, treeId);
    if (DgnDbStatus::Success != status)
      return status;

    json["id"] = treeId.c_str();
    json["maxTilesToSkip"] = 1;
    json["maxInitialTilesToSkip"] = tree.m_isEmpty ? 0 : 3;
    json["formatVersion"] = IMDL_FORMAT_VERSION;
    json["tileScreenSize"] = tree.m_tileScreenSize;

    auto location = json["location"];
    location.SetEmptyArray();
    double translation[3] = {tree.m_center.x, tree.m_center.y, tree.m_center.z};
    for (int row = 0; row < 3; ++row) {
      auto rowJson = location.appendArray();
      for (int col = 0; col < 3; ++col)
        rowJson.appendValue() = row == col ? 1.0 : 0.0;

      rowJson.appendValue() = translation[row];
    }

    if (!tree.m_isEmpty)
      writeRange(json["contentRange"], tree.m_contentRange);

    auto root = json["rootTile"];
    root["contentId"] = "0/0/0/0/1";
    root["isLeaf"] = tree.m_isEmpty;
    root["maximumSize"] = tree.m_isEmpty ? 0u : tree.m_tileScreenSize;
    writeRange(root["range"], tree.m_range);
    if (!tree.m_isEmpty)
      writeRange(root["contentRange"], tree.m_contentRange);

    return DgnDbStatus::Success;
  }

  void PurgeTileTrees(DgnDbR db, bvector<DgnModelId> const*) override {
    std::lock_guard<std::mutex> lock(m_mutex);
    Utf8String prefix = Utf8PrintfString("%p|", (void*)&db);
    for (auto it = m_trees.begin(); it != m_trees.end(); )
      it = 0 == it->first.compare(0, prefix.size(), prefix) ? m_trees.erase(it) : std::next(it);
  }

  TileContentResult GetTileContent(DgnDbR db, Utf8StringCR treeId, Utf8StringCR contentId) override {
    return Generate(db, treeId, contentId);
  }

  PollContentResult PollTileContent(ICancellableP, DgnDbR db, Utf8StringCR treeId, Utf8StringCR contentId) override {
    Utf8String key = MakeKey(db, treeId, contentId);
    std::lock_guard<std::mutex> lock(m_mutex);
    auto found = m_requests.find(key);
    if (m_requests.end() == found) {
      auto request = std::make_shared<ContentRequest>();
      request->m_db = &db;
      request->m_treeId = treeId;
      request->m_contentId = contentId;
      m_requests[key] = request;
      m_queue.push_back(request);
      if (m_workers.empty()) {
        unsigned count = std::max(1u, std::min(4u, std::thread::hardware_concurrency() / 2));
        for (unsigned i = 0; i < count; ++i)
          m_workers.emplace_back([this] { Work(); });
      }

      m_cv.notify_one();
      return PollState::New;
    }

    ContentRequestPtr request = found->second;
    if (!request->m_done)
      return request->m_state;

    m_requests.erase(found);
    if (request->m_content.IsNull())
      return request->m_status;

    return PolledTileContent(*request->m_content, request->m_elapsedSeconds);
  }

  void CancelContentRequests(DgnDbR db, Utf8StringCR treeId, bvector<Utf8String> const& contentIds) override {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto const& contentId : contentIds) {
      auto found = m_requests.find(MakeKey(db, treeId, contentId));
      if (m_requests.end() == found || PollState::Pending != found->second->m_state)
        continue;

      auto request = found->second;
      m_queue.erase(std::remove(m_queue.begin(), m_queue.end(), request), m_queue.end());
      m_requests.erase(found);
    }
  }

  ElementGraphicsRequestsUPtr CreateElementGraphicsRequests(DgnDbR db) override {
    return std::make_unique<TileElementGraphicsRequests>(db);
  }
};

} // namespace

Visualization* iTwinVisualization_create(Render::System* renderSystem) {
  return new TileVisualization(renderSystem);
}
