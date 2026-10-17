/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
#include "SmKernel.h"

#include <SmApiBrep.h>
#include <SmApiGeneral.h>
#include <SmApiPrimitives.h>
#include <SmApiQueries.h>
#include <SmApiTrimmedSurfaces.h>
#include <SmPrimitiveCreation.h>
#include <SmBrep.h>
#include <SmBrepData.h>
#include <SmCurve.h>
#include <SmDatabaseIO.h>
#include <SmEdge.h>
#include <SmFace.h>
#include <SmPoly.h>
#include <SmSurface.h>

#include <cmath>
#include <cstring>
#include <map>
#include <sstream>
#include <unordered_map>

namespace SmKernel {

namespace {

// Marks SMLib data so it is never confused with the Parasolid data in existing iModels.
constexpr char kMagic[4] = {'S', 'M', 'L', 'B'};
constexpr uint32_t kFormatVersion = 1;

void PutU32(std::vector<uint8_t>& out, uint32_t value) {
    uint8_t bytes[4];
    std::memcpy(bytes, &value, 4);
    out.insert(out.end(), bytes, bytes + 4);
}

bool GetU32(uint8_t const*& data, uint8_t const* end, uint32_t& value) {
    if (end - data < 4)
        return false;
    std::memcpy(&value, data, 4);
    data += 4;
    return true;
}

SmContext& Context() { return *SmApiGetOrCreateContext(); }

//! Surface parameters of a point on a face, (0,0) if they can't be found.
SmPoint2d SurfaceParams(SmFace* face, SmPoint3d const& point) {
    SmSurface* surface = face ? face->GetSurface() : nullptr;
    if (nullptr == surface)
        return SmPoint2d(0, 0);
    SmBoolean success = FALSE, multivalued = FALSE;
    SmPoint2d uv(0, 0);
    double gap = 0.0;
    if (SM_SUCCESS != surface->DropPoint(point, surface->GetNaturalUVDomain(), nullptr, success, uv, gap, multivalued) || !success)
        return SmPoint2d(0, 0);
    return uv;
}

void GetFaces(SmBrep const& brep, SmTArray<SmFace*>& faces) {
    const_cast<SmBrep&>(brep).GetFaces(faces);
}

} // namespace

struct Body {
    SmBrep* m_brep;
    explicit Body(SmBrep* brep) : m_brep(brep) { }
    ~Body() { delete m_brep; }
};

std::recursive_mutex& Mutex() {
    static std::recursive_mutex s_mutex;
    return s_mutex;
}

void Delete(Body* body) { delete body; }

static BodyPtr Wrap(SmBrep* brep) { return brep ? BodyPtr(new Body(brep)) : BodyPtr(); }

/*---------------------------------------------------------------------------------**//**
* The payload is the magic, a format version, the face tags (count then values), then SMLib's
* binary brep database stream with the same 'V'/'H' header SmBrepData::WriteToFile writes.
+---------------+---------------+---------------+---------------+---------------+------*/
bool Save(Body const& body, std::vector<uint32_t> const& faceTags, std::vector<uint8_t>& out) {
    try {
        SmBrepData* data = new (Context()) SmBrepData(FALSE, SM_DS_SMLIB);
        SmObjDelete cleanup(data);
        SmTArray<SmAttribute*> attributes;
        if (SM_SUCCESS != data->FromBrep(*body.m_brep, attributes))
            return false;

        std::ostringstream stream(std::ios::binary);
        SmDatabaseIO db(SM_BINARY, &stream);
        SmMapTypeToType<SmAttribute*, ULONG> attributeMap;
        if (SM_SUCCESS != db.BeginWriting() ||
            SM_SUCCESS != db.WriteChar('V') || SM_SUCCESS != db.WriteLong(SM_CURRENT_DATABASE_VERSION) ||
            SM_SUCCESS != db.WriteChar('H') || SM_SUCCESS != db.WriteLong(SM_HEALER_VERSION) ||
            SM_SUCCESS != data->WriteToDB(Context(), SM_CURRENT_DATABASE_VERSION, attributes, attributeMap, db, FALSE, 0.0, FALSE) ||
            SM_SUCCESS != db.EndWriting())
            return false;

        std::string const bytes = stream.str();
        out.clear();
        out.insert(out.end(), kMagic, kMagic + 4);
        PutU32(out, kFormatVersion);
        PutU32(out, static_cast<uint32_t>(faceTags.size()));
        for (uint32_t tag : faceTags)
            PutU32(out, tag);
        out.insert(out.end(), bytes.begin(), bytes.end());
        return true;
    } catch (...) {
        return false;
    }
}

BodyPtr Restore(uint8_t const* data, size_t size, std::vector<uint32_t>& faceTags) {
    if (nullptr == data || size < 8 || 0 != std::memcmp(data, kMagic, 4))
        return nullptr;
    uint8_t const* cursor = data + 4;
    uint8_t const* end = data + size;
    uint32_t version;
    uint32_t tagCount;
    if (!GetU32(cursor, end, version) || version != kFormatVersion || !GetU32(cursor, end, tagCount) || tagCount > static_cast<size_t>(end - cursor) / 4)
        return nullptr;
    faceTags.resize(tagCount);
    for (uint32_t& tag : faceTags)
        GetU32(cursor, end, tag);

    try {
        std::istringstream stream(std::string(reinterpret_cast<char const*>(cursor), end - cursor), std::ios::binary);
        SmDatabaseIO db(SM_BINARY, &stream);
        if (SM_SUCCESS != db.BeginReading())
            return nullptr;

        char header;
        long dbVersion = 0, healerVersion = 0;
        if (SM_SUCCESS != db.ReadChar(header) || 'V' != header || SM_SUCCESS != db.ReadLong(dbVersion) ||
            SM_SUCCESS != db.ReadChar(header) || 'H' != header || SM_SUCCESS != db.ReadLong(healerVersion))
            return nullptr;
        db.SetVersionNum(dbVersion);

        SmTArray<SmAttribute*> attributes;
        SmBrepData* brepData = nullptr;
        if (SM_SUCCESS != SmBrepData::ReadFromDB(Context(), dbVersion, attributes, brepData, db, FALSE) || nullptr == brepData)
            return nullptr;
        SmObjDelete dataCleanup(brepData);
        brepData->SetHealerVersion(healerVersion);

        SmBrep* brep = new (Context()) SmBrep();
        SmObjDelete brepCleanup(brep);
        if (SM_SUCCESS != brep->MakeTopologyFromData(brepData, attributes, FALSE, TRUE, TRUE))
            return nullptr;
        db.EndReading();

        brepCleanup.Clear();
        return Wrap(brep);
    } catch (...) {
        return nullptr;
    }
}

BodyPtr Copy(Body const& body) {
    SmBrep* copy = nullptr;
    try {
        if (SM_SUCCESS != SmApiBrepCopy(body.m_brep, copy))
            return nullptr;
    } catch (...) {
        return nullptr;
    }
    return Wrap(copy);
}

Kind GetKind(Body const& body) {
    SmTArray<SmFace*> faces;
    GetFaces(*body.m_brep, faces);
    if (0 == faces.GetSize())
        return Kind::Wire;
    SmBoolean isSolid = FALSE;
    if (SM_SUCCESS == SmApiBrepIsManifoldSolid(body.m_brep, isSolid) && isSolid)
        return Kind::Solid;
    return Kind::Sheet;
}

size_t GetFaceCount(Body const& body) {
    SmTArray<SmFace*> faces;
    GetFaces(*body.m_brep, faces);
    return faces.GetSize();
}

bool GetRange(Body const& body, double low[3], double high[3]) {
    SmPoint3d min, max;
    if (SM_SUCCESS != SmApiBrepBoundingBox(body.m_brep, TRUE, min, max))
        return false;
    low[0] = min.x; low[1] = min.y; low[2] = min.z;
    high[0] = max.x; high[1] = max.y; high[2] = max.z;
    return true;
}

bool HasCurvedFaceOrEdge(Body const& body) {
    SmTArray<SmFace*> faces;
    GetFaces(*body.m_brep, faces);
    for (ULONG i = 0; i < faces.GetSize(); ++i) {
        SmSurface* surface = faces[i]->GetSurface();
        if (surface && !surface->IsPlanar())
            return true;
    }
    SmTArray<SmEdge*> edges;
    body.m_brep->GetEdges(edges);
    for (ULONG i = 0; i < edges.GetSize(); ++i) {
        SmCurve* curve = edges[i]->GetCurve();
        if (curve && !curve->IsLinear())
            return true;
    }
    return false;
}

bool Transform(Body& body, double const m[3][4]) {
    SmVector3d x(m[0][0], m[1][0], m[2][0]), y(m[0][1], m[1][1], m[2][1]), z(m[0][2], m[1][2], m[2][2]);
    double scale = x.Length();
    double const tolerance = 1.0e-10 * std::max(1.0, scale);
    if (scale <= 0.0 || std::fabs(y.Length() - scale) > tolerance || std::fabs(z.Length() - scale) > tolerance)
        return false; // only rigid motions with uniform scale
    try {
        if (std::fabs(scale - 1.0) > 1.0e-12 && SM_SUCCESS != SmApiScale(body.m_brep, SmVector3d(scale, scale, scale)))
            return false;
        SmAxis2Placement placement(SmPoint3d(m[0][3], m[1][3], m[2][3]), x / scale, y / scale);
        return SM_SUCCESS == SmApiTransform(body.m_brep, placement);
    } catch (...) {
        return false;
    }
}

bool Tessellate(Body const& body, TessellationOptions const& options, Mesh& mesh) {
    SmTessellationParams params;
    params.dChordHeightTol = options.chordTolerance;
    params.dCurveAngleTolDeg = options.angleToleranceDegrees;
    params.dSurfaceAngleTolDeg = options.angleToleranceDegrees;
    params.dMaxEdgeLength = options.maxEdgeLength;

    SmPolyBrep* polyBrep = nullptr;
    try {
        if (SM_SUCCESS != SmApiTessellate(body.m_brep, polyBrep, params) || nullptr == polyBrep)
            return false;
    } catch (...) {
        return false;
    }
    SmObjDelete cleanup(polyBrep);

    SmTArray<SmFace*> faces;
    GetFaces(*body.m_brep, faces);
    std::unordered_map<SmFace const*, uint32_t> faceIndex;
    for (ULONG i = 0; i < faces.GetSize(); ++i)
        faceIndex[faces[i]] = i;

    SmTArray<SmPolyVertex*> vertices;
    polyBrep->GetPolyVertices(vertices);
    std::unordered_map<SmPolyVertex const*, uint32_t> vertexIndex;
    mesh = Mesh();
    mesh.points.reserve(vertices.GetSize() * 3);
    for (ULONG i = 0; i < vertices.GetSize(); ++i) {
        vertexIndex[vertices[i]] = i;
        SmPoint3d const point = vertices[i]->GetPoint();
        mesh.points.insert(mesh.points.end(), {point.x, point.y, point.z});
    }

    // Params are per (vertex, face): a vertex on a face boundary has a uv on each face.
    std::map<std::pair<SmPolyVertex const*, SmFace const*>, SmPoint2d> uvs;

    SmTArray<SmPolyFace*> polyFaces;
    polyBrep->GetPolyFaces(polyFaces);
    for (ULONG i = 0; i < polyFaces.GetSize(); ++i) {
        SmPolyFace* polyFace = polyFaces[i];
        SmPolyLoop* loop = polyFace->GetOuterPolyLoop();
        if (nullptr == loop)
            continue;
        SmTArray<SmPolyEdge*> edges;
        loop->GetPolyEdges(edges);
        if (edges.GetSize() < 3)
            continue;

        SmFace* original = polyFace->GetOriginalFace();
        auto face = faceIndex.find(original);
        SmVector3d facetNormal = polyFace->GetNormal(TRUE, FALSE);
        size_t const firstIndex = mesh.indices.size();
        bool valid = true;
        for (ULONG j = 0; j < edges.GetSize() && valid; ++j) {
            SmPolyVertex* vertex = edges[j]->GetStartPolyVertex();
            auto found = vertexIndex.find(vertex);
            if (found == vertexIndex.end()) {
                valid = false;
                break;
            }
            mesh.indices.push_back(found->second);

            // The surface normal of the original face at this vertex, else the facet normal.
            SmVector3d normal = facetNormal;
            auto const& vertexFaces = vertex->GetFacesRef();
            auto const& vertexNormals = vertex->GetNormalsRef();
            for (ULONG k = 0; k < vertexFaces.GetSize() && k < vertexNormals.GetSize(); ++k) {
                if (vertexFaces[k] == static_cast<SmTopology*>(original)) {
                    normal = vertexNormals[k];
                    break;
                }
            }
            double const length = normal.Length();
            if (length > 1.0e-30)
                normal = normal / length;
            mesh.normals.insert(mesh.normals.end(), {normal.x, normal.y, normal.z});

            auto key = std::make_pair(static_cast<SmPolyVertex const*>(vertex), static_cast<SmFace const*>(original));
            auto param = uvs.find(key);
            if (param == uvs.end())
                param = uvs.emplace(key, SurfaceParams(original, vertex->GetPoint())).first;
            mesh.params.insert(mesh.params.end(), {param->second.x, param->second.y});
        }
        if (!valid) {
            mesh.indices.resize(firstIndex);
            mesh.normals.resize(firstIndex * 3);
            mesh.params.resize(firstIndex * 2);
            continue;
        }
        mesh.facetSizes.push_back(static_cast<uint32_t>(edges.GetSize()));
        mesh.facetFaces.push_back(face == faceIndex.end() ? 0 : face->second);
    }
    return !mesh.facetSizes.empty();
}

bool GetMassProperties(Body const& body, double relativeAccuracy, MassProperties& props) {
    double area, volume, mass;
    SmPoint3d centroid;
    SmVector3d moments, products;
    try {
        // A first pass for the centroid, then the moments about it.
        if (SM_SUCCESS != SmApiBrepComputeMassProperties(body.m_brep, relativeAccuracy, 1.0, SmPoint3d(0, 0, 0), area, volume, mass, centroid, moments, products))
            return false;
        if (SM_SUCCESS != SmApiBrepComputeMassProperties(body.m_brep, relativeAccuracy, 1.0, centroid, area, volume, mass, centroid, moments, products))
            return false;
    } catch (...) {
        return false;
    }
    props.volume = volume;
    props.area = area;
    props.centroid[0] = centroid.x; props.centroid[1] = centroid.y; props.centroid[2] = centroid.z;
    props.moments[0] = moments.x; props.moments[1] = moments.y; props.moments[2] = moments.z;
    // SMLib orders products (Iyz, Izx, Ixy).
    props.products[0] = products.z;
    props.products[1] = products.y;
    props.products[2] = products.x;
    return true;
}

/*---------------------------------------------------------------------------------**//**
* Primitives
+---------------+---------------+---------------+---------------+---------------+------*/
static SmVector3d Vec(double const v[3]) { return SmVector3d(v[0], v[1], v[2]); }

BodyPtr CreateBox(double const origin[3], double const size[3]) {
    SmBrep* brep = nullptr;
    try {
        if (SM_SUCCESS != SmApiCreateBox(Vec(origin), size[0], size[1], size[2], brep))
            return nullptr;
    } catch (...) {
        return nullptr;
    }
    return Wrap(brep);
}

BodyPtr CreateSphere(double const center[3], double radius) {
    SmBrep* brep = nullptr;
    try {
        if (SM_SUCCESS != SmApiCreateSphere(Vec(center), radius, brep))
            return nullptr;
    } catch (...) {
        return nullptr;
    }
    return Wrap(brep);
}

//! Move a body built around the origin's +z axis so that axis runs from `origin` along `axis`.
static bool PlaceOnAxis(SmBrep* brep, SmVector3d const& origin, SmVector3d axis) {
    axis = axis / axis.Length();
    SmVector3d const reference = std::fabs(axis.z) < 0.9 ? SmVector3d(0, 0, 1) : SmVector3d(1, 0, 0);
    SmVector3d x = reference * axis; // cross product
    x = x / x.Length();
    SmVector3d const y = axis * x;
    SmAxis2Placement placement(SmPoint3d(origin.x, origin.y, origin.z), x, y);
    return SM_SUCCESS == SmApiTransform(brep, placement);
}

BodyPtr CreateCone(double const base[3], double const top[3], double baseRadius, double topRadius) {
    SmVector3d const axis = Vec(top) - Vec(base);
    double const height = axis.Length();
    if (height <= 0.0)
        return nullptr;
    SmBrep* brep = nullptr;
    try {
        SmApiStatus status = baseRadius == topRadius
            ? SmApiCreateCylinder(SmVector3d(0, 0, 0), baseRadius, height, brep)
            : SmApiCreateCone(SmVector3d(0, 0, 0), baseRadius, topRadius, height, brep);
        if (SM_SUCCESS != status)
            return nullptr;
        BodyPtr body = Wrap(brep);
        if (!PlaceOnAxis(brep, Vec(base), axis))
            return nullptr;
        return body;
    } catch (...) {
        return nullptr;
    }
}

BodyPtr CreateTorus(double const center[3], double const axis[3], double majorRadius, double minorRadius) {
    SmBrep* brep = nullptr;
    try {
        if (SM_SUCCESS != SmApiCreateTorus(SmVector3d(0, 0, 0), majorRadius, minorRadius, brep))
            return nullptr;
        BodyPtr body = Wrap(brep);
        if (!PlaceOnAxis(brep, Vec(center), Vec(axis)))
            return nullptr;
        return body;
    } catch (...) {
        return nullptr;
    }
}

bool Boolean(BodyPtr& target, BodyPtr tool, BooleanOp op) {
    if (!target || !tool)
        return false;
    SmBrep* result = nullptr;
    try {
        SmApiStatus status;
        switch (op) {
            case BooleanOp::Unite: status = SmApiBooleanUnion(target->m_brep, tool->m_brep, result); break;
            case BooleanOp::Subtract: status = SmApiBooleanDifference(target->m_brep, tool->m_brep, result); break;
            case BooleanOp::Intersect: status = SmApiBooleanIntersection(target->m_brep, tool->m_brep, result); break;
            default: return false;
        }
        if (SM_SUCCESS != status || nullptr == result)
            return false;
    } catch (...) {
        return false;
    }
    // On success SMLib deleted the tool's brep and modified the target's in place.
    tool->m_brep = nullptr;
    target->m_brep = result;
    return true;
}

/*---------------------------------------------------------------------------------**//**
* Curves and the operations built from them
+---------------+---------------+---------------+---------------+---------------+------*/
namespace {

//! Owns SMLib curves until they are released to an operation that consumes them.
struct CurveArray {
    SmTArray<SmCurve*> m_curves;
    ~CurveArray() {
        for (ULONG i = 0; i < m_curves.GetSize(); ++i)
            delete m_curves[i];
    }
    void Release() { m_curves.SetSize(0); }
};

SmBSplineCurve* ToSmCurve(Curve const& curve) {
    size_t const count = curve.poles.size() / 3;
    if (curve.degree < 1 || count < 2 || curve.knots.size() != count + curve.degree + 1 || (!curve.weights.empty() && curve.weights.size() != count))
        return nullptr;
    SmTArray<SmPoint3d> poles;
    for (size_t i = 0; i < count; ++i)
        poles.Add(SmPoint3d(curve.poles[3 * i], curve.poles[3 * i + 1], curve.poles[3 * i + 2]));
    SmTArray<double> knots;
    SmTArray<ULONG> multiplicities;
    double const span = curve.knots.back() - curve.knots.front();
    for (double knot : curve.knots) {
        if (knots.GetSize() > 0 && std::fabs(knot - knots[knots.GetSize() - 1]) <= 1.0e-12 * span) {
            multiplicities[multiplicities.GetSize() - 1]++;
        } else {
            knots.Add(knot);
            multiplicities.Add(1);
        }
    }
    SmTArray<double> weights;
    for (double weight : curve.weights)
        weights.Add(weight);
    SmBSplineCurve* result = nullptr;
    if (SM_SUCCESS != SmBSplineCurve::CreateCanonical(Context(), 3, curve.degree, poles, SM_CF_UNSPECIFIED, multiplicities, knots, SM_KT_UNSPECIFIED, curve.weights.empty() ? nullptr : &weights, nullptr, result))
        return nullptr;
    return result;
}

bool AddCurves(Curves const& curves, CurveArray& out) {
    for (Curve const& curve : curves) {
        SmBSplineCurve* smCurve = ToSmCurve(curve);
        if (nullptr == smCurve)
            return false;
        out.m_curves.Add(smCurve);
    }
    return true;
}

SmBrep* Release(BodyPtr& body) {
    SmBrep* brep = body->m_brep;
    body->m_brep = nullptr;
    body.reset();
    return brep;
}

} // namespace

BodyPtr CreatePlanarSheet(std::vector<Curves> const& loops) {
    try {
        CurveArray curves;
        for (Curves const& loop : loops) {
            if (!AddCurves(loop, curves))
                return nullptr;
        }
        SmBrep* brep = nullptr;
        if (SM_SUCCESS != SmApiCreatePlanarFaces(curves.m_curves, brep) || nullptr == brep)
            return nullptr;
        curves.Release();
        return Wrap(brep);
    } catch (...) {
        return nullptr;
    }
}

BodyPtr Extrude(std::vector<Curves> const& loops, double const direction[3], double distance, bool cap) {
    try {
        CurveArray curves;
        for (Curves const& loop : loops) {
            if (!AddCurves(loop, curves))
                return nullptr;
        }
        SmBrep* brep = nullptr;
        if (SM_SUCCESS != SmApiCreateLinearSweep(curves.m_curves, Vec(direction), distance, cap ? TRUE : FALSE, brep) || nullptr == brep)
            return nullptr;
        return Wrap(brep);
    } catch (...) {
        return nullptr;
    }
}

namespace {

using Vec3 = SmVector3d;

Vec3 PoleOf(Curve const& curve, size_t i) { return Vec3(curve.poles[3 * i], curve.poles[3 * i + 1], curve.poles[3 * i + 2]); }

//! Apply x -> pivot + R (x - pivot) + shift to every pole, where R rotates `from` onto `to`.
void Move(Curves& curves, Vec3 const& pivot, Vec3 const& from, Vec3 const& to, Vec3 const& shift) {
    Vec3 axis = from * to;
    double const sine = axis.Length(), cosine = from.Dot(to);
    if (sine > 1.0e-14)
        axis = axis / sine;
    for (Curve& curve : curves) {
        for (size_t i = 0; i < curve.poles.size() / 3; ++i) {
            Vec3 v = PoleOf(curve, i) - pivot;
            if (sine > 1.0e-14) // Rodrigues
                v = v * cosine + (axis * v) * sine + axis * (axis.Dot(v) * (1.0 - cosine));
            v = v + pivot + shift;
            curve.poles[3 * i] = v.x;
            curve.poles[3 * i + 1] = v.y;
            curve.poles[3 * i + 2] = v.z;
        }
    }
}

//! Keep the part of the body on the side of the plane its normal points to.
bool ClipToHalfSpace(BodyPtr& body, Vec3 const& origin, Vec3 normal, double size) {
    normal = normal / normal.Length();
    Vec3 const reference = std::fabs(normal.z) < 0.9 ? Vec3(0, 0, 1) : Vec3(1, 0, 0);
    Vec3 x = reference * normal;
    x = x / x.Length();
    Vec3 const y = normal * x;
    SmBrep* box = nullptr;
    if (SM_SUCCESS != SmApiCreateBox(Vec3(-size, -size, 0), 2 * size, 2 * size, size, box))
        return false;
    BodyPtr halfSpace = Wrap(box);
    if (SM_SUCCESS != SmApiTransform(box, SmAxis2Placement(origin, x, y)))
        return false;
    return Boolean(body, std::move(halfSpace), BooleanOp::Intersect);
}

//! Sweep along line segments with mitered corners: one extrusion per segment, each clipped by
//! the planes that bisect its corners, all united.
BodyPtr SweepPolyline(Curves profile, std::vector<Vec3> const& points) {
    size_t const segments = points.size() - 1;
    std::vector<Vec3> directions;
    std::vector<double> lengths;
    double total = 0.0;
    for (size_t i = 0; i < segments; ++i) {
        Vec3 d = points[i + 1] - points[i];
        lengths.push_back(d.Length());
        directions.push_back(d / lengths.back());
        total += lengths.back();
    }
    double radius = 0.0;
    for (Curve const& curve : profile)
        for (size_t i = 0; i < curve.poles.size() / 3; ++i)
            radius = std::max(radius, Vec3(PoleOf(curve, i) - points[0]).Length());
    double const margin = total + 4.0 * radius;
    double const size = 10.0 * (total + radius);

    BodyPtr result;
    for (size_t i = 0; i < segments; ++i) {
        if (i > 0) {
            if (directions[i - 1].Dot(directions[i]) < -0.99)
                return nullptr; // turns back on itself
            Move(profile, points[i], directions[i - 1], directions[i], Vec3(0, 0, 0));
        }
        double const back = i > 0 ? margin : 0.0, ahead = i + 1 < segments ? margin : 0.0;
        Curves start = profile;
        Move(start, points[i], directions[i], directions[i], directions[i] * -back);
        double const direction[3] = {directions[i].x, directions[i].y, directions[i].z};
        BodyPtr piece = Extrude({start}, direction, back + lengths[i] + ahead);
        if (!piece)
            return nullptr;
        if (i > 0 && !ClipToHalfSpace(piece, points[i], directions[i - 1] + directions[i], size))
            return nullptr;
        if (i + 1 < segments && !ClipToHalfSpace(piece, points[i + 1], (directions[i] + directions[i + 1]) * -1.0, size))
            return nullptr;
        if (!result)
            result = std::move(piece);
        else if (!Boolean(result, std::move(piece), BooleanOp::Unite))
            return nullptr;
        Move(profile, points[i], directions[i], directions[i], directions[i] * lengths[i]);
    }
    return result;
}

} // namespace

BodyPtr Sweep(Curves const& profile, Curves const& path) {
    if (path.empty())
        return nullptr;
    try {
        // A path of line segments: sweep piecewise with mitered corners.
        std::vector<Vec3> points;
        bool linear = true;
        for (Curve const& curve : path) {
            if (1 != curve.degree) {
                linear = false;
                break;
            }
            for (size_t i = 0; i < curve.poles.size() / 3; ++i) {
                Vec3 const point = PoleOf(curve, i);
                if (points.empty() || Vec3(point - points.back()).Length() > 1.0e-10)
                    points.push_back(point);
            }
        }
        if (linear)
            return points.size() < 2 ? nullptr : SweepPolyline(profile, points);
        if (1 != path.size())
            return nullptr;

        CurveArray curves;
        if (!AddCurves(profile, curves))
            return nullptr;
        SmBSplineCurve* smPath = ToSmCurve(path.front());
        if (nullptr == smPath)
            return nullptr;
        SmObjDelete pathCleanup(smPath);
        SmSweepOptions options;
        options.bCapEndsArg = TRUE;
        options.bMoveToProfile = TRUE;
        SmBrep* brep = nullptr;
        if (SM_SUCCESS != SmApiCreateCurveSweep(curves.m_curves, smPath, nullptr, nullptr, &options, brep, nullptr, nullptr, nullptr) || nullptr == brep)
            return nullptr;
        return Wrap(brep);
    } catch (...) {
        return nullptr;
    }
}

BodyPtr Loft(std::vector<Curves> const& profiles) {
    if (profiles.size() < 2)
        return nullptr;
    try {
        CurveArray curves;
        SmTArray<ULONG> counts;
        for (Curves const& profile : profiles) {
            if (!AddCurves(profile, curves))
                return nullptr;
            counts.Add(static_cast<ULONG>(profile.size()));
        }
        SmTArray<SmCurve const*> constCurves;
        for (ULONG i = 0; i < curves.m_curves.GetSize(); ++i)
            constCurves.Add(curves.m_curves[i]);
        ULONG const degree = static_cast<ULONG>(std::min<size_t>(3, profiles.size() - 1));
        SmBrep* brep = nullptr;
        if (SM_SUCCESS != SmApiCreateSkinPrimitive(counts, constCurves, degree, TRUE, brep, nullptr, nullptr, nullptr) || nullptr == brep)
            return nullptr;
        return Wrap(brep);
    } catch (...) {
        return nullptr;
    }
}

BodyPtr Sew(std::vector<BodyPtr> sheets) {
    if (sheets.empty() || !sheets.front())
        return nullptr;
    try {
        BodyPtr result = std::move(sheets.front());
        for (size_t i = 1; i < sheets.size(); ++i) {
            if (!sheets[i] || SM_SUCCESS != result->m_brep->MergeBrep(*sheets[i]->m_brep))
                return nullptr;
        }
        SmBoolean solid = FALSE;
        ULONG stitched = 0;
        double vertexGap = 0.0, edgeGap = 0.0;
        if (SM_SUCCESS != SmApiStitchIntoSolid(result->m_brep, solid, stitched, vertexGap, edgeGap))
            return nullptr;
        return result;
    } catch (...) {
        return nullptr;
    }
}

namespace {

//! The order to try SMLib's offset modes in. Extended offsets keep corners sharp, but on curved
//! bodies they tend to fail, and a failed shell or offset can leave SMLib state that breaks later
//! operations, so curved bodies try rounded corners first.
std::vector<SmBoolean> OffsetModes(Body const& body) {
    if (HasCurvedFaceOrEdge(body))
        return {FALSE, TRUE};
    return {TRUE, FALSE};
}

//! A solid result must enclose a positive volume. SMLib sometimes reports success for a broken one.
bool IsValidResult(Body const& body, Kind expected) {
    if (GetKind(body) != expected)
        return false;
    if (Kind::Solid != expected)
        return true;
    MassProperties props;
    return GetMassProperties(body, 1.0e-3, props) && std::isfinite(props.volume) && props.volume > 0.0;
}

} // namespace

bool Offset(BodyPtr& body, double distance) {
    try {
        Kind const kind = GetKind(*body);
        // The input may be left modified on failure, so each attempt works on a copy.
        for (SmBoolean extended : OffsetModes(*body)) {
            BodyPtr copy = Copy(*body);
            SmBrep* result = nullptr;
            if (copy && SM_SUCCESS == SmApiOffsetBrepFull(copy->m_brep, distance, extended, TRUE, result) && nullptr != result) {
                BodyPtr offset = result == copy->m_brep ? std::move(copy) : Wrap(result);
                if (!IsValidResult(*offset, kind))
                    return false;
                body = std::move(offset);
                return true;
            }
        }
        return false;
    } catch (...) {
        return false;
    }
}

bool Hollow(BodyPtr& body, double distance) {
    try {
        for (SmBoolean extended : OffsetModes(*body)) {
            SmBrep* result = nullptr;
            SmTArray<SmFace const*> keep;
            if (SM_SUCCESS == SmApiShellBrepFull(body->m_brep, distance, extended, TRUE, FALSE, keep, result) && nullptr != result) {
                BodyPtr shell = Wrap(result);
                if (!IsValidResult(*shell, Kind::Solid))
                    return false;
                body = std::move(shell);
                return true;
            }
        }
        return false;
    } catch (...) {
        return false;
    }
}

bool Thicken(BodyPtr& body, double front, double back) {
    if (front + back <= 0.0)
        return false;
    if (back > 0.0 && !Offset(body, -back))
        return false;
    try {
        for (SmBoolean extended : OffsetModes(*body)) {
            SmBrep* result = nullptr;
            SmTArray<SmFace const*> keep;
            if (SM_SUCCESS == SmApiShellBrepFull(body->m_brep, front + back, extended, TRUE, TRUE, keep, result) && nullptr != result) {
                BodyPtr solid = Wrap(result);
                if (!IsValidResult(*solid, Kind::Solid))
                    return false;
                body = std::move(solid);
                return true;
            }
        }
        return false;
    } catch (...) {
        return false;
    }
}

bool Round(BodyPtr& body, double radius) {
    try {
        SmTArray<SmEdge*> all, sharp;
        body->m_brep->GetEdges(all);
        for (ULONG i = 0; i < all.GetSize(); ++i) {
            if (!all[i]->IsLamina() && !all[i]->IsSeam())
                sharp.Add(all[i]);
        }
        if (0 == sharp.GetSize())
            return false;
        return SM_SUCCESS == SmApiFilletEdges(body->m_brep, sharp, radius, 1, 1, 1.0);
    } catch (...) {
        return false;
    }
}

bool KeepHalfSpace(BodyPtr& body, double const origin[3], double const normal[3]) {
    try {
        double low[3], high[3];
        if (!GetRange(*body, low, high))
            return false;
        Vec3 const point = Vec(origin);
        double const size = 10.0 * (Vec3(Vec(high) - Vec(low)).Length() + Vec3(Vec(low) - point).Length() + 1.0);
        return ClipToHalfSpace(body, point, Vec(normal), size);
    } catch (...) {
        return false;
    }
}

std::vector<BodyPtr> Separate(BodyPtr body) {
    std::vector<BodyPtr> result;
    try {
        SmBrep* brep = Release(body);
        SmTArray<SmBrep*> parts;
        if (SM_SUCCESS != SmBrep::CreateOneBrepPerBody(brep, parts)) {
            delete brep;
            return result;
        }
        for (ULONG i = 0; i < parts.GetSize(); ++i)
            result.push_back(Wrap(parts[i]));
    } catch (...) {
    }
    return result;
}

bool GetPlane(Body const& body, double origin[3], double normal[3]) {
    SmTArray<SmFace*> faces;
    GetFaces(*body.m_brep, faces);
    if (0 == faces.GetSize())
        return false;
    SmPoint3d point;
    SmVector3d direction;
    for (ULONG i = 0; i < faces.GetSize(); ++i) {
        SmSurface* surface = faces[i]->GetSurface();
        SmPoint3d facePoint;
        SmVector3d faceNormal;
        if (nullptr == surface || !surface->IsPlanar(1.0e-8, &facePoint, &faceNormal))
            return false;
        if (0 == i) {
            point = facePoint;
            direction = faceNormal;
        } else if (std::fabs(std::fabs(faceNormal.Dot(direction)) - 1.0) > 1.0e-8 || std::fabs(SmVector3d(facePoint - point).Dot(direction)) > 1.0e-8) {
            return false;
        }
    }
    origin[0] = point.x; origin[1] = point.y; origin[2] = point.z;
    normal[0] = direction.x; normal[1] = direction.y; normal[2] = direction.z;
    return true;
}

} // namespace SmKernel
