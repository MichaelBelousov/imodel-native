/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
#include <SMBRepGeometry/SMBRepGeometry.h>
#include <DgnPlatform/ElementGeometry.h>
#include <Napi/napi.h>
#include "SmKernel.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>

USING_NAMESPACE_BENTLEY_DGN

namespace {

using Lock = std::lock_guard<std::recursive_mutex>;
using BodyRef = std::shared_ptr<SmKernel::Body>;

BodyRef Share(SmKernel::BodyPtr body) {
    return BodyRef(body.release(), [](SmKernel::Body* b) {
        Lock lock(SmKernel::Mutex());
        SmKernel::Delete(b);
    });
}

struct FaceAttachments : RefCounted<IFaceMaterialAttachments> {
    T_FaceAttachmentsVec m_attachments;
    T_FaceAttachmentsVec const& _GetFaceAttachmentsVec() const override { return m_attachments; }
    T_FaceAttachmentsVec& _GetFaceAttachmentsVecR() override { return m_attachments; }
};

//=======================================================================================
// An SMLib body in local coordinates plus its body-to-world transform. Instances made by
// CreateInstance share the body.
//=======================================================================================
struct SMLibEntity : RefCounted<IBRepEntity> {
    BodyRef m_body;
    Transform m_transform;
    RefCountedPtr<FaceAttachments> m_attachments;
    //! For each face (in SMLib's face order), the index of its attachment.
    bvector<uint32_t> m_faceSymbology;
    //! Output of _SaveEntityToMemory, which the entity owns.
    mutable std::vector<uint8_t> m_saved;
    mutable std::optional<DRange3d> m_localRange;

    SMLibEntity(BodyRef body, TransformCR transform) : m_body(std::move(body)), m_transform(transform) { }

    SmKernel::Body const& Body() const { return *m_body; }

    static SMLibEntity const* From(IBRepEntityCR entity) { return dynamic_cast<SMLibEntity const*>(&entity); }

    //! The attachment index for each face, defaulting to 0 (the base symbology).
    uint32_t SymbologyOf(uint32_t face) const {
        if (!m_attachments.IsValid() || face >= m_faceSymbology.size())
            return 0;
        uint32_t index = m_faceSymbology[face];
        return index < m_attachments->m_attachments.size() ? index : 0;
    }

    bool _IsEqual(IBRepEntityCR other) const override {
        auto o = From(other);
        return nullptr != o && o->m_body == m_body && o->m_transform.IsEqual(m_transform);
    }

    EntityType _GetEntityType() const override {
        Lock lock(SmKernel::Mutex());
        switch (SmKernel::GetKind(Body())) {
            case SmKernel::Kind::Solid: return EntityType::Solid;
            case SmKernel::Kind::Sheet: return EntityType::Sheet;
            default: return EntityType::Wire;
        }
    }

    DRange3d _GetLocalEntityRange() const override {
        if (!m_localRange) {
            double low[3], high[3];
            Lock lock(SmKernel::Mutex());
            m_localRange = SmKernel::GetRange(Body(), low, high) ? DRange3d::From(DPoint3d::FromArray(low), DPoint3d::FromArray(high)) : DRange3d::NullRange();
        }
        return *m_localRange;
    }

    DRange3d _GetEntityRange() const override {
        DRange3d range = _GetLocalEntityRange();
        if (!range.IsNull())
            m_transform.Multiply(range, range);
        return range;
    }

    bool _HasCurvedFaceOrEdge() const override {
        Lock lock(SmKernel::Mutex());
        return SmKernel::HasCurvedFaceOrEdge(Body());
    }

    Transform _GetEntityTransform() const override { return m_transform; }
    bool _SetEntityTransform(TransformCR transform) override { m_transform = transform; return true; }

    IFaceMaterialAttachmentsCP _GetFaceMaterialAttachments() const override { return m_attachments.get(); }

    //! @note faceIndexToAttachmentIndex uses 1-based face indices, matching how older b-reps
    //! recorded face symbology.
    void _SetFaceMaterialAttachments(T_FaceAttachmentsVec const& attachments, T_FaceIndexToAttachmentIndexVec const* faceIndexToAttachmentIndex) override {
        if (attachments.empty()) {
            m_attachments = nullptr;
            m_faceSymbology.clear();
            return;
        }
        m_attachments = new FaceAttachments();
        m_attachments->m_attachments = attachments;
        if (nullptr == faceIndexToAttachmentIndex)
            return;
        size_t faceCount;
        {
            Lock lock(SmKernel::Mutex());
            faceCount = SmKernel::GetFaceCount(Body());
        }
        m_faceSymbology.assign(faceCount, 0);
        for (auto const& entry : *faceIndexToAttachmentIndex) {
            if (entry.m_faceIndex >= 1 && entry.m_faceIndex <= faceCount)
                m_faceSymbology[entry.m_faceIndex - 1] = entry.m_symbIndex;
        }
    }

    IBRepEntityPtr _Clone() const override {
        SmKernel::BodyPtr copy;
        {
            Lock lock(SmKernel::Mutex());
            copy = SmKernel::Copy(Body());
        }
        if (!copy)
            return nullptr;
        auto clone = new SMLibEntity(Share(std::move(copy)), m_transform);
        clone->m_faceSymbology = m_faceSymbology;
        if (m_attachments.IsValid())
            clone->_SetFaceMaterialAttachments(m_attachments->m_attachments, nullptr);
        return clone;
    }

    IBRepEntityPtr _CreateInstance(bool) const override {
        auto instance = new SMLibEntity(m_body, m_transform);
        instance->m_localRange = m_localRange;
        instance->m_faceSymbology = m_faceSymbology;
        if (m_attachments.IsValid())
            instance->_SetFaceMaterialAttachments(m_attachments->m_attachments, nullptr);
        return instance;
    }

    bool _IsAlive() const override { return nullptr != m_body; }
    void _ChangePartition() const override { }

    bool _IsSameStructureAndGeometry(IBRepEntityCR other, double tolerance) const override {
        auto o = From(other);
        if (nullptr == o)
            return false;
        if (o->m_body == m_body)
            return true;
        {
            Lock lock(SmKernel::Mutex());
            if (SmKernel::GetFaceCount(Body()) != SmKernel::GetFaceCount(o->Body()) || SmKernel::GetKind(Body()) != SmKernel::GetKind(o->Body()))
                return false;
        }
        DRange3d a = _GetEntityRange(), b = other.GetEntityRange();
        return a.low.Distance(b.low) <= tolerance && a.high.Distance(b.high) <= tolerance;
    }
};

//! Largest scale the transform applies to any direction (for converting world tolerances).
double MaxScale(TransformCR transform) {
    DVec3d x, y, z;
    transform.Matrix().GetColumns(x, y, z);
    return std::max({x.Magnitude(), y.Magnitude(), z.Magnitude()});
}

SmKernel::TessellationOptions ToKernelOptions(IFacetOptionsCR options, TransformCR transform) {
    double const scale = MaxScale(transform);
    SmKernel::TessellationOptions kernel;
    if (options.GetChordTolerance() > 0.0 && scale > 0.0)
        kernel.chordTolerance = options.GetChordTolerance() / scale;
    if (options.GetMaxEdgeLength() > 0.0 && scale > 0.0)
        kernel.maxEdgeLength = options.GetMaxEdgeLength() / scale;
    if (options.GetAngleTolerance() > 0.0)
        kernel.angleToleranceDegrees = Angle::RadiansToDegrees(options.GetAngleTolerance());
    if (kernel.chordTolerance <= 0.0 && kernel.angleToleranceDegrees <= 0.0)
        kernel.angleToleranceDegrees = 25.0;
    return kernel;
}

//! Polyfaces from a tessellation, one per group, where `groupOf(face)` assigns body faces to groups.
template <typename GroupOf>
bvector<PolyfaceHeaderPtr> ToPolyfaces(SmKernel::Mesh const& mesh, size_t groupCount, GroupOf groupOf, SMLibEntity const& entity, IFacetOptionsCR options) {
    TransformCR transform = entity.m_transform;
    RotMatrix normalMatrix;
    bool const normals = options.GetNormalsRequired();
    bool const params = options.GetParamsRequired();
    if (normals) {
        RotMatrix matrix = transform.Matrix();
        normalMatrix.InverseOf(matrix);
        normalMatrix.Transpose();
    }

    // An edge between two facets of the same body face is interior to that face; hide it so only
    // the b-rep's own edges draw.
    std::map<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, int>> edgeUse; // edge -> (face, uses)
    std::vector<size_t> facetStart(mesh.facetSizes.size());
    for (size_t f = 0, start = 0; f < mesh.facetSizes.size(); start += mesh.facetSizes[f++]) {
        facetStart[f] = start;
        for (uint32_t i = 0; i < mesh.facetSizes[f]; ++i) {
            uint32_t a = mesh.indices[start + i], b = mesh.indices[start + (i + 1) % mesh.facetSizes[f]];
            auto key = std::make_pair(std::min(a, b), std::max(a, b));
            auto it = edgeUse.find(key);
            if (it == edgeUse.end())
                edgeUse[key] = {mesh.facetFaces[f], 1};
            else if (it->second.first == mesh.facetFaces[f])
                it->second.second++;
            else
                it->second.second = -1; // shared by different faces: a b-rep edge
        }
    }

    bool const twoSided = IBRepEntity::EntityType::Solid != entity.GetEntityType();
    bvector<PolyfaceHeaderPtr> polyfaces(groupCount);
    bvector<bmap<uint32_t, int>> pointMaps(groupCount); // mesh point -> 1-based polyface point
    for (size_t f = 0; f < mesh.facetSizes.size(); ++f) {
        size_t group = groupOf(mesh.facetFaces[f]);
        auto& polyface = polyfaces[group];
        if (polyface.IsNull()) {
            polyface = PolyfaceHeader::CreateVariableSizeIndexed();
            polyface->SetTwoSided(twoSided);
        }
        uint32_t const size = mesh.facetSizes[f];
        bvector<int> pointIndices, normalIndices, paramIndices;
        for (uint32_t i = 0; i < size; ++i) {
            size_t const corner = facetStart[f] + i;
            uint32_t const point = mesh.indices[corner];
            auto found = pointMaps[group].find(point);
            int index;
            if (found == pointMaps[group].end()) {
                DPoint3d xyz = DPoint3d::From(mesh.points[3 * point], mesh.points[3 * point + 1], mesh.points[3 * point + 2]);
                transform.Multiply(xyz);
                polyface->Point().push_back(xyz);
                index = static_cast<int>(polyface->Point().size());
                pointMaps[group][point] = index;
            } else {
                index = found->second;
            }

            uint32_t const next = mesh.indices[facetStart[f] + (i + 1) % size];
            auto use = edgeUse.find(std::make_pair(std::min(point, next), std::max(point, next)));
            bool const hidden = options.GetEdgeHiding() && use != edgeUse.end() && use->second.second == 2;
            pointIndices.push_back(hidden ? -index : index);

            if (normals) {
                DVec3d normal = DVec3d::From(mesh.normals[3 * corner], mesh.normals[3 * corner + 1], mesh.normals[3 * corner + 2]);
                normalMatrix.Multiply(normal);
                normal.Normalize();
                polyface->Normal().push_back(normal);
                normalIndices.push_back(static_cast<int>(polyface->Normal().size()));
            }
            if (params) {
                polyface->Param().push_back(DPoint2d::From(mesh.params[2 * corner], mesh.params[2 * corner + 1]));
                paramIndices.push_back(static_cast<int>(polyface->Param().size()));
            }
        }
        polyface->PointIndex().AddAndTerminate(pointIndices.data(), pointIndices.size());
        if (normals)
            polyface->NormalIndex().AddAndTerminate(normalIndices.data(), normalIndices.size());
        if (params)
            polyface->ParamIndex().AddAndTerminate(paramIndices.data(), paramIndices.size());
    }
    int const maxPerFace = options.GetMaxPerFace();
    for (auto& polyface : polyfaces) {
        if (polyface.IsNull())
            continue;
        polyface->SetActiveFlagsByAvailableData();
        if (maxPerFace >= 3)
            polyface->Triangulate(maxPerFace, true, nullptr);
    }
    return polyfaces;
}

bool Tessellate(SMLibEntity const& entity, IFacetOptionsCR options, SmKernel::Mesh& mesh) {
    Lock lock(SmKernel::Mutex());
    return SmKernel::Tessellate(entity.Body(), ToKernelOptions(options, entity.m_transform), mesh);
}

//! Central second moments of a region, from its products about the origin.
struct Moments {
    double amount = 0.0;
    DPoint3d centroid = DPoint3d::FromZero();
    double xx = 0, yy = 0, zz = 0, xy = 0, xz = 0, yz = 0; //!< central products

    void ToOutputs(DPoint3dR moments, double& iXY, double& iXZ, double& iYZ) const {
        moments.Init(yy + zz, xx + zz, xx + yy);
        iXY = xy;
        iXZ = xz;
        iYZ = yz;
    }
};

} // namespace

/*---------------------------------------------------------------------------------**//**
* Session. SMLib needs no setup; Mutex() serializes all access to it.
+---------------+---------------+---------------+---------------+---------------+------*/
static bool s_sessionStarted = false;
void SMLibKernelAdmin::_StartSession() const { s_sessionStarted = true; }
void SMLibKernelAdmin::_StopSession() const { s_sessionStarted = false; }
bool SMLibKernelAdmin::_IsSessionStarted() const { return s_sessionStarted; }

/*---------------------------------------------------------------------------------**//**
* @bsimethod
+---------------+---------------+---------------+---------------+---------------+------*/
BentleyStatus SMLibKernelAdmin::_RestoreEntityFromMemory(IBRepEntityPtr& out, uint8_t const* buffer, size_t bufferSize, TransformCR transform) const {
    std::vector<uint32_t> faceSymbology;
    SmKernel::BodyPtr body;
    {
        Lock lock(SmKernel::Mutex());
        body = SmKernel::Restore(buffer, bufferSize, faceSymbology);
    }
    if (!body)
        return ERROR;
    auto entity = new SMLibEntity(Share(std::move(body)), transform);
    entity->m_faceSymbology.assign(faceSymbology.begin(), faceSymbology.end());
    out = entity;
    return SUCCESS;
}

BentleyStatus SMLibKernelAdmin::_SaveEntityToMemory(uint8_t** buffer, size_t& bufferSize, IBRepEntityCR in, bool) const {
    auto entity = SMLibEntity::From(in);
    if (nullptr == entity)
        return ERROR;
    std::vector<uint32_t> faceSymbology(entity->m_faceSymbology.begin(), entity->m_faceSymbology.end());
    {
        Lock lock(SmKernel::Mutex());
        if (!SmKernel::Save(entity->Body(), faceSymbology, entity->m_saved))
            return ERROR;
    }
    *buffer = entity->m_saved.data();
    bufferSize = entity->m_saved.size();
    return SUCCESS;
}

/*---------------------------------------------------------------------------------**//**
* @bsimethod
+---------------+---------------+---------------+---------------+---------------+------*/
PolyfaceHeaderPtr SMLibKernelAdmin::_FacetEntity(IBRepEntityCR in, IFacetOptionsCR options) const {
    auto entity = SMLibEntity::From(in);
    SmKernel::Mesh mesh;
    if (nullptr == entity || !Tessellate(*entity, options, mesh))
        return nullptr;
    return ToPolyfaces(mesh, 1, [](uint32_t) { return size_t(0); }, *entity, options).front();
}

bool SMLibKernelAdmin::_FacetEntity(IBRepEntityCR in, bvector<PolyfaceHeaderPtr>& polyfaces, bvector<FaceAttachment>& params, IFacetOptionsCR options) const {
    auto entity = SMLibEntity::From(in);
    SmKernel::Mesh mesh;
    if (nullptr == entity || !Tessellate(*entity, options, mesh))
        return false;

    T_FaceAttachmentsVec attachments;
    if (entity->m_attachments.IsValid() && !options.GetIgnoreFaceMaterialAttachments())
        attachments = entity->m_attachments->m_attachments;
    if (attachments.empty())
        attachments.push_back(FaceAttachment());

    bool const useAttachments = attachments.size() > 1;
    auto grouped = ToPolyfaces(mesh, attachments.size(), [&](uint32_t face) { return size_t(useAttachments ? entity->SymbologyOf(face) : 0); }, *entity, options);
    for (size_t i = 0; i < grouped.size(); ++i) {
        if (grouped[i].IsNull())
            continue;
        polyfaces.push_back(grouped[i]);
        params.push_back(attachments[i]);
    }
    return !polyfaces.empty();
}

/*---------------------------------------------------------------------------------**//**
* Area properties come from a fine tessellation, as iModelPlatform does for meshes.
+---------------+---------------+---------------+---------------+---------------+------*/
BentleyStatus SMLibKernelAdmin::_GetAreaProperties(IBRepEntityCR in, double& area, double& perimeter, DPoint3dR centroid, DPoint3dR moments, double& iXY, double& iXZ, double& iYZ, double tolerance) const {
    auto entity = SMLibEntity::From(in);
    if (nullptr == entity)
        return ERROR;

    IFacetOptionsPtr options = IFacetOptions::Create();
    options->SetChordTolerance(tolerance > 0.0 ? tolerance : 0.0);
    options->SetAngleTolerance(Angle::DegreesToRadians(5.0));
    options->SetNormalsRequired(false);
    options->SetEdgeHiding(false);
    SmKernel::Mesh mesh;
    if (!Tessellate(*entity, *options, mesh))
        return ERROR;
    auto polyface = ToPolyfaces(mesh, 1, [](uint32_t) { return size_t(0); }, *entity, *options).front();

    DMatrix4d products;
    Moments m;
    m.amount = polyface->SumFacetSecondAreaMomentProducts(DPoint3d::FromZero(), products);
    if (m.amount <= 0.0)
        return ERROR;
    m.centroid.Init(products.coff[0][3] / m.amount, products.coff[1][3] / m.amount, products.coff[2][3] / m.amount);
    DPoint3dCR c = m.centroid;
    m.xx = products.coff[0][0] - m.amount * c.x * c.x;
    m.yy = products.coff[1][1] - m.amount * c.y * c.y;
    m.zz = products.coff[2][2] - m.amount * c.z * c.z;
    m.xy = products.coff[0][1] - m.amount * c.x * c.y;
    m.xz = products.coff[0][2] - m.amount * c.x * c.z;
    m.yz = products.coff[1][2] - m.amount * c.y * c.z;

    // Perimeter: edges used by a single facet bound an open sheet.
    perimeter = 0.0;
    std::map<std::pair<uint32_t, uint32_t>, int> edgeCount;
    for (size_t f = 0, start = 0; f < mesh.facetSizes.size(); start += mesh.facetSizes[f++]) {
        for (uint32_t i = 0; i < mesh.facetSizes[f]; ++i) {
            uint32_t a = mesh.indices[start + i], b = mesh.indices[start + (i + 1) % mesh.facetSizes[f]];
            edgeCount[std::make_pair(std::min(a, b), std::max(a, b))]++;
        }
    }
    for (auto const& [edge, count] : edgeCount) {
        if (count != 1)
            continue;
        DPoint3d a = DPoint3d::FromArray(&mesh.points[3 * edge.first]), b = DPoint3d::FromArray(&mesh.points[3 * edge.second]);
        entity->m_transform.Multiply(a);
        entity->m_transform.Multiply(b);
        perimeter += a.Distance(b);
    }

    area = m.amount;
    centroid = m.centroid;
    m.ToOutputs(moments, iXY, iXZ, iYZ);
    return SUCCESS;
}

/*---------------------------------------------------------------------------------**//**
* Volume properties are exact from SMLib, then mapped through the entity transform: for
* x' = Ax + t, the central second-moment matrix becomes |det A| A S A^T.
+---------------+---------------+---------------+---------------+---------------+------*/
BentleyStatus SMLibKernelAdmin::_GetVolumeProperties(IBRepEntityCR in, double& volume, double& area, DPoint3dR centroid, DPoint3dR moments, double& iXY, double& iXZ, double& iYZ, double tolerance) const {
    auto entity = SMLibEntity::From(in);
    if (nullptr == entity || IBRepEntity::EntityType::Solid != in.GetEntityType())
        return ERROR;

    DRange3d range = in.GetLocalEntityRange();
    double const diagonal = range.IsNull() ? 1.0 : range.low.Distance(range.high);
    double const scale = MaxScale(entity->m_transform);
    double const relative = std::clamp(tolerance > 0.0 && diagonal > 0.0 && scale > 0.0 ? tolerance / (diagonal * scale) : 1.0e-4, 1.0e-4, 1.0e-1);

    SmKernel::MassProperties props;
    {
        Lock lock(SmKernel::Mutex());
        if (!SmKernel::GetMassProperties(entity->Body(), relative, props))
            return ERROR;
    }

    // Central second moments S from the inertia moments: Ixx = Syy + Szz, ...
    double const half = 0.5 * (props.moments[0] + props.moments[1] + props.moments[2]);
    RotMatrix S = RotMatrix::FromRowValues(
        half - props.moments[0], props.products[0], props.products[1],
        props.products[0], half - props.moments[1], props.products[2],
        props.products[1], props.products[2], half - props.moments[2]);

    RotMatrix A = entity->m_transform.Matrix();
    double const det = std::fabs(A.Determinant());
    RotMatrix AT;
    AT.TransposeOf(A);
    RotMatrix world;
    world.InitProduct(A, S, AT);
    world.ScaleColumns(world, det, det, det);

    Moments m;
    m.amount = props.volume * det;
    m.centroid = DPoint3d::FromArray(props.centroid);
    entity->m_transform.Multiply(m.centroid);
    m.xx = world.form3d[0][0]; m.yy = world.form3d[1][1]; m.zz = world.form3d[2][2];
    m.xy = world.form3d[0][1]; m.xz = world.form3d[0][2]; m.yz = world.form3d[1][2];

    volume = m.amount;
    area = props.area * std::pow(det, 2.0 / 3.0);
    centroid = m.centroid;
    m.ToOutputs(moments, iXY, iXZ, iYZ);
    return SUCCESS;
}

/*---------------------------------------------------------------------------------**//**
* createBRepGeometry
+---------------+---------------+---------------+---------------+---------------+------*/
namespace {

//! A row-major 3x4 matrix of a transform.
void ToMatrix(TransformCR transform, double matrix[3][4]) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j)
            matrix[i][j] = transform.form3d[i][j];
}

//! A body for a right box, full sphere, right circular cone or cylinder, or full torus. Null for
//! other primitives.
SmKernel::BodyPtr BodyFromSolidPrimitive(ISolidPrimitiveCR primitive) {
    double const tolerance = Angle::SmallAngle();
    DgnBoxDetail box;
    DgnSphereDetail sphere;
    DgnConeDetail cone;
    DgnTorusPipeDetail torus;
    if (primitive.TryGetDgnBoxDetail(box)) {
        DVec3d x = box.m_vectorX, y = box.m_vectorY;
        if (!x.Normalize() || !y.Normalize() || !x.IsPerpendicularTo(y) || !box.m_capped ||
            std::fabs(box.m_baseX - box.m_topX) > tolerance * box.m_baseX || std::fabs(box.m_baseY - box.m_topY) > tolerance * box.m_baseY)
            return nullptr;
        DVec3d z = DVec3d::FromCrossProduct(x, y);
        DVec3d rise = DVec3d::FromStartEnd(box.m_baseOrigin, box.m_topOrigin);
        double height = rise.DotProduct(z);
        if (height <= 0.0 || !rise.IsParallelTo(z))
            return nullptr;
        double const origin[3] = {0, 0, 0}, size[3] = {box.m_baseX, box.m_baseY, height};
        auto body = SmKernel::CreateBox(origin, size);
        double matrix[3][4];
        ToMatrix(Transform::FromOriginAndVectors(box.m_baseOrigin, x, y, z), matrix);
        return body && SmKernel::Transform(*body, matrix) ? std::move(body) : nullptr;
    }
    if (primitive.TryGetDgnSphereDetail(sphere)) {
        DPoint3d center;
        RotMatrix axes;
        double radius;
        if (!sphere.IsTrueSphere(center, axes, radius))
            return nullptr;
        double const c[3] = {center.x, center.y, center.z};
        return SmKernel::CreateSphere(c, radius);
    }
    if (primitive.TryGetDgnConeDetail(cone)) {
        DVec3d axis = DVec3d::FromStartEnd(cone.m_centerA, cone.m_centerB);
        if (!cone.m_capped || !axis.IsPerpendicularTo(cone.m_vector0) || !axis.IsPerpendicularTo(cone.m_vector90))
            return nullptr;
        double const a[3] = {cone.m_centerA.x, cone.m_centerA.y, cone.m_centerA.z}, b[3] = {cone.m_centerB.x, cone.m_centerB.y, cone.m_centerB.z};
        return SmKernel::CreateCone(a, b, cone.m_radiusA, cone.m_radiusB);
    }
    if (primitive.TryGetDgnTorusPipeDetail(torus)) {
        if (std::fabs(std::fabs(torus.m_sweepAngle) - Angle::TwoPi()) > tolerance)
            return nullptr;
        DVec3d axis = DVec3d::FromCrossProduct(torus.m_vectorX, torus.m_vectorY);
        double const c[3] = {torus.m_center.x, torus.m_center.y, torus.m_center.z}, n[3] = {axis.x, axis.y, axis.z};
        return SmKernel::CreateTorus(c, n, torus.m_majorRadius, torus.m_minorRadius);
    }
    return nullptr;
}

//! Polylines become one curve per segment so their corners are vertices.
bool AddPrimitive(ICurvePrimitiveCR primitive, SmKernel::Curves& out) {
    if (auto points = primitive.GetLineStringCP()) {
        for (size_t i = 0; i + 1 < points->size(); ++i) {
            DPoint3dCR a = (*points)[i], b = (*points)[i + 1];
            if (a.IsEqual(b))
                continue;
            SmKernel::Curve line;
            line.poles = {a.x, a.y, a.z, b.x, b.y, b.z};
            line.knots = {0, 0, 1, 1};
            out.push_back(std::move(line));
        }
        return true;
    }
    MSBsplineCurvePtr bspline = primitive.GetMSBsplineCurvePtr();
    if (bspline.IsNull() || (bspline->IsClosed() && MSB_SUCCESS != bspline->MakeOpen(0.0)))
        return false;
    SmKernel::Curve curve;
    curve.degree = bspline->GetIntOrder() - 1;
    bvector<DPoint3d> poles;
    bspline->GetUnWeightedPoles(poles);
    for (DPoint3dCR pole : poles)
        curve.poles.insert(curve.poles.end(), {pole.x, pole.y, pole.z});
    if (bspline->rational) {
        bvector<double> weights;
        bspline->GetWeights(weights);
        curve.weights.assign(weights.begin(), weights.end());
    }
    bvector<double> knots;
    bspline->GetKnots(knots);
    curve.knots.assign(knots.begin(), knots.end());
    out.push_back(std::move(curve));
    return true;
}

bool ToCurves(CurveVectorCR chain, SmKernel::Curves& out) {
    for (auto const& primitive : chain) {
        if (auto child = primitive->GetChildCurveVectorCP()) {
            if (!ToCurves(*child, out))
                return false;
        } else if (!AddPrimitive(*primitive, out)) {
            return false;
        }
    }
    return !out.empty();
}

//! The loops of a region, or the one chain of a path.
bool ToLoops(CurveVectorCR curves, std::vector<SmKernel::Curves>& loops) {
    if (curves.IsParityRegion()) {
        for (auto const& primitive : curves) {
            auto loop = primitive->GetChildCurveVectorCP();
            if (nullptr == loop || !ToLoops(*loop, loops))
                return false;
        }
        return !loops.empty();
    }
    if (!curves.IsClosedPath() && !curves.IsOpenPath())
        return false;
    loops.emplace_back();
    return ToCurves(curves, loops.back());
}

void Translate(std::vector<SmKernel::Curves>& loops, DVec3dCR shift) {
    for (auto& loop : loops)
        for (auto& curve : loop)
            for (size_t i = 0; i < curve.poles.size(); i += 3) {
                curve.poles[i] += shift.x;
                curve.poles[i + 1] += shift.y;
                curve.poles[i + 2] += shift.z;
            }
}

//! One createBRepGeometry input: a body, or curves that become a sheet body when one is needed.
struct Input {
    SmKernel::BodyPtr m_body;
    CurveVectorPtr m_curves;

    bool IsRegion() const { return m_curves.IsValid() && m_curves->IsAnyRegionType(); }

    SmKernel::Kind Kind() {
        if (m_body)
            return SmKernel::GetKind(*m_body);
        return IsRegion() ? SmKernel::Kind::Sheet : SmKernel::Kind::Wire;
    }

    //! The body, made from region curves if needed. Null for open curves.
    SmKernel::BodyPtr TakeBody() {
        if (!m_body && IsRegion()) {
            std::vector<SmKernel::Curves> loops;
            if (ToLoops(*m_curves, loops))
                m_body = SmKernel::CreatePlanarSheet(loops);
        }
        return std::move(m_body);
    }

    bool GetLoops(std::vector<SmKernel::Curves>& loops) const { return m_curves.IsValid() && ToLoops(*m_curves, loops); }

    //! The plane and orientation of planar region curves.
    bool GetPlane(DPoint3dR origin, DVec3dR normal) const {
        double area;
        return IsRegion() && m_curves->CentroidNormalArea(origin, normal, area) && normal.Normalize() > 0.0;
    }
};

bool ReadInput(GeometryStreamIO::Reader const& reader, GeometryStreamIO::Operation const& op, Input& input) {
    GeometricPrimitivePtr geometry;
    if (!reader.Get(op, geometry) || geometry.IsNull())
        return false;
    if (auto curves = geometry->GetAsCurveVector(); curves.IsValid()) {
        input.m_curves = curves;
        return true;
    }
    if (auto curve = geometry->GetAsICurvePrimitive(); curve.IsValid()) {
        input.m_curves = CurveVector::Create(CurveVector::BOUNDARY_TYPE_Open, curve);
        return true;
    }
    if (auto entity = geometry->GetAsIBRepEntity(); entity.IsValid()) {
        auto smEntity = SMLibEntity::From(*entity);
        if (nullptr == smEntity)
            return false;
        input.m_body = SmKernel::Copy(smEntity->Body());
        double matrix[3][4];
        ToMatrix(smEntity->m_transform, matrix);
        return input.m_body && SmKernel::Transform(*input.m_body, matrix);
    }
    if (auto primitive = geometry->GetAsISolidPrimitive(); primitive.IsValid()) {
        input.m_body = BodyFromSolidPrimitive(*primitive);
        return nullptr != input.m_body;
    }
    return false;
}

DRange3d RangeOf(SmKernel::Body const& body) {
    double low[3], high[3];
    if (!SmKernel::GetRange(body, low, high))
        return DRange3d::NullRange();
    return DRange3d::From(DPoint3d::FromArray(low), DPoint3d::FromArray(high));
}

//! Signed distances of the range's corners from a plane.
void Extent(DRange3dCR range, DPoint3dCR origin, DVec3dCR normal, double& low, double& high) {
    DPoint3d corners[8];
    range.Get8Corners(corners);
    low = DBL_MAX;
    high = -DBL_MAX;
    for (DPoint3dCR corner : corners) {
        double const d = normal.DotProduct(DVec3d::FromStartEnd(origin, corner));
        low = std::min(low, d);
        high = std::max(high, d);
    }
}

SmKernel::BodyPtr Extrude(std::vector<SmKernel::Curves> const& loops, DVec3dCR direction, double distance, bool cap = true) {
    double const d[3] = {direction.x, direction.y, direction.z};
    return SmKernel::Extrude(loops, d, distance, cap);
}

//! Booleans of the target with the tools, where intersecting with several tools intersects with
//! their union. A planar sheet tool is a half space bounded by its plane, with the material on the
//! side opposite its normal.
DgnDbStatus Combine(std::vector<Input>& inputs, SmKernel::BooleanOp op, SmKernel::BodyPtr& result) {
    if (inputs.size() < 2)
        return DgnDbStatus::BadArg;
    if (SmKernel::BooleanOp::Intersect == op && inputs.size() > 2) {
        SmKernel::BodyPtr tools;
        for (size_t i = 1; i < inputs.size(); ++i) {
            if (SmKernel::Kind::Solid != inputs[i].Kind())
                return DgnDbStatus::BadArg;
            auto tool = inputs[i].TakeBody();
            if (!tools)
                tools = std::move(tool);
            else if (!SmKernel::Boolean(tools, std::move(tool), SmKernel::BooleanOp::Unite))
                return DgnDbStatus::BadRequest;
        }
        inputs.resize(2);
        inputs[1].m_body = std::move(tools);
    }
    result = inputs[0].TakeBody();
    if (!result)
        return DgnDbStatus::BadArg;
    for (size_t i = 1; i < inputs.size(); ++i) {
        if (SmKernel::Kind::Sheet == inputs[i].Kind() && SmKernel::Kind::Solid == SmKernel::GetKind(*result)) {
            if (SmKernel::BooleanOp::Unite == op)
                return DgnDbStatus::BadArg;
            auto sheet = inputs[i].TakeBody();
            double origin[3], normal[3];
            if (!sheet || !SmKernel::GetPlane(*sheet, origin, normal))
                return DgnDbStatus::NotEnabled; // only planar sheets
            if (SmKernel::BooleanOp::Intersect == op)
                normal[0] = -normal[0], normal[1] = -normal[1], normal[2] = -normal[2];
            if (!SmKernel::KeepHalfSpace(result, origin, normal))
                return DgnDbStatus::BadRequest;
            continue;
        }
        auto tool = inputs[i].TakeBody();
        if (!tool)
            return DgnDbStatus::BadArg;
        if (!SmKernel::Boolean(result, std::move(tool), op))
            return DgnDbStatus::BadRequest;
    }
    return DgnDbStatus::Success;
}

//! Remove the material swept by a planar region along its normal, through the target or to a depth.
DgnDbStatus Cut(std::vector<Input>& inputs, Napi::Object const& parameters, SmKernel::BodyPtr& result) {
    std::vector<SmKernel::Curves> loops;
    DPoint3d origin;
    DVec3d normal;
    if (inputs.size() != 2 || !inputs[1].GetLoops(loops) || !inputs[1].GetPlane(origin, normal))
        return DgnDbStatus::BadArg;
    result = inputs[0].TakeBody();
    if (!result || SmKernel::Kind::Solid != SmKernel::GetKind(*result))
        return DgnDbStatus::BadArg;

    auto distanceValue = parameters.Get("distance");
    bool const both = parameters.Get("bothDirections").ToBoolean();
    double distance;
    if (distanceValue.IsNumber() && distanceValue.ToNumber().DoubleValue() > 0.0) {
        distance = distanceValue.ToNumber().DoubleValue();
    } else {
        double low, high;
        Extent(RangeOf(*result), origin, normal, low, high);
        distance = std::max(std::fabs(low), std::fabs(high)) * 1.1 + 1.0;
    }
    if (both)
        Translate(loops, DVec3d::FromScale(normal, -distance));
    auto tool = Extrude(loops, normal, both ? 2.0 * distance : distance);
    if (!tool || !SmKernel::Boolean(result, std::move(tool), SmKernel::BooleanOp::Subtract))
        return DgnDbStatus::BadRequest;
    return DgnDbStatus::Success;
}

//! Move a planar region against its normal until it reaches the target: a pad where the region
//! starts outside a solid, a pocket where it starts inside, a dent in a planar sheet.
DgnDbStatus Emboss(std::vector<Input>& inputs, SmKernel::BodyPtr& result) {
    std::vector<SmKernel::Curves> loops;
    DPoint3d origin;
    DVec3d normal;
    if (inputs.size() != 2 || !inputs[1].GetLoops(loops) || !inputs[1].GetPlane(origin, normal))
        return DgnDbStatus::BadArg;
    DVec3d const down = DVec3d::FromScale(normal, -1.0);

    if (SmKernel::Kind::Sheet == inputs[0].Kind()) {
        std::vector<SmKernel::Curves> targetLoops;
        DPoint3d targetOrigin;
        DVec3d targetNormal;
        if (!inputs[0].GetLoops(targetLoops) || !inputs[0].GetPlane(targetOrigin, targetNormal))
            return DgnDbStatus::NotEnabled; // only planar regions
        double const height = normal.DotProduct(DVec3d::FromStartEnd(targetOrigin, origin));
        if (!targetNormal.IsParallelTo(normal) || height <= 0.0)
            return DgnDbStatus::BadArg;
        auto walls = Extrude(loops, down, height, false);
        auto cap = SmKernel::CreatePlanarSheet(loops);
        auto projected = loops;
        Translate(projected, DVec3d::FromScale(down, height));
        targetLoops.insert(targetLoops.end(), projected.begin(), projected.end());
        auto holed = SmKernel::CreatePlanarSheet(targetLoops);
        if (!walls || !cap || !holed)
            return DgnDbStatus::BadRequest;
        std::vector<SmKernel::BodyPtr> sheets;
        sheets.push_back(std::move(holed));
        sheets.push_back(std::move(walls));
        sheets.push_back(std::move(cap));
        result = SmKernel::Sew(std::move(sheets));
        return result ? DgnDbStatus::Success : DgnDbStatus::BadRequest;
    }

    result = inputs[0].TakeBody();
    if (!result || SmKernel::Kind::Solid != SmKernel::GetKind(*result))
        return DgnDbStatus::BadArg;
    double low, high;
    Extent(RangeOf(*result), origin, normal, low, high);
    double const tolerance = 1.0e-8 * (high - low);
    SmKernel::BooleanOp op;
    double depth;
    if (high <= tolerance) {
        op = SmKernel::BooleanOp::Unite; // into the middle of the target, so the pad joins it
        depth = -high + 0.5 * (high - low);
    } else if (low < -tolerance) {
        op = SmKernel::BooleanOp::Subtract;
        depth = -low * 1.1 + tolerance;
    } else {
        return DgnDbStatus::BadArg; // the target is in front of the region
    }
    auto tool = Extrude(loops, down, depth);
    if (!tool || !SmKernel::Boolean(result, std::move(tool), op))
        return DgnDbStatus::BadRequest;
    return DgnDbStatus::Success;
}

double NumberOf(Napi::Object const& parameters, char const* name, double fallback) {
    auto value = parameters.Get(name);
    return value.IsNumber() ? value.ToNumber().DoubleValue() : fallback;
}

DgnDbStatus Apply(int operation, std::vector<Input>& inputs, Napi::Object const& parameters, SmKernel::BodyPtr& result) {
    if (inputs.empty())
        return DgnDbStatus::BadArg;
    auto single = [&](SmKernel::Kind kind) -> bool {
        if (1 != inputs.size() || kind != inputs[0].Kind())
            return false;
        result = inputs[0].TakeBody();
        return nullptr != result;
    };
    switch (operation) {
        case 0: return Combine(inputs, SmKernel::BooleanOp::Unite, result);
        case 1: return Combine(inputs, SmKernel::BooleanOp::Subtract, result);
        case 2: return Combine(inputs, SmKernel::BooleanOp::Intersect, result);
        case 3: { // Sew
            std::vector<SmKernel::BodyPtr> sheets;
            for (auto& input : inputs) {
                if (SmKernel::Kind::Sheet != input.Kind())
                    return DgnDbStatus::BadArg;
                sheets.push_back(input.TakeBody());
                if (!sheets.back())
                    return DgnDbStatus::BadArg;
            }
            result = SmKernel::Sew(std::move(sheets));
            break;
        }
        case 4: return Cut(inputs, parameters, result);
        case 5: return Emboss(inputs, result);
        case 6: { // Thicken
            double const front = std::max(0.0, NumberOf(parameters, "frontDistance", 0.0)), back = std::max(0.0, NumberOf(parameters, "backDistance", 0.0));
            if (!single(SmKernel::Kind::Sheet) || front + back <= 0.0)
                return DgnDbStatus::BadArg;
            if (!SmKernel::Thicken(result, front, back))
                result.reset();
            break;
        }
        case 7: { // Hollow
            double const distance = NumberOf(parameters, "distance", 0.0);
            if (!single(SmKernel::Kind::Solid) || 0.0 == distance)
                return DgnDbStatus::BadArg;
            if (!SmKernel::Hollow(result, distance))
                result.reset();
            break;
        }
        case 8: { // Sweep
            std::vector<SmKernel::Curves> profile, path;
            if (2 != inputs.size() || !inputs[0].GetLoops(profile) || 1 != profile.size() || !inputs[1].GetLoops(path) || 1 != path.size())
                return DgnDbStatus::BadArg;
            result = SmKernel::Sweep(profile.front(), path.front());
            break;
        }
        case 9: { // Loft
            std::vector<SmKernel::Curves> profiles;
            for (auto const& input : inputs) {
                std::vector<SmKernel::Curves> loops;
                if (!input.GetLoops(loops) || 1 != loops.size())
                    return DgnDbStatus::BadArg;
                profiles.push_back(std::move(loops.front()));
            }
            result = SmKernel::Loft(profiles);
            break;
        }
        case 10: { // Round
            double const radius = NumberOf(parameters, "radius", 0.0);
            if (radius <= 0.0 || 1 != inputs.size() || !(result = inputs[0].TakeBody()))
                return DgnDbStatus::BadArg;
            if (!SmKernel::Round(result, radius))
                result.reset();
            break;
        }
        case 11: { // Offset
            double const distance = NumberOf(parameters, "distance", 0.0);
            if (0.0 == distance || 1 != inputs.size() || !(result = inputs[0].TakeBody()))
                return DgnDbStatus::BadArg;
            if (!SmKernel::Offset(result, distance))
                result.reset();
            break;
        }
        default:
            return DgnDbStatus::BadArg;
    }
    return result ? DgnDbStatus::Success : DgnDbStatus::BadRequest;
}

} // namespace

DgnDbStatus SMLibKernelAdmin::_CreateBRepGeometry(DgnDbR db, Napi::Object const& createProps, Napi::Env env) const {
    auto onResult = createProps.Get("onResult");
    auto entries = createProps.Get("entryArray");
    if (!onResult.IsFunction() || !entries.IsArray())
        return DgnDbStatus::BadArg;
    auto parametersValue = createProps.Get("parameters");
    Napi::Object parameters = parametersValue.IsObject() ? parametersValue.As<Napi::Object>() : Napi::Object::New(env);

    auto entryArray = entries.As<Napi::Array>();
    GeometryStreamIO::Reader reader(db);
    std::vector<IBRepEntityPtr> entities;
    {
        Lock lock(SmKernel::Mutex());
        std::vector<Input> inputs(entryArray.Length());
        for (uint32_t i = 0; i < entryArray.Length(); ++i) {
            if (!ReadInput(reader, GeometryStreamIO::ToOperation(entryArray.Get(i)), inputs[i]))
                return DgnDbStatus::BadArg;
        }
        SmKernel::BodyPtr result;
        DgnDbStatus status = Apply(createProps.Get("operation").ToNumber().Int32Value(), inputs, parameters, result);
        if (DgnDbStatus::Success != status)
            return status;

        std::vector<SmKernel::BodyPtr> bodies;
        if (createProps.Get("separateDisjoint").ToBoolean())
            bodies = SmKernel::Separate(std::move(result));
        else
            bodies.push_back(std::move(result));
        for (auto& body : bodies) {
            if (body && SmKernel::GetFaceCount(*body) > 0) // an empty result is a success with no geometry
                entities.push_back(new SMLibEntity(Share(std::move(body)), Transform::FromIdentity()));
        }
    }

    Napi::Array resultEntries = Napi::Array::New(env);
    uint32_t index = 0;
    for (auto const& entity : entities) {
        GeometryStreamIO::Writer writer(db);
        writer.Append(*entity);
        for (auto const& egOp : GeometryStreamIO::Collection(writer.m_buffer.data(), writer.m_buffer.size())) {
            if (GeometryStreamIO::OpCode::Header != egOp.m_opCode)
                resultEntries.Set(index++, GeometryStreamIO::FromOperation(egOp, env));
        }
    }

    auto info = Napi::Object::New(env);
    info.Set("entryArray", resultEntries);
    onResult.As<Napi::Function>().Call({info});
    return DgnDbStatus::Success;
}
