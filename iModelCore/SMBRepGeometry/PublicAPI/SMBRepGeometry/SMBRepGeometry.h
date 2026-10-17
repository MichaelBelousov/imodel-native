/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
#pragma once

#include <DgnPlatform/PlatformLib.h>

BEGIN_BENTLEY_DGN_NAMESPACE

//=======================================================================================
//! A BRep geometry admin backed by the open-source SMLib solid modeling kernel.
//!
//! B-reps are persisted in SMLib's own format under GeometryStreamIO::OpCode::ParasolidBRep, tagged
//! so they are never mistaken for Parasolid data. Parasolid b-reps in existing iModels are rebuilt
//! in SMLib from their transmit data when restored.
//=======================================================================================
struct SMLibKernelAdmin : PlatformLib::Host::BRepGeometryAdmin
{
    void _StartSession() const override;
    void _StopSession() const override;
    bool _IsSessionStarted() const override;

    BentleyStatus _RestoreEntityFromMemory(IBRepEntityPtr& out, uint8_t const* buffer, size_t bufferSize, TransformCR transform) const override;
    BentleyStatus _SaveEntityToMemory(uint8_t** buffer, size_t& bufferSize, IBRepEntityCR in, bool checkConsistency = true) const override;

    PolyfaceHeaderPtr _FacetEntity(IBRepEntityCR entity, IFacetOptionsCR facetOptions) const override;
    bool _FacetEntity(IBRepEntityCR entity, bvector<PolyfaceHeaderPtr>& polyfaces, bvector<FaceAttachment>& params, IFacetOptionsCR facetOptions) const override;

    BentleyStatus _GetAreaProperties(IBRepEntityCR entity, double& area, double& perimeter, DPoint3dR centroid, DPoint3dR moments, double& iXY, double& iXZ, double& iYZ, double tolerance) const override;
    BentleyStatus _GetVolumeProperties(IBRepEntityCR entity, double& volume, double& area, DPoint3dR centroid, DPoint3dR moments, double& iXY, double& iXZ, double& iYZ, double tolerance) const override;

    DgnDbStatus _CreateBRepGeometry(DgnDbR db, Napi::Object const& createProps, Napi::Env env) const override;
};

END_BENTLEY_DGN_NAMESPACE
