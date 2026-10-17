/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
#pragma once

// Shared by the SmKernel sources, which see SMLib's headers. Nothing else may include this.

#include "SmKernel.h"

#include <SmBrep.h>
#include <SmBSplineCurve.h>
#include <SmContext.h>

#include <string>

namespace SmKernel {

struct Body {
    SmBrep* m_brep;
    explicit Body(SmBrep* brep) : m_brep(brep) { }
    ~Body() { delete m_brep; }
};

SmContext& Context();
BodyPtr Wrap(SmBrep* brep);
SmBSplineCurve* ToSmCurve(Curve const& curve);

//! Rebuild a body from Parasolid transmit data, `scale` times the size the data describes. Null if
//! the data isn't Parasolid or the body can't be rebuilt. Problems are appended to `log` if given.
BodyPtr RestoreParasolid(uint8_t const* data, size_t size, std::vector<uint32_t>& faceTags, double& scale, std::string* log = nullptr);

} // namespace SmKernel
