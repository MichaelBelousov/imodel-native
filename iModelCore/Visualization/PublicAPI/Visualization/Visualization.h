/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
#pragma once

#include <DgnPlatform/Visualization.h>

// Open-source stand-in for Bentley's private Visualization module. Produces no tiles and no
// element graphics; requests for either report that there is nothing to show.
BentleyApi::Dgn::Visualization* iTwinVisualization_create(BentleyApi::Dgn::Render::System* renderSystem);
