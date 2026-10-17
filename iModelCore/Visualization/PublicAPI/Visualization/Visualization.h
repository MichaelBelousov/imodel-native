/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
#pragma once

#include <DgnPlatform/Visualization.h>

// Open-source stand-in for Bentley's private Visualization module. Produces iMdl tiles for 3d
// spatial models; element graphics requests report that there is nothing to show.
BentleyApi::Dgn::Visualization* iTwinVisualization_create(BentleyApi::Dgn::Render::System* renderSystem);
