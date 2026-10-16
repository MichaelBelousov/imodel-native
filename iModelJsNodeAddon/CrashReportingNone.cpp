/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
// Crash reporting for builds without crashpad. Matches the inline fallbacks IModelJsNative.h uses
// on platforms where BENTLEYCONFIG_CRASHPAD is not defined.
#include "IModelJsNative.h"

using namespace IModelJsNative;

bool JsInterop::InitializeCrashReporting(CrashReportingConfig const&) { return true; }

void JsInterop::SetCrashReportProperty(Utf8CP, Utf8CP) { }

std::map<std::string, std::string> JsInterop::GetCrashReportProperties() { return {}; }
