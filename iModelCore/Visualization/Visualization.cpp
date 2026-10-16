/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
#include <Visualization/Visualization.h>

USING_NAMESPACE_BENTLEY_DGN

namespace {

// Must match ElementGraphicsStatus in iModelJsNodeAddon/api_package/ts/src/NativeLibrary.ts.
constexpr int ElementGraphicsStatus_NoGeometry = 2;

struct StubElementGraphicsRequests : ElementGraphicsRequests {
  void Terminate() override { }
  void Cancel(bvector<Utf8String> const&) override { }

  Napi::Value Enqueue(BeJsConst, Napi::Env env) override {
    auto deferred = Napi::Promise::Deferred::New(env);
    auto result = Napi::Object::New(env);
    result.Set("status", Napi::Number::New(env, ElementGraphicsStatus_NoGeometry));
    deferred.Resolve(result);
    return deferred.Promise();
  }
};

struct StubVisualization : Visualization {
  explicit StubVisualization(Render::System* renderSystem) : Visualization(renderSystem) { }

  ElementGraphicsRequestsUPtr CreateElementGraphicsRequests(DgnDbR) override {
    return std::make_unique<StubElementGraphicsRequests>();
  }
};

} // namespace

Visualization* iTwinVisualization_create(Render::System* renderSystem) {
  return new StubVisualization(renderSystem);
}
