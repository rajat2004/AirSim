#include "RenderRequest.h"
#include "TextureResource.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Async/TaskGraphInterfaces.h"
#include "ImageUtils.h"

#include "AirBlueprintLib.h"
#include "Async/Async.h"

RenderRequest::RenderRequest(UGameViewportClient * game_viewport, std::function<void()>&& query_camera_pose_cb)
    : params_(nullptr), result_(nullptr), wait_signal_(new msr::airlib::WorkerThreadSignal),
    game_viewport_(game_viewport), query_camera_pose_cb_(std::move(query_camera_pose_cb))
{
}

RenderRequest::~RenderRequest()
{
}

// read pixels from render target using render thread, then compress the result into PNG
// argument on the thread that calls this method.
void RenderRequest::getScreenshot(std::shared_ptr<RenderParams> params, std::shared_ptr<RenderResult> result, bool use_safe_method)
{
    //TODO: is below really needed?
    /*for (unsigned int i = 0; i < req_size; ++i) {
        results.push_back(std::make_shared<RenderResult>());

        if (!params[i]->pixels_as_float)
            results[i]->bmp.Reset();
        else
            results[i]->bmp_float.Reset();
        results[i]->time_stamp = 0;
    }*/
    if (!params->pixels_as_float)
        result->bmp.Reset();
    else
        result->bmp_float.Reset();
    result->time_stamp = 0;

    //make sure we are not on the rendering thread
    CheckNotBlockedOnRenderThread();

    if (use_safe_method) {
        //for (unsigned int i = 0; i < req_size; ++i) {
            //TODO: below doesn't work right now because it must be running in game thread
            FIntPoint img_size;
            if (!params->pixels_as_float) {
                //below is documented method but more expensive because it forces flush
                FTextureRenderTargetResource* rt_resource = params->render_target->GameThread_GetRenderTargetResource();
                auto flags = setupRenderResource(rt_resource, params.get(), result.get(), img_size);
                rt_resource->ReadPixels(result->bmp, flags);
            }
            else {
                FTextureRenderTargetResource* rt_resource = params->render_target->GetRenderTargetResource();
                setupRenderResource(rt_resource, params.get(), result.get(), img_size);
                rt_resource->ReadFloat16Pixels(result->bmp_float);
            }
        //}
    }
    else {
        //wait for render thread to pick up our task
        params_ = params;
        result_ = result;
        //req_size_ = req_size;

        // Queue up the task of querying camera pose in the game thread and synchronizing render thread with camera pose
        AsyncTask(ENamedThreads::GameThread, [this]() {
            check(IsInGameThread());

            saved_DisableWorldRendering_ = game_viewport_->bDisableWorldRendering;
            game_viewport_->bDisableWorldRendering = 0;
            end_draw_handle_ = game_viewport_->OnEndDraw().AddLambda([this] {
                check(IsInGameThread());

                // capture CameraPose for this frame
                //query_camera_pose_cb_();

                // The completion is called immeidately after GameThread sends the
                // rendering commands to RenderThread. Hence, our ExecuteTask will
                // execute *immediately* after RenderThread renders the scene!
                RenderRequest* This = this;
                ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)(
                [This](FRHICommandListImmediate& RHICmdList)
                {
                    This->ExecuteTask();
                });

                game_viewport_->bDisableWorldRendering = saved_DisableWorldRendering_;

                assert(end_draw_handle_.IsValid());
                game_viewport_->OnEndDraw().Remove(end_draw_handle_);
            });

            // while we're still on GameThread, enqueue request for capture the scene!
            //for (unsigned int i = 0; i < req_size_; ++i) {
                params_->render_component->CaptureSceneDeferred();
            //}
        });

        // wait for this task to complete
        while (!wait_signal_->waitFor(5)) {
            // log a message and continue wait
            // lamda function still references a few objects for which there is no refcount.
            // Walking away will cause memory corruption, which is much more difficult to debug.
            UE_LOG(LogTemp, Warning, TEXT("Failed: timeout waiting for screenshot"));
        }
    }

    //for (unsigned int i = 0; i < req_size; ++i) {
        if (!params->pixels_as_float) {
            if (result->width != 0 && result->height != 0) {
                result->image_data_uint8.SetNumUninitialized(result->width * result->height * 3, false);
                if (params->compress)
                    UAirBlueprintLib::CompressImageArray(result->width, result->height, result->bmp, result->image_data_uint8);
                else {
                    uint8* ptr = result->image_data_uint8.GetData();
                    for (const auto& item : result->bmp) {
                        *ptr++ = item.B;
                        *ptr++ = item.G;
                        *ptr++ = item.R;
                    }
                }
            }
        }
        else {
            result->image_data_float.SetNumUninitialized(result->width * result->height);
            float* ptr = result->image_data_float.GetData();
            for (const auto& item : result->bmp_float) {
                *ptr++ = item.R.GetFloat();
            }
        }
    //}
}

FReadSurfaceDataFlags RenderRequest::setupRenderResource(const FTextureRenderTargetResource* rt_resource, const RenderParams* params, RenderResult* result, FIntPoint& size)
{
    size = rt_resource->GetSizeXY();
    result->width = size.X;
    result->height = size.Y;
    FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);
    flags.SetLinearToGamma(false);

    return flags;
}

void RenderRequest::ExecuteTask()
{
    if (params_ != nullptr)
    {
        //for (unsigned int i = 0; i < req_size_; ++i) {
            FRHICommandListImmediate& RHICmdList = GetImmediateCommandList_ForRenderCommand();
            auto rt_resource = params_->render_target->GetRenderTargetResource();
            if (rt_resource != nullptr) {
                const FTexture2DRHIRef& rhi_texture = rt_resource->GetRenderTargetTexture();
                FIntPoint size;
                auto flags = setupRenderResource(rt_resource, params_.get(), result_.get(), size);

                //should we be using ENQUEUE_UNIQUE_RENDER_COMMAND_ONEPARAMETER which was in original commit by @saihv
                //https://github.com/Microsoft/AirSim/pull/162/commits/63e80c43812300a8570b04ed42714a3f6949e63f#diff-56b790f9394f7ca1949ddbb320d8456fR64
                if (!params_->pixels_as_float) {
                    //below is undocumented method that avoids flushing, but it seems to segfault every 2000 or so calls
                    RHICmdList.ReadSurfaceData(
                        rhi_texture,
                        FIntRect(0, 0, size.X, size.Y),
                        result_->bmp,
                        flags);
                }
                else {
                    RHICmdList.ReadSurfaceFloatData(
                        rhi_texture,
                        FIntRect(0, 0, size.X, size.Y),
                        result_->bmp_float,
                        CubeFace_PosX, 0, 0
                    );
                }
            }

            result_->time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
        //}

        //req_size_ = 0;
        params_ = nullptr;
        result_ = nullptr;

        wait_signal_->signal();
    }
}
