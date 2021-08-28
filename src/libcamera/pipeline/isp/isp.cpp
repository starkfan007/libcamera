/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Siyuan Fan <siyuan.fan@foxmail.com>
 *
 * isp.cpp - The software ISP-based pipeline handler
 */

#include "../../swisp/cpu/isp.h"

#include <math.h>
#include <queue>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>

#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(ISP)

class ISPCameraData : public CameraData
{
public:
       ISPCameraData(PipelineHandler *pipe, MediaDevice *media)
               : CameraData(pipe), media_(media), video_(nullptr)
       {
       }

       ~ISPCameraData()
       {
               delete video_;
       }

       int init();
       void bufferReady(FrameBuffer *buffer);
       void ISPCompleted(FrameBuffer *rawbuffer, FrameBuffer *rgbBuffer);

       Stream stream_;
       ISPCPU isp_;
       int width_;
       int height_;

       std::vector<std::unique_ptr<FrameBuffer>> rawBuffers_;
       std::queue<FrameBuffer *> rawBufferQueue_;
       std::queue<FrameBuffer *> rgbBufferQueue_;

       MediaDevice *media_;
       V4L2VideoDevice *video_;
};


class ISPCameraConfiguration : public CameraConfiguration
{
public:
       ISPCameraConfiguration();

       Status validate() override;
};

class PipelineHandlerISP : public PipelineHandler
{
public:
       PipelineHandlerISP(CameraManager *manager);

       CameraConfiguration *generateConfiguration(Camera *camera,
                                                   const StreamRoles &roles) override;
       int configure(Camera *camera, CameraConfiguration *config) override;

       int exportFrameBuffers(Camera *camera, Stream *stream,
                               std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

       int start(Camera *camera, const ControlList *controls) override;
       void stop(Camera *camera) override;

       int queueRequestDevice(Camera *camera, Request *request) override;

       bool match(DeviceEnumerator *enumerator) override;

private:
       ISPCameraData *cameraData(const Camera *camera)
       {
               return static_cast<ISPCameraData *>(
                       PipelineHandler::cameraData(camera));
       }
};

ISPCameraConfiguration::ISPCameraConfiguration()
       : CameraConfiguration()
{
}

CameraConfiguration::Status ISPCameraConfiguration::validate()
{
       Status status = Valid;

       // if (config_.empty())
       //        return Invalid;

       // if (config_.size() > 1) {
       //        config_.resize(1);
       //        status = Adjusted;
       // }

       // StreamConfiguration &cfg = config_[0];
       // const std::vector<libcamera::PixelFormat> formats = cfg.formats().pixelformats();
       // if (std::find(formats.begin(), formats.end(), cfg.pixelFormat) == formats.end()) {
       //        cfg.pixelFormat = formats::RGB888;
       //        LOG(ISP, Debug) << "Adjusting format to " << cfg.pixelFormat.toString();
       //        status = Adjusted;
       // }

       // cfg.bufferCount = 4;

       return status;
}

PipelineHandlerISP::PipelineHandlerISP(CameraManager *manager)
       : PipelineHandler(manager)
{
}

CameraConfiguration *PipelineHandlerISP::generateConfiguration(Camera *camera,
                                                               const StreamRoles &roles)
{
       ISPCameraData *data = cameraData(camera);
       CameraConfiguration *config = new ISPCameraConfiguration();

       if (roles.empty())
              return config;

       std::map<V4L2PixelFormat, std::vector<SizeRange>> v4l2Formats =
              data->video_->formats();
       std::map<PixelFormat, std::vector<SizeRange>> deviceFormats;
       std::transform(v4l2Formats.begin(), v4l2Formats.end(),
              std::inserter(deviceFormats, deviceFormats.begin()),
              [&](const decltype(v4l2Formats)::value_type &format) {
                  return decltype(deviceFormats)::value_type{
                     format.first.toPixelFormat(),
                     format.second
                  };
              });

       StreamFormats formats(deviceFormats);
       StreamConfiguration cfg(formats);

       cfg.pixelFormat = formats::RGB888;
       cfg.size = { 640, 480 };
       cfg.bufferCount = 4;

       config->addConfiguration(cfg);

       config->validate();

       return config;
}

int PipelineHandlerISP::configure(Camera *camera, CameraConfiguration *config)
{
       ISPCameraData *data = cameraData(camera);
       StreamConfiguration &cfg = config->at(0);

       PixelFormat bayerFormat = formats::SBGGR10;
       V4L2DeviceFormat format = {};
       format.fourcc = data->video_->toV4L2PixelFormat(bayerFormat);;
       format.size = {640, 480};

       int ret = data->isp_.configure(bayerFormat, cfg.pixelFormat, format.size, cfg.size);
       if (ret) {
              LOG(ISP, Error) << "No matching the software ISP input/output format or size.";
              return ret;
       }

       data->width_ = format.size.width;
       data->height_ = format.size.height;

       ret = data->video_->setFormat(&format);
       if (ret)
               return ret;

       cfg.setStream(&data->stream_);
       cfg.stride = format.planes[0].bpl;

       return 0;
}

int PipelineHandlerISP::exportFrameBuffers(Camera *camera, Stream *stream,
                                           std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
       unsigned int count = stream->configuration().bufferCount;
       ISPCameraData *data = cameraData(camera);

       count = data->isp_.exportBuffers(buffers, count, data->width_, data->height_);

       return count;       

}

int PipelineHandlerISP::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
       ISPCameraData *data = cameraData(camera);
       unsigned int count = data->stream_.configuration().bufferCount;

       int ret = data->video_->allocateBuffers(count, &data->rawBuffers_);
       if (ret < 0) {
              LOG(ISP, Error) << strerror(-ret);
              return ret;
       }

       for (unsigned int i = 0; i < count; i++) {
              data->rawBufferQueue_.push(data->rawBuffers_[i].get());
       }
       

       ret = data->video_->streamOn();
       if (ret < 0) {
               data->video_->releaseBuffers();
               return ret;
       }

       data->isp_.start();

       return 0;
}

void PipelineHandlerISP::stop(Camera *camera)
{
       ISPCameraData *data = cameraData(camera);

       if (!(data->rawBuffers_.empty())) {
              data->rawBuffers_.clear();
       }

       data->isp_.stop();

       data->video_->streamOff();
       data->video_->releaseBuffers();
}

int PipelineHandlerISP::queueRequestDevice(Camera *camera, Request *request)
{
       ISPCameraData *data = cameraData(camera);
       FrameBuffer *rgbBuffer = request->findBuffer(&data->stream_);       
       if (!rgbBuffer) {
               LOG(ISP, Error) << "Attempt to queue request with invalid stream";
               return -ENOENT;
       }
       data->rgbBufferQueue_.push(rgbBuffer);

       FrameBuffer *buffer = data->rawBufferQueue_.front();
       int ret = data->video_->queueBuffer(buffer);
       if (ret < 0) {
               LOG(ISP, Error) << "Queue raw buffer error";
               return ret;
       }
       data->rawBufferQueue_.pop();

       return 0;
}

bool PipelineHandlerISP::match(DeviceEnumerator *enumerator)
{
       DeviceMatch unicam("unicam");

       unicam.add("unicam-embedded");
       unicam.add("unicam-image");

       MediaDevice *unicam_ = acquireMediaDevice(enumerator, unicam);
       if (!unicam_) {
               LOG(ISP, Debug) << "unicam Device not found";
               return false;
       }

       LOG(ISP, Debug) << "unicam Device Identified";

       std::unique_ptr<ISPCameraData> data = std::make_unique<ISPCameraData>(this, unicam_);

       if(data->init()) return false;

       std::set<Stream *> streams{&data->stream_};
       std::shared_ptr<Camera> camera = Camera::create(this, data->video_->deviceName(), streams);
       registerCamera(std::move(camera), std::move(data));

       return true;
}


void ISPCameraData::ISPCompleted(FrameBuffer *rawBuffer, FrameBuffer *rgbBuffer)
{
       Request *request = rgbBuffer->request();

       rawBufferQueue_.push(rawBuffer);

       pipe_->completeBuffer(request, rgbBuffer);
       pipe_->completeRequest(request);

}

void ISPCameraData::bufferReady(FrameBuffer *buffer)
{
       FrameBuffer *rgbBuffer = rgbBufferQueue_.front();
       isp_.invokeMethod(&ISPCPU::processing, ConnectionTypeQueued, buffer, rgbBuffer, width_, height_);
       rgbBufferQueue_.pop();
       
}

int ISPCameraData::init()
{
       video_ = new V4L2VideoDevice(media_->getEntityByName("unicam-image"));
       if (video_->open())
               return -ENODEV;

       video_->bufferReady.connect(this, &ISPCameraData::bufferReady);
       isp_.ispCompleted.connect(this, &ISPCameraData::ISPCompleted);

       return 0;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerISP)

} /* namespace libcamera */
