/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Siyuan Fan <siyuan.fan@foxmail.com>
 *
 * isp.h - The software ISP abstract base class
 */
#ifndef __LIBCAMERA_INTERNAL_ISP_H__
#define __LIBCAMERA_INTERNAL_ISP_H__

#include <vector>
#include <memory>

#include <libcamera/formats.h>
#include <libcamera/framebuffer.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

#include "libcamera/base/object.h"
#include "libcamera/base/signal.h"

namespace libcamera{

class ISP : public Object
{
public:
        ISP() {}

        virtual ~ISP() {}

        virtual int configure(PixelFormat inputFormat, PixelFormat outputFormat, Size inputSize, Size outputSize) = 0;

        virtual void processing(std::vector<std::string> op, FrameBuffer *srcBuffer, FrameBuffer *dstBuffer,
                                int width, int height) = 0;

        virtual int exportBuffers(std::vector<std::unique_ptr<FrameBuffer>> *buffers,
                                  unsigned int count, int width, int height) = 0;

        virtual void start() = 0;

        virtual void stop() = 0;

        Signal<FrameBuffer *, FrameBuffer *> ispCompleted;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_ISP_H__ */
