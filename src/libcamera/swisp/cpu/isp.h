/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Siyuan Fan <siyuan.fan@foxmail.com>
 *
 * isp.h - The CPU implementation of the software ISP
 */
#ifndef __LIBCAMERA_SWISP_CPU_ISP_H__
#define __LIBCAMERA_SWISP_CPU_ISP_H__

#include <map>
#include <set>

#include <libcamera/internal/isp.h>

#include "libcamera/base/thread.h"

namespace libcamera{

using std::uint16_t;
using std::uint8_t;

class ISPCPU : public ISP
{
public:
        int configure(PixelFormat inputFormat, PixelFormat outputFormat, Size inputSize, Size outputSize) override;

        void processing(std::vector<std::string> op, FrameBuffer *srcBuffer, FrameBuffer *dstBuffer,
                        int width, int height) override;

        int exportBuffers(std::vector<std::unique_ptr<FrameBuffer>> *buffers,
                           unsigned int count, int width, int height) override;

        void start() override;

        void stop() override;

        std::multimap<PixelFormat, PixelFormat> ISPSupportFormat = {
                {formats::SBGGR10, formats::RGB888},
                {formats::SBGGR10, formats::BGR888}
        };

        std::set<Size> ISPSupportSize = {{640, 480}};

        PixelFormat format;

private:
        void autoContrast(uint16_t *data, float lowCut, float highCut, int width, int height);

        void blackLevelCorrect(uint16_t *data, uint16_t offset, int width, int height);

        void readChannels(uint16_t *data, uint16_t *R, uint16_t *G, uint16_t *B,
                          int width, int height);

        void firstPixelInsert(uint16_t *src, uint16_t *dst, int width, int height);

        void twoPixelInsert(uint16_t *src, uint16_t *dst, int width, int height);

        void lastPixelInsert(uint16_t *src, uint16_t *dst, int width, int height);

        void demosaic(uint16_t *data, uint16_t *R, uint16_t *G, uint16_t *B,
                      int width, int height);

        void autoWhiteBalance(uint16_t *R, uint16_t *G, uint16_t *B, int width, int height);

        void gammaCorrect(uint16_t *R, uint16_t *G, uint16_t *B, float val, int width, int height);

        float distance(int x, int y, int i, int j);

        double gaussian(float x, double sigma);

        void bilateralFilter(uint16_t *R, uint16_t *G, uint16_t *B,
                             int diameter, double sigmaI, double sigmaS,
                             int width, int height);

        void noiseReduction(uint16_t *R, uint16_t *G, uint16_t *B, int width, int height);

        void compressAndTransformFormat(uint16_t *src, uint8_t *dst, int width, int height);

        bool blc_wrapper(std::vector<void*> args);

        bool demosaic_wrapper(std::vector<void*> args);

        bool awb_wrapper(std::vector<void*> args);

        bool toneMapping_wrapper(std::vector<void*> args);

        bool gamma_wrapper(std::vector<void*> args);

        bool nr_wrapper(std::vector<void*> args);

        template <typename... Args>
        bool callISPFunction(std::string fname, Args... args);

        std::map<std::string, std::function<bool(std::vector<void*>)>> funcMap = 
        {
                {"BLC", blc_wrapper},
                {"Demosaic", demosaic_wrapper},
                {"AWB", awb_wrapper},
                {"ToneMapping", toneMapping_wrapper},
                {"Gamma", gamma_wrapper},
                {"NR", nr_wrapper}
        };

        Thread thread_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_SWISP_CPU_ISP_H__ */
