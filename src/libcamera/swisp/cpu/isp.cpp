/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Siyuan Fan <siyuan.fan@foxmail.com>
 *
 * isp.cpp - The software ISP class
 */

#include "isp.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <libcamera/request.h>
#include <libcamera/file_descriptor.h>

#include "libcamera/base/log.h"

namespace libcamera{

LOG_DECLARE_CATEGORY(ISP)

void ISPCPU::autoContrast(uint16_t *data, float lowCut, float highCut, int width, int height)
{
        int blue, gr, gb, red;
        int histBlue[1024] = {0}, histGb[1024] = {0}, histGr[1024] = {0}, histRed[1024] = {0};

        int index;
        for (int i = 0; i < height; i++) {
                index = i * width;
                for (int j = 0; j < width; j++) {
                        if (i % 2 == 0 && j % 2 == 0) {
                                blue = data[index];
                                histBlue[blue]++;
                        }
                        else if ((i % 2 == 0 && j % 2 == 1)) {
                                gb = data[index];
                                histGb[gb]++;
                        }
                        else if ((i % 2 == 1 && j % 2 == 0)) {
                                gr = data[index];
                                histGr[gr]++;
                        }
                        else {
                                red = data[index];
                                histRed[red]++;
                        }
                        index++;
                }    
        }

        int pixelAmount = width * height;
        int sum = 0;
        int minBlue;
        for (int i = 0; i < 1024; i++){
                sum = sum + histBlue[i];
                if (sum >= pixelAmount * lowCut * 0.01) {
                        minBlue = i;
                        break;
                }
        }

        sum = 0;
        int maxBlue;
        for (int i = 1023; i >= 0; i--){
                sum = sum + histBlue[i];
                if (sum >= pixelAmount * highCut * 0.01) {
                        maxBlue = i;
                        break;
                }
        }

        sum = 0;
        int minGb;
        for (int i = 0; i < 1024; i++){
                sum = sum + histGb[i];
                if (sum >= pixelAmount * lowCut * 0.01) {
                        minGb = i;
                        break;
                }
        }

        sum = 0;
        int maxGb;
        for (int i = 1023; i >= 0; i--){
                sum = sum + histGb[i];
                if (sum >= pixelAmount * highCut * 0.01) {
                        maxGb = i;
                        break;
                }
        }

        sum = 0;
        int minGr;
        for (int i = 0; i < 1024; i++){
                sum = sum + histGr[i];
                if (sum >= pixelAmount * lowCut * 0.01) {
                        minGr = i;
                        break;
                }
        }

        sum = 0;
        int maxGr;
        for (int i = 1023; i >= 0; i--){
                sum = sum + histGr[i];
                if (sum >= pixelAmount * highCut * 0.01) {
                        maxGr = i;
                        break;
                }
        }

        sum = 0;
        int minRed;
        for (int i = 0; i < 1024; i++){
                sum = sum + histRed[i];
                if (sum >= pixelAmount * lowCut * 0.01) {
                        minRed = i;
                        break;
                }
        }

        sum = 0;
        int maxRed;
        for (int i = 1023; i >= 0; i--){
                sum = sum + histRed[i];
                if (sum >= pixelAmount * highCut * 0.01) {
                        maxRed = i;
                        break;
                }
        }

        int blueMap[1024];
        float norb = 1.0 / (maxBlue - minBlue);
        for (int i = 0; i < 1024; i++) {
                if (i < minBlue) {
                        blueMap[i] = 0;
                }
                else if (i > maxBlue) {
                        blueMap[i] = 1023;
                }
                else {
                        blueMap[i] = (i - minBlue) * norb * 1023;
                }
                if (blueMap[i] > 1023) blueMap[i] = 1023;
        }

        int gbMap[1024];
        float norgb = 1.0 / (maxGb - minGb);
        for (int i = 0; i < 1024; i++) {
                if (i < minGb) {
                        gbMap[i] = 0;
                }
                else if (i > maxGb) {
                        gbMap[i] = 1023;
                }
                else {
                        gbMap[i] = (i - minGb) * norgb * 1023;
                }
                if (gbMap[i] > 1023) gbMap[i] = 1023;
        }

        int grMap[1024];
        float norgr = 1.0 / (maxGr - minGr);
        for (int i = 0; i < 1024; i++) {
                if (i < minGr) {
                        grMap[i] = 0;
                }
                else if (i > maxGr) {
                        grMap[i] = 1023;
                }
                else {
                        grMap[i] = (i - minGr) * norgr * 1023;
                }
                if (grMap[i] > 1023) grMap[i] = 1023;
        }

        int redMap[1024];
        float norr = 1.0 / (maxRed - minRed);
        for (int i = 0; i < 1024; i++) {
                if (i < minRed) {
                        redMap[i] = 0;
                }
                else if (i > maxRed) {
                        redMap[i] = 1023;
                }
                else{
                        redMap[i] = (i - minRed) * norr * 1023;
                }
                if (redMap[i] > 1023) redMap[i] = 1023;
        }

        for (int i = 0;i < height; i++) {
                for (int j = 0; j < width; j++){
                        index = i * width;
                        if (i % 2 == 0 && j % 2 == 0) {
                                data[index] = blueMap[data[index]];
                        }    
                        else if (i % 2 == 0 && j % 2 == 1) {
                                data[index] = gbMap[data[index]];
                        }    
                        else if (i % 2 == 1 && j % 2 == 0) {
                                data[index] = grMap[data[index]];
                        }    
                        else {
                                data[index] = redMap[data[index]];
                        }
                        index++;
                }
        }
}

void ISPCPU::blackLevelCorrect(uint16_t *data, uint16_t offset, int width, int height)
{
        int len = width * height;
        for(int i = 0; i < len; i++) {
                if (data[i] < offset){
                        data[i] = 0;
                }
                else {
                        data[i] -= offset;
                }
        }
}

void ISPCPU::readChannels(uint16_t *data, uint16_t *R, uint16_t *G, uint16_t *B,
                       int width, int height)
{
        int index;
        for (int i = 0; i < height; i++) {
                index = i * width;
                for (int j = 0; j < width; j++) {
                        if (i % 2 == 0 && j % 2 == 0) {
                                B[index] = data[index];
                        }
                        else if ((i % 2 == 0 && j % 2 == 1) || (i % 2 == 1 && j % 2 == 0)){
                                G[index] = data[index];
                        }
                        else {
                                R[index] = data[index];
                        }
                        index++;
                }
        }
}

void ISPCPU::firstPixelInsert(uint16_t *src, uint16_t *dst, int width, int height)
{
        int index;
        for (int i = 0; i < height; i++) {
                index = i * width;
                for (int j = 0; j < width; j++){
                        if (i % 2 == 0 && j % 2 == 1) {
                                if (j == (width - 1)) {
                                        dst[index] = src[index - 1];
                                }
                                else {
                                        dst[index] = (src[index - 1] +
                                                src[index + 1]) >> 1;
                                }
                        }

                        if (i % 2 == 1 && j % 2 == 0) {
                                if(i == height - 1) {
                                        dst[index] = src[index - width];
                                }
                                else {
                                        dst[index] = (src[index - width]+
                                                src[index + width]) >> 1;
                                }
                        }
    
                        if (i % 2 == 1 && j % 2 == 1) {
                                if (j < width - 1 && i < height - 1) {
                                        dst[index] = (src[index - width - 1] +
                                                src[index - width + 1] +
                                                src[index + width - 1] +
                                                src[index + width + 1]) >> 2;
                                }
                                else if (i == height - 1 && j < width - 1) {
                                        dst[index] = (src[index - width - 1] +
                                                src[index - width + 1]) >> 1;
                                }
                                else if (i < height - 1 && j == width - 1) {
                                        dst[index] = (src[index - width - 1] +
                                                src[index + width - 1]) >> 1;
                                }
                                else {
                                        dst[index] = src[index - width - 1];
                                }          
                        }
                        index++;
                }
        }
}

void ISPCPU::twoPixelInsert(uint16_t *src, uint16_t *dst, int width, int height)
{
        int index;
        for (int i = 0; i < height; i++) {
                index = i * width;
                for (int j = 0; j < width; j++) {
                        if (i == 0 && j == 0) {
                                dst[index] = (src[index + width] +
                                        src[index + 1]) >> 1;
                        }
                        else if (i == 0 && j > 0 && j % 2 == 0) {
                                dst[index] = (src[index - 1] +
                                        src[index + width] +
                                        src[index + 1]) / 3;
                        }
                        else if (i > 0 && j == 0 && i % 2 == 0) {
                                dst[index] = (src[index - width] +
                                        src[index + 1] +
                                        src[index + width]) / 3;
                        }
                        else if (i == (height - 1) && j < (width - 1) && j % 2 == 1) {
                                dst[index] = (src[index - 1] +
                                        src[index - width] +
                                        src[index + 1]) / 3;
                        }
                        else if (i < (height - 1) && j == (width - 1) && i % 2 == 1) {
                                dst[index] = (src[index - width] +
                                        src[index - 1] +
                                        src[index + width]) / 3;
                        }
                        else if (i == (height - 1) && j == (width - 1)) {
                                dst[index] = (src[index - width] +
                                        src[index - 1]) >> 1;
                        }
                        else if ((i % 2 == 0 && j % 2 == 0) || (i % 2 == 1 && j % 2 == 1)) {
                                dst[index] = (src[index - 1] +
                                        src[index + 1] +
                                        src[index - width] +
                                        src[index + width]) / 4;
                        }
                        index++;
                }
        }
}

void ISPCPU::lastPixelInsert(uint16_t *src, uint16_t *dst, int width, int height)
{
        int index;
        for (int i = 0; i < height; i++) {
                index = i * width;
                for (int j = 0; j < width; j++){
                        if (i % 2 == 1 && j % 2 == 0) {
                                if (j == 0) {
                                        dst[index] = src[index + 1];
                                }
                                else {
                                        dst[index] = (src[index - 1] +
                                                src[index + 1]) >> 1;
                                }
                        }

                        if (i % 2 == 0 && j % 2 == 1) {
                                if(i == 0) {
                                        dst[index] = src[index + width];
                                }
                                else {
                                        dst[index] = (src[index - width]+
                                                src[index + width]) >> 1;
                                }
                        }

                        if (i % 2 == 0 && j % 2 == 0) {
                                if (i > 0 && j > 0) {
                                        dst[index] = (src[index - width - 1] +
                                                src[index - width + 1] +
                                                src[index + width - 1] +
                                                src[index + width + 1]) >> 2;
                                }
                                else if (i == 0 && j > 0) {
                                        dst[index] = (src[index + width - 1] +
                                                src[index + width + 1]) >> 1;
                                }
                                else if (i > 0 && j == 0) {
                                        dst[index] = (src[index - width + 1] +
                                                src[index + width + 1]) >> 1;
                                }
                                else {
                                        dst[index] = src[index + width + 1];
                                }
                        }
                        index++;
                }
        }
}

void ISPCPU::demosaic(uint16_t *data, uint16_t *R, uint16_t *G, uint16_t *B,
                       int width, int height)
{
        readChannels(data, R, G, B, width, height);
        firstPixelInsert(data, B, width, height);
        twoPixelInsert(data, G, width, height);
        lastPixelInsert(data, R, width, height);
}

void ISPCPU::autoWhiteBalance(uint16_t *R, uint16_t *G, uint16_t *B, int width, int height)
{
        float aveB = 0, aveG = 0, aveR = 0;
        float Kb, Kg, Kr;

        for (int i = 0; i < width * height; i++) {
                aveB += 1.0 * B[i];
                aveG += 1.0 * G[i];
                aveR += 1.0 * R[i];
        }

        aveB *= (1.0 / (width * height));
        aveG *= (1.0 / (width * height));
        aveR *= (1.0 / (width * height));

        Kr = (aveB + aveG + aveR) / aveR * (1.0 / 3.0);
        Kg = (aveB + aveG + aveR) / aveG * (1.0 / 3.0);
        Kb = (aveB + aveG + aveR) / aveB * (1.0 / 3.0);

        for (int i = 0; i < width * height; i++) {
                B[i] = B[i] * Kb;
                G[i] = G[i] * Kg;
                R[i] = R[i] * Kr;

                if (R[i] > 1023) R[i] = 1023;
                if (G[i] > 1023) G[i] = 1023;
                if (R[i] > 1023) B[i] = 1023;
        }
}

void ISPCPU::gammaCorrect(uint16_t *R, uint16_t *G, uint16_t *B, float val, int width, int height)
{
        float nor = 1.0 / 1023.0;
        float gamma = 1.0 / val;
        for (int i = 0; i < width * height; i++) {
                R[i] = pow(R[i] * nor, gamma) * 1023;
                G[i] = pow(G[i] * nor, gamma) * 1023;
                B[i] = pow(B[i] * nor, gamma) * 1023;

                if (R[i] > 1023) R[i] = 1023;
                if (G[i] > 1023) G[i] = 1023;
                if (B[i] > 1023) B[i] = 1023;
        }
}

void ISPCPU::compressAndTransformFormat(uint16_t *src, uint8_t *dst, int width, int height)
{
    switch(format.fourcc())
    {
        case formats::RGB888.fourcc(): {
            int j = 0;
            for (int i = 0; i < width * height; i++, j += 3) {
                    dst[i] = src[j] >> 2 & 0xff;
            }
            
            j = 1;
            for (int i = 0; i < width * height; i++, j += 3) {
                    dst[i + width * height] = src[j] >> 2 & 0xff;
            }

            j = 2;
            for (int i = 0; i < width * height; i++, j += 3) {
                    dst[i + width * height *2] = src[j] >> 2 & 0xff;
            }
            break;
        }

        case formats::BGR888.fourcc(): {
            int j = 2;
            for (int i = 0; i < width * height; i++, j += 3) {
                    dst[i] = src[j] >> 2 & 0xff;
            }
            
            j = 1;
            for (int i = 0; i < width * height; i++, j += 3) {
                    dst[i + width * height] = src[j] >> 2 & 0xff;
            }

            j = 0;
            for (int i = 0; i < width * height; i++, j += 3) {
                    dst[i + width * height *2] = src[j] >> 2 & 0xff;
            }
            break;
        }
    }
}

float ISPCPU::distance(int x, int y, int i, int j)
{
    return float(sqrt(pow(x - i, 2) + pow(y - j, 2)));
}

double ISPCPU::gaussian(float x, double sigma)
{
    return exp(-(pow(x, 2)) / (2 * pow(sigma, 2))) / (2 * 3.1415926 * pow(sigma, 2));
}

void ISPCPU::bilateralFilter(uint16_t *R, uint16_t *G, uint16_t *B,
                          int diameter, double sigmaI,
                          double sigmaS, int width, int height)
{
        for (int i = 2; i < height - 2; i++) {
                for (int j = 2; j < width - 2; j++) {
                        double iFiltered = 0;
                        double wp = 0;
                        int neighbor_x = 0;
                        int neighbor_y = 0;
                        int half = diameter / 2;

                        for (int k = 0; k < diameter; k++) {
                                for (int l = 0; l < diameter; l++) {
                                        neighbor_x = i - (half - k);
                                        neighbor_y = j - (half - l);
                                        double gi = gaussian(R[neighbor_x * width + neighbor_y] - R[i * width +j], sigmaI);
                                        double gs = gaussian(distance(i, j, neighbor_x, neighbor_y), sigmaS);
                                        double w = gi * gs;
                                        iFiltered = iFiltered + R[neighbor_x * width + neighbor_y] * w;
                                        wp = wp + w;
                                }    
                        }

                        iFiltered = iFiltered / wp;
                        R[i * width + j] = iFiltered;
                }
        }

        for (int i = 2; i < height - 2; i++) {
                for (int j = 2; j < width - 2; j++) {
                        double iFiltered = 0;
                        double wp = 0;
                        int neighbor_x = 0;
                        int neighbor_y = 0;
                        int half = diameter / 2;

                        for (int k = 0; k < diameter; k++) {
                                for (int l = 0; l < diameter; l++) {
                                        neighbor_x = i - (half - k);
                                        neighbor_y = j - (half - l);
                                        double gi = gaussian(G[neighbor_x * width + neighbor_y] - G[i * width +j], sigmaI);
                                        double gs = gaussian(distance(i, j, neighbor_x, neighbor_y), sigmaS);
                                        double w = gi * gs;
                                        iFiltered = iFiltered + G[neighbor_x * width + neighbor_y] * w;
                                        wp = wp + w;
                                }    
                        }

                        iFiltered = iFiltered / wp;
                        G[i * width + j] = iFiltered;
                }
        }

        for (int i = 2; i < height - 2; i++) {
                for (int j = 2; j < width - 2; j++) {
                        double iFiltered = 0;
                        double wp = 0;
                        int neighbor_x = 0;
                        int neighbor_y = 0;
                        int half = diameter / 2;

                        for (int k = 0; k < diameter; k++) {
                                for (int l = 0; l < diameter; l++) {
                                        neighbor_x = i - (half - k);
                                        neighbor_y = j - (half - l);
                                        double gi = gaussian(B[neighbor_x * width + neighbor_y] - B[i * width +j], sigmaI);
                                        double gs = gaussian(distance(i, j, neighbor_x, neighbor_y), sigmaS);
                                        double w = gi * gs;
                                        iFiltered = iFiltered + B[neighbor_x * width + neighbor_y] * w;
                                        wp = wp + w;
                                }    
                        }

                        iFiltered = iFiltered / wp;
                        B[i * width + j] = iFiltered;
                }
        }
}

void ISPCPU::noiseReduction(uint16_t *R, uint16_t *G, uint16_t *B, int width, int height)
{
        bilateralFilter(R, G, B, 5, 24.0, 32.0, width, height);
}

int ISPCPU::configure(PixelFormat inputFormat, PixelFormat outputFormat, Size inputSize, Size outputSize)
{
       int ret = -1;

       auto it = ISPSupportFormat.find(inputFormat);
       auto cit = ISPSupportFormat.count(inputFormat);
       for (; cit > 0; cit--, it++) {
               if (it->second == outputFormat) {
                       ret = 0;
                       break;
               }    
       }

       if (ret == -1) return ret;
       format = it->second;

       std::set<Size>::iterator it1;
       it1 = ISPSupportSize.find(inputSize);
       if (it1 == ISPSupportSize.end() || *it1 != outputSize) {
               ret = -1;
               return ret;
       }

       return ret; 
}

template <typename... Args>
bool ISPCPU::callISPFunction(std::string fname, Args... args)
{
        auto func_iter = funcMap.find(fname);
        if (func_iter == funcMap.end()) {
                return false;
        }
        else
        {
                std::vector<void*> arg_pointers;
                (arg_pointers.push_back(reinterpret_cast<void*>(std::addressof(args))),...);
                return func_iter->second(arg_pointers);
        }
}

void ISPCPU::processing(std::vector<std::string> op, FrameBuffer *srcBuffer, FrameBuffer *dstBuffer, int width, int height)
{
        uint16_t *inputBuf;
        uint16_t *rgbBuf = new std::uint16_t[width * height * 3];

        uint16_t *rData = rgbBuf;
        uint16_t *gData = rData + width * height;
        uint16_t *bData = gData + width * height;
        memset(rgbBuf, 0x0, width * height * 3);

        const FrameBuffer::Plane &plane =  srcBuffer->planes()[0]; 
        inputBuf = (uint16_t *)mmap(NULL, plane.length, PROT_READ|PROT_WRITE, MAP_SHARED, plane.fd.fd(), 0);
        if (inputBuf == MAP_FAILED) {
                LOG(ISP, Error) << "Read raw data failed";
                ispCompleted.emit(srcBuffer, dstBuffer);
        }

        for (auto iter = op.begin(); iter != op.end(); iter++) {
                if (*iter == "BLC")
                    callISPFunction("BLC", 16, width, height);
                if (*iter == "Demosaic")
                    callISPFunction("Demosaic", inputBuf, rData, gData, bData, width, height);
                if (*iter == "AWB")
                    callISPFunction("AWB", rData, gData, bData, width, height);
                if (*iter == "ToneMapping")
                    callISPFunction("ToneMapping", rData, gData, bData, 0.01, 0.01, width, height);
                if (*iter == "Gamma")
                    callISPFunction("Gamma", rData, gData, bData, 2.2, width, height);
                if (*iter == "NR")
                    callISPFunction("NR", rData, gData, bData, width, height);
        }

        blackLevelCorrect(inputBuf, 16, width, height);
        demosaic(inputBuf, rData, gData, bData, width, height);
        autoWhiteBalance(rData, gData, bData, width, height);
        autoContrast(rData, 0.01, 0.01, width, height);
        autoContrast(gData, 0.01, 0.01, width, height);
        autoContrast(bData, 0.01, 0.01, width, height);
        gammaCorrect(rData, gData, bData, 2.2, width, height);
        //bilateralFilter(rData, gData, bData, 5, 24.0, 32.0, width, height);
    
        const FrameBuffer::Plane &rgbPlane = dstBuffer->planes()[0];
        uint8_t *outputBuf;
        outputBuf = (uint8_t *)mmap(NULL, rgbPlane.length, PROT_READ|PROT_WRITE, MAP_SHARED, rgbPlane.fd.fd(), 0);
        if (outputBuf == MAP_FAILED) {
                LOG(ISP, Error) << "Read rgb data failed";
                ispCompleted.emit(srcBuffer, dstBuffer);
        }

        compressAndTransformFormat(rgbBuf, outputBuf, width, height);

        dstBuffer->metadata_.status = srcBuffer->metadata().status;
        dstBuffer->metadata_.sequence = srcBuffer->metadata().sequence;
        dstBuffer->metadata_.timestamp = srcBuffer->metadata().timestamp;

        dstBuffer->metadata_.planes.clear();
        dstBuffer->metadata_.planes.push_back({rgbPlane.length});

        delete[] rgbBuf;

        ispCompleted.emit(srcBuffer, dstBuffer);
}

int ISPCPU::exportBuffers(std::vector<std::unique_ptr<FrameBuffer>> *buffers,
                       unsigned int count, int width, int height)
{
        int bufferByte = width * height * 3;

        for (unsigned int i = 0; i < count; i++) {
                std::string name = "frame-" + std::to_string(i);

                const int isp_fd = memfd_create(name.c_str(), 0);
                int ret = ftruncate(isp_fd, bufferByte);
                if (ret < 0) {
                        LOG(ISP, Error) << "Failed to resize memfd" << strerror(-ret);
                        return ret;
                }

                FrameBuffer::Plane rgbPlane;
                rgbPlane.fd = FileDescriptor(std::move(isp_fd));
                rgbPlane.length = bufferByte;

                std::vector<FrameBuffer::Plane> planes{rgbPlane};
                buffers->emplace_back(std::make_unique<FrameBuffer>(std::move(planes)));
        }

        return count;
}

void ISPCPU::start()
{
        moveToThread(&thread_);
        thread_.start();
}

void ISPCPU::stop()
{
        thread_.exit();
        thread_.wait();
}

} /* namespace libcamera */
