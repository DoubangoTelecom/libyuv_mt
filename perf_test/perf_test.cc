/* Copyright (C) 2015 Doubango Telecom.
*
* DOUBANGO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DOUBANGO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DOUBANGO.
*/
#if defined(WIN32)|| defined(_WIN32) || defined(_WIN32_WCE)
#include <windows.h>
#endif
#include "stdafx.h"

#include <assert.h>
#include <libyuv.h>

#define YUV_ASSERT(x) { \
	bool __b_ret = (x); \
	assert(__b_ret); \
}
#define YUV_FREE(ptr) if (ptr) free(ptr), ptr = NULL

#define YUV_INPUT_CHROMA		libyuv::FOURCC_ARGB
#define YUV_OUTPUT_CHROMA		libyuv::FOURCC_I420

#define YUV_INPUT_WIDTH			1024
#define YUV_INPUT_HEIGHT		768

#define YUV_Y_STRIDE		YUV_INPUT_WIDTH
#define YUV_U_STRIDE		((YUV_Y_STRIDE + 1) >> 1)
#define YUV_V_STRIDE		YUV_U_STRIDE

#define YUV_Y_START			0
#define YUV_U_START			(YUV_Y_STRIDE * YUV_INPUT_HEIGHT)
#define YUV_V_START			YUV_U_START + (YUV_U_START >> 2)

#define YUV_CROP_X			0
#define YUV_CROP_Y			0

#define YUV_LOOP_COUNT		1000

static size_t YuvGetChromaSize(enum libyuv::FourCC chroma, size_t width, size_t height)
{
	switch (chroma){
	case libyuv::FOURCC_24BG:
		return (width * height * 3);
	case libyuv::FOURCC_RGBP:
		return ((width * height) << 1);
	case libyuv::FOURCC_ARGB:
		return ((width * height) << 2);
	case libyuv::FOURCC_NV21:
		return ((width * height * 3) >> 1);
	case libyuv::FOURCC_NV12:
		return ((width * height * 3) >> 1);
	case libyuv::FOURCC_I422:
		return ((width * height) << 1);
	case libyuv::FOURCC_UYVY:
	case libyuv::FOURCC_YUY2:
		return ((width * height) << 1);
	case libyuv::FOURCC_I420:
		return ((width * height * 3) >> 1);
	default:
		YUV_ASSERT(false);
		return 0;
	}
}

static void YuvFillSamples(uint8 *ptr, size_t size)
{
	for (size_t i = 0; i < size; ++i)
	{
		ptr[i] = rand() % 10;
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	uint8 *inPtr = NULL, *outPtr = NULL;
	size_t inSize, outSize;
	int ret = 0;
	uint64 timeStart, timeEnd;

	libyuv::YUVDebugSetLevel(libyuv::YUV_DEBUG_LEVEL_INFO);

	YUV_DEBUG_INFO("=== Starting perf test ===");

	inSize = YuvGetChromaSize(YUV_INPUT_CHROMA, YUV_INPUT_WIDTH, YUV_INPUT_HEIGHT);
	outSize = YuvGetChromaSize(YUV_OUTPUT_CHROMA, YUV_INPUT_WIDTH, YUV_INPUT_HEIGHT);
	YUV_ASSERT((inPtr = (uint8 *)malloc(inSize)) != NULL);
	YUV_ASSERT((outPtr = (uint8 *)malloc(outSize)) != NULL);

	YUV_ASSERT (YUV_INPUT_CHROMA == libyuv::FOURCC_I420 || YUV_OUTPUT_CHROMA == libyuv::FOURCC_I420);
	
	YuvFillSamples(inPtr, inSize);

	timeStart = libyuv::YuvTimeNow();

	if (YUV_OUTPUT_CHROMA == libyuv::FOURCC_I420)
	{
		for (size_t index = 0; index < YUV_LOOP_COUNT; ++index)
		{
			ret = libyuv::ConvertToI420(inPtr, inSize,
				&outPtr[YUV_Y_START], YUV_Y_STRIDE,
				&outPtr[YUV_U_START], YUV_U_STRIDE,
				&outPtr[YUV_V_START], YUV_V_STRIDE,
				YUV_CROP_X, YUV_CROP_Y,
				YUV_INPUT_WIDTH, YUV_INPUT_HEIGHT,
				YUV_INPUT_WIDTH, YUV_INPUT_HEIGHT,
				libyuv::kRotate0,
				YUV_INPUT_CHROMA);
			YUV_ASSERT(ret == 0);
		}
	}
	else
	{
		for (size_t index = 0; index < YUV_LOOP_COUNT; ++index)
		{
			ret = libyuv::ConvertFromI420(
				&inPtr[YUV_Y_START], YUV_Y_STRIDE,
				&inPtr[YUV_U_START], YUV_U_STRIDE,
				&inPtr[YUV_V_START], YUV_V_STRIDE,
				outPtr, 0/* dst_sample_stride */,
				YUV_INPUT_WIDTH, YUV_INPUT_HEIGHT,
				YUV_OUTPUT_CHROMA);
			YUV_ASSERT(ret == 0);
		}
	}

	timeEnd = libyuv::YuvTimeNow();

	YUV_DEBUG_INFO("Converted %d frames in %llu millis\n --> Time for each frame=%.2f millis\n --> Speed=%.2f frames/sec",
                  YUV_LOOP_COUNT,
                  (timeEnd - timeStart),
				  ((float)timeEnd - (float)timeStart) / (float)YUV_LOOP_COUNT,
                  ((float)YUV_LOOP_COUNT / (((float)timeEnd - (float)timeStart) / (float)1000)));

	YUV_FREE(inPtr);
	YUV_FREE(outPtr);

	YUV_DEBUG_INFO("=== Ending perf test ===");

	getchar();

	return 0;
}

