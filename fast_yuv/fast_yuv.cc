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
// https://software.intel.com/sites/landingpage/legacy/mmx/MMX_App_RGB_YUV.pdf
// https://software.intel.com/sites/landingpage/IntrinsicsGuide/
#if defined(WIN32)|| defined(_WIN32) || defined(_WIN32_WCE)
#include <windows.h>
#endif

#include "stdafx.h"

#include <assert.h>
#include <libyuv.h>

#if !YUV_UNDER_WINDOWS_CE
#include <intrin.h>
#endif

#define FAST_INPUT_WIDTH		1024
#define FAST_INPUT_HEIGHT		768

#define FAST_INPUT_SIZE			((FAST_INPUT_WIDTH * FAST_INPUT_HEIGHT) << 2) // RGB32
#define FAST_OUTPUT_SIZE		((FAST_INPUT_WIDTH * FAST_INPUT_HEIGHT * 3) >> 1) // I420

#define FAST_ALIGNMENT			16 // MAX(AVX (32), SSE (16))

#define FAST_LOOP_COUNT			1000 // FIXME

#define FAST_ASSERT(x) { \
	bool __b_ret = (x); \
	assert(__b_ret); \
}

#define FAST_IS_ALIGNED(p, a) (!((uintptr_t)(p) & ((a) - 1)))

static __declspec(align(FAST_ALIGNMENT)) const int8 kYCoeffs[16] = {
		33, 65, 13, 0,
		33, 65, 13, 0,
		33, 65, 13, 0,
		33, 65, 13, 0,
	};
	static __declspec(align(FAST_ALIGNMENT)) const int8 kUCoeffs[16] = {
		-38, -74, 112, 0,
		-38, -74, 112, 0,
		-38, -74, 112, 0,
		-38, -74, 112, 0,
	};
	static __declspec(align(FAST_ALIGNMENT)) const int8 kVCoeffs[16] = {
		112, -94, -18, 0,
		112, -94, -18, 0,
		112, -94, -18, 0,
		112, -94, -18, 0,
	};
	static __declspec(align(FAST_ALIGNMENT)) const  int32 kRGBAShuffleDuplicate[4] = { 0x03020100, 0x0b0a0908, 0x03020100, 0x0b0a0908 }; // RGBA(X) || RGBA(X + 2) || RGBA(X) || RGBA(X + 2) = 2U || 2V
	static __declspec(align(FAST_ALIGNMENT)) const uint16 kY16[8] = {
		16, 16, 16, 16,
		16, 16, 16, 16
	};
	static __declspec(align(FAST_ALIGNMENT)) const uint16 kUV128[8] = {
		128, 128, 128, 128,
		128, 128, 128, 128
	};

static void* FastMallocAligned(size_t size, size_t alignment);
static void* FastReallocAligned(void* ptr, size_t size, size_t alignment);
static void FastFreeAligned(void* mem);

static void* FastMallocAligned(size_t size, size_t alignment)
{
#if YUV_UNDER_WINDOWS_CE
	void* ret = malloc(size + alignment);
    if (ret) {
        long pad = ((~(long)ret) % alignment) + 1;
        ret = ((uint8*)ret) + pad; // pad
        ((uint8*)ret)[-1] = (uint8)pad; // store the pad for later use
    }
	return ret;
#else
	return _aligned_malloc(size, alignment);
#endif
}

static void* FastReallocAligned(void* ptr, size_t size, size_t alignment)
{
#if YUV_UNDER_WINDOWS_CE
	FastFreeAligned(ptr);
	return FastMallocAligned(size, alignment);
#else
	return _aligned_realloc(ptr, size, alignment);
#endif
}

static void FastFreeAligned(void* mem)
{
#if YUV_UNDER_WINDOWS_CE
	if (mem) free((((uint8*)mem) - ((uint8*)mem)[-1]));
#else
	return _aligned_free(mem);
#endif
}

#if !YUV_UNDER_WINDOWS_CE
#endif

static void RGB32ToI420_C(uint8 *yuvPtr, const uint8 *rgbPtr, const size_t width, const size_t height)
{
	const size_t ySize = (width * height);
	uint8 *dst_y = yuvPtr;
	uint8 *dst_u = yuvPtr + ySize;
	uint8 *dst_v = dst_u + (ySize >> 2);
#define COMP_COUNT 4 // 4 = RGB32, 3 = RGB24
	// Y plane
	for(size_t i = 0; i < ySize; ++i)
	{
#if 0
		*dst_y++ = ( (66*rgbPtr[COMP_COUNT*i] + 129*rgbPtr[COMP_COUNT*i+1] + 25*rgbPtr[COMP_COUNT*i+2] ) >> 8 ) + 16;
#else
		// divide the coeff by 2
		*dst_y++ = ( (33*rgbPtr[COMP_COUNT*i] + 65*rgbPtr[COMP_COUNT*i+1] + 13*rgbPtr[COMP_COUNT*i+2] ) >> 7 ) + 16;
#endif
	}
	// U+V planes
	for(size_t y=0; y<height; y+=2)
	{
		for(size_t x=0; x<width; x+=2)
		{
			const size_t i = y*width + x;
			*dst_u++ = ( ( -38*rgbPtr[COMP_COUNT*i] + -74*rgbPtr[COMP_COUNT*i+1] + 112*rgbPtr[COMP_COUNT*i+2] ) >> 8 ) + 128;
			*dst_v++ = ( ( 112*rgbPtr[COMP_COUNT*i] + -94*rgbPtr[COMP_COUNT*i+1] + -18*rgbPtr[COMP_COUNT*i+2] ) >> 8 ) + 128;
		}
	}
}

__declspec(naked) __declspec(align(FAST_ALIGNMENT))
void RGB32ToI420_Asm_SSSE3(uint8 *yuvPtr, const uint8 *rgbPtr, int width, int height)
{
	__asm {
	push esi
    push edi
	push ebx
	/*** Y Samples ***/
	mov edx, [esp + 12 + 4]   // yuvPtr
    mov eax, [esp + 12 + 8]   // rgbPtr
	mov ecx, [esp + 12 + 12] // width
	imul ecx, [esp + 12 + 16] // (width * height) = samplesCount

	movdqa xmm7, kYCoeffs // yCoeffs
	movdqa xmm6, kY16 // y16
	/* loopY start */
loopY:
	// load 16 RGBA samples
	movdqa xmm0, [eax] // mmRgb0
	movdqa xmm1, [eax + 16] // mmRgb1
	movdqa xmm2, [eax + 32] // mmRgb2
	movdqa xmm3, [eax + 48] // mmRgb3
	lea eax, [eax + 64] // rgbPtr_ += 64
	// (yCoeffs[0] * mmRgbX[0]) + (yCoeffs[1] * mmRgbX[1])
	pmaddubsw xmm0, xmm7
	pmaddubsw xmm1, xmm7
	pmaddubsw xmm2, xmm7
	pmaddubsw xmm3, xmm7
	// horizontal add
	phaddw xmm0, xmm1
	phaddw xmm2, xmm3
	// >> 7
	psraw xmm0, 7
	psraw xmm2, 7
	// + 16
	paddw xmm0, xmm6
	paddw xmm2, xmm6
	// Saturate(I16 -> U8) - Packs
	packuswb xmm0, xmm2
	// Copy to yuvPtr
	movdqa [edx], xmm0
	lea edx, [edx + 16] // yPtr_ += 16
	sub ecx, 16 // samplesCount -= 16
	jnz loopY // goto loop if (samplesCount != 0)

	//==================================//
	//=========== UV Samples ===========//
	//==================================//
	mov esi, [esp + 12 + 4]   // yuvPtr
    mov eax, [esp + 12 + 8]   // rgbPtr
	mov ecx, [esp + 12 + 12] // width
	imul ecx, [esp + 12 + 16] // (width * height) = samplesCount
	mov edx, ecx
	shr edx, 2 // edx = samplesCount / 4
	add esi,  ecx // [[esi = uPtr_]]
	mov edi, esi // edi = uPtr_
	add edi, edx // [[edi = uPtr_ + edx = uPtr_ + (samplesCount / 4) = vPtr_]]
	xor edx, edx // edx = 0 = i
	mov ebx, [esp + 12 + 12] // ebx = width
	sub ebx, 1 // ebx = (width - 1)

	movdqa xmm7, kUCoeffs // uCoeffs
	movdqa xmm6, kVCoeffs // vCoeffs
	movdqa xmm5, kRGBAShuffleDuplicate // rgbaShuffleDuplicate
	movdqa xmm4, kUV128 // uv128

	/* loopUV start */
loopUV:
	// load 16 RGBA samples
	movdqa xmm0, [eax] // mmRgb0
	movdqa xmm1, [eax + 16] // mmRgb1
	movdqa xmm2, [eax + 32] // mmRgb2
	movdqa xmm3, [eax + 48] // mmRgb3
	lea eax, [eax + 64] // rgbPtr_ += 64

	pshufb xmm0, xmm5
	pshufb xmm1, xmm5
	pshufb xmm2, xmm5
	pshufb xmm3, xmm5

	punpcklqdq xmm0, xmm1 // mmRgbU0
	movdqa xmm1, xmm0 // mmRgbV0
	punpcklqdq xmm2, xmm3 // mmRgbU1
	movdqa xmm3, xmm2 // mmRgbV1

	pmaddubsw xmm0, xmm7 // mmRgbU0
	pmaddubsw xmm1, xmm6 // mmRgbV0
	pmaddubsw xmm2, xmm7 // mmRgbU1
	pmaddubsw xmm3, xmm6 // mmRgbV1

	phaddw xmm0, xmm2 // mmY0
	phaddw xmm1, xmm3 // mmY1

	psraw xmm0, 8
	psraw xmm1, 8

	paddw xmm0, xmm4
	paddw xmm1, xmm4

	packuswb xmm0, xmm1

	movaps xmm1, xmm0
	movlps [esi], xmm1
	movhps [edi], xmm1

	lea esi, [esi + 8]
	lea edi, [edi + 8]

	add edx, 16 // i += 16;
	push edx // save edx
	and edx, ebx // edx = (ebx & ebx) = (ebx & (width - 1)) = (ebx % width)
	cmp edx, 0 // (ebx % width) == 0 ?
	pop edx // restore edx
	jne loopUV_NextLine
	
	// loopUV_EndOfLine: ((ebx % width) == 0)
	add ebx, 1// change ebx value from width-1 to width
	add edx, ebx // i += width
	lea eax, [eax + 4 * ebx]// rgbPtr_ += (width * 4);
	sub ebx, 1// change back ebx value to width - 1
loopUV_NextLine:
	cmp edx, ecx
	jl loopUV

	pop ebx
	pop edi
    pop esi
    ret
	}
}

#if !YUV_UNDER_WINDOWS_CE
static void RGB32ToI420_Intrin_SSSE3(uint8 *yuvPtr, const uint8 *rgbPtr, const size_t width, const size_t height)
{
	// rgbPtr contains (samplesCount * 16) bytes
	// yPtr contains samplesCount bytes
	const size_t samplesCount = (width * height); // "width" and "height" are in samples
	const uint8 *rgbPtr_;
	uint8* yPtr_ = yuvPtr, *uPtr_ = (yPtr_ + samplesCount), *vPtr_ = uPtr_ + (samplesCount >> 2);
	FAST_ASSERT(FAST_IS_ALIGNED(yuvPtr, 16));
	FAST_ASSERT(FAST_IS_ALIGNED(rgbPtr, 16));
	FAST_ASSERT(FAST_IS_ALIGNED(width, 16));
	FAST_ASSERT(FAST_IS_ALIGNED(height, 16));

	// Convert 16 RGBA samples to 16 Y samples
	rgbPtr_ = rgbPtr;
	/* const */__m128i yCoeffs = _mm_load_si128((__m128i*)kYCoeffs);
	/* const */__m128i y16 = _mm_load_si128((__m128i*)kY16);
	for(size_t i = 0; i < samplesCount; i += 16)
	{
		// load 16 RGBA samples
		__m128i mmRgb0 = _mm_load_si128((__m128i*)rgbPtr_); // 4 RGBA samples
		__m128i mmRgb1 = _mm_load_si128((__m128i*)&rgbPtr_[16]); // 4 RGBA samples
		__m128i mmRgb2 = _mm_load_si128((__m128i*)&rgbPtr_[32]); // 4 RGBA samples
		__m128i mmRgb3 = _mm_load_si128((__m128i*)&rgbPtr_[48]); // 4 RGBA samples
		
		mmRgb0 = _mm_maddubs_epi16(mmRgb0/*unsigned*/, yCoeffs/*signed*/); // mmRgb0 = ((yCoeffs[j] * mmRgb0[j]) +  (yCoeffs[j + 1] * mmRgb0[j + 1]))
		mmRgb1 = _mm_maddubs_epi16(mmRgb1/*unsigned*/, yCoeffs/*signed*/);
		mmRgb2 = _mm_maddubs_epi16(mmRgb2/*unsigned*/, yCoeffs/*signed*/);
		mmRgb3 = _mm_maddubs_epi16(mmRgb3/*unsigned*/, yCoeffs/*signed*/);

		__m128i mmY0 = _mm_hadd_epi16(mmRgb0, mmRgb1); // horizontal add
		__m128i mmY1 = _mm_hadd_epi16(mmRgb2, mmRgb3);
		
		mmY0 = _mm_srai_epi16(mmY0, 7); // >> 7
		mmY1 = _mm_srai_epi16(mmY1, 7);

		mmY0 = _mm_add_epi16(mmY0, y16); // + 16
		mmY1 = _mm_add_epi16(mmY1, y16);

		__m128i mmY = _mm_packus_epi16(mmY0, mmY1); // Saturate(I16 -> U8)

		_mm_store_si128((__m128i*)yPtr_, mmY);

		rgbPtr_ += 64; // 16samples * 4bytes
		yPtr_ += 16; // 16samples * 1byte
	}

	// U+V planes
	/* const */__m128i uCoeffs = _mm_load_si128((__m128i*)kUCoeffs);
	/* const */__m128i vCoeffs = _mm_load_si128((__m128i*)kVCoeffs);
	/* const */__m128i rgbaShuffleDuplicate = _mm_load_si128((__m128i*)kRGBAShuffleDuplicate);
	/* const */__m128i uv128 = _mm_load_si128((__m128i*)kUV128);
#if 1
	rgbPtr_ = rgbPtr;
	for(size_t i = 0; i < samplesCount; )
	{
		// load 16 RGBA samples
		__m128i mmRgb0 = _mm_load_si128((__m128i*)rgbPtr_); // 4 RGBA samples
		__m128i mmRgb1 = _mm_load_si128((__m128i*)&rgbPtr_[16]); // 4 RGBA samples
		__m128i mmRgb2 = _mm_load_si128((__m128i*)&rgbPtr_[32]); // 4 RGBA samples
		__m128i mmRgb3 = _mm_load_si128((__m128i*)&rgbPtr_[48]); // 4 RGBA samples

		mmRgb0 = _mm_shuffle_epi8(mmRgb0, rgbaShuffleDuplicate);
		mmRgb1 = _mm_shuffle_epi8(mmRgb1, rgbaShuffleDuplicate);
		mmRgb2 = _mm_shuffle_epi8(mmRgb2, rgbaShuffleDuplicate);
		mmRgb3 = _mm_shuffle_epi8(mmRgb3, rgbaShuffleDuplicate);

		__m128i mmRgbU0 = _mm_unpacklo_epi64(mmRgb0, mmRgb1);
		__m128i mmRgbV0 = _mm_unpackhi_epi64(mmRgb0, mmRgb1); // same as mmRgbU0: Use _mm_store_si128??
		__m128i mmRgbU1 = _mm_unpacklo_epi64(mmRgb2, mmRgb3);
		__m128i mmRgbV1 = _mm_unpackhi_epi64(mmRgb2, mmRgb3); // same as mmRgbU0: Use _mm_store_si128??

		mmRgbU0 = _mm_maddubs_epi16(mmRgbU0/*unsigned*/, uCoeffs/*signed*/);
		mmRgbV0 = _mm_maddubs_epi16(mmRgbV0/*unsigned*/, vCoeffs/*signed*/);
		mmRgbU1 = _mm_maddubs_epi16(mmRgbU1/*unsigned*/, uCoeffs/*signed*/);
		mmRgbV1 = _mm_maddubs_epi16(mmRgbV1/*unsigned*/, vCoeffs/*signed*/);

		__m128i mmY0 = _mm_hadd_epi16(mmRgbU0, mmRgbU1); // horizontal add
		__m128i mmY1 = _mm_hadd_epi16(mmRgbV0, mmRgbV1);

		mmY0 = _mm_srai_epi16(mmY0, 8); // >> 8
		mmY1 = _mm_srai_epi16(mmY1, 8);

		mmY0 = _mm_add_epi16(mmY0, uv128); // + 128
		mmY1 = _mm_add_epi16(mmY1, uv128);

		// Y contains 8 samples for U then 8 samples for V
		__m128i mmY = _mm_packus_epi16(mmY0, mmY1); // Saturate(I16 -> U8)
		_mm_storel_pi((__m64*)uPtr_, _mm_load_ps((float*)&mmY));
		_mm_storeh_pi((__m64*)vPtr_, _mm_load_ps((float*)&mmY));
		
		uPtr_ += 8; // 8samples * 1byte
		vPtr_ += 8; // 8samples * 1byte
		
		// move to next 16 samples
		i += 16;
		rgbPtr_ += 64; // 16samples * 4bytes
		
		if (/*i % width == 0*/ !(i & (width - 1)))
		{
			// skip next line
			i += width;
			rgbPtr_ += (width * 4);
		}
	}
#else
	for (size_t y=0; y<height; y+=2)
	{
		rgbPtr_ = rgbPtr + (y * width * 4);
		for (size_t x = 0; x < width; x += 16)
		{
			// load 16 RGBA samples
			__m128i mmRgb0 = _mm_load_si128((__m128i*)rgbPtr_); // 4 RGBA samples
			__m128i mmRgb1 = _mm_load_si128((__m128i*)&rgbPtr_[16]); // 4 RGBA samples
			__m128i mmRgb2 = _mm_load_si128((__m128i*)&rgbPtr_[32]); // 4 RGBA samples
			__m128i mmRgb3 = _mm_load_si128((__m128i*)&rgbPtr_[48]); // 4 RGBA samples

			mmRgb0 = _mm_shuffle_epi8(mmRgb0, rgbaShuffleDuplicate);
			mmRgb1 = _mm_shuffle_epi8(mmRgb1, rgbaShuffleDuplicate);
			mmRgb2 = _mm_shuffle_epi8(mmRgb2, rgbaShuffleDuplicate);
			mmRgb3 = _mm_shuffle_epi8(mmRgb3, rgbaShuffleDuplicate);

			__m128i mmRgbU0 = _mm_unpacklo_epi64(mmRgb0, mmRgb1);
			__m128i mmRgbV0 = _mm_unpackhi_epi64(mmRgb0, mmRgb1); // same as mmRgbU0: Use _mm_store_si128??
			__m128i mmRgbU1 = _mm_unpacklo_epi64(mmRgb2, mmRgb3);
			__m128i mmRgbV1 = _mm_unpackhi_epi64(mmRgb2, mmRgb3); // same as mmRgbU0: Use _mm_store_si128??

			mmRgbU0 = _mm_maddubs_epi16(mmRgbU0/*unsigned*/, uCoeffs/*signed*/);
			mmRgbV0 = _mm_maddubs_epi16(mmRgbV0/*unsigned*/, vCoeffs/*signed*/);
			mmRgbU1 = _mm_maddubs_epi16(mmRgbU1/*unsigned*/, uCoeffs/*signed*/);
			mmRgbV1 = _mm_maddubs_epi16(mmRgbV1/*unsigned*/, vCoeffs/*signed*/);

			__m128i mmY0 = _mm_hadd_epi16(mmRgbU0, mmRgbU1); // horizontal add
			__m128i mmY1 = _mm_hadd_epi16(mmRgbV0, mmRgbV1);

			mmY0 = _mm_srai_epi16(mmY0, 8); // >> 8
			mmY1 = _mm_srai_epi16(mmY1, 8);

			mmY0 = _mm_add_epi16(mmY0, uv128); // + 128
			mmY1 = _mm_add_epi16(mmY1, uv128);

			// Y contains 8 samples for U then 8 samples for V
			__m128i mmY = _mm_packus_epi16(mmY0, mmY1); // Saturate(I16 -> U8)
			_mm_storel_pi((__m64*)uPtr_, _mm_load_ps((float*)&mmY));
			_mm_storeh_pi((__m64*)vPtr_, _mm_load_ps((float*)&mmY));

			rgbPtr_ += 64; // 16samples * 4bytes
			uPtr_ += 8; // 8samples * 1byte
			vPtr_ += 8; // 8samples * 1byte
		}
	}
#endif
}
#endif

static void RGBFillSamples(uint8 *ptr, size_t size)
{
	for (size_t i = 0; i < size; ++i)
	{
		ptr[i] = rand();
	}
}

static BOOL BuffersAreEqual(const uint8 *ptr1, const uint8 *ptr2, size_t size)
{
	for (size_t i = 0; i < size; ++i)
	{
		if (ptr1[i] != ptr2[i])
		{
			return FALSE;
		}
	}
	return TRUE;
}

int _tmain(int argc, _TCHAR* argv[])
{
	uint8 *inPtr = NULL, *outPtr = NULL;
	int ret = 0;
	uint64 timeStart, timeEnd;

	libyuv::YUVDebugSetLevel(libyuv::YUV_DEBUG_LEVEL_INFO);

	YUV_DEBUG_INFO("=== Starting fast yuv test ===");

	inPtr = (uint8 *)FastMallocAligned(FAST_INPUT_SIZE, FAST_ALIGNMENT);
	FAST_ASSERT(inPtr != NULL);
	FAST_ASSERT(FAST_IS_ALIGNED(inPtr, FAST_ALIGNMENT));
	outPtr = (uint8 *)FastMallocAligned(FAST_OUTPUT_SIZE, FAST_ALIGNMENT);
	FAST_ASSERT(outPtr != NULL);
	FAST_ASSERT(FAST_IS_ALIGNED(outPtr, FAST_ALIGNMENT));

	RGBFillSamples(inPtr, FAST_INPUT_SIZE);

	// Conversion
	timeStart = libyuv::YuvTimeNow();
	for (size_t index = 0; index < FAST_LOOP_COUNT; ++index)
	{
		RGB32ToI420_Asm_SSSE3(outPtr, inPtr, FAST_INPUT_WIDTH, FAST_INPUT_HEIGHT);
		//RGB32ToI420_C(outPtr, inPtr, FAST_INPUT_WIDTH, FAST_INPUT_HEIGHT);
		//RGB32ToI420_Intrin_SSSE3(outPtr, inPtr, FAST_INPUT_WIDTH, FAST_INPUT_HEIGHT);
	}
	timeEnd = libyuv::YuvTimeNow();

	// Comparing results
	{
		uint8 *outPtr_ = (uint8 *)malloc(FAST_OUTPUT_SIZE);
		FAST_ASSERT(outPtr_ != NULL);
		RGB32ToI420_C(outPtr_, inPtr, FAST_INPUT_WIDTH, FAST_INPUT_HEIGHT);
		if (!BuffersAreEqual(outPtr_, outPtr, FAST_OUTPUT_SIZE))
		{
			YUV_DEBUG_FATAL("Result mismatch");
		}
		free(outPtr_);
	}


	YUV_DEBUG_INFO("Converted %d frames in %llu millis\n --> Time for each frame=%.2f millis\n --> Speed=%.2f frames/sec",
		FAST_LOOP_COUNT,
		(timeEnd - timeStart),
		((float)timeEnd - (float)timeStart) / (float)FAST_LOOP_COUNT,
		((float)FAST_LOOP_COUNT / (((float)timeEnd - (float)timeStart) / (float)1000)));

	FastFreeAligned(inPtr);
	FastFreeAligned(outPtr);

	YUV_DEBUG_INFO("=== Ending fast yuv test ===");

	getchar();

	return 0;
}

