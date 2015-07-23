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
#if !defined (DOUBANGO_YUV_DEBUG_H)
#define DOUBANGO_YUV_DEBUG_H

#include "libyuv/yuv_config.h"
#include <stdio.h>

#ifdef __cplusplus
namespace libyuv {
extern "C" {
#endif

typedef enum YUV_DEBUG_LEVEL
{
	YUV_DEBUG_LEVEL_INFO = 4,
	YUV_DEBUG_LEVEL_WARN = 3,
	YUV_DEBUG_LEVEL_ERROR = 2,
	YUV_DEBUG_LEVEL_FATAL = 1
}
YUV_DEBUG_LEVEL;

typedef int (*YUVDebugFuncPtr)(const void* arg, const char* fmt, ...);

	/* INFO */
#define YUV_DEBUG_INFO(FMT, ...)		\
	if (libyuv::YUVDebugGetLevel() >= libyuv::YUV_DEBUG_LEVEL_INFO) { \
		if (libyuv::YUVDebugGetInfoCallback()) \
			libyuv::YUVDebugGetInfoCallback()(libyuv::YUVDebugGetArgData(), "*[DOUBANGO INFO]: " FMT "\n", ##__VA_ARGS__); \
		else \
			fprintf(stderr, "*[LIBYUV INFO]: " FMT "\n", ##__VA_ARGS__); \
	}


	/* WARN */
#define YUV_DEBUG_WARN(FMT, ...)		\
	if (libyuv::YUVDebugGetLevel() >= libyuv::YUV_DEBUG_LEVEL_WARN) { \
		if (libyuv::YUVDebugGetWarnCallback()) \
			libyuv::YUVDebugGetWarnCallback()(libyuv::YUVDebugGetArgData(), "**[DOUBANGO WARN]: function: \"%s()\" \nfile: \"%s\" \nline: \"%u\" \nMSG: " FMT "\n", __FUNCTION__,  __FILE__, __LINE__, ##__VA_ARGS__); \
		else \
			fprintf(stderr, "**[LIBYUV WARN]: function: \"%s()\" \nfile: \"%s\" \nline: \"%u\" \nMSG: " FMT "\n", __FUNCTION__,  __FILE__, __LINE__, ##__VA_ARGS__); \
	}

	/* ERROR */
#define YUV_DEBUG_ERROR(FMT, ...) 		\
	if (libyuv::YUVDebugGetLevel() >= libyuv::YUV_DEBUG_LEVEL_ERROR) { \
		if (libyuv::YUVDebugGetErrorCallback()) \
			libyuv::YUVDebugGetErrorCallback()(libyuv::YUVDebugGetArgData(), "***[DOUBANGO ERROR]: function: \"%s()\" \nfile: \"%s\" \nline: \"%u\" \nMSG: " FMT "\n", __FUNCTION__,  __FILE__, __LINE__, ##__VA_ARGS__); \
		else \
			fprintf(stderr, "***[LIBYUV ERROR]: function: \"%s()\" \nfile: \"%s\" \nline: \"%u\" \nMSG: " FMT "\n", __FUNCTION__,  __FILE__, __LINE__, ##__VA_ARGS__); \
	}


	/* FATAL */
#define YUV_DEBUG_FATAL(FMT, ...) 		\
	if (libyuv::YUVDebugGetLevel() >= libyuv::YUV_DEBUG_LEVEL_FATAL) { \
		if (libyuv::YUVDebugGetFatalCallback()) \
			libyuv::YUVDebugGetFatalCallback()(libyuv::YUVDebugGetArgData(), "****[DOUBANGO FATAL]: function: \"%s()\" \nfile: \"%s\" \nline: \"%u\" \nMSG: " FMT "\n", __FUNCTION__,  __FILE__, __LINE__, ##__VA_ARGS__); \
		else \
			fprintf(stderr, "****[LIBYUV FATAL]: function: \"%s()\" \nfile: \"%s\" \nline: \"%u\" \nMSG: " FMT "\n", __FUNCTION__,  __FILE__, __LINE__, ##__VA_ARGS__); \
	}


LIBYUV_API void YUVDebugSetArgData(const void*);
LIBYUV_API const void* YUVDebugGetArgData();
LIBYUV_API void YUVDebugSetInfoCallback(YUVDebugFuncPtr );
LIBYUV_API YUVDebugFuncPtr YUVDebugGetInfoCallback();
LIBYUV_API void YUVDebugSetWarnCallback(YUVDebugFuncPtr );
LIBYUV_API YUVDebugFuncPtr YUVDebugGetWarnCallback();
LIBYUV_API void YUVDebugSetErrorCallback(YUVDebugFuncPtr );
LIBYUV_API YUVDebugFuncPtr YUVDebugGetErrorCallback( );
LIBYUV_API void YUVDebugSetFatalCallback(YUVDebugFuncPtr );
LIBYUV_API YUVDebugFuncPtr YUVDebugGetFatalCallback( );
LIBYUV_API YUV_DEBUG_LEVEL YUVDebugGetLevel( );
LIBYUV_API void YUVDebugSetLevel(YUV_DEBUG_LEVEL );


#ifdef __cplusplus
}  // extern "C"
}  // namespace libyuv
#endif

#endif /* DOUBANGO_YUV_DEBUG_H */