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
#if !defined (DOUBANGO_YUV_CONFIG_H)
#define DOUBANGO_YUV_CONFIG_H

#include "libyuv/basic_types.h"

#ifdef __cplusplus
namespace libyuv {
extern "C" {
#endif

#ifdef __SYMBIAN32__
#undef _WIN32 /* Because of WINSCW */
#endif

// Windows (XP/Vista/7/CE and Windows Mobile) macro definition.
#if defined(WIN32)|| defined(_WIN32) || defined(_WIN32_WCE)
#	define YUV_UNDER_WINDOWS	1
#	if defined(_WIN32_WCE) || defined(UNDER_CE)
#		define YUV_UNDER_WINDOWS_CE	1
#		define YUV_STDCALL			__cdecl
#	else
#		define YUV_STDCALL __stdcall
#	endif
#	if defined(WINAPI_FAMILY) && (WINAPI_FAMILY == WINAPI_FAMILY_PHONE_APP || WINAPI_FAMILY == WINAPI_FAMILY_APP)
#		define YUV_UNDER_WINDOWS_RT		1
#	endif
#else
#	define YUV_STDCALL
#endif

// OS X or iOS
#if defined(__APPLE__)
#	define YUV_UNDER_APPLE				1
#   include <TargetConditionals.h>
#   include <Availability.h>
#endif
#if TARGET_OS_MAC
#	define YUV_UNDER_MAC				1
#endif
#if TARGET_OS_IPHONE
#	define YUV_UNDER_IPHONE				1
#endif
#if TARGET_IPHONE_SIMULATOR
#	define YUV_UNDER_IPHONE_SIMULATOR	1
#endif

/* Disable some well-known warnings
*/
#ifdef _MSC_VER
#	if !defined(_CRT_SECURE_NO_WARNINGS)
#		define _CRT_SECURE_NO_WARNINGS
#	endif /* _CRT_SECURE_NO_WARNINGS */
#endif

#ifdef __cplusplus
}  // extern "C"
}  // namespace libyuv
#endif

#endif /* DOUBANGO_YUV_CONFIG_H */