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
#if !defined (DOUBANGO_YUV_TIME_H)
#define DOUBANGO_YUV_TIME_H

#include "libyuv/yuv_config.h"
#include "libyuv/basic_types.h"

#ifdef __cplusplus
namespace libyuv {
extern "C" {
#endif

LIBYUV_API uint64 YuvTimeEpoch();
LIBYUV_API uint64 YuvTimeNow();

#ifdef __cplusplus
}  // extern "C"
}  // namespace libyuv
#endif

#endif /* DOUBANGO_YUV_TIME_H */
