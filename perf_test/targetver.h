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
#if !defined(PERF_TEST_TARGETVER_H)
#define PERF_TEST_TARGETVER_H

// The following macros define the minimum required platform.  The minimum required platform
// is the earliest version of Windows, Internet Explorer etc. that has the necessary features to run 
// your application.  The macros work by enabling all features available on platform versions up to and 
// including the version specified.

// Modify the following defines if you have to target a platform prior to the ones specified below.
// Refer to MSDN for the latest info on corresponding values for different platforms.
#if !defined(_WIN32_WINNT) && !defined(_WIN32_WCE)            // Specifies that the minimum required platform is Windows Vista.
#define _WIN32_WINNT 0x0501     // Change this to the appropriate value to target other versions of Windows.
#endif
#endif /* PERF_TEST_TARGETVER_H */
