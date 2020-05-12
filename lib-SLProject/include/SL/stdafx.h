//#############################################################################
//  File:      SL/stdafx.h
//  Author:    Marcus Hudritsch
//  Date:      July 2014
//  Codestyle: https://github.com/cpvrlab/SLProject/wiki/SLProject-Coding-Style
//  Copyright: Marcus Hudritsch
//             This software is provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//  Purpose:   Include file for standard system include files, or project
//             specific include files that are used frequently, but are changed
//             infrequently. You must set the property C/C++/Precompiled Header
//             as "Use Precompiled Header"
//#############################################################################

#ifndef STDAFX_H
#define STDAFX_H

#define _USE_MATH_DEFINES
//#define NOMINMAX

// Include standard C++ libraries
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <typeinfo>
#include <vector>
#include <memory>
//-----------------------------------------------------------------------------
// Include standard C libraries
#include <cassert> // for debug asserts
#include <cfloat>  // for defines like FLT_MAX & DBL_MAX
#include <climits> // for defines like UINT_MAX
#include <cmath>   // for math functions
#include <cstdio>  // for the old ANSI C IO functions
#include <cstdlib> // srand, rand
#include <cstring> // for string functions
#include <ctime>   // for clock()
//-----------------------------------------------------------------------------
// Core header files used by all files
#include <SL.h>
#include <SLEnums.h>
#include <SLMat3.h>
#include <SLMat4.h>
#include <SLObject.h>
#include <SLQuat4.h>
#include <SLVec2.h>
#include <SLVec3.h>
#include <SLVec4.h>
//-----------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#endif
