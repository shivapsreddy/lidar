// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LIDAR_THERMAL_CPP__VISIBILITY_CONTROL_H_
#define LIDAR_THERMAL_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

//#define LIDAR_THERMAL_CPP_STD_MSG 0
// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LIDAR_THERMAL_CPP_EXPORT __attribute__ ((dllexport))
    #define LIDAR_THERMAL_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define LIDAR_THERMAL_CPP_EXPORT __declspec(dllexport)
    #define LIDAR_THERMAL_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef LIDAR_THERMAL_CPP_BUILDING_DLL
    #define LIDAR_THERMAL_CPP_PUBLIC LIDAR_THERMAL_CPP_EXPORT
  #else
    #define LIDAR_THERMAL_CPP_PUBLIC LIDAR_THERMAL_CPP_IMPORT
  #endif
  #define LIDAR_THERMAL_CPP_PUBLIC_TYPE LIDAR_THERMAL_CPP_PUBLIC
  #define LIDAR_THERMAL_CPP_LOCAL
#else
  #define LIDAR_THERMAL_CPP_EXPORT __attribute__ ((visibility("default")))
  #define LIDAR_THERMAL_CPP_IMPORT
  #if __GNUC__ >= 4
    #define LIDAR_THERMAL_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define LIDAR_THERMAL_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LIDAR_THERMAL_CPP_PUBLIC
    #define LIDAR_THERMAL_CPP_LOCAL
  #endif
  #define LIDAR_THERMAL_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_THERMAL_CPP__VISIBILITY_CONTROL_H_
