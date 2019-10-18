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

#ifndef DYNAMIC_TUTORIAL__VISIBILITY_CONTROL_H_
#define DYNAMIC_TUTORIAL__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DYNAMIC_TUTORIAL_EXPORT __attribute__ ((dllexport))
    #define DYNAMIC_TUTORIAL_IMPORT __attribute__ ((dllimport))
  #else
    #define DYNAMIC_TUTORIAL_EXPORT __declspec(dllexport)
    #define DYNAMIC_TUTORIAL_IMPORT __declspec(dllimport)
  #endif
  #ifdef DYNAMIC_TUTORIAL_BUILDING_DLL
    #define DYNAMIC_TUTORIAL_PUBLIC DYNAMIC_TUTORIAL_EXPORT
  #else
    #define DYNAMIC_TUTORIAL_PUBLIC DYNAMIC_TUTORIAL_IMPORT
  #endif
  #define DYNAMIC_TUTORIAL_PUBLIC_TYPE DYNAMIC_TUTORIAL_PUBLIC
  #define DYNAMIC_TUTORIAL_LOCAL
#else
  #define DYNAMIC_TUTORIAL_EXPORT __attribute__ ((visibility("default")))
  #define DYNAMIC_TUTORIAL_IMPORT
  #if __GNUC__ >= 4
    #define DYNAMIC_TUTORIAL_PUBLIC __attribute__ ((visibility("default")))
    #define DYNAMIC_TUTORIAL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DYNAMIC_TUTORIAL_PUBLIC
    #define DYNAMIC_TUTORIAL_LOCAL
  #endif
  #define DYNAMIC_TUTORIAL_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIC_TUTORIAL__VISIBILITY_CONTROL_H_