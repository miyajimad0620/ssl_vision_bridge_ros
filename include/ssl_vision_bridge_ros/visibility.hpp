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

#ifndef SSL_VISION_BRIDGE_ROS__VISIBILITY_HPP_
#define SSL_VISION_BRIDGE_ROS__VISIBILITY_HPP_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borroexampled (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define SSL_VISION_BRIDGE_ROS_EXPORT __attribute__((dllexport))
#define SSL_VISION_BRIDGE_ROS_IMPORT __attribute__((dllimport))
#else
#define SSL_VISION_BRIDGE_ROS_EXPORT __declspec(dllexport)
#define SSL_VISION_BRIDGE_ROS_IMPORT __declspec(dllimport)
#endif

#ifdef SSL_VISION_BRIDGE_ROS_DLL
#define SSL_VISION_BRIDGE_ROS_PUBLIC SSL_VISION_BRIDGE_ROS_EXPORT
#else
#define SSL_VISION_BRIDGE_ROS_PUBLIC SSL_VISION_BRIDGE_ROS_IMPORT
#endif

#define SSL_VISION_BRIDGE_ROS_PUBLIC_TYPE SSL_VISION_BRIDGE_ROS_PUBLIC

#define SSL_VISION_BRIDGE_ROS_LOCAL

#else

#define SSL_VISION_BRIDGE_ROS_EXPORT __attribute__((visibility("default")))
#define SSL_VISION_BRIDGE_ROS_IMPORT

#if __GNUC__ >= 4
#define SSL_VISION_BRIDGE_ROS_PUBLIC __attribute__((visibility("default")))
#define SSL_VISION_BRIDGE_ROS_LOCAL __attribute__((visibility("hidden")))
#else
#define SSL_VISION_BRIDGE_ROS_PUBLIC
#define SSL_VISION_BRIDGE_ROS_LOCAL
#endif

#define SSL_VISION_BRIDGE_ROS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // SSL_VISION_BRIDGE_ROS__VISIBILITY_HPP_
