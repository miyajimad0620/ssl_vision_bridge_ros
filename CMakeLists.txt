# Copyright 2024 TRAPS

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# cmakeの設定
cmake_minimum_required(VERSION 3.8)
project(ssl_vision_bridge_ros)
string(TOUPPER "${PROJECT_NAME}" PROJECT_NAME_UPPER)

# コンパイルオプション
add_compile_options(-O2)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 必要なパッケージのinclude
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Protobuf REQUIRED)

#protoファイルのビルド
file(GLOB proto_files
  LIST_DIRECTORIES false
  "${CMAKE_CURRENT_SOURCE_DIR}/proto/*")
protobuf_generate_cpp(proto_src_files proto_hdr_files "${proto_files}")
add_library("${PROJECT_NAME}_proto_lib" "${proto_src_files}")
target_link_libraries("${PROJECT_NAME}_proto_lib"
  PUBLIC   protobuf::libprotobuf
)
target_include_directories("${PROJECT_NAME}_proto_lib"
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
)
target_compile_options("${PROJECT_NAME}_proto_lib" PRIVATE -fPIC)

# interfaceファイル(msg, srv, action)のビルド
file(GLOB msg_files
  LIST_DIRECTORIES false
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "msg/*")
file(GLOB srv_files
  LIST_DIRECTORIES false
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "srv/*")
file(GLOB action_files
  LIST_DIRECTORIES false
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "action/*")
list(APPEND interface_files ${msg_files} ${srv_files} ${action_files})
list(LENGTH interface_files interface_files_count)
if(interface_files_count GREATER 0)
  rosidl_generate_interfaces("${PROJECT_NAME}"
    "${interface_files}"
    DEPENDENCIES "${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}")
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
endif()

# c++ファイルのビルド
file(GLOB_RECURSE "${PROJECT_NAME}_lib_files"
  LIST_DIRECTORIES false
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/*/*")
list(LENGTH ${PROJECT_NAME}_lib_files ${PROJECT_NAME}_lib_files_count)
if(${PROJECT_NAME}_lib_files_count GREATER 0)
  ament_auto_add_library("${PROJECT_NAME}_lib" SHARED "${${PROJECT_NAME}_lib_files}")
  target_compile_features("${PROJECT_NAME}_lib" PUBLIC c_std_99 cxx_std_17)
  target_link_libraries("${PROJECT_NAME}_lib" "${cpp_typesupport_target}" "${PROJECT_NAME}_proto_lib" fmt Eigen3::Eigen)
endif()

# ノードプラグインの設定
rclcpp_components_register_node("${PROJECT_NAME}_lib"
  PLUGIN "${PROJECT_NAME}::WrapperAsyncBridgeNode"
  EXECUTABLE wrapper_async_bridge_node)

# ディレクトリのinstall
set(install_directories config;launch;${${PROJECT_NAME_UPPER}_INSTALL_DIRECTORIES})
foreach(install_directory IN LISTS install_directories)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${install_directory}")
    install(DIRECTORY
      "${install_directory}"
      DESTINATION share/${PROJECT_NAME}/)
  endif()
endforeach()

# テスト
if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
  # file(GLOB_RECURSE "${PROJECT_NAME}_test_src_files"
  #   LIST_DIRECTORIES false
  #   RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  #   "test/*.cpp")
  # ament_auto_add_gtest(${PROJECT_NAME}_test
  #   "${${PROJECT_NAME}_test_src_files}"
  #   TIMEOUT "30")
  # target_compile_features("${PROJECT_NAME}_test" PUBLIC c_std_99 cxx_std_17)
  # target_link_libraries("${PROJECT_NAME}_test" "${cpp_typesupport_target}" fmt)
endif()

# パッケージング
ament_auto_package()
