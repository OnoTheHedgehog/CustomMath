cmake_minimum_required(VERSION 3.9)

project(maths VERSION 1.0.0 DESCRIPTION "Maths Library")

add_library(maths SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mat4.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/quat.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/vec3.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/transform.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/dualQuaternion.cpp
)


set_target_properties(maths PROPERTIES
            CXX_STANDARD 17)

if (MSVC)
  set_target_properties(maths PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()
target_include_directories(maths PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

