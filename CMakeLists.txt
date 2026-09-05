cmake_minimum_required(VERSION 3.16)
project(gravity_comp VERSION 0.3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(GCT_ENABLE_MUJOCO "Build MuJoCo simulator, scene, and related tools" OFF)
option(GCT_BUILD_EXAMPLES "Build example programs" ON)
option(GCT_BUILD_TESTS "Build unit tests" ON)

# Pinocchio / Eigen / Catch2: activated conda env, ROS, or GCT_PINOCCHIO_ROOT.
set(GCT_PINOCCHIO_ROOT "" CACHE PATH
    "Optional extra prefix for Pinocchio (e.g. ROS). Default: \$CONDA_PREFIX")
if(GCT_PINOCCHIO_ROOT STREQUAL "${CMAKE_SOURCE_DIR}/.deps")
  message(STATUS "Ignoring stale GCT_PINOCCHIO_ROOT=.deps; activate a conda env instead")
  unset(GCT_PINOCCHIO_ROOT CACHE)
  set(GCT_PINOCCHIO_ROOT "" CACHE PATH
      "Optional extra prefix for Pinocchio (e.g. ROS). Default: \$CONDA_PREFIX")
endif()

set(_gct_prefix "${GCT_PINOCCHIO_ROOT}")
if(NOT _gct_prefix AND DEFINED ENV{CONDA_PREFIX})
  set(_gct_prefix "$ENV{CONDA_PREFIX}")
endif()
if(_gct_prefix)
  list(PREPEND CMAKE_PREFIX_PATH "${_gct_prefix}")
  if(EXISTS "${_gct_prefix}/lib")
    set(CMAKE_BUILD_RPATH "${_gct_prefix}/lib")
    set(CMAKE_INSTALL_RPATH "${_gct_prefix}/lib")
  endif()
  message(STATUS "Dependency prefix: ${_gct_prefix}")
endif()

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_library(gct
  src/pin_model.cpp
  src/gravity_compensator.cpp
)
add_library(gct::gct ALIAS gct)
target_include_directories(gct
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(gct PUBLIC Eigen3::Eigen pinocchio::pinocchio)
target_compile_options(gct PRIVATE -Wall -Wextra -Wpedantic)

# MuJoCo scene keeps Menagerie MJCF armature unless GCT_JOINT_ARMATURE is set.
set(GCT_JOINT_ARMATURE "" CACHE STRING
    "Optional uniform MuJoCo armature override; empty keeps Menagerie MJCF values")
set(GCT_MENAGERIE_UR5E "${CMAKE_SOURCE_DIR}/third_party/mujoco_menagerie/universal_robots_ur5e")
set(GCT_MJCF_PATH "${GCT_MENAGERIE_UR5E}/ur5e.xml")
set(GCT_MODEL_PATH "${GCT_MJCF_PATH}")
set(GCT_MUJOCO_SCENE_PATH
    "${CMAKE_BINARY_DIR}/models/ur5e_menagerie_torque.mjb")

function(gct_use_test_headers target)
  target_include_directories(${target} PRIVATE
    "${CMAKE_SOURCE_DIR}/tests/fixtures"
  )
  target_compile_definitions(${target} PRIVATE
    GCT_TEST_MODEL_PATH="${GCT_MODEL_PATH}"
  )
endfunction()

if(GCT_ENABLE_MUJOCO)
  set(GCT_MUJOCO_ROOT "" CACHE PATH
      "Directory containing libmujoco and include/mujoco")

  set(_gct_mj_roots)
  if(GCT_MUJOCO_ROOT)
    list(APPEND _gct_mj_roots "${GCT_MUJOCO_ROOT}")
  endif()
  if(DEFINED ENV{CONDA_PREFIX})
    list(APPEND _gct_mj_roots "$ENV{CONDA_PREFIX}")
    file(GLOB _gct_conda_mj
      "$ENV{CONDA_PREFIX}/lib/python*/site-packages/mujoco")
    list(APPEND _gct_mj_roots ${_gct_conda_mj})
  endif()

  set(MUJOCO_LIBRARY "")
  set(_gct_mj_root "")
  foreach(_root IN LISTS _gct_mj_roots)
    if(NOT IS_DIRECTORY "${_root}")
      continue()
    endif()
    set(_lib "")
    if(EXISTS "${_root}/lib/libmujoco.so")
      set(_lib "${_root}/lib/libmujoco.so")
    else()
      file(GLOB _libs "${_root}/libmujoco.so*")
      if(_libs)
        list(GET _libs 0 _lib)
      endif()
    endif()
    if(_lib)
      set(MUJOCO_LIBRARY "${_lib}")
      set(_gct_mj_root "${_root}")
      break()
    endif()
  endforeach()

  if(NOT MUJOCO_LIBRARY)
    find_library(MUJOCO_LIBRARY NAMES mujoco)
  endif()
  if(NOT MUJOCO_LIBRARY)
    message(FATAL_ERROR
      "MuJoCo not found. Pass -DGCT_MUJOCO_ROOT=/path/to/mujoco "
      "(libmujoco + include/mujoco) or -DGCT_ENABLE_MUJOCO=OFF.")
  endif()
  if(NOT _gct_mj_root)
    get_filename_component(_gct_mj_root "${MUJOCO_LIBRARY}" DIRECTORY)
  endif()

  set(GCT_MUJOCO_INCLUDE "")
  foreach(_inc IN ITEMS
      "${_gct_mj_root}/include"
      "${_gct_mj_root}/../include")
    if(EXISTS "${_inc}/mujoco/mujoco.h")
      get_filename_component(GCT_MUJOCO_INCLUDE "${_inc}" ABSOLUTE)
      break()
    endif()
  endforeach()
  if(NOT GCT_MUJOCO_INCLUDE)
    find_path(GCT_MUJOCO_INCLUDE mujoco/mujoco.h)
  endif()
  if(NOT GCT_MUJOCO_INCLUDE)
    message(FATAL_ERROR "MuJoCo headers not found next to ${MUJOCO_LIBRARY}")
  endif()

  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  set(_gct_scene_python "${Python3_EXECUTABLE}")
  if(_gct_mj_root MATCHES "site-packages/mujoco/?$")
    get_filename_component(_gct_mj_site "${_gct_mj_root}" DIRECTORY)
    get_filename_component(_gct_mj_pyver "${_gct_mj_site}" DIRECTORY)
    get_filename_component(_gct_mj_lib "${_gct_mj_pyver}" DIRECTORY)
    get_filename_component(_gct_mj_env "${_gct_mj_lib}" DIRECTORY)
    foreach(_py IN ITEMS "${_gct_mj_env}/bin/python3" "${_gct_mj_env}/bin/python")
      if(EXISTS "${_py}")
        set(_gct_scene_python "${_py}")
        break()
      endif()
    endforeach()
  endif()

  if(NOT EXISTS "${GCT_MJCF_PATH}")
    message(FATAL_ERROR
      "MuJoCo Menagerie UR5e not found at ${GCT_MJCF_PATH}. "
      "Run: ./tools/setup_menagerie.sh")
  endif()

  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/models")
  if(GCT_JOINT_ARMATURE STREQUAL "")
    set(_gct_armature_args)
  else()
    set(_gct_armature_args --armature "${GCT_JOINT_ARMATURE}")
  endif()

  add_custom_command(
    OUTPUT "${GCT_MUJOCO_SCENE_PATH}"
    COMMAND "${_gct_scene_python}" "${CMAKE_SOURCE_DIR}/tools/build_menagerie_torque_scene.py"
            --mjcf "${GCT_MJCF_PATH}"
            --output "${GCT_MUJOCO_SCENE_PATH}"
            ${_gct_armature_args}
    DEPENDS "${GCT_MJCF_PATH}" "${CMAKE_SOURCE_DIR}/tools/build_menagerie_torque_scene.py"
    COMMENT "Building Menagerie UR5e torque scene (ur5e_menagerie_torque.mjb)"
    VERBATIM
  )
  add_custom_target(gct_mujoco_scene DEPENDS "${GCT_MUJOCO_SCENE_PATH}")

  add_library(gct_mujoco
    src/mujoco_arm_simulator.cpp
    src/gravity_validation.cpp
  )
  add_library(gct::mujoco ALIAS gct_mujoco)
  target_link_libraries(gct_mujoco PUBLIC gct)
  target_link_libraries(gct_mujoco PRIVATE "${MUJOCO_LIBRARY}")
  target_include_directories(gct_mujoco
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
      "${GCT_MUJOCO_INCLUDE}"
  )
  target_compile_options(gct_mujoco PRIVATE -Wall -Wextra -Wpedantic)
  get_filename_component(MUJOCO_LIB_DIR "${MUJOCO_LIBRARY}" DIRECTORY)
  set_property(TARGET gct_mujoco APPEND PROPERTY BUILD_RPATH "${MUJOCO_LIB_DIR}")
  if(_gct_prefix AND EXISTS "${_gct_prefix}/lib")
    set_property(TARGET gct_mujoco APPEND PROPERTY BUILD_RPATH "${_gct_prefix}/lib")
  endif()
endif()

if(GCT_BUILD_EXAMPLES)
  if(GCT_ENABLE_MUJOCO)
    add_custom_target(ur5e_mujoco_viewer
      COMMAND "${_gct_scene_python}" "${CMAKE_SOURCE_DIR}/examples/ur5e_mujoco_viewer.py"
              --mode compare --real-time
      DEPENDS gct_mujoco_scene
      USES_TERMINAL
      COMMENT "Launch UR5e viewer: all three modes in sequence"
    )
    add_custom_target(ur5e_mujoco_viewer_no_gc
      COMMAND "${_gct_scene_python}" "${CMAKE_SOURCE_DIR}/examples/ur5e_mujoco_viewer.py"
              --mode no-gc --real-time
      DEPENDS gct_mujoco_scene
      USES_TERMINAL
      COMMENT "Launch UR5e viewer: mode 1 no gravity compensation"
    )
    add_custom_target(ur5e_mujoco_viewer_hold
      COMMAND "${_gct_scene_python}" "${CMAKE_SOURCE_DIR}/examples/ur5e_mujoco_viewer.py"
              --mode hold --real-time
      DEPENDS gct_mujoco_scene
      USES_TERMINAL
      COMMENT "Launch UR5e viewer: mode 2 gravity hold"
    )
    add_custom_target(ur5e_mujoco_viewer_push
      COMMAND "${_gct_scene_python}" "${CMAKE_SOURCE_DIR}/examples/ur5e_mujoco_viewer.py"
              --mode push --law gravity --real-time
      DEPENDS gct_mujoco_scene
      USES_TERMINAL
      COMMENT "Launch UR5e viewer: push with gravity only"
    )
    add_custom_target(ur5e_mujoco_viewer_pd
      COMMAND "${_gct_scene_python}" "${CMAKE_SOURCE_DIR}/examples/ur5e_mujoco_viewer.py"
              --mode push --law pd --real-time
      DEPENDS gct_mujoco_scene
      USES_TERMINAL
      COMMENT "Launch UR5e viewer: push with PD+gravity (recovers home)"
    )
    add_custom_target(ur5e_mujoco_viewer_servo
      COMMAND "${_gct_scene_python}" "${CMAKE_SOURCE_DIR}/examples/ur5e_mujoco_viewer.py"
              --mode push --law position-servo --real-time
      DEPENDS gct_mujoco_scene
      USES_TERMINAL
      COMMENT "Launch UR5e viewer: push with position-servo (recovers home)"
    )
  endif()
endif()

if(GCT_BUILD_TESTS)
  find_package(Catch2 3 REQUIRED)
  set(GCT_TEST_SOURCES
    tests/test_gravity_comp.cpp
  )
  if(GCT_ENABLE_MUJOCO)
    list(APPEND GCT_TEST_SOURCES tests/test_mujoco.cpp)
  endif()
  add_executable(gct_tests ${GCT_TEST_SOURCES})
  target_link_libraries(gct_tests PRIVATE gct Catch2::Catch2 Catch2::Catch2WithMain)
  gct_use_test_headers(gct_tests)
  target_compile_options(gct_tests PRIVATE -Wall -Wextra -Wpedantic)
  if(GCT_ENABLE_MUJOCO)
    add_dependencies(gct_tests gct_mujoco_scene)
    target_link_libraries(gct_tests PRIVATE gct_mujoco)
    target_compile_definitions(gct_tests PRIVATE
      GCT_TEST_MUJOCO_SCENE_PATH="${GCT_MUJOCO_SCENE_PATH}"
    )
    set_property(TARGET gct_tests APPEND PROPERTY BUILD_RPATH "${MUJOCO_LIB_DIR}")
  endif()

  enable_testing()
  add_test(NAME gct_tests COMMAND gct_tests)
  set(_gct_test_rpath "")
  if(_gct_prefix)
    set(_gct_test_rpath "${_gct_prefix}/lib")
  endif()
  if(GCT_ENABLE_MUJOCO)
    set(_gct_test_rpath "${MUJOCO_LIB_DIR}:${_gct_test_rpath}")
  endif()
  set_tests_properties(gct_tests PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=${_gct_test_rpath}")
endif()
