cmake_minimum_required(VERSION 3.8)
project(LR LANGUAGES CXX)

# 기존 설정
set(CMAKE_DEBUG_POSTFIX "d")
set(BUILD_SHARED_LIBS FALSE)
set(JOINTNUM 6 CACHE STRING "Number of joints")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()
find_package(urdfdom REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ruckig REQUIRED)
find_package(jsoncpp REQUIRED)

file(REMOVE ${PROJECT_SOURCE_DIR}/include/type.h)
configure_file(${PROJECT_SOURCE_DIR}/include/type_template.h.in ${PROJECT_SOURCE_DIR}/include/type.h)
set(COMMON_SRCS src/liegroup_robotics.cpp )
set(CONTROL_SRCS src/LR_Control.cpp)
set(TRAJ_SRCS src/LR_Trajectory.cpp)

add_library(Control STATIC ${CONTROL_SRCS} ${COMMON_SRCS})
add_library(Trajectory STATIC ${TRAJ_SRCS} ${COMMON_SRCS})
add_library(LR::Control ALIAS Control)
add_library(LR::Trajectory ALIAS Trajectory)



set(RUCKIG_THIRDPARTY /opt/ruckig/third_party/)

target_include_directories(Control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/LR>
)
target_include_directories(Trajectory PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/LR>
  ${RUCKIG_THIRDPARTY}
)

target_link_libraries(Control PUBLIC Eigen3::Eigen pinocchio::pinocchio ruckig::ruckig ${urdfdom_LIBRARIES})
target_link_libraries(Trajectory PUBLIC Eigen3::Eigen ruckig::ruckig )

# 설치 설정

install(TARGETS Control EXPORT LRTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
install(TARGETS Trajectory EXPORT LRTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
        
install(DIRECTORY ${PROJECT_SOURCE_DIR}/
        DESTINATION include/LR
        FILES_MATCHING PATTERN "*.h")
# Export Targets 설정
install(EXPORT LRTargets
        FILE LRTargets.cmake
        NAMESPACE LR::
        DESTINATION lib/cmake/LR)

# CMake 패키지 설정 파일 생성
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/LRConfig.cmake"
    INSTALL_DESTINATION lib/cmake/LR
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/LRConfig.cmake"
        DESTINATION lib/cmake/LR)

