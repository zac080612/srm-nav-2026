cmake_minimum_required(VERSION 3.16)
project(nav_core CXX)

# ---------------------------------------------------------------------------
# core/ 是「纯 C++」层：零第三方依赖、零 ROS 依赖。
# 目的：Windows / WSL / NUC 上都能编译和跑单测，把上车才会咬人的逻辑
#       （协议、控制律、状态机、R_bc 映射与标定算法）在离机阶段就验完。
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# ---- 核心库 ----
add_library(nav_core STATIC
  src/crc.cpp
  src/frame.cpp
  src/safe_cmd.cpp
  src/rbc_map.cpp
  src/control.cpp
  src/state_machine.cpp
  src/calib.cpp
  src/pile_detect.cpp
  src/pile_frame.cpp
)
target_include_directories(nav_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(MSVC)
  # /utf-8：源文件是 UTF-8（中文注释），MSVC 默认按本地代码页 GBK 解析会报错
  # _USE_MATH_DEFINES：MSVC 的 <cmath> 默认不提供 M_PI
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /utf-8 /D_USE_MATH_DEFINES")
  target_compile_options(nav_core PRIVATE /W4)
else()
  target_compile_options(nav_core PRIVATE -Wall -Wextra)
  # hypot / isfinite 在部分工具链上需要显式链接数学库
  target_link_libraries(nav_core PUBLIC m)
endif()

# ---- 测试：每个测试一个可执行文件 + 一个 ctest 用例 ----
# 用自带的 test_util.hpp（零依赖），所以不需要联网拉 googletest。
# 以后在 ROS 环境里想换 gtest：把下面的 link 换成 ament_cmake_gtest / GTest::gtest_main，
# 并把 test_util.hpp 的宏替换掉即可，测试文件本身的写法不用改。
enable_testing()

set(NAV_TESTS
  test_crc
  test_frame
  test_safe_cmd
  test_rbc
  test_control
  test_state_machine
  test_calib
  test_pile_detect
  test_pile_frame
  test_pile_structure
)

foreach(t ${NAV_TESTS})
  add_executable(${t} test/${t}.cpp)
  # 每个测试自带 main()（返回失败数），所以**不要**链接 gtest_main 之类
  target_link_libraries(${t} PRIVATE nav_core)
  target_include_directories(${t} PRIVATE include)
  add_test(NAME ${t} COMMAND ${t})
endforeach()

# ---- 安装：让 ros/ 包能 find_package(nav_core) 链接本库 ----
# 用法：cmake --install <build> --prefix <某目录>，然后 ros 包构建时加
#       -DCMAKE_PREFIX_PATH=<该目录>。脚本 scripts/build.sh 已经封装好这一步。
include(GNUInstallDirs)
install(TARGETS nav_core
        EXPORT nav_coreTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT nav_coreTargets
        FILE nav_coreTargets.cmake
        NAMESPACE nav::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nav_core)
install(FILES cmake/nav_coreConfig.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nav_core)
