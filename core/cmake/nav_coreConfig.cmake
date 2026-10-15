# nav_core 的 CMake 包配置文件：只负责把导出的 target 引进来。
# find_package(nav_core) 之后即可使用 target：nav::nav_core
include("${CMAKE_CURRENT_LIST_DIR}/nav_coreTargets.cmake")
