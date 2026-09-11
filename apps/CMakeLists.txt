# check_deps: proves that GTSAM, small_gicp and the core library link together.
add_executable(check_deps check_deps.cpp)
target_link_libraries(check_deps PRIVATE ranger_lio::ranger_lio)
