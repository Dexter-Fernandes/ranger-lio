include(FetchContent)
# The devcontainer image ships the pinned GoogleTest source at $RANGER_GTEST_SRC so
# configure never touches the network; outside the image FetchContent downloads it.
if(DEFINED ENV{RANGER_GTEST_SRC} AND EXISTS "$ENV{RANGER_GTEST_SRC}/CMakeLists.txt")
  set(FETCHCONTENT_SOURCE_DIR_GOOGLETEST "$ENV{RANGER_GTEST_SRC}")
endif()
FetchContent_Declare(googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.18.0
  GIT_SHALLOW    TRUE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

add_library(ranger_test_support INTERFACE)
target_include_directories(ranger_test_support INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

function(ranger_add_test name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE ranger_lio::ranger_lio ranger_test_support GTest::gtest_main)
  gtest_discover_tests(${name})
endfunction()

ranger_add_test(test_so3)
ranger_add_test(test_se3)
