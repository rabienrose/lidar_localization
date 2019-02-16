cmake_minimum_required(VERSION 2.8.3)
project(desc_matcher)

find_package(catkin_simple 0.1.0 REQUIRED)
catkin_simple()

add_definitions(-std=c++11 -DBENCHMARK_ENABLE)

cs_add_executable(desc_matcher
  src/main.cpp
  src/desc_matcher.cpp
  src/chamo_util.cpp
)

cs_install()
cs_export()
