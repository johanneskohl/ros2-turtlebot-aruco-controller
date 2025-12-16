cmake_minimum_required(VERSION 3.8)


target_sources(main
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/node.cpp
)
target_sources(make-marker
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/make-marker-main.cpp
)
