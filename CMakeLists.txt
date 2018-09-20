cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

project (raspigcd2 CXX)

set (raspigcd2_VERSION_MAJOR 0)
set (raspigcd2_VERSION_MINOR 1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_definitions(-Wall)

install(DIRECTORY include/ DESTINATION include/raspigcd2
        FILES_MATCHING PATTERN "*.hpp")


FIND_PACKAGE( OpenMP )
  if(OPENMP_FOUND)
     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
     set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  endif()
include_directories(SYSTEM ${OpenMP_INCLUDE_PATH})


include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/thirdparty")



set(raspigcd2_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/raspigcd.cpp )
file(GLOB_RECURSE lib_SOURCES "src/*.cpp")
file(GLOB_RECURSE raspigcd2_TESTS "tests/*.cpp")
list(REMOVE_ITEM lib_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/raspigcd.cpp)

add_library(raspigcd2 SHARED ${lib_SOURCES})
add_executable(gcd ${raspigcd2_SOURCES} ${lib_SOURCES})

install (TARGETS raspigcd2 DESTINATION lib)


enable_testing()

add_executable(executor_t_test ${CMAKE_CURRENT_SOURCE_DIR}/tests/executor_t_test.cpp ${lib_SOURCES})
target_link_libraries(executor_t_test LINK_INTERFACE_LIBRARIES raspigcd2)
add_test(NAME executor_t_test COMMAND "${CMAKE_BINARY_DIR}/executor_t_test" WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" )


add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND}
                  DEPENDS executor_t_test)


SET(CPACK_PACKAGE_VERSION_MAJOR ${raspigcd2_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${raspigcd2_VERSION_MINOR})
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
SET(CPACK_GENERATOR "DEB" "TGZ")

SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Tadeusz Puźniakowski")
INCLUDE(CPack)


 
# To use this:
# make package

