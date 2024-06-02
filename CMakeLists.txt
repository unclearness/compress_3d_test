cmake_minimum_required(VERSION 3.17)

set(PROJECT_NAME compress_3d_test)
project(${PROJECT_NAME} LANGUAGES CXX C VERSION 0.0.1 DESCRIPTION "compress_3d_test")

set(CMAKE_VERBOSE_MAKEFILE TRUE)

# .lib
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# .dll and .exe
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin) 

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

if (WIN32)
# option for Visual Studio
# -EHsc (enable proper Exxeption Handling) needs to avoid C4530
# -Wall is too noisy so that set -W4. 
# https://docs.microsoft.com/en-us/cpp/build/reference/compiler-option-warning-level?view=vs-2017
# "However, for a new project, it may be best to use /W4 in all compilations;
# this will ensure the fewest possible hard-to-find code defects."
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 /EHsc /MP /bigobj")

add_definitions(-DNOMINMAX)

else()
# g++/clang option for *nix
set(CMAKE_CXX_EXTENSIONS OFF) #Set this to ON if you want to use GNU++
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O2")
endif()

get_directory_property(hasParent PARENT_DIRECTORY)
if(hasParent)
  message(STATUS "Has a parent scope.")
else()
  message(STATUS "Doesn't have a parent scope.")
endif()

function(set_as_cache ARG_NAME ARG_PATH ARG_TEXT)
    set(${ARG_NAME} ${ARG_PATH} CACHE PATH ${ARG_TEXT} FORCE)
endfunction(set_as_cache)

# Use draco eigen for all libs and apps
set_as_cache(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/draco/third_party/eigen" " ")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third_party/draco)

set_as_cache(UGU_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ugu" " ")

# Disable all optional dependencies except OpenMP
set(UGU_USE_GLFW OFF CACHE BOOL "" FORCE)
set(UGU_USE_FREETYPE OFF CACHE BOOL "" FORCE)
set(UGU_BUILD_GUI_APP OFF CACHE BOOL "" FORCE)
set(UGU_USE_POISSON_RECONSTRUCTION OFF CACHE BOOL "" FORCE)
#set(UGU_USE_TINYOBJLOADER OFF CACHE BOOL "" FORCE)
set(UGU_USE_NANORT OFF CACHE BOOL "" FORCE)
set(UGU_USE_TINYCOLORMAP OFF CACHE BOOL "" FORCE)
set(UGU_USE_JSON OFF CACHE BOOL "" FORCE)
set(UGU_USE_NANOFLANN OFF CACHE BOOL "" FORCE)
set(UGU_USE_CXXOPTS OFF CACHE BOOL "" FORCE)
#set(UGU_USE_STB OFF CACHE BOOL "" FORCE)
#set(UGU_USE_LODEPNG OFF CACHE BOOL "" FORCE)

add_subdirectory(${UGU_INSTALL_DIR})

add_executable(compress_3d_test app.cpp draco_encode.h draco_encode.cpp draco_decode.h draco_decode.cpp)
target_include_directories(compress_3d_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/win_build ${CMAKE_CURRENT_SOURCE_DIR}/third_party/draco/src ${Ugu_INCLUDE_DIRS})
target_link_libraries(compress_3d_test PRIVATE draco ${Ugu_LIBS})

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_LIST})