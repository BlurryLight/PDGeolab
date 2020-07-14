# Project definition
cmake_minimum_required(VERSION 3.14)
project(PD_GEO_LAB)

include(FetchContent)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON CACHE BOOL "" FORCE)
#set(FETCHCONTENT_QUIET OFF CACHE BOOL "" FORCE)
find_package(glfw3)
find_package(glm)
find_package(assimp)

if(NOT glfw3_FOUND)
    message("GLFW NOT FOUND! DOWNLOAD GLFW FROM GITHUB!")
    include(cmake/glfw.cmake)
endif()

if(NOT glm_FOUND)
    message("GLM NOT FOUND! DOWNLOAD GLM FROM GITHUB!")
    include(cmake/glm.cmake)
endif()

if(NOT assimp_FOUND)
    message("ASSIMP NOT FOUND! DOWNLOAD ASSIMP FROM GITHUB!")
    include(cmake/assimp.cmake)
endif()


# Source files
set(SRC_DIR "src")

set(SOURCES
    ${SRC_DIR}/main.cc
    ${SRC_DIR}/resource_path_searcher.h
    ${SRC_DIR}/resource_path_searcher.cc
    ${SRC_DIR}/glsupport.h
    ${SRC_DIR}/glsupport.cc
    )
file(GLOB IMGUI_SOURCE third_party/imgui/*)
#deps
add_subdirectory(third_party/glad/)

# generate header
set(ROOT_DIR_VAR ${CMAKE_SOURCE_DIR})
if (DEFINED CMAKE_BUILD_TYPE)
    set(BUILD_TYPE_VAR ${CMAKE_BUILD_TYPE})
else()
    # In Linux the default is empty.
    set(BUILD_TYPE_VAR "Debug")
endif()
set(BUILD_SYSTEM_VERSION_VAR ${CMAKE_SYSTEM_VERSION})
set(BUILD_SYSTEM_NAME_VAR ${CMAKE_SYSTEM_NAME})
string(TIMESTAMP BUILD_UTC_TIMESTAMP_VAR UTC)
configure_file(${CMAKE_SOURCE_DIR}/src/cmake_vars.h.in ${CMAKE_SOURCE_DIR}/src/cmake_vars.h)

# Executable definition and properties
add_executable(${PROJECT_NAME} ${SOURCES} ${IMGUI_SOURCE})
target_include_directories(${PROJECT_NAME} PRIVATE third_party/)
target_include_directories(${PROJECT_NAME} PRIVATE ${glm_SOURCE_DIR} ${tinyobjloader_SOURCE_DIR})
message(${glm_SOURCE_DIR})
target_compile_definitions(${PROJECT_NAME} PRIVATE -DIMGUI_IMPL_OPENGL_LOADER_GLAD)
target_link_libraries(${PROJECT_NAME} PRIVATE
  glad
  glfw
  assimp
  ${CMAKE_DL_LIBS}
  )
set_target_properties(${PROJECT_NAME}
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" )
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0 AND NOT APPLE)
        target_compile_definitions(${PROJECT_NAME} PRIVATE -DFILESYSTEM_TS)
        target_link_libraries(${PROJECT_NAME} PRIVATE stdc++fs)
    elseif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0 AND APPLE)
        target_compile_definitions(${PROJECT_NAME} PRIVATE -DFILESYSTEM_TS)
        target_link_libraries(${PROJECT_NAME} PRIVATE c++fs)
    endif()
endif (CMAKE_CXX_COMPILER_ID MATCHES "Clang" )

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.1)
        target_compile_definitions(${PROJECT_NAME} PRIVATE -DFILESYSTEM_TS)
        target_link_libraries(${PROJECT_NAME} PRIVATE stdc++fs)
    endif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.1)
endif()
