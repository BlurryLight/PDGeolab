FetchContent_Declare(
        openmesh
        GIT_REPOSITORY https://www.graphics.rwth-aachen.de:9000/OpenMesh/OpenMesh.git
        GIT_TAG OpenMesh-8.1
)
if(NOT openmesh_POPULATED)
    FetchContent_Populate(openmesh)
    set(CMAKE_BUILD_TYPE Release)
    add_subdirectory(${openmesh_SOURCE_DIR} ${openmesh_BINARY_DIR})
    set(CMAKE_BUILD_TYPE)
endif()
