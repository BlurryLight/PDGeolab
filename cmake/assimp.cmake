FetchContent_Declare(
        assimp
        GIT_REPOSITORY https://github.com/assimp/assimp.git
        GIT_TAG v5.0.1
)
FetchContent_GetProperties(assimp)
if(NOT assimp_POPULATED)
    FetchContent_Populate(assimp)
    set(ASSIMP_NO_EXPORT ON CACHE BOOL "")
    set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "")
    set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "")
    add_subdirectory(${assimp_SOURCE_DIR} ${assimp_BINARY_DIR})
endif()
