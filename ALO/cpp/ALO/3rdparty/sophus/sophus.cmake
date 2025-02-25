include(FetchContent)

set(SOPHUS_USE_BASIC_LOGGING ON CACHE BOOL "Don't use fmt for Sophus libraru")
set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "Don't build Sophus tests")
set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "Don't build Sophus Examples")

# TODO: after https://github.com/strasdat/Sophus/pull/502 gets merged go back to mainstream
FetchContent_Declare(sophus SYSTEM URL https://github.com/nachovizzo/Sophus/archive/refs/tags/1.22.11.tar.gz)
FetchContent_MakeAvailable(sophus)
