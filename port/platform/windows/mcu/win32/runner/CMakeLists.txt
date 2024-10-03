cmake_minimum_required(VERSION 3.5)

project(runner_win32)

# Get the generic cmake definitions for ubxlib
include(../../../windows.cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /WX-")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /WX- /W3")
include_directories("C:/Users/JordanTrajkov/Desktop/ubxlibSARA/ubxlib/port/api")
include_directories("C:/Users/JordanTrajkov/Desktop/ubxlibSARA/ubxlib/cell/api")
include_directories("C:/Users/JordanTrajkov/Desktop/ubxlibSARA/ubxlib/port/src")
include_directories("C:/Users/JordanTrajkov/Desktop/ubxlibSARA/ubxlib/cell/src")
include_directories("C:/Users/JordanTrajkov/Desktop/ubxlibSARA/ubxlib/gnss/api")
include_directories("C:/Users/JordanTrajkov/Desktop/ubxlibSARA/ubxlib/gnss/src")
include_directories("C:/Users/JordanTrajkov/Desktop/ubxlibSARA/ubxlib/common/device/api")
include_directories("C:/Users/JordanTrajkov/Desktop/ubxlibSARA/ubxlib/common/location/api")
include_directories("C:/Users/JordanTrajkov/Desktop/ubxlibSARA/ubxlib/cell/api")
include_directories("C:/Users/JordanTrajkov/Desktop/ubxlibSARA/ubxlib/common/at_client/api")
include_directories("C:/Users/JordanTrajkov/Desktop/ubxlibSARA/ubxlib/cell/api")
include_directories("C:/Users/JordanTrajkov/Desktop/ubxlibSARA/ubxlib/common/device/api")
include_directories("C:/Users/JordanTrajkov/Desktop/ubxlibSARA/ubxlib/common/network/api")
include_directories("C:/Users/JordanTrajkov/Desktop/ubxlibSARA/ubxlib/common/sock/api")

if (DEFINED ENV{UNITY_PATH})
    set(UNITY_PATH $ENV{UNITY_PATH} CACHE PATH "the path to the Unity directory")
else()
    # Adjusted the path since Unity is in the same directory as ubxlib
    set(UNITY_PATH "${UBXLIB_BASE}/Unity" CACHE PATH "the path to the Unity directory")
endif()
message("UNITY_PATH will be \"${UNITY_PATH}\"")

# Conditionally apply -Wno-error only for GCC/Clang compilers
if (CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-error")
endif()

set(UBXLIB_TEST_SRC_PORT
  ${UBXLIB_BASE}/port/platform/common/runner/u_runner.c)
set(UBXLIB_PRIVATE_TEST_INC_PORT
  ${UBXLIB_BASE}/port/platform/common/runner)

# Add Unity and its headers
# Unity requires a definition of "noreturn" and this may be
# missing from MSVC, depending on which Windows SDK CMake
# decides to employ.  To work around this, we provide a
# definition here which lets it evaluate to nothing.
add_compile_options("-DUNITY_NORETURN=")
add_subdirectory(${UNITY_PATH} unity)

add_compile_options("-DU_AT_CLIENT_PRINT_WITH_TIMESTAMP")


# Create a library containing the ubxlib tests
# These files must be compiled as C++ so that the "runner" macro
# which creates the actual test functions works
# This is created as an OBJECT library so that the linker doesn't
# throw away the constructors we need
set_source_files_properties(${UBXLIB_TEST_SRC} PROPERTIES LANGUAGE CXX )
set_source_files_properties(${UBXLIB_TEST_SRC_PORT} PROPERTIES LANGUAGE CXX )
add_library(ubxlib_test OBJECT ${UBXLIB_TEST_SRC} ${UBXLIB_TEST_SRC_PORT})
target_compile_options(ubxlib_test PRIVATE ${UBXLIB_COMPILE_OPTIONS})
target_include_directories(ubxlib_test PRIVATE
                           ${UBXLIB_TEST_INC}
                           ${UBXLIB_PRIVATE_TEST_INC_PORT}
                           ${UBXLIB_INC}
                           ${UBXLIB_PRIVATE_INC}
                           ${UBXLIB_PUBLIC_INC_PORT}
                           ${UBXLIB_PRIVATE_INC_PORT}
                           ${UNITY_PATH}/src)

# Create the test target for ubxlib, including in it u_main.c
add_executable(ubxlib_test_main ${UBXLIB_BASE}/port/platform/${UBXLIB_PLATFORM}/app/u_main.c)
target_include_directories(ubxlib_test_main PUBLIC ${UBXLIB_INC} ${UBXLIB_PUBLIC_INC_PORT})
target_include_directories(ubxlib_test_main PRIVATE ${UBXLIB_PRIVATE_TEST_INC_PORT} ${UBXLIB_PRIVATE_INC})
# Link the ubxlib test target with the ubxlib tests library and Unity
# plus any additional libraries that have been brought in
target_link_libraries (ubxlib_test_main PRIVATE ubxlib unity ubxlib_test ${UBXLIB_EXTRA_LIBS})
