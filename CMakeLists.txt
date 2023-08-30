cmake_minimum_required(VERSION 3.8)
project(vive_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT PLATFORM)
  if(CMAKE_SIZEOF_VOID_P MATCHES 8)
    set(PLATFORM 64)
  else()
    set(PLATFORM 32)
  endif()
endif()
message(STATUS "Compilation set for ${PLATFORM}bits architectures.")

set(SDL_REQUIRED_LIBRARIES SDL2)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  add_definitions(-DLINUX -DPOSIX)
  set(ARCH_TARGET linux64)

  if(${PLATFORM} MATCHES 32)
    message(WARNING "OpenVR x86 binaries not provided on GNU/Linux.")
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CMAKE_MACOSX_RPATH 0)
  add_definitions(-DOSX -DPOSIX)
  set(ARCH_TARGET osx32)

elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(SDL_REQUIRED_LIBRARIES ${SDL_REQUIRED_LIBRARIES} SDL2main)
  add_definitions(-D_WIN32)
  set(ARCH_TARGET win${PLATFORM})

  # Binaries path for thirdparties are not generics so we try to guess their suffixes.
  set(WINDOWS_PATH_SUFFIXES win${PLATFORM} Win${PLATFORM} x${PLATFORM})

  if(${PLATFORM} MATCHES 64)
    message(WARNING "SDL x64 runtime binaries not provided on Windows.")
  endif()
endif()

# -----------------------------------------------------------------------------
## COMPILER DETECTION ##
if(   (${CMAKE_CXX_COMPILER_ID} MATCHES "GNU")
   OR (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang"))
  # Better to use the prebuilt GNU preprocessor define __GNUC__,
  # kept for legacy reason with the sample code.
  add_definitions(-DGNUC)

  set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++11 -include ${SHARED_SRC_DIR}/compat.h")
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -pedantic -g")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

  # Handles x86 compilation support on x64 arch.
  if(${PLATFORM} MATCHES 32)
    set(CMAKE_CXX_FLAGS        "${CMAKE_CXX_FLAGS} -m32")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -m32")
  endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} /W2 /DEBUG")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MP /INCREMENTAL:NO")
else()
  message(FATAL_ERROR "Unsupported compiler '${CMAKE_CXX_COMPILER_ID}'")
endif()

###
set(OPENVR_SRC "c:/Users/jskuser/source/openvr-1.26.7") ## option OPENVR_SRC, OPENVR_DIR
set(OPENVR_DIR "c:/local/openvr")
set(THIRDPARTY_DIR ${OPENVR_SRC}/samples/thirdparty)
set(SHARED_SRC_DIR ${OPENVR_SRC}/samples/shared)
#set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++11 -include ${SHARED_SRC_DIR}/compat.h")
#set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -pedantic -g")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

## find openvr(windows)
find_library(OPENVR_LIBRARIES
  NAMES
    openvr_api64
  PATHS
    ${OPENVR_DIR}/bin
    ${OPENVR_DIR}/lib
  PATH_SUFFIXES
    win${PLATFORM}
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH
)
set(OPENVR_INCLUDE_DIR ${OPENVR_DIR}/include/openvr ${OPENVR_SRC}/samples)
set(OPENVR_FOUND TRUE)
##add_definitions(-DOPENVR_BUILD_STATIC)##

## OpenGL / GLU
find_package(OpenGL REQUIRED)

## GLEW 1.11
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  # On GNU/Linux, glew can be found on the package manager.
  find_package(GLEW 1.11 REQUIRED)
else()
  # Otherwise, use the shipped binaries.
  find_library(GLEW_LIBRARIES
    NAMES
      GLEW
      glew32
    PATHS
      ${THIRDPARTY_DIR}/glew/glew-1.11.0/lib/Release
    PATH_SUFFIXES
      osx32
      ${WINDOWS_PATH_SUFFIXES}
  )
  set(GLEW_INCLUDE_DIR ${THIRDPARTY_DIR}/glew/glew-1.11.0/include)
  set(GLEW_FOUND TRUE)
endif()

## find SDL
set(SDL_REQUIRED_LIBRARIES SDL2)
foreach(lib ${SDL_REQUIRED_LIBRARIES})
  find_library(${lib}_LIBRARY
    NAMES
      ${lib}
    PATHS
      ${THIRDPARTY_DIR}/sdl2-2.0.3/bin
    PATH_SUFFIXES
      linux${PLATFORM}
      ${WINDOWS_PATH_SUFFIXES}
    )
  list(APPEND SDL2_LIBRARIES ${${lib}_LIBRARY})
endforeach()
set(SDL2_INCLUDE_DIR ${THIRDPARTY_DIR}/sdl2-2.0.3/include)
set(SDL2_FOUND TRUE)

## ## vulkan
## find_library(VULKAN_LIBRARY
##     NAMES
##       vulkan-1
##       vulkan
##     PATHS
##       ${THIRDPARTY_DIR}/vulkan-1.0.49.0/lib
##     PATH_SUFFIXES
##       linux${PLATFORM}
##       ${WINDOWS_PATH_SUFFIXES}
##     NO_DEFAULT_PATH
## )
## set(VULKAN_INCLUDE_DIR ${THIRDPARTY_DIR}/vulkan-1.0.49.0/include)
## set(VULKAN_FOUND TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_ADDITIONAL_VERSIONS "1.62" "1.63" "1.64" "1.65" "1.66" "1.67" "1.68" "1.69" "1.70" "1.71" "1.72" "1.73" "1.74" "1.75" "1.81")
find_package(Boost 1.58.0 REQUIRED)

## if(BUILD_TESTING)
##   find_package(ament_lint_auto REQUIRED)
##   # the following line skips the linter which checks for copyrights
##   # comment the line when a copyright and license is added to all source files
##   set(ament_cmake_copyright_FOUND TRUE)
##   # the following line skips cpplint (only works in a git repo)
##   # comment the line when this package is in a git repo and when
##   # a copyright and license is added to all source files
##   set(ament_cmake_cpplint_FOUND TRUE)
##   ament_lint_auto_find_test_dependencies()
## endif()
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

file(GLOB SHARED_SRC_FILES
  ${SHARED_SRC_DIR}/*.cpp
  ${SHARED_SRC_DIR}/*.h
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIR}
  ${SDL2_INCLUDE_DIR}
  ${OPENVR_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR} ## (OpenGL specific)
  ${GLEW_INCLUDE_DIR} ## (OpenGL specific)
)

add_executable(test_openvr
  ${SHARED_SRC_FILES}
  src/vive_node_w_image.cpp
  src/vr_interface.cpp
  )
#add_executable(test_openvr
#  src/no_ros.cpp
#  )

message("OPENGL: ${OPENGL_LIBRARIES}")
message("GLEW: ${GLEW_LIBRARIES}")
message("SDL2: ${SDL2_LIBRARIES}")
message("OPENVR: ${OPENVR_LIBRARIES}")
target_link_libraries(test_openvr
  ${Boost_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${SDL2_LIBRARIES}
  ${OPENVR_LIBRARIES}
  )

ament_target_dependencies(test_openvr
rclcpp
std_msgs
std_srvs
image_transport
cv_bridge
geometry_msgs
tf2_ros
tf2_eigen
)

install(TARGETS
  test_openvr
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
