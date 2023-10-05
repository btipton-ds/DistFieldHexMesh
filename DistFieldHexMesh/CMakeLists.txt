# CMakeList.txt : CMake project for DistFieldHexMesh, include source and define
# project specific logic here.
#

find_package(wxWidgets)
include(${wxWidgets_USE_FILE})
message("wxWidgets_USE_FILE: ${wxWidgets_USE_FILE}")
message("wxWidgets_INCLUDE_DIRS: ${wxWidgets_INCLUDE_DIRS}")

include_directories(
	"../../ThirdParty/eigen/"
	"../../triMesh/triMesh/include/"
	"../../triMesh/stlReader/include/"
	"../../utilities/threads/include/"
	"../../utilities/opengl/include/"
  "include"
  "${wxWidgets_INCLUDE_DIRS}"
)

# Add source to this project's executable.

set(triMeshLibDir "../../triMesh/x64/${CMAKE_BUILD_TYPE}")

link_directories(
	"C:/Program\ Files/wxWidgets/lib/vc_x64_lib/"
	${triMeshLibDir}
)

add_executable (DistFieldHexMesh         
	WIN32
	"src/main.cpp" 
	"src/volume.cpp"
	"src/mainFrame.cpp"
	"src/app.cpp"
	"src/graphicsCanvas.cpp"
	"../../utilities/opengl/src/OGLExtensions.cpp"
	"../../utilities/opengl/src/OGLMultiVbo.cpp"
	"../../utilities/opengl/src/OGLMultiVbo.cpp"
	"../../utilities/opengl/src/OGLShader.cpp"
 )

message("CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
	target_link_libraries("DistFieldHexMesh" 
	PUBLIC triMesh
	PUBLIC stlReader

	#windows & mfc libraries
	PUBLIC comctl32
	PUBLIC rpcrt4

	#wxWidgests Libraries
	PUBLIC wxbase32ud
	PUBLIC wxmsw32ud_core
	PUBLIC wxpngd
	PUBLIC wxzlibd
	PUBLIC wxmsw32ud_gl
	)
else()
	target_link_libraries("DistFieldHexMesh" 
	PUBLIC triMesh
	PUBLIC stlReader

	#windows & mfc libraries
	PUBLIC comctl32
	PUBLIC rpcrt4

	#wxWidgests Libraries
	PUBLIC wxbase32u
	PUBLIC wxmsw32u_core
	PUBLIC wxpng
	PUBLIC wxzlib
	PUBLIC wxmsw32u_gl
	)
endif()

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET DistFieldHexMesh PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
