# CMakeList.txt : CMake project for DistFieldHexMesh, include source and define
# project specific logic here.
#

include_directories(
	"../../ThirdParty/eigen/"
	"../../triMesh/triMesh/include/"
	"C:/Program\ Files/wxWidgets/include/"
	"C:/Program\ Files/wxWidgets/lib/vc_x64_lib/mswu/"
)

# Add source to this project's executable.

set(triMeshLibDir "D:/Users/BobT/Documents/Projects/Code/triMesh/x64/${CMAKE_BUILD_TYPE}")

link_directories(
	"C:/Program\ Files/wxWidgets/lib/vc_x64_lib/"
	${triMeshLibDir}
)

add_executable (DistFieldHexMesh "main.cpp" "volume.cpp")

if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
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
	)
else()
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
	)
endif()

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET DistFieldHexMesh PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
