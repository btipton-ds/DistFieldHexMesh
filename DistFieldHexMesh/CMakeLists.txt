﻿# CMakeList.txt : CMake project for DistFieldHexMesh, include source and define
# project specific logic here.
#

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/${CMAKE_BUILD_TYPE}")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/${CMAKE_BUILD_TYPE}")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/${CMAKE_BUILD_TYPE}")

find_package(wxWidgets)
include(${wxWidgets_USE_FILE})

set(OpenGlDir "${PROJECT_SOURCE_DIR}/../utilities/opengl")
set(triMeshLibDir "../../triMesh/x64/${CMAKE_BUILD_TYPE}")

if(WIN32)
set(OpenGlPlatform "win")
set(OpenGlPlatformDecl WIN32)
else(WIN32)
set(OpenGlPlatform "linux")
set(OpenGlPlatformDecl )
endif(WIN32)

include_directories(
	"include"
	"../../ThirdParty/eigen/"
	"../../triMesh/triMesh/include/"
	"../../triMesh/stlReader/include/"
	"../../utilities/threads/include/"
	"${OpenGlDir}/include/"
	"${OpenGlDir}/include/${OpenGlPlatform}/"
	"${wxWidgets_INCLUDE_DIRS}"
)

if(WIN32)
link_directories(
	"C:/Program\ Files/wxWidgets/lib/vc_x64_lib/"
	${triMeshLibDir}
)
else()
link_directories(
	"/usr/local/lib/libwx_gtk3u-3.2"
	"/usr/local/lib/x86_64-linux-gnu"
	${triMeshLibDir}
)
endif()

add_executable (DistFieldHexMesh         
	${OpenGlPlatformDecl}
	"src/app.cpp"
    "src/appData.cpp"
	
	"src/block.cpp"	
    "src/edge.cpp"
    "src/index3D.cpp"
	"src/graphicsCanvas.cpp"
	"src/main.cpp" 
	"src/mainFrame.cpp"
	"src/makeBlockDlg.cpp"
	"src/objectPool.cpp" 
	"src/polygon.cpp"
	"src/polyHedron.cpp"
	
	"src/vertex.cpp"
	"src/volume.cpp"
	"${OpenGlDir}/src/${OpenGlPlatform}/"
	"${OpenGlDir}/src/${OpenGlPlatform}/OGLExtensions.cpp"
	"${OpenGlDir}/src/OGLShader.cpp"
	"${OpenGlDir}/src/OGLMultiVbo.cpp"
	"${OpenGlDir}/src/OGLMultiVboHandler.cpp"
 	  )

add_executable (TestDFM         
	
	"src/block.cpp"	
 	"src/blockTest.cpp" 
 	"src/edge.cpp" 
    "src/index3D.cpp"
	"src/mainTest.cpp"
	"src/objectPool.cpp" 
	"src/polygon.cpp"
	"src/polyHedron.cpp"
	
	"src/vertex.cpp"
	"src/volume.cpp" 	 )

if (WIN32)
if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
	target_link_libraries("DistFieldHexMesh" 
		PUBLIC triMesh
		PUBLIC stlReader

		#windows & mfc libraries
		PUBLIC comctl32
		PUBLIC rpcrt4

		#wxWidgets Libraries
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

	#wxWidgets Libraries
	PUBLIC wxbase32u
	PUBLIC wxmsw32u_core
	PUBLIC wxpng
	PUBLIC wxzlib
	PUBLIC wxmsw32u_gl
	)
endif()
else(WIN32)
target_link_libraries("DistFieldHexMesh" 
	PUBLIC triMesh
	PUBLIC stlReader

	PUBLIC GL
	PUBLIC GLU
	PUBLIC GLX

	#wxWidgets Libraries
	PUBLIC wx_gtk3u-3.2
	PUBLIC wx_gtk3u_gl-3.2
)
endif(WIN32)

target_link_libraries("TestDFM" 
	PUBLIC triMesh
	PUBLIC stlReader
)

add_custom_command(
        TARGET DistFieldHexMesh POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                "${OpenGlDir}/src/phong.vert"
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders/phong.vert"

        COMMAND ${CMAKE_COMMAND} -E copy
                "${OpenGlDir}/src/phong.frag"
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders/phong.frag"
)

if(WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++20")
add_definitions(-D_CRT_SECURE_NO_WARNINGS)
else(WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-enum-enum-conversion")
target_compile_features(DistFieldHexMesh PRIVATE cxx_std_20)
endif(WIN32)


# TODO: Add tests and install targets if needed.
