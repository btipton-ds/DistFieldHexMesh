# CMakeList.txt : CMake project for DistFieldHexMesh, include source and define
# project specific logic here.
#

find_package(wxWidgets)
include(${wxWidgets_USE_FILE})

set(OpenGlDir "${PROJECT_SOURCE_DIR}/../utilities/opengl")

if(WIN32)
set(OpenGlPlatform "win")
set(OpenGlPlatformDecl WIN32)
else(WIN32)
set(OpenGlPlatform "linux")
set(OpenGlPlatformDecl )
endif(WIN32)

if(WIN32)
set(triMeshLibDir "../../triMesh/x64/win/${CMAKE_BUILD_TYPE}")
set (CMAKE_SHADER_SOURCE_DIR  "${CMAKE_SOURCE_DIR}/DistFieldHexMesh/shaders")
message(STATUS "CMAKE_SHADER_SOURCE_DIR: ${CMAKE_SHADER_SOURCE_DIR}")
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/win/${CMAKE_BUILD_TYPE}")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/win/${CMAKE_BUILD_TYPE}")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/win/${CMAKE_BUILD_TYPE}")

include_directories(
	"include"
	"../../ThirdParty/eigen/"
	"../../triMesh/stlReader/include/"
	"../../triMesh/triMesh/include/"
	"../../triMesh/tests/include/"
	"../../utilities/threads/include/"
	"${OpenGlDir}/include/"
	"${OpenGlDir}/include/${OpenGlPlatform}/"
	"${wxWidgets_INCLUDE_DIRS}"
)
else()
set(triMeshLibDir "../../triMesh/x64/linux/${CMAKE_BUILD_TYPE}")
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/linux/${CMAKE_BUILD_TYPE}")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/linux/${CMAKE_BUILD_TYPE}")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/linux/${CMAKE_BUILD_TYPE}")
include_directories(
	"include"
	"usr/include/GL/"
	"../../ThirdParty/eigen/"
	"../../triMesh/triMesh/include/"
	"../../triMesh/stlReader/include/"
	"../../triMesh/tests/include/"
	"../../utilities/threads/include/"
	"${OpenGlDir}/include/"
	"${OpenGlDir}/include/${OpenGlPlatform}/"
	"${wxWidgets_INCLUDE_DIRS}"
)
endif()

if(WIN32)
link_directories(
	"C:/Program\ Files/wxWidgets/lib/vc_x64_lib/"
	${triMeshLibDir}
)
else()
link_directories(
	"/usr/local/lib/"
	"/usr/lib/x86_64-linux-gnu/"
	${triMeshLibDir}
)
endif()

add_executable (DistFieldHexMesh         
	${OpenGlPlatformDecl}
	"src/app.cpp"
    "src/appData.cpp"
	
	"src/block.cpp"	
	"src/createBaseMeshDlg.cpp"
    "src/edge.cpp"
	"src/edgeKey.cpp"
	"src/divideHexMeshDlg.cpp"
	"src/drawMesh.cpp"
	"src/drawHexMesh.cpp" 
	"src/drawModelMesh.cpp"
    "src/index3D.cpp"
	"src/fixedPoint.cpp"
	"src/gradingOp.cpp"
	"src/graphicsCanvas.cpp"
	"src/graphicsVBORec.cpp"
	"src/graphicsDebugCanvas.cpp"
	"src/logger.cpp"
	"src/main.cpp" 
	"src/mainFrame.cpp"
	"src/makeBlockDlg.cpp"
	"src/meshData.cpp"
	"src/objectPool.cpp" 
	"src/objectTreeCtrl.cpp"
	"src/polygon.cpp"
	"src/polyHedron.cpp"
    "src/splitter2D.cpp" 
	"src/splitter3D.cpp"
	"src/polymeshTables.cpp"
	"src/selectBlocksDlg.cpp"
	"src/splitLongTrisDlg.cpp"
	"src/splitParams.cpp"
	"src/unalignedBBox.cpp"
	"src/utils.cpp"
	"src/vertexSpatialTree.cpp"
	"src/vertex.cpp"
	"src/volume.cpp"
 	"../../utilities/threads/src/MultiCoreUtil.cpp"
	"../../utilities/threads/src/local_heap.cpp"
	"${OpenGlDir}/src/${OpenGlPlatform}/"
	"${OpenGlDir}/src/${OpenGlPlatform}/OGLExtensions.cpp"
	"${OpenGlDir}/src/OGLShader.cpp"
	"${OpenGlDir}/src/OGLMultiVbo.cpp"
	"${OpenGlDir}/src/OGLMultiVboHandler.cpp"
	"${OpenGlDir}/src/rgbaColor.cpp"
)

add_executable (TestDFM         
	
	"src/block.cpp"	
 	"src/edge.cpp" 
	"src/edgeKey.cpp"
	"src/gradingOp.cpp"
    "src/index3D.cpp"
	"src/logger.cpp"
	"src/objectPool.cpp" 
	"src/polygon.cpp"
	"src/polyHedron.cpp"	
	"src/splitter3D.cpp" 
	"src/splitter2D.cpp"
	"src/polymeshTables.cpp"
	"src/splitParams.cpp"
	"src/unalignedBBox.cpp"
	"src/utils.cpp"
	"src/vertexSpatialTree.cpp"
	"src/vertex.cpp"
	"src/volume.cpp"
 	"src/testBlock.cpp" 
	"src/testMain.cpp"
	"src/testPoolMemory.cpp" 
	"src/testMultiCore.cpp"
	"../../utilities/threads/src/MultiCoreUtil.cpp"
	"../../utilities/threads/src/local_heap.cpp")

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
                "${CMAKE_SHADER_SOURCE_DIR}/phong.vert"
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders/phong.vert"
				
        COMMAND ${CMAKE_COMMAND} -E copy
                "${CMAKE_SHADER_SOURCE_DIR}/phong.frag"
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders/phong.frag"

        COMMAND ${CMAKE_COMMAND} -E copy
                "${CMAKE_SHADER_SOURCE_DIR}/dual_peeling_screenRect.vert"
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders/dual_peeling_screenRect.vert"
				
        COMMAND ${CMAKE_COMMAND} -E copy
                "${CMAKE_SHADER_SOURCE_DIR}/dual_peeling_blend.frag"
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders/dual_peeling_blend.frag"

        COMMAND ${CMAKE_COMMAND} -E copy
                "${CMAKE_SHADER_SOURCE_DIR}/dual_peeling_final.frag"
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders/dual_peeling_final.frag"

        COMMAND ${CMAKE_COMMAND} -E copy
                "${CMAKE_SHADER_SOURCE_DIR}/dual_peeling_init.vert"
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders/dual_peeling_init.vert"
				
        COMMAND ${CMAKE_COMMAND} -E copy
                "${CMAKE_SHADER_SOURCE_DIR}/dual_peeling_init.frag"
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders/dual_peeling_init.frag"

        COMMAND ${CMAKE_COMMAND} -E copy
                "${CMAKE_SHADER_SOURCE_DIR}/dual_peeling_peel.vert"
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders/dual_peeling_peel.vert"
				
        COMMAND ${CMAKE_COMMAND} -E copy
                "${CMAKE_SHADER_SOURCE_DIR}/dual_peeling_peel.frag"
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders/dual_peeling_peel.frag"

        COMMAND ${CMAKE_COMMAND} -E copy
                "${CMAKE_SHADER_SOURCE_DIR}/debug.vert"
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders/debug.vert"
				
        COMMAND ${CMAKE_COMMAND} -E copy
                "${CMAKE_SHADER_SOURCE_DIR}/debug.frag"
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders/debug.frag"
)

if(WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++20")
add_definitions(-D_CRT_SECURE_NO_WARNINGS)
else(WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -Wno-deprecated-enum-enum-conversion -lX11 ")
target_compile_features(DistFieldHexMesh PRIVATE cxx_std_20) 
endif(WIN32)


# TODO: Add tests and install targets if needed.
