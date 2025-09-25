# CMakeList.txt : CMake project for DistFieldHexMesh, include source and define
# project specific logic here.
#

find_package(wxWidgets COMPONENTS core base gl)
include(${wxWidgets_USE_FILE})
message(WARNING "wxWidgets_INCLUDE_DIRS: ${wxWidgets_INCLUDE_DIRS}")

set(OpenGlDir "${PROJECT_SOURCE_DIR}/../utilities/opengl")
if(WIN32)
	set(OpenGlPlatform "win")
	set(OpenGlPlatformDecl WIN32)
else(WIN32)
	set(OpenGL_GL_PREFERENCE LEGACY)
	find_package(OpenGL)
	set(OpenGlPlatform "linux")
	set(OpenGlPlatformDecl )
endif(WIN32)

set (CMAKE_SHADER_SOURCE_DIR  "${CMAKE_SOURCE_DIR}/DistFieldHexMesh/shaders")
set (CMAKE_ASSETS_SOURCE_DIR  "${CMAKE_SOURCE_DIR}/DistFieldHexMesh/assets")
if(WIN32)
	set(triMeshLibDir "../../triMesh/x64/win/${CMAKE_BUILD_TYPE}")
	set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/win/${CMAKE_BUILD_TYPE}")
	set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/win/${CMAKE_BUILD_TYPE}")
	set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/win/${CMAKE_BUILD_TYPE}")

	include_directories(
		"include"
		"geometry/include/"
		"graphics/include/"
		"test/include/"
		"UI/include/"
		"../../ThirdParty/eigen/"
		"../../triMesh/stlReader/include/"
		"../../triMesh/triMesh/include/"
		"../../triMesh/tests/include/"
		"../../utilities/threads/include/"
		"${OpenGlDir}/include/"
		"${OpenGlDir}/include/${OpenGlPlatform}/"
		"C:/Program Files/wxWidgets/lib/vc_x64_lib/mswu"
		"C:/Program Files/wxWidgets/include"
	)

	link_directories(
			"C:/Program\ Files/wxWidgets/lib/vc_x64_lib/"
			${triMeshLibDir}
	)
else(WIN32)
	set(triMeshLibDir "../../triMesh/x64/linux/${CMAKE_BUILD_TYPE}")
	set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/linux/${CMAKE_BUILD_TYPE}")
	set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/linux/${CMAKE_BUILD_TYPE}")
	set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/linux/${CMAKE_BUILD_TYPE}")
	include_directories(
		"include"
		"/usr/include/GL/"
		"/usr/include/eigen3/"
		"geometry/include/"
		"graphics/include/"
		"test/include/"
		"UI/include/"
		"../../triMesh/triMesh/include/"
		"../../triMesh/stlReader/include/"
		"../../triMesh/tests/include/"
		"../../utilities/threads/include/"
		"${OpenGlDir}/include/"
		"${OPENGL_INCLUDE_DIRS}"
		"${wxWidgets_INCLUDE_DIRS}"
	)
	link_directories(
			"/usr/local/lib/"
			"/usr/lib/x86_64-linux-gnu/"
			usr/lib/X11
			${wxWidgets_LIBRARIES}
			${triMeshLibDir}
	)
endif(WIN32)

add_executable (DistFieldHexMesh         
	${OpenGlPlatformDecl}
	"src/app.cpp"
    "src/appData.cpp"
    "src/index3D.cpp"
	"src/logger.cpp"
	"src/main.cpp" 
	"src/polymeshTables.cpp"
	"src/splitParams.cpp"
	"src/utils.cpp"

	"geometry/src/block.cpp"
	"geometry/src/debugMeshData.cpp"
    "geometry/src/edge.cpp"
	"geometry/src/edgeKey.cpp"
	"geometry/src/fixedPoint.cpp"
	"geometry/src/gradingOp.cpp"
	"geometry/src/lineSegment2D.cpp"
	"geometry/src/meshData.cpp"
	"geometry/src/model.cpp"
	"geometry/src/objectPool.cpp" 
	"geometry/src/polygon.cpp"
	"geometry/src/polygonSearchKey.cpp"
	"geometry/src/polyHedron.cpp"
	"geometry/src/polyMesh.cpp"
    "geometry/src/splitter2D.cpp" 
	"geometry/src/splitter3D.cpp"
	"geometry/src/unalignedBBox.cpp"
	"geometry/src/vertexSpatialTree.cpp"
	"geometry/src/vertex.cpp"
	"geometry/src/vector2d.cpp"
	"geometry/src/volume.cpp"

	"graphics/src/drawCrossSectionEdges.cpp"
	"graphics/src/drawDebugMesh.cpp"
	"graphics/src/drawMesh.cpp"
	"graphics/src/drawHexMesh.cpp" 
	"graphics/src/drawModelMesh.cpp"
	"graphics/src/graphicsVBORec.cpp"

	"UI/src/createBaseMeshDlg.cpp"
	"UI/src/editPrefsDlg.cpp"
	"UI/src/divideHexMeshDlg.cpp"
	"UI/src/graphicsCanvas.cpp"
	"UI/src/mainFrame.cpp"
	"UI/src/makeBlockDlg.cpp"
	"UI/src/objectTreeCtrl.cpp"
	"UI/src/selectBlocksDlg.cpp"
	"UI/src/splitLongTrisDlg.cpp"

 	"../../utilities/threads/src/MultiCoreUtil.cpp"
	"../../utilities/threads/src/local_heap.cpp"

	"${OpenGlDir}/src/OGLExtensions.cpp"
	"${OpenGlDir}/src/OGLShader.cpp"
	"${OpenGlDir}/src/OGLMultiVbo.cpp"
	"${OpenGlDir}/src/OGLMultiVboHandler.cpp"
	"${OpenGlDir}/src/rgbaColor.cpp"
   )

if ((${CMAKE_BUILD_TYPE} STREQUAL "Release"))
	add_definitions("-DDEBUGGING_MUTEXES_AVAIL=0")
else()
	add_definitions("-DDEBUGGING_MUTEXES_AVAIL=1")
endif()

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
	else(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
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
	endif(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
else(WIN32)
	target_link_libraries("DistFieldHexMesh"
		PUBLIC triMesh
		PUBLIC stlReader

		${OPENGL_LIBRARIES} GLEW

		#wxWidgets Libraries
		${wxWidgets_LIBRARIES} X11
	)
endif(WIN32)

add_executable (TestDFM         
    "src/index3D.cpp"
	"src/logger.cpp"
	"src/polymeshTables.cpp"
	"src/splitParams.cpp"
	"src/utils.cpp"

	"geometry/src/block.cpp"
	"geometry/src/debugMeshData.cpp"
    "geometry/src/edge.cpp"
	"geometry/src/edgeKey.cpp"
	"geometry/src/fixedPoint.cpp"
	"geometry/src/gradingOp.cpp"
	"geometry/src/lineSegment2D.cpp"
	"geometry/src/meshData.cpp"
	"geometry/src/model.cpp"
	"geometry/src/objectPool.cpp" 
	"geometry/src/polygon.cpp"
	"geometry/src/polygonSearchKey.cpp"
	"geometry/src/polyHedron.cpp"
	"geometry/src/polyMesh.cpp"
    "geometry/src/splitter2D.cpp" 
	"geometry/src/splitter3D.cpp"
	"geometry/src/unalignedBBox.cpp"
	"geometry/src/vertexSpatialTree.cpp"
	"geometry/src/vector2d.cpp"
	"geometry/src/vertex.cpp"
	"geometry/src/volume.cpp"

 	"../../utilities/threads/src/MultiCoreUtil.cpp"
	"../../utilities/threads/src/local_heap.cpp"

 	"test/src/testBlock.cpp" 
	"test/src/testMain.cpp"
	"test/src/testPoolMemory.cpp" 
	"test/src/testMultiCore.cpp"
)

target_link_libraries("TestDFM" 
	PUBLIC triMesh
	PUBLIC stlReader
)

add_custom_command(
        TARGET DistFieldHexMesh POST_BUILD

        COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${CMAKE_SHADER_SOURCE_DIR}/"
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders"

# Only needed if the assets folder has contents, which it currently does not
#        COMMAND ${CMAKE_COMMAND} -E copy_directory
#                "${CMAKE_ASSETS_SOURCE_DIR}/"
#                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets"
)

if(WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
add_definitions(-D_CRT_SECURE_NO_WARNINGS)
else(WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -Wno-deprecated-enum-enum-conversion -lX11")
target_compile_features(DistFieldHexMesh PRIVATE cxx_std_20)
endif(WIN32)


# TODO: Add tests and install targets if needed.
