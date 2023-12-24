# CMakeList.txt : CMake project for DistFieldHexMesh, include source and define
# project specific logic here.
#

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/${CMAKE_BUILD_TYPE}")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/${CMAKE_BUILD_TYPE}")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/${CMAKE_BUILD_TYPE}")

find_package(wxWidgets)
include(${wxWidgets_USE_FILE})

set(OpenGlDir "${PROJECT_SOURCE_DIR}/../utilities/opengl")
set(triMeshLibDir "../../triMesh/x64/${CMAKE_BUILD_TYPE}")

include_directories(
	"include"
	"../../ThirdParty/eigen/"
	"../../triMesh/triMesh/include/"
	"../../triMesh/stlReader/include/"
	"../../utilities/threads/include/"
	"${OpenGlDir}/include/"
	"${wxWidgets_INCLUDE_DIRS}"
)


link_directories(
	"C:/Program\ Files/wxWidgets/lib/vc_x64_lib/"
	${triMeshLibDir}
)

add_executable (DistFieldHexMesh         
	WIN32
	"src/main.cpp" 
	"src/volume.cpp"
	"src/mainFrame.cpp"
	"src/makeBlockDlg.cpp"
	"src/app.cpp"
    "src/appData.cpp"
	"src/graphicsCanvas.cpp"
	"src/objectPool.cpp" 
	"src/block.cpp"	
	"src/cell.cpp"
	"src/polygon.cpp"
	"src/polyHedron.cpp"
	"src/vertex.cpp"
	"${OpenGlDir}/src/OGLExtensions.cpp"
	"${OpenGlDir}/src/OGLShader.cpp"
	"${OpenGlDir}/src/OGLMultiVbo.cpp"
	"${OpenGlDir}/src/OGLMultiVboHandler.cpp"
 "src/dataPool.cpp" "src/vertexManager.cpp")

add_executable (TestDFM         
	"src/mainTest.cpp"
	"src/volume.cpp"
	"src/objectPool.cpp" 
	"src/block.cpp"	
	"src/cell.cpp"
	"src/polygon.cpp"
	"src/polyHedron.cpp"
	"src/vertex.cpp"
 "src/blockTest.cpp" "src/dataPool.cpp" "src/vertexManager.cpp")

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

if(MSVC)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++20")
else(MSVC)
target_compile_features(DistFieldHexMesh PRIVATE cxx_std_17)
endif(MSVC)


# TODO: Add tests and install targets if needed.
