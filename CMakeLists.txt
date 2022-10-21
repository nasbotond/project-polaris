cmake_minimum_required (VERSION 3.10)

# add_definitions ( -std=c++17 )
set ( CMAKE_CXX_STANDARD 17 )

project ( project-polaris )

set ( CMAKE_BUILD_TYPE Debug )
# if ( UNIX )
#     set ( OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../opencv/opencv_build/" )
# endif()
set ( source_dir "${CMAKE_CURRENT_SOURCE_DIR}/src/" )
set ( filters "${CMAKE_CURRENT_SOURCE_DIR}/include/project-polaris/" )

find_package ( OpenCV REQUIRED )
# include_directories ( ${OpenCV_DIR} )
include_directories ( ${OpenCV_INCLUDE_DIRS} )
include_directories ( ${filters} )

set(VTK_DIR "/opt/homebrew/Cellar/vtk/9.2.2/lib/cmake/vtk-9.2/")
find_package(VTK COMPONENTS 
CommonColor
CommonCore
CommonDataModel
CommonTransforms
FiltersSources
InteractionStyle
RenderingAnnotation
RenderingContextOpenGL2
RenderingCore
RenderingFreeType
RenderingGL2PSOpenGL2
RenderingOpenGL2 )

# include(${VTK_USE_FILE})

if (NOT VTK_FOUND)
  message(FATAL_ERROR "Filter: Unable to find the VTK build folder.")
endif()

add_executable ( filter ${source_dir}/main.cpp ${source_dir}/madgwick_filter.cpp ${source_dir}/comp_filter.cpp ${source_dir}/csv_reader.cpp ) # MACOSX_BUNDLE

target_link_libraries ( filter ${OpenCV_LIBS} )
target_link_libraries( filter ${VTK_LIBRARIES} )

# vtk_module_autoinit is needed
vtk_module_autoinit(
  TARGETS filter
  MODULES ${VTK_LIBRARIES}
)