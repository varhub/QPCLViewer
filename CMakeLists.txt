cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project(pcl_qtwidget)


find_package (PCL REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

find_package (VTK REQUIRED)
set(VTK_LIBRARIES vtkRendering vtkGraphics vtkHybrid QVTK)

find_package (Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})



set(HEADERS
    pcl_qtwidget.hpp
)

set(SOURCES
    pcl_qtwidget.cpp
)

set(LIBRARIES
    ${QT_LIBRARIES}
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
)

add_executable(pcl_qtwidget_demo demo.cpp ${SOURCES})
target_link_libraries(pcl_qtwidget_demo ${LIBRARIES})

