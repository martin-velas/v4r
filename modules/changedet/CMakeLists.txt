set(the_description "Change detection")
v4r_add_module(changedet REQUIRED v4r_common v4r_io v4r_features v4r_segmentation pcl opencv OPTIONAL edt siftgpu)

include_directories(${CMAKE_CURRENT_LIST_DIR}/../../3rdparty/metslib
					${CMAKE_CURRENT_LIST_DIR}/include/
					${CMAKE_CURRENT_LIST_DIR}/../io/include/
					${CMAKE_CURRENT_LIST_DIR}/../core/include/)

SET(SOURCE_CPP
  src/Visualizer3D.cpp
)

SET(SOURCE_HPP
  "${CMAKE_CURRENT_LIST_DIR}/include/v4r/${name}/change_detection.h"
  "${CMAKE_CURRENT_LIST_DIR}/include/v4r/${name}/impl/change_detection.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/include/v4r/${name}/miscellaneous.h"
  "${CMAKE_CURRENT_LIST_DIR}/include/v4r/${name}/impl/miscellaneous.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/include/v4r/${name}/Visualizer3D.h"
  "${CMAKE_CURRENT_LIST_DIR}/include/v4r/${name}/occlusion_checker.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/include/v4r/${name}/viewport_checker.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/include/v4r/${name}/ObjectDetection.h"
  "${CMAKE_CURRENT_LIST_DIR}/include/v4r/${name}/ObjectHistory.h"
  "${CMAKE_CURRENT_LIST_DIR}/include/v4r/${name}/impl/ObjectHistory.hpp"
)

v4r_set_module_sources(HEADERS ${SOURCE_HPP}
                       SOURCES ${SOURCE_CPP})
v4r_module_include_directories()
v4r_create_module()
