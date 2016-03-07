cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(superquadric_fitter)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wall")
set(CMAKE_BUILD_TYPE "Debug")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  sensor_msgs
  cv_bridge
  message_filters
  pcl_ros
  tf
  eigen_conversions
  tf_conversions
  message_generation
)

include_directories(${PCL_INCLUDE_DIRS} include/)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
find_package (Ceres REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS
  io
  common
  segmentation
  sample_consensus
  surface
  visualization
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()



## Generate services in the 'srv' folder
 add_service_files(
   FILES
   estimateCapsicumPose.srv
 )

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
  superquadric_fitter
)


catkin_package(
   INCLUDE_DIRS include
   #fit_superquadric_to_samples
   #split_and_merge
   #fit_superquadric_ceres
   #tabletop_segmentation
   #segmentation_n_fitting_pipeline
   #segmentation_n_fitting_pipeline_test
   #segmentation_n_detection
   #segmentation_n_detection_test
)

include_directories (include
  ${CERES_INCLUDES}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

set (framework_incs include/fit_superquadric_lm.h 
					include/fit_superquadric_ceres.h
                    include/superquadric_formulas.h
                    include/sample_superquadric.h
                    include/sample_superquadric_uniform.h
                    include/segmentation_utils.h
                    include/tabletop_segmentation.h
                    include/superquadric_detection.h
                    include/superquadric_rigid_registration.h)
set (framework_impls include/impl/fit_superquadric_lm.hpp
					 include/impl/fit_superquadric_ceres.hpp
					 include/impl/superquadric_formulas.hpp
                     include/impl/sample_superquadric.hpp
                     include/impl/sample_superquadric_uniform.hpp
                     include/impl/segmentation_utils.hpp
                     include/impl/tabletop_segmentation.hpp
                     include/impl/superquadric_detection.hpp
                     include/impl/superquadric_rigid_registration.hpp)
set (framework_srcs src/fit_superquadric_lm.cpp
                    src/fit_superquadric_ceres.cpp
                    src/sample_superquadric.cpp
                    src/sample_superquadric_uniform.cpp
                    src/segmentation_utils.cpp
                    src/tabletop_segmentation.cpp
                    src/superquadric_detection.cpp
                    src/superquadric_rigid_registration.cpp)

ADD_LIBRARY (superquadrics_framework ${framework_incs} ${framework_impls} ${framework_srcs})
TARGET_LINK_LIBRARIES (superquadrics_framework)

set (libs_to_link ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} superquadrics_framework)

#add_executable (sample_superquadric_vtk MACOSX_BUNDLE src/main_sample_superquadric_vtk.cpp)
#target_link_libraries (sample_superquadric_vtk ${PCL_LIBRARIES})

#add_executable (sample_superquadric src/main_sample_superquadric.cpp)
#target_link_libraries (sample_superquadric ${libs_to_link})

#add_executable (fit_superquadric_to_samples MACOSX_BUNDLE src/main_fit_superquadric_to_samples.cpp)
#target_link_libraries (fit_superquadric_to_samples ${PCL_LIBRARIES} superquadrics_framework)

#add_executable (split_and_merge MACOSX_BUNDLE src/main_split_and_merge.cpp)
#target_link_libraries (split_and_merge ${libs_to_link})

#add_executable (fit_superquadric_ceres src/main_fit_superquadric_ceres.cpp)
#target_link_libraries (fit_superquadric_ceres ${libs_to_link})

#add_executable (tabletop_segmentation MACOSX_BUNDLE src/main_tabletop_segmentation.cpp)
#target_link_libraries (tabletop_segmentation ${libs_to_link})

#add_executable (segmentation_n_fitting_pipeline MACOSX_BUNDLE src/main_segmentation_n_fitting_pipeline.cpp)
#target_link_libraries (segmentation_n_fitting_pipeline ${libs_to_link})

add_executable (superquadric_fitter src/superquadric_fitter_subsriber.cpp)
target_link_libraries (superquadric_fitter ${libs_to_link})

add_dependencies( superquadric_fitter ${PROJECT_NAME}_generate_messages_cpp )

#add_executable (segmentation_n_detection MACOSX_BUNDLE src/main_segmentation_n_detection.cpp)
#target_link_libraries (segmentation_n_detection ${libs_to_link})

#add_executable (segmentation_n_detection_test MACOSX_BUNDLE src/main_segmentation_n_detection_test.cpp)
#target_link_libraries (segmentation_n_detection_test ${libs_to_link})

##add_executable (tests src/main_tests.cpp)
##target_link_libraries (tests ${libs_to_link})
