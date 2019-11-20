cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O1")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -w -Ofast")

project(Mirror-Skull)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(HEADER_DIR ${PROJECT_SOURCE_DIR}/include)
set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)
set(SOURCE_FILES ${SOURCE_DIR}/main.cpp  ${SOURCE_DIR}/mirror.cpp ${SOURCE_DIR}/areapick.cpp )
set(HEADER_FILES ${HEADER_DIR}/mirror.hpp  ${HEADER_DIR}/areapick.h )
option(WITH_OPENMP "WITH_OPENMP" ON)


find_package(PCL 1.7 REQUIRED COMPONENTS common io kdtree features visualization recognition)

if(WITH_OPENMP)
find_package(OpenMP)
find_package(OpenCV REQUIRED)

if(OPENMP_FOUND)
	message("OpenMP found")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

endif()




if(PCL_FOUND)
	message("PCL found")

	include_directories(${PCL_INCLUDE_DIRS})
	include_directories(${HEADER_DIR})
	include_directories(${OpenCV_INCLUDE_DIRS})
	
	add_definitions(${PCL_DEFINITIONS})

	add_executable(mirror_pointcloud_tool ${SOURCE_FILES} ${HEADER_FILES})

	
	link_directories(${OpenCV_LIBRARY_DIRS})
	target_link_libraries(mirror_pointcloud_tool ${OpenCV_LIBS})
    target_link_libraries(mirror_pointcloud_tool gflags)
    target_link_libraries(mirror_pointcloud_tool ${PCL_COMMON_LIBRARIES} 
	${PCL_IO_LIBRARIES} ${PCL_KDTREE_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_RECOGNITION_LIBRARIES})
endif()
