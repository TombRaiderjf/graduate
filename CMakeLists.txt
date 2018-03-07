#1.cmake verson，指定cmake版本 
cmake_minimum_required(VERSION 3.5.1)

###following three rows convert to debug mode
#SET(CMAKE_BUILD_TYPE "Debug") 
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

#2.project name，指定项目的名称，一般和项目的文件夹名称对应
PROJECT(myprogram)

set(CMAKE_CXX_FLAGS "-std=c++11")
if(USE_32BITS)
	message(STATUS "using 32 bits")
else(USE_32BITS)
	message(STATUS "using 64 bits")
endif(USE_32BITS)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)
find_package(freenect2 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
FIND_PACKAGE( OpenMP)
find_package(PCL REQUIRED)
  if(OPENMP_FOUND)
     message("-- OPENMP FOUND")
     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
     set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  endif()
include_directories(SYSTEM ${OpenMP_INCLUDE_PATH})

#3.head file path，头文件目录
INCLUDE_DIRECTORIES(
include
/home/jf/freenect2/include
/home/jf/libfreenect2/src/Openni2
/usr/local/include/openMVG_dependencies/cereal/include
/usr/local/include/openMVG_dependencies
/usr/local/include/eigen3
/usr/local/include/pcl-1.8
${OpenCV_INCLUDE_DIRS}
)

include(${CGAL_USE_FILE})

#4.source directory，源文件目录
AUX_SOURCE_DIRECTORY(src DIR_SRCS)

#5.set environment variable，设置环境变量，编译用到的源文件全部都要放到这里，否则编译能够通过，但是执行的时候会出现各种问题，比如"symbol lookup error xxxxx , undefined symbol"
#SET(TEST_MATH
#${DIR_SRCS}
#)

#link_directories(
#/home/jf/freenect2/lib/OpenNI2/Drivers
#/home/jf/OpenNI-Linux-x64-2.2/Redist/OpenNI2/Drivers
#)

ADD_EXECUTABLE(myexe ${DIR_SRCS})

#6.add link library，添加可执行文件所需要的库，比如我们用到了libm.so（命名规则：lib+name+.so），就添加该库的名称
TARGET_LINK_LIBRARIES(myexe 
  ${OPENMVG_LIBRARIES}
  ${freenect2_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${CGAL_LIBRARIES} 
  ${CGAL_3RD_PARTY_LIBRARIES} 
  ${OpenMP_LIBRARIES}
  ${PCL_LIBRARIES}
  cxsparse
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_matching
  openMVG_geometry
  openMVG_numeric
  openMVG_multiview
  openMVG_sfm
  openMVG_kvld
  easyexif
  fast
  lib_clp
  lib_Osi
  openMVG_linearProgramming
  openMVG_lInftyComputerVision
  lemon
  vlsift  
  jpeg
  png12
  tiff
  stlplus)
 #7.add executable file，添加要编译的可执行文件
 

