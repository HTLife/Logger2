cmake_minimum_required(VERSION 2.8)
find_package(ZLIB REQUIRED)
find_package(Qt4 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Boost COMPONENTS date_time REQUIRED)
find_package( Eigen3 REQUIRED )

include(${QT_USE_FILE})
 
qt4_wrap_cpp(main_moc_SRCS
             main.h)

IF(UNIX)
    set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3")
ENDIF (UNIX)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/Include)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${ZLIB_INCLUDE_DIR})

include_directories(${EIGEN_INCLUDE_DIRS})

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(BOOST_ALL_DYN_LINK ON)   # force dynamic linking for all libraries

set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3")


  if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
    endif ()
  else ()
    set (CMAKE_CXX_STANDARD 11)
  endif ()

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#Magic hack to check if host is 32 or 64 bit
if(APPLE)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/libOpenNI2.dylib ${CMAKE_CURRENT_BINARY_DIR}/libOpenNI2.dylib COPYONLY)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/libOniFile.dylib ${CMAKE_CURRENT_BINARY_DIR}/OpenNI2/Drivers/libOniFile.dylib COPYONLY)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/libPS1080.dylib ${CMAKE_CURRENT_BINARY_DIR}/OpenNI2/Drivers/libPS1080.dylib COPYONLY)
elseif(UNIX)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/libOpenNI2x64.so ${CMAKE_CURRENT_BINARY_DIR}/libOpenNI2.so COPYONLY)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/libDummyDevicex64.so ${CMAKE_CURRENT_BINARY_DIR}/OpenNI2/Drivers/libDummyDevice.so COPYONLY)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/libOniFilex64.so ${CMAKE_CURRENT_BINARY_DIR}/OpenNI2/Drivers/libOniFile.so COPYONLY)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/libPS1080x64.so ${CMAKE_CURRENT_BINARY_DIR}/OpenNI2/Drivers/libPS1080.so COPYONLY)
    else()
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/libOpenNI2x86.so ${CMAKE_CURRENT_BINARY_DIR}/libOpenNI2.so COPYONLY)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/libDummyDevicex86.so ${CMAKE_CURRENT_BINARY_DIR}/OpenNI2/Drivers/libDummyDevice.so COPYONLY)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/libOniFilex86.so ${CMAKE_CURRENT_BINARY_DIR}/OpenNI2/Drivers/libOniFile.so COPYONLY)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/libPS1080x86.so ${CMAKE_CURRENT_BINARY_DIR}/OpenNI2/Drivers/libPS1080.so COPYONLY)
    endif()
endif()

set(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath,. -L${CMAKE_CURRENT_BINARY_DIR}")

set(srcs main.cpp
         Logger2.cpp
         Communicator.cpp
         common.cpp
         common_3d.cpp
         ${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/OpenNI2Interface.cpp)


set(MSCL_INC "/home/rvl/code/MSCL/MSCL/source") 

include_directories(${MSCL_INC})

set(MSCL_LIB "/home/rvl/code/MSCL/Output/C++/Release/libmscl.so")
#LINK_DIRECTORIES("/home/rvl/code/MSCL/Output/C++/Release")

add_executable(Logger2 ${main_moc_SRCS} ${srcs})

target_link_libraries(Logger2 -ljpeg                       
                      ${ZLIB_LIBRARY}
                      ${Boost_LIBRARIES}
                      ${OpenCV_LIBS} 
                      ${QT_LIBRARIES}
                      ${MSCL_LIB}
                      boost_system
                      boost_filesystem
                      boost_thread
                      OpenNI2)

