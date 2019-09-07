


FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters )

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

find_package( OpenCV REQUIRED)
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARY_DIRS})







# 增加一个可执行的二进制
ADD_EXECUTABLE( generatePointCloud generatePointCloud.cpp )
target_link_libraries(generatePointCloud 
                      ${OpenCV_LIBS} 
                      ${PCL_LIBRARIES}
                      )