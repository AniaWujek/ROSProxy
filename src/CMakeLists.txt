SET(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(.)

# ##############################################################################
# Find required libraries
# ##############################################################################

# Find Boost, at least ver. 1.41
FIND_PACKAGE(Boost 1.41.0 REQUIRED COMPONENTS system thread filesystem date_time)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

# Find another necessary libraries

# # OpenCV library
# FIND_PACKAGE( OpenCV REQUIRED )

# Find another DCLs our one depends on
# Use macro: DISCODE_FIND_DCL(<DCL_NAME>)
DISCODE_FIND_DCL(CvCoreTypes)


SET(DisCODe_LIBRARIES ${DisCODe_LIBRARIES} ${Boost_LIBRARIES})

SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,defs") 


# ##############################################################################
# Add source directories
# ##############################################################################

# CvBlobs components
ADD_SUBDIRECTORY(Components)

# CvBlobs types
ADD_SUBDIRECTORY(Types)

# Prepare config file to use from another DCLs
CONFIGURE_FILE(ROSProxyConfig.cmake.in ${CMAKE_INSTALL_PREFIX}/ROSProxyConfig.cmake @ONLY)
