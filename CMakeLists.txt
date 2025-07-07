CMAKE_MINIMUM_REQUIRED(VERSION 3.22)
CMAKE_POLICY(VERSION 3.22)

PROJECT(tnm116-interactions)

INCLUDE(GNUInstallDirs)
LIST (APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")
IF (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  # If not set otherwise, install into a common bin folder
  # regardless of build folder or config
  SET (CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "Install path prefix, prepended onto install directories." FORCE)
ENDIF()

SET (APP_LIBS CACHE INTERNAL "The list of libraries that the app should link against.")

# Gramods dependencies
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(GLEW REQUIRED)
FIND_PACKAGE(SDL2 REQUIRED)
FIND_PACKAGE(tinyxml2 REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(FreeImage REQUIRED)
FIND_PACKAGE(OpenVR)

# Build against Gramods
FIND_PACKAGE(Gramods REQUIRED gmCore gmMisc gmTrack gmGraphics gmNetwork)
LIST (APPEND APP_LIBS gmCore gmMisc gmTrack gmGraphics gmNetwork)

ADD_EXECUTABLE(main main.cpp MyApp.cpp MyApp.hh)

TARGET_LINK_LIBRARIES(main ${APP_LIBS})
SET_PROPERTY(TARGET main PROPERTY CXX_STANDARD 17)
INSTALL(TARGETS main
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
