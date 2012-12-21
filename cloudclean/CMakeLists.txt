project (cloudclean)
cmake_minimum_required (VERSION 2.8.6)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

if(WIN32)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
else()
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
endif()

set(CMAKE_BUILD_TYPE Release)

find_package(Qt4 REQUIRED)
set(QT_USE_OPENGL TRUE)
include(${QT_USE_FILE})

add_definitions(
        -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
        -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED # Hack
)

if(CMAKE_BUILD_TOOL MATCHES "(msdev|devenv|nmake)")
    add_definitions(
        /W2 # Mute warnings in ms
        -DNOMINMAX # Workaround for windows.h bug
    )
else()
    add_definitions(
        -Wall -std=c++0x
        -fopenmp
        -Wno-deprecated-declarations
        -Wno-unknown-pragmas
        -fstack-protector-all
        #-stdlib=libc++
    )
endif()

list(APPEND QMAKE_CXXFLAGS_RELEASE -O3)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(PCL 1.7 REQUIRED COMPONENTS
	common
	io
	octree
	kdtree
	search
	features
	filters
    segmentation
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenGL REQUIRED)
find_package(OpenCL REQUIRED)

include_directories(
	ext
	src/common
    src/cloudclean
	${OPENCL_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIR}
)


# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

# the RPATH to be used when installing
SET(CMAKE_INSTALL_RPATH "/usr/lib/cloudclean")

# don't add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

add_subdirectory(src/cloudclean)
add_subdirectory(src/main)
add_subdirectory(src/plugins)

install(FILES debian/cloudclean.png DESTINATION share/pixmaps/)
install(FILES debian/cloudclean.desktop DESTINATION share/applications/)

set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "1")
set(CPACK_PACKAGE_VERSION_PATCH "0")
set(CPACK_DEBIAN_PACKAGE_NAME, "cloudclean")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.3.1-6), libgcc1 (>= 1:3.4.2-12), libqt4-opengl (>=4.8.0), libqt4-core (>=4.8.0), libqt4-gui (>=4.8.0), libglu1-mesa" )
set(CPACK_PACKAGE_DESCRIPTION "A pointcloud editing framework.")
set(CPACK_PACKAGE_CONTACT "Rickert Mulder <circlingthesun@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/debian/postinst")
include(CPack)
