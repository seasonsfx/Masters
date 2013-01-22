project (CloudClean)
cmake_minimum_required (VERSION 2.8.6)
set(CMAKE_BUILD_TYPE Release)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

if(CMAKE_BUILD_TOOL MATCHES "(msdev|devenv|nmake)")
    SET(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} /W2") # Mute warnings in ms
    add_definitions(-DNOMINMAX) # Workaround for windows.h bug
else()
    SET(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -Wall")
    SET(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -std=c++0x")
    SET(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -fopenmp")
    SET(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
    SET(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -Wno-unknown-pragmas")
    SET(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -fstack-protector-all")

    #SET(CMAKE_C_COMPILER             "clang")
    #SET(CMAKE_CXX_COMPILER             "clang++")
    #SET(CMAKE_AR      "llvm-ar")
    #SET(CMAKE_LINKER  "llvm-ld")
    #SET(CMAKE_NM      "llvm-nm")
    #SET(CMAKE_OBJDUMP "llvm-objdump")
    #SET(CMAKE_RANLIB  "llvm-ranlib")

endif()

list(APPEND CMAKE_CXXFLAGS_RELEASE -O3)

add_definitions(
    -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
    -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED # Hack
)

include_directories(
        ${PROJECT_SOURCE_DIR}/src
)

if(WIN32)
    set(BIN_DIR_SUFFIX bin)
    set(LIB_DIR_SUFFIX bin)
    set(PLUGIN_DIR_SUFFIX plugins)

    set(BIN_INSTALL_DIR_SUFFIX ${BIN_DIR_SUFFIX})
    set(LIB_INSTALL_DIR_SUFFIX ${LIB_DIR_SUFFIX})
    set(PLUGIN_INSTALL_DIR_SUFFIX plugins)
else()
    set(BIN_DIR_SUFFIX bin)
    set(LIB_DIR_SUFFIX lib)
    set(PLUGIN_DIR_SUFFIX plugins)

    set(BIN_INSTALL_DIR_SUFFIX ${BIN_DIR_SUFFIX})
    set(LIB_INSTALL_DIR_SUFFIX ${LIB_DIR_SUFFIX}/cloudclean)
    set(PLUGIN_INSTALL_DIR_SUFFIX ${LIB_DIR_SUFFIX}/cloudclean/plugins)
endif()


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/${BIN_DIR_SUFFIX})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/${LIB_DIR_SUFFIX})

find_package(Qt4 REQUIRED)
set(QT_USE_OPENGL TRUE)

include(${QT_USE_FILE})

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

SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "/usr/lib/cloudclean")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

add_subdirectory(src/cloudclean)
add_subdirectory(src/main)
add_subdirectory(src/plugins)

# INSTALL
function(find_dlls lib_files dll_dir dll_files)

    list( REMOVE_ITEM lib_files debug optimized)

    foreach( library ${lib_files} )
        get_filename_component( library_name ${library} NAME_WE )
        get_filename_component( library_path ${library} PATH )

        string( REGEX REPLACE "^lib(.*)" "\\1" library_name ${library_name} )
        
        set(file_name ${library_name}.dll)
        list(APPEND search_paths ${library_path} ${dll_dir})
        
        find_file(
            ${library_name}
            ${file_name}
            HINTS
            ${search_paths}
            $ENV{LIBPATH}
            $ENV{PATH}
            $ENV{SYSTEMROOT}/system32
            $ENV{VCINSTALLDIR}/bin
        )

        if( ${library_name} )
            list( APPEND dll_file_list ${${library_name}} )
            message("Found : " ${library_name})
        else()
            message("not found: " ${library_name})
        endif()

    endforeach()
    if(NOT "${dll_file_list}" STREQUAL "")
        list( REMOVE_DUPLICATES dll_file_list )
    endif()
    set(${dll_files} ${dll_file_list} PARENT_SCOPE)
endfunction()

function(followsym _files real_files)
    set (_resolvedFiles "")
    foreach (_file ${_files})
        get_filename_component(_resolvedFile "${_file}" REALPATH)
        list (APPEND _resolvedFiles "${_resolvedFile}")
    endforeach()
    set(${real_files} ${_resolvedFiles} PARENT_SCOPE)
endfunction()

if(WIN32)

    find_dlls("${QT_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTOPENGL_LIBRARY}" ${QT_BINARY_DIR} qt_dll_files)
    install( FILES ${qt_dll_files} DESTINATION ${LIB_DIR_SUFFIX} COMPONENT QT )

    find_dlls("${PCL_LIBRARIES}" $ENV{PROGRAMW6432}/PCL/bin pcl_dll_files)
    install( FILES ${pcl_dll_files} DESTINATION ${LIB_DIR_SUFFIX} COMPONENT PCL )

    find_dlls("${GLEW_LIBRARIES}" "" glew_dll_files)
    install(FILES ${glew_dll_files} DESTINATION ${LIB_DIR_SUFFIX} COMPONENT GLEW )

else()
    install(FILES debian/cloudclean.png DESTINATION share/pixmaps/)
    install(FILES debian/cloudclean.desktop DESTINATION share/applications/)

    list( REMOVE_ITEM PCL_LIBRARIES debug optimized)
    followsym("${PCL_LIBRARIES}" PCL_LIBRARIES)
    followsym("${QT_LIBRARIES};${QT_QTCORE_LIBRARY};${QT_QTGUI_LIBRARY};${QT_QTOPENGL_LIBRARY}" QT_LIBRARIES)
    install(FILES ${PCL_LIBRARIES} DESTINATION ${LIB_INSTALL_DIR_SUFFIX} COMPONENT PCL)
    install(FILES ${QT_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTOPENGL_LIBRARY} DESTINATION ${LIB_INSTALL_DIR_SUFFIX} COMPONENT QT)

endif()


# Packaging
set(CPACK_PACKAGE_DESCRIPTION "A pointcloud editing framework.")
set(CPACK_PACKAGE_CONTACT "Rickert Mulder <circlingthesun@gmail.com>")

SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENCE.txt")

set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "1")
set(CPACK_PACKAGE_VERSION_PATCH "0")

SET(CPACK_PACKAGE_INSTALL_DIRECTORY "CloudClean ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}")

if(WIN32)
    set(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP TRUE)
    include(InstallRequiredSystemLibraries)
    install(PROGRAMS ${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS}
        DESTINATION ${LIB_INSTALL_DIR_SUFFIX} COMPONENT VisualCPPRuntime)

    SET(OMP_PATH $ENV{VCINSTALLDIR}/redist/x64/Microsoft.VC110.OpenMP/vcomp110.dll)
    STRING(REGEX REPLACE "\\\\" "/" OMP_PATH ${OMP_PATH}) 

    install(PROGRAMS ${OMP_PATH}
        DESTINATION ${LIB_INSTALL_DIR_SUFFIX} COMPONENT VisualCPPOpenMPRuntime)
    
    set(CPACK_GENERATOR "NSIS")
    SET(CPACK_PACKAGE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/debian\\\\cloudclean.bmp")
    SET(CPACK_NSIS_INSTALLED_ICON_NAME "${CMAKE_CURRENT_SOURCE_DIR}/src/main/cloudclean.ico")
    SET(CPACK_NSIS_MUI_ICON "${CMAKE_CURRENT_SOURCE_DIR}/src/main/cloudclean.ico")
    SET(CPACK_NSIS_DISPLAY_NAME "CloudClean")
    SET(CPACK_NSIS_CREATE_ICONS "CreateShortCut '\$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\${PROJECT_NAME}.lnk' '\$INSTDIR\\\\bin\\\\run_cloudclean.exe'")
    SET(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\people.cs.uct.ac.za/~rmulder")
    SET(CPACK_NSIS_CONTACT ${CPACK_PACKAGE_CONTACT})
    SET(CPACK_NSIS_MODIFY_PATH ON)
    SET(CPACK_NSIS_MUI_FINISHPAGE_RUN "run_cloudclean.exe")
    SET(CPACK_NSIS_MODIFY_PATH "OFF")


else()
    set(CPACK_GENERATOR "DEB")
    set(CPACK_DEBIAN_PACKAGE_NAME, "cloudclean")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
    #set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.3.1-6), libgcc1 (>= 1:3.4.2-12), libqt4-opengl (>=4.8.0), libqt4-core (>=4.8.0), libqt4-gui (>=4.8.0), libglu1-mesa" )
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "libglu1-mesa" )
    set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/debian/postinst")
    set( CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/debian/postinst;${CMAKE_CURRENT_SOURCE_DIR}/debian/prerm;" )
endif()

include(CPack)
