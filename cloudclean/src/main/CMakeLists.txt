include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

if(WIN32)
    set(WIN_RESOURCE_FILE cloudclean.rc)
    add_executable(cloudclean WIN32 main.cpp ${WIN_RESOURCE_FILE})
else()
    add_executable(cloudclean main.cpp)
endif()

target_link_libraries(
	cloudclean
    cloudclean_gui
    cloudclean_resources
    cloudclean_model
)

qt5_use_modules(cloudclean Widgets Gui OpenGL)
