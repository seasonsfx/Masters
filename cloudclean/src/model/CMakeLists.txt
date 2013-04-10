add_definitions(-Dmodel_EXPORTS)

include_directories(
    ${QT_QTCORE_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FILE (GLOB model_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)

add_library(cloudclean_model SHARED ${model_SOURCE})
target_link_libraries(
    cloudclean_model
    ${QT_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_IOSTREAMS_LIBRARY}
)

INSTALL(TARGETS cloudclean_model DESTINATION ${LIB_INSTALL} COMPONENT CloudClean)

#qt5_use_modules(cloudclean_model Core)
