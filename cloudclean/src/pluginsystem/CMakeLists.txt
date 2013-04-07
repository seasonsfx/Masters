add_definitions(-Dpluginsystem_EXPORTS)

FILE (GLOB pluginsystem_SRCS *.cpp *.cxx *.cc *.C *.c *.h *.hpp)

add_library(
    cloudclean_pluginsystem STATIC
    ${pluginsystem_SRCS}
)

target_link_libraries(
    cloudclean_pluginsystem
    ${QT_LIBRARIES}
    cloudclean_model
    cloudclean_gui
)
