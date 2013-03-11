FILE (GLOB pluginsystem_SRCS *.cpp *.cxx *.cc *.C *.c *.h *.hpp)

add_library(
    pluginsystem STATIC
    ${pluginsystem_SRCS}
)

target_link_libraries(
    pluginsystem
    ${QT_LIBRARIES}
)