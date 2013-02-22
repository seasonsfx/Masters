FILE (GLOB data_normals_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)

add_library(data_normals SHARED ${data_normals_SOURCE})

target_link_libraries(data_normals
    cloudclean_gui
    cloudclean_model
)
