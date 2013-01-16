SET(edit_flood_normals_SOURCES edit_flood_normals.cpp settings.cpp)
SET(edit_flood_normals_HEADERS edit_flood_normals.h settings.h)
SET(edit_flood_normals_RESOURCES edit_flood_normals.qrc)
SET(edit_flood_normals_FORMS settings.ui)

QT4_WRAP_CPP(edit_flood_normals_HEADERS_MOC ${edit_flood_normals_HEADERS})
QT4_ADD_RESOURCES(edit_flood_normals_RESOURCES_RCC ${edit_flood_normals_RESOURCES})
QT4_WRAP_UI(edit_flood_normals_FORMS_HEADERS ${edit_flood_normals_FORMS})

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
add_library (edit_flood_normals SHARED ${edit_flood_normals_SOURCES} ${edit_flood_normals_HEADERS_MOC} ${edit_flood_normals_RESOURCES_RCC} ${edit_flood_normals_FORMS_HEADERS})
target_link_libraries(edit_flood_normals
    cloudclean
)
INSTALL(TARGETS edit_flood_normals DESTINATION ${PLUGIN_INSTALL_DIR_SUFFIX} COMPONENT CloudClean)
