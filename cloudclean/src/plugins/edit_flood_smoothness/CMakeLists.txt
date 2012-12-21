SET(edit_flood_smoothness_SOURCES edit_flood_smoothness.cpp settings.cpp)
SET(edit_flood_smoothness_HEADERS edit_flood_smoothness.h settings.h)
SET(edit_flood_smoothness_RESOURCES edit_flood_smoothness.qrc)
SET(edit_flood_smoothness_FORMS settings.ui)

QT4_WRAP_CPP(edit_flood_smoothness_HEADERS_MOC ${edit_flood_smoothness_HEADERS})
QT4_ADD_RESOURCES(edit_flood_smoothness_RESOURCES_RCC ${edit_flood_smoothness_RESOURCES})
QT4_WRAP_UI(edit_flood_smoothness_FORMS_HEADERS ${edit_flood_smoothness_FORMS})

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
add_library (edit_flood_smoothness SHARED ${edit_flood_smoothness_SOURCES} ${edit_flood_smoothness_HEADERS_MOC} ${edit_flood_smoothness_RESOURCES_RCC} ${edit_flood_smoothness_FORMS_HEADERS})
target_link_libraries(edit_flood_smoothness
    cloudclean
)

INSTALL(TARGETS edit_flood_smoothness DESTINATION lib/cloudclean/plugins)
