SET(edit_mincut3_SOURCES edit_mincut3.cpp settings.cpp mincut.cpp)
SET(edit_mincut3_HEADERS edit_mincut3.h settings.h mincut.h)
SET(edit_mincut3_RESOURCES edit_mincut3.qrc)
SET(edit_mincut3_FORMS settings.ui)

QT4_WRAP_CPP(edit_mincut3_HEADERS_MOC ${edit_mincut3_HEADERS})
QT4_ADD_RESOURCES(edit_mincut3_RESOURCES_RCC ${edit_mincut3_RESOURCES})
QT4_WRAP_UI(edit_mincut3_FORMS_HEADERS ${edit_mincut3_FORMS})

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

add_library (edit_mincut3 SHARED ${edit_mincut3_SOURCES} ${edit_mincut3_HEADERS_MOC} ${edit_mincut3_RESOURCES_RCC} ${edit_mincut3_FORMS_HEADERS})
target_link_libraries(edit_mincut3
    ${PCL_LIBRARIES}
    cloudclean
)

INSTALL(TARGETS edit_mincut3 DESTINATION lib/cloudclean/plugins)
