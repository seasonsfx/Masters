SET(edit_mincut2_SOURCES edit_mincut2.cpp settings.cpp mincut.cpp compgeom.cpp lasso.cpp)
SET(edit_mincut2_HEADERS edit_mincut2.h settings.h mincut.h compgeom.h lasso.h)
SET(edit_mincut2_RESOURCES edit_mincut2.qrc)
SET(edit_mincut2_FORMS settings.ui)

QT4_WRAP_CPP(edit_mincut2_HEADERS_MOC ${edit_mincut2_HEADERS})
QT4_ADD_RESOURCES(edit_mincut2_RESOURCES_RCC ${edit_mincut2_RESOURCES})
QT4_WRAP_UI(edit_mincut2_FORMS_HEADERS ${edit_mincut2_FORMS})

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
add_library (edit_mincut2 SHARED ${edit_mincut2_SOURCES} ${edit_mincut2_HEADERS_MOC} ${edit_mincut2_RESOURCES_RCC} ${edit_mincut2_FORMS_HEADERS})
target_link_libraries(edit_mincut2
    ${PCL_LIBRARIES}
    cloudclean
)

INSTALL(TARGETS edit_mincut2 DESTINATION lib/cloudclean/plugins)
