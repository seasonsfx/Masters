SET(edit_mincut_SOURCES edit_mincut.cpp mincut.cpp settings.cpp)
SET(edit_mincut_HEADERS edit_mincut.h settings.h mincut.h)
SET(edit_mincut_RESOURCES edit_mincut.qrc)
SET(edit_mincut_FORMS settings.ui)

QT4_WRAP_CPP(edit_mincut_HEADERS_MOC ${edit_mincut_HEADERS})
QT4_ADD_RESOURCES(edit_mincut_RESOURCES_RCC ${edit_mincut_RESOURCES})
QT4_WRAP_UI(edit_mincut_FORMS_HEADERS ${edit_mincut_FORMS})

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
add_library (edit_mincut SHARED ${edit_mincut_SOURCES} ${edit_mincut_HEADERS_MOC} ${edit_mincut_RESOURCES_RCC} ${edit_mincut_FORMS_HEADERS})

target_link_libraries(edit_mincut
    #${QT_LIBRARIES}
    #${QT_QTCORE_LIBRARY}
    #${QT_QTGUI_LIBRARY}
    #${QT_QTCORE_LIBRARY}
    #${QT_QTGUI_LIBRARY}
    #${QT_QTOPENGL_LIBRARY}
    ${PCL_LIBRARIES}
)
