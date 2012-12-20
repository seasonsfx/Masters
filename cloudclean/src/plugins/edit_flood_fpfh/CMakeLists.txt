SET(edit_flood_fpfh_SOURCES edit_flood_fpfh.cpp settings.cpp)
SET(edit_flood_fpfh_HEADERS edit_flood_fpfh.h settings.h)
SET(edit_flood_fpfh_RESOURCES edit_flood_fpfh.qrc)
SET(edit_flood_fpfh_FORMS settings.ui)

QT4_WRAP_CPP(edit_flood_fpfh_HEADERS_MOC ${edit_flood_fpfh_HEADERS})
QT4_ADD_RESOURCES(edit_flood_fpfh_RESOURCES_RCC ${edit_flood_fpfh_RESOURCES})
QT4_WRAP_UI(edit_flood_fpfh_FORMS_HEADERS ${edit_flood_fpfh_FORMS})

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
add_library(edit_flood_fpfh SHARED ${edit_flood_fpfh_SOURCES} ${edit_flood_fpfh_HEADERS_MOC} ${edit_flood_fpfh_RESOURCES_RCC} ${edit_flood_fpfh_FORMS_HEADERS})
target_link_libraries(edit_flood_fpfh
    cloudclean
)
