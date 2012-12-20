SET(edit_flood_pca_SOURCES edit_flood_pca.cpp settings.cpp)
SET(edit_flood_pca_HEADERS edit_flood_pca.h settings.h)
SET(edit_flood_pca_RESOURCES edit_flood_pca.qrc)
SET(edit_flood_pca_FORMS settings.ui)

QT4_WRAP_CPP(edit_flood_pca_HEADERS_MOC ${edit_flood_pca_HEADERS})
QT4_ADD_RESOURCES(edit_flood_pca_RESOURCES_RCC ${edit_flood_pca_RESOURCES})
QT4_WRAP_UI(edit_flood_pca_FORMS_HEADERS ${edit_flood_pca_FORMS})

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
add_library (edit_flood_pca SHARED ${edit_flood_pca_SOURCES} ${edit_flood_pca_HEADERS_MOC} ${edit_flood_pca_RESOURCES_RCC} ${edit_flood_pca_FORMS_HEADERS})
target_link_libraries(edit_flood_pca
    cloudclean
)
