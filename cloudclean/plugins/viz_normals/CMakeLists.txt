SET(viz_normals_SOURCES viz_normals.cpp)
SET(viz_normals_HEADERS viz_normals.h)
SET(viz_normals_RESOURCES viz_normals.qrc)

QT4_WRAP_CPP(viz_normals_HEADERS_MOC ${viz_normals_HEADERS})
QT4_ADD_RESOURCES(viz_normals_RESOURCES_RCC ${viz_normals_RESOURCES})
QT4_WRAP_UI(viz_normals_FORMS_HEADERS ${viz_normals_FORMS})

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
add_library (viz_normals SHARED ${viz_normals_SOURCES} ${viz_normals_HEADERS_MOC} ${viz_normals_RESOURCES_RCC})
