SET(edit_lasso_SOURCES edit_lasso.cpp)
SET(edit_lasso_HEADERS edit_lasso.h)
SET(edit_lasso_RESOURCES edit_lasso.qrc)

include_directories(${PROJECT_SOURCE_DIR}/cloudclean)

QT4_WRAP_CPP(edit_lasso_HEADERS_MOC ${edit_lasso_HEADERS})
QT4_ADD_RESOURCES(helloworld_RESOURCES_RCC ${helloworld_RESOURCES})

add_library (edit_lasso SHARED edit_lasso.cpp)
