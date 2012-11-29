SET(edit_lasso_SOURCES edit_lasso.cpp settings.cpp)
SET(edit_lasso_HEADERS edit_lasso.h settings.h)
SET(edit_lasso_RESOURCES edit_lasso.qrc)
SET(edit_lasso_FORMS settings.ui)

QT4_WRAP_CPP(edit_lasso_HEADERS_MOC ${edit_lasso_HEADERS})
QT4_ADD_RESOURCES(edit_lasso_RESOURCES_RCC ${edit_lasso_RESOURCES})
QT4_WRAP_UI(edit_lasso_FORMS_HEADERS ${edit_lasso_FORMS})

add_library (edit_lasso SHARED ${edit_lasso_SOURCES} ${edit_lasso_HEADERS_MOC} ${edit_lasso_RESOURCES_RCC} ${edit_lasso_FORMS_HEADERS})
