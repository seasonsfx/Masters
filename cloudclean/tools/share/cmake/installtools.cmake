function(find_dlls lib_files dll_dir dll_files)

    list( REMOVE_ITEM lib_files debug optimized)

    foreach( library ${lib_files} )
        get_filename_component( library_name ${library} NAME_WE )
        get_filename_component( library_path ${library} PATH )

        string( REGEX REPLACE "^lib(.*)" "\\1" library_name ${library_name} )
        
        set(file_name ${library_name}.dll)
        list(APPEND search_paths ${library_path} ${dll_dir})
        
        find_file(
            ${library_name}
            ${file_name}
            HINTS
            ${search_paths}
            $ENV{LIBPATH}
            $ENV{PATH}
            $ENV{SYSTEMROOT}/system32
            $ENV{VCINSTALLDIR}/bin
        )

        if( ${library_name} )
            list( APPEND dll_file_list ${${library_name}} )
            message("Found : " ${library_name})
        else()
            message("not found: " ${library_name})
        endif()

    endforeach()
    if(NOT "${dll_file_list}" STREQUAL "")
        list( REMOVE_DUPLICATES dll_file_list )
    endif()
    set(${dll_files} ${dll_file_list} PARENT_SCOPE)
endfunction()

function(followsym _files real_files)
    set (_resolvedFiles "")
    foreach (_file ${_files})
        get_filename_component(_resolvedFile "${_file}" REALPATH)
        list (APPEND _resolvedFiles "${_resolvedFile}")
    endforeach()
    set(${real_files} ${_resolvedFiles} PARENT_SCOPE)
endfunction()
