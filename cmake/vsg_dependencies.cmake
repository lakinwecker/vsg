function(vsg_find_dependencies_thirdparty)
    find_package(doctest REQUIRED)
    find_package(immer REQUIRED)
endfunction()

function (vsg_dependency_targets)
    set(VSG_DEPENDENCY_TARGETS 
        doctest::doctest
        immer::immer
        PARENT_SCOPE)
    # NOTE: remove duplicates so it can be called more than once.
    list(REMOVE_DUPLICATES VSG_DEPENDENCY_TARGETS)
endfunction()
