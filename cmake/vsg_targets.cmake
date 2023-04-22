
function(vsg_declare_library)
    add_library(vsg_vsg INTERFACE)
    add_library(vsg::vsg ALIAS vsg_vsg)

    set_property(
        TARGET vsg_vsg PROPERTY
        EXPORT_NAME vsg
    )

    target_include_directories(
        vsg_vsg ${warning_guard}
        INTERFACE
        "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    )

    target_compile_features(vsg_vsg INTERFACE cxx_std_20)
endfunction()

