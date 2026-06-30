function(voloop_add_c_test name)
    add_executable(${name} ${ARGN})

    target_link_libraries(${name}
        PRIVATE
            voloop::voloop
    )

    target_compile_features(${name}
        PRIVATE
            c_std_99
    )

    add_test(
        NAME ${name}
        COMMAND $<TARGET_FILE:${name}>
    )
endfunction()
