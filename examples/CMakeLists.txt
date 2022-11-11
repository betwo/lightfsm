function(add_lightfsm_example_example NAME)
    add_executable(${PROJECT_NAME}_example_${NAME}
        ${NAME}_example.cpp
    )
    target_link_libraries(${PROJECT_NAME}_example_${NAME}
        ${PROJECT_NAME}
    )

    install(TARGETS
        ${PROJECT_NAME}_example_${NAME}
        DESTINATION lib/${PROJECT_NAME}
    )
endfunction()

add_lightfsm_example_example(simple_fsm)