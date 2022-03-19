function(sbc_example NAME)
    add_executable(sbc15_fsm_example_${NAME}
        ${NAME}_example.cpp
    )
    target_link_libraries(sbc15_fsm_example_${NAME}
        sbc15_state_machine
        ${catkin_LIBRARIES}
    )
endfunction()


sbc_example(map_exploration)
sbc_example(find_cup)
sbc_example(pick_up_cup)
sbc_example(place_cup)
sbc_example(approach_object)
sbc_example(pick_up)