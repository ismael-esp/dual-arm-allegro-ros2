# Build basic example, test Allegro Hand Driver 
add_executable(test_allegro_hand_driver example_driver.cpp)
target_link_libraries(test_allegro_hand_driver allegro_hand::AllegroHandDriver)

# Build basic example, test Allegro Hand Library 
add_executable(test_allegro_hand_controller example_controller.cpp)
target_link_libraries(test_allegro_hand_controller allegro_hand::AllegroHandDriver)

# Build basic example, test Allegro Hand Library 
add_executable(test_allegro_hand example.cpp)
target_link_libraries(test_allegro_hand allegro_hand::AllegroHandDriver)

# Build basic example, test FK and IK all fingers
add_executable(test_FK_and_IK example_FK_and_IK_all_fingers.cpp)
target_link_libraries(test_FK_and_IK allegro_hand::AllegroHandDriver)

# Build basic example, test motionTypesOneTwo
add_executable(test_motionTypesOneTwo motionTypesOneTwo.cpp)
target_link_libraries(test_motionTypesOneTwo allegro_hand::AllegroHandDriver)

# Build basic example, test motionTypesOneTwo
add_executable(test_solver solver.cpp)
target_link_libraries(test_solver allegro_hand::AllegroHandDriver)

find_package(yaml-cpp REQUIRED)
# Calculate minimum and maximum for all the joints of the hand. If desired it can rewrite the YAML configuration file.
add_executable(joint_limits calculate_joint_limits.cpp)
target_link_libraries(joint_limits allegro_hand::AllegroHandDriver yaml-cpp)