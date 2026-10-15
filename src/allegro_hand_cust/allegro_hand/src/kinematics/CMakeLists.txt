cmake_minimum_required(VERSION 3.5)
project(AllegroFingers
        VERSION 0.0.1
        DESCRIPTION "Allegro fingers modeling for dexterous manipulation"
        LANGUAGES CXX)


add_executable(mainBinary main.cpp src/AllegroFinger.cpp src/operations.cpp src/IMAFinger.cpp src/ThumbFinger.cpp)
add_executable(fk_ik_tests FK_and_IK_all_fingers.cpp src/AllegroFinger.cpp src/operations.cpp src/IMAFinger.cpp src/ThumbFinger.cpp)
add_executable(ik_thumb_test IK_thumb_finger.cpp src/AllegroFinger.cpp src/operations.cpp src/IMAFinger.cpp src/ThumbFinger.cpp)
add_executable(motion_types_test motionTypesOneTwo.cpp src/AllegroFinger.cpp src/operations.cpp src/IMAFinger.cpp src/ThumbFinger.cpp)


target_compile_features(mainBinary PRIVATE cxx_std_20)
target_include_directories(mainBinary PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(fk_ik_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(ik_thumb_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(motion_types_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)