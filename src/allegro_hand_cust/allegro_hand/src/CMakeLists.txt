# AllegroHandDriver library (CAN 2.0)

if(NOT DEFINED BUILD_SHARED_LIBS)
  option(BUILD_SHARED_LIBS "Build dynamically-linked binaries" ON)
endif()

# alllegro_hand sources
# it includes the urdf-to-kdl code
# and the kinematics solver code
set(AllegroHand_SRC 	AllegroHandDriver.cpp
			controllers/AllegroHandController.cpp
			controllers/AllegroHandControllerGravityCompensator.cpp
			controllers/AllegroHandControllerJointPositionPD.cpp
			controllers/AllegroHandControllerJointPositionPID.cpp
			controllers/AllegroHandControllerTorque.cpp
			controllers/AllegroHandControllerFake.cpp
			AllegroHand.cpp
			AllegroHandConfigurations.cpp
			AllegroHandControl.cpp
			AllegroHandJointLimits.cpp
			AllegroHandKinematics.cpp
		    CAN_AH.cpp
			urdf-to-kdl/kdl_parser.cpp
			urdf-to-kdl/model.cpp
			kinematics/AllegroFinger.cpp
			kinematics/IMAFinger.cpp
			kinematics/operations.cpp
			kinematics/ThumbFinger.cpp
)

add_library(${PROJECT_NAME} ${AllegroHand_SRC})
add_library(${PROJECT_NAME}::AllegroHandDriver ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}>"
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>
)
target_link_libraries(${PROJECT_NAME} PRIVATE ${ALLEGROHAND_LIBS})
#message(status "Allegro hand libs are : " ${ALLEGROHAND_LIBS})
#set_target_properties(AllegroHandDriver PROPERTIES LINK_FLAGS "-Wl,--export-dynamic")
