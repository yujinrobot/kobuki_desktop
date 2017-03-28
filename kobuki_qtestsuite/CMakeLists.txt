cmake_minimum_required(VERSION 2.8.3)
project(kobuki_qtestsuite)

find_package(catkin REQUIRED COMPONENTS rospy
                                        kobuki_testsuite
                                        qt_gui_py_common
                                        rqt_gui
                                        rqt_gui_py
                                        rqt_py_common
                                        rqt_plot
                                        geometry_msgs
                                        nav_msgs)

catkin_python_setup()

catkin_package()

set(DIR src/kobuki_qtestsuite/detail)
set(UI_DIR src/kobuki_qtestsuite/ui)
set(RESOURCES_DIR src/kobuki_qtestsuite/resources)

add_custom_target(${PROJECT_NAME}
	          ALL
                  COMMAND pyrcc5 -o ${DIR}/common_rc.py ${RESOURCES_DIR}/common.qrc
	          COMMAND pyrcc5 -o ${DIR}/common_rc.py ${RESOURCES_DIR}/common.qrc
	          COMMAND pyrcc5 -o ${DIR}/text_rc.py ${RESOURCES_DIR}/text.qrc
	          COMMAND pyuic5 -o ${DIR}/testsuite_ui.py ${UI_DIR}/testsuite.ui
	          COMMAND pyuic5 -o ${DIR}/configuration_dock_ui.py ${UI_DIR}/configuration_dock.ui
	          COMMAND pyuic5 -o ${DIR}/battery_profile_frame_ui.py ${UI_DIR}/battery_profile_frame.ui
	          COMMAND pyuic5 -o ${DIR}/gyro_drift_frame_ui.py ${UI_DIR}/gyro_drift_frame.ui
	          COMMAND pyuic5 -o ${DIR}/payload_frame_ui.py ${UI_DIR}/payload_frame.ui
	          COMMAND pyuic5 -o ${DIR}/cliff_sensor_frame_ui.py ${UI_DIR}/cliff_sensor_frame.ui
	          COMMAND pyuic5 -o ${DIR}/life_frame_ui.py ${UI_DIR}/life_frame.ui
	          COMMAND pyuic5 -o ${DIR}/wandering_frame_ui.py ${UI_DIR}/wandering_frame.ui
	          COMMAND pyuic5 -o ${DIR}/climbing_frame_ui.py ${UI_DIR}/climbing_frame.ui
	          WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

install(PROGRAMS scripts/kobuki_qtestsuite
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})   
