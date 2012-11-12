#include $(shell rospack find mk)/cmake.mk

# Transfer this to the CMakeLists.txt later.

# Note: xxx_rc is the expected name for resources consuemd by ui files.

DIR = src/kobuki_qtestsuite/detail
UI_DIR = src/kobuki_qtestsuite/ui
RESOURCES_DIR = src/kobuki_qtestsuite/resources

all:
	mkdir -p ${DIR}
	touch ${DIR}/__init__.py
	pyrcc4 -o ${DIR}/common_rc.py ${RESOURCES_DIR}/common.qrc
	pyrcc4 -o ${DIR}/text_rc.py ${RESOURCES_DIR}/text.qrc
	pyuic4 -o ${DIR}/testsuite_ui.py ${UI_DIR}/testsuite.ui 
	pyuic4 -o ${DIR}/configuration_dock_ui.py ${UI_DIR}/configuration_dock.ui
	pyuic4 -o ${DIR}/climbing_frame_ui.py ${UI_DIR}/climbing_frame.ui
	pyuic4 -o ${DIR}/wandering_frame_ui.py ${UI_DIR}/wandering_frame.ui
	pyuic4 -o ${DIR}/battery_profile_frame_ui.py ${UI_DIR}/battery_profile_frame.ui
	
clean:
	rm -rf  ${DIR}
