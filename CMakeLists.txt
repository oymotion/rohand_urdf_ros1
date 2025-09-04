cmake_minimum_required(VERSION 2.8.3)

project(rohand_urdf_ros1)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

set(PYTHON_SCRIPTS
  scripts/FingerMathURDF.py
  scripts/rohand_joint_state_gui.py
  scripts/rohand_urdf.py
)

foreach(dir config launch meshes_r meshes_l urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
