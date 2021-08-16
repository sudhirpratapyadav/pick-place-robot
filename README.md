# pick-place-robot

## Credits
Following works are taken and modfied

* Gripper (robotiq_2f_85) Models (Meshes, Urdf)
	1. [robotiq ros-industrial package](https://github.com/ros-industrial/robotiq)
	2. [robotiq_arg85_descritipion](https://github.com/a-price/robotiq_arg85_description): robotiq_2f_85 gripper description package with urdf file

* ur5 Model (Meshes, Urdf) and pybullet integration
	1. [pybullet-playground](https://github.com/zswang666/pybullet-playground)
		* **custom_robot.py** file very useful: Standard template for importing urdf file in pybullet
	2. [ur5pybullet](https://github.com/sholtodouglas/ur5pybullet)


##Current Inssues
* pybullet only loads last link color (from material tag) and apply to all robot links
* Need to update **urdf/robotiq_2f_85_actual_mesh.urdf** 