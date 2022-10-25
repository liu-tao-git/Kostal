Flexiv Description
==================

This package contains urdf for flexiv robots.
The urdf contains kinematic and dynamic parameters, used by Robot Control App.
The kinematic frames are identical to those found in the User Manual.

Running colcon build will re-generate the urdf from the xacro files,
via xacro_add_files() in the CMakeLists.txt

Package              | File                    | DOF | Description
-------------------- | ----------------------- | --- | -----------
flexiv_description   | A02L-P.urdf            |   7 | flexiv light arm
flexiv_description   | A02L-M.urdf            |   7 | flexiv light arm
flexiv_description   | A02LMR-M.urdf          |   7 | flexiv mirrored light arm
flexiv_description   | A02LS-D.urdf         |   7 | flexiv light arm with force-torque sensor
flexiv_description   | A02LS-P.urdf         |   7 | flexiv light arm with force-torque sensor
flexiv_description   | A02LD-X.urdf       |  14 | flexiv dual arm, based on A02L-M and A02LMR-M
flexiv_description   | A02LD_left-X.urdf  |   7 | flexiv dual arm, mounted left arm only
flexiv_description   | A02LD_right-X.urdf |   7 | flexiv dual arm, mounted right arm only
flexiv_description   | A02H-D.urdf           |   7 | flexiv heavy arm
flexiv_description   | A02H-P.urdf           |   7 | flexiv heavy arm