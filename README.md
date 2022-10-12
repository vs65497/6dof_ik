# Robot Arm (6 Degrees of Freedom)

<img src="https://zanzivyr.github.io/home/images/demo/arm_crop.jpg" height="360"> [![IK Solving Test for Rectangular Path 1](https://img.youtube.com/vi/lz0mPY3OvsU/0.jpg)](https://youtu.be/lz0mPY3OvsU)

August 2021 - January 2022, Personal Project. Wrote an Inverse Kinematics Solver for robotic arm with 6 degrees of freedom in python. Number crunching using Matlab. I attempted many things in this project. Though the design of the arm is from Thingiverse, the rest of the assembly and code of the arm is original work. The documentation below is to organize aspects of the project.

**The STL files for the arm come from here:** https://github.com/RoboLabHub/RobotArm_v1.0

### Skills include:
- Python, Javascript, Matlab
- Linear Algebra, Calculus
  - Inverse Kinematics (6dof), Bezier Curves, Convex Shapes
- Electronics
  - LX-16A motor driver and configuration
  - Serial communications
  - PSMove Controller teardown (basic reverse engineering)
- PSMoveAPI
- 3D printing (PLA, Ender3), Cura Slicer, Fusion360 (CAD)
- Product Design
- BOM Creation

## A. Arm, 6 Degrees of Freedom

### Videos
 - _IK Test for Rectangular Path_: https://youtu.be/lz0mPY3OvsU
 - _Test 2_: https://youtu.be/SbTDIftg9qU
 - _Servo Test_: https://youtu.be/0N8iMFF4bWM

### Code (Python)
Here is code for actually running the arm.
 - _PS4 Game Pad and Servo Controller_: https://github.com/zanzivyr/6dof_ik/blob/main/main.py
 - _IK Solver_: https://github.com/zanzivyr/6dof_ik/blob/main/ikSolver.py

### Math Notes
Most of this project was actually mathematics. Below are the documents I wrote before coding.
 - _Inverse Kinematics Math Notes_: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/ik_calculations_v2.pdf
 - _Jacobian Pseudo Inverse Math Notes_: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/jacobian_pseudo_inverse.pdf
 - _Matlab_: https://github.com/zanzivyr/6dof_ik/blob/main/arm_matlab.mat

## B. Custom 3D Engine (Simulation)
Converted my 3D engine for simulating the arm. Also ported from Javascript to Python. This helped to sort out some of my problems before getting to the physical arm.
- _Simulator Update_: https://youtu.be/RmtcbHyHOPE
  - Pathing Simulator (Incomplete): https://github.com/zanzivyr/6dof_ik/blob/main/pathing_sim.py
- _Original 3D engine via Quaternions_: https://youtube.com/shorts/hPEd5JOKDyQ
  - Original Javascript 3D Engine: https://github.com/zanzivyr/3d-light
  - Python Port, 3D Engine: https://github.com/zanzivyr/6dof_ik/blob/main/graphicsEngine.py
  - Custom Linear Algebra Package for 3D engine: https://github.com/zanzivyr/6dof_ik/blob/main/linalg_graphics.py

## C. Path Planner
Attempted to write a smooth path planner for the end-effector. The intended useage was the following:
1) The user would use the controller to record multiple set points in the workspace
2) These set points would be stored as a point-to-point path in the order received
3) The Path Planner would extrapolate a smooth trajectory over these points via 3D Bezier Curves

### Original Algorithm, Ellipse Method (Failed)
The Ellipse Method takes 3 points in R^3 space and tries to generate an ellipse. This ellipse then can give tangents which are used as handlebars for Bezier Curves. Thus creating a smooth path. However this did not work because of non-convex shapes (the order of the points is important). Generating an ellipse requires some constraints, but some edge cases of this approach did not follow those constraints.
- _Ellipse Method Math Notes_: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/path_planner_derivation.pdf
- _Orphan Set Points Math Notes_: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/arm_orphans.pdf
- _Code_: https://github.com/zanzivyr/6dof_ik/blob/main/pathing.py

### Curvature Method (Newest, Untested)
After a lot of introspection, I changed my approach to use vectors and curvature. Combined with Bezier Curves, I imagined this would be effective. But I ended the project before testing due to schoolwork.
- _Math Notes_: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/pathing_curvature.pdf
- _Code_: https://github.com/zanzivyr/6dof_ik/blob/main/pathing_curvature.py

## D. Motion Controller
The ultimate goal of this project was to create a motion controller which would pair with the arm allowing simple control of the end effector. It was inspired by this video: https://www.youtube.com/watch?v=xfJCUf1uD3M (Website: https://thp.io/2010/psmove/)

Unfortunately I couldn't get my computer to connect with my PSMove controller. So I set out to design my own controller using salvaged components from the PSMove and an MPU6050 9-axis accelerometer. Below is the intended design.
- _Attempt to reproduce PSMove Controller_ (Failed): https://youtu.be/7PIn4dRMIpQ
- _Custom Motion Controller Design_: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/motion_controller.pdf

## Miscellaneous
- _Misc. Notes 1_: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/jacobian_pseudo_inverse_matlab_notes.pdf
- _Motor Limits Notes_: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/motor_limits_math.pdf
- _Keyboard Test_: https://github.com/zanzivyr/6dof_ik/blob/main/keyboard_test.py
- _Servo Test_: https://github.com/zanzivyr/6dof_ik/blob/main/servo_test.py

If you want to use LX-16A Motors you'll need this zip.
https://drive.google.com/file/d/1dBZVQSwtvPDnMTtPonr_dPYOlMWDc_09/view?usp=sharing
