# Robot Arm (6 Degrees of Freedom)

<img src="https://zanzivyr.github.io/home/images/demo/arm_crop.jpg" height="360"> [![IK Solving Test for Rectangular Path 1](https://img.youtube.com/vi/0N8iMFF4bWM/0.jpg)](https://youtu.be/0N8iMFF4bWM)

Wrote an Inverse Kinematics Solver for robotic arm with 6 degrees of freedom in python. Number crunching using Matlab. I attempted many things in this project. The documentation below is to organize aspects of the project.

### Topics include:
- Python, Matlab
- Linear Algebra, Calculus
  - Inverse Kinematics (6dof), Bezier Curves, Convex Shapes
- Electronics
  - LX-16A motor driver and configuration
  - Serial communications
  - PSMove Controller teardown (basic reverse engineering)
- PSMoveAPI
- 3D printing (PLA), Cura Slicer
- Product Design
- BOM Creation

## Videos
 - _IK Test for Rectangular Path_: https://youtu.be/0N8iMFF4bWM
 - _Test 2_: https://youtu.be/SbTDIftg9qU
 - _Servo Test_: https://youtu.be/0N8iMFF4bWM
## Code
Here is code for actually running the arm.
 - _PS4 Game Pad and Servo Controller_: https://github.com/zanzivyr/6dof_ik/blob/main/main.py
 - _IK Solver_: https://github.com/zanzivyr/6dof_ik/blob/main/ikSolver.py
## Notes
Most of this project was actually mathematics. Below are the documents I wrote before coding.
 - _Inverse Kinematics Algorithm_: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/ik_calculations_v2.pdf
 - _Jacobian Pseudo Inverse_: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/jacobian_pseudo_inverse.pdf
 - _Matlab_: https://github.com/zanzivyr/6dof_ik/blob/main/arm_matlab.mat

Converted my 3D engine for simulating the arm. This helped to sort out some of my problems before getting to the physical arm.
- _Simulator Update_: https://youtu.be/RmtcbHyHOPE
  - Pathing Simulator (Incomplete): https://github.com/zanzivyr/6dof_ik/blob/main/pathing_sim.py
- _Original 3D engine via Quaternions_: https://youtube.com/shorts/hPEd5JOKDyQ
  - 3D Engine: https://github.com/zanzivyr/6dof_ik/blob/main/graphicsEngine.py
  - Custom Linear Algebra Package for 3D engine: https://github.com/zanzivyr/6dof_ik/blob/main/linalg_graphics.py

## Path Planner
Attempted to write a smooth path planner for the end-effector. The intended useage was the following:
1) The user would use the controller to record multiple set points in the workspace
2) These set points would be stored as a point-to-point path in the order received
3) The Path Planner would extrapolate a smooth trajectory over these points via 3D Bezier Curves

### Curvature Method (Newest)
After a lot of introspection, the original method of using ellipses did not work because of non-convex shapes (the order of the points is important). So, instead, I used vectors and curvature. Combined with Bezier Curves, I imagined this would be much more efficient. But I ended the project before testing because of schoolwork.
- _Derivation_: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/pathing_curvature.pdf
- _Code_: https://github.com/zanzivyr/6dof_ik/blob/main/pathing_curvature.py

### Original Algorithm, Ellipse Method (Out of date)
The Ellipse Method takes 3 points in R^3 space and tries to generate an ellipse. This ellipse then can give tangents which are used as handlebars for Bezier Curves. Thus creating a smooth path.
- _Derivation_: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/path_planner_derivation.pdf
- _Code_: https://github.com/zanzivyr/6dof_ik/blob/main/pathing.py

## Motion Controller
The ultimate goal of this project was to create a motion controller which would pair with the arm allowing simple control of the end effector. It was inspired by this video: https://www.youtube.com/watch?v=xfJCUf1uD3M (Website: https://thp.io/2010/psmove/)

Unfortunately I couldn't get my computer to connect with my PSMove controller. So I set out to design my own controller using salvaged components from the PSMove and an MPU6050 9-axis accelerometer. Below is the intended design.
- _Motion Controller Design_: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/motion_controller.pdf

## Miscellaneous
- _Misc. Notes 1_: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/jacobian_pseudo_inverse_matlab_notes.pdf
- _Motor Limits_: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/motor_limits_math.pdf
- _Keyboard Test_: https://github.com/zanzivyr/6dof_ik/blob/main/keyboard_test.py
- _Servo Test_: https://github.com/zanzivyr/6dof_ik/blob/main/servo_test.py

If you want to use LX-16A Motors like I did, then you'll need this zip.
https://drive.google.com/file/d/1dBZVQSwtvPDnMTtPonr_dPYOlMWDc_09/view?usp=sharing
