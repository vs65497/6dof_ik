# Robot Arm (6 Degrees of Freedom)

Wrote the Inverse Kinematics Solver for this in python. Number crunching using Matlab. 

This was a long and winding project in which I attempted many things. The files and code are not well laid out, so the documentation below is my attempt to clarify it.

## Code
 - MAIN: PS4 Game Pad and Servo Controller: https://github.com/zanzivyr/6dof_ik/blob/main/main.py
 - IK Solver: https://github.com/zanzivyr/6dof_ik/blob/main/ikSolver.py
 - Matlab: https://github.com/zanzivyr/6dof_ik/blob/main/arm_matlab.mat
## Notes
 - Inverse Kinematics Algorithm: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/ik_calculations_v2.pdf
 - Jacobian Pseudo Inverse: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/jacobian_pseudo_inverse.pdf
## Videos
 - IK Solving Test for Rectangular Path 1: https://youtu.be/0N8iMFF4bWM
 - Test 2: https://youtu.be/SbTDIftg9qU
 - Servo Test: https://youtu.be/0N8iMFF4bWM

Converted my 3D engine for simulating the arm. This helped to sort out some of my problems before getting to the physical arm.
- Simulator Update: https://youtu.be/RmtcbHyHOPE
  - Pathing Simulator (Incomplete): https://github.com/zanzivyr/6dof_ik/blob/main/pathing_sim.py
- Original 3D engine via Quaternions: https://youtube.com/shorts/hPEd5JOKDyQ
  - 3D Engine: https://github.com/zanzivyr/6dof_ik/blob/main/graphicsEngine.py
  - Custom Linear Algebra Package for 3D engine: https://github.com/zanzivyr/6dof_ik/blob/main/linalg_graphics.py

## Path Planner
Attempted to write a smooth path planner for the end-effector. The intended useage was the following:
1) The user would use the controller to record multiple set points in the workspace
2) These set points would be stored as a point-to-point path in the order received
3) The Path Planner would extrapolate a smooth trajectory over these points via 3D Bezier Curves

### Curvature Method (Newest)
After a lot of introspection, the original method of using ellipses did not work because of non-convex shapes (the order of the points is important). So, instead, I used vectors and curvature. Combined with Bezier Curves, I imagined this would be much more efficient. But I ended the project before testing because of schoolwork.
- Derivation: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/pathing_curvature.pdf
- Code: https://github.com/zanzivyr/6dof_ik/blob/main/pathing_curvature.py

### Original Algorithm, Ellipse Method (Out of date)
The Ellipse Method takes 3 points in R^3 space and tries to generate an ellipse. This ellipse then can give tangents which are used as handlebars for Bezier Curves. Thus creating a smooth path.
- Derivation: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/path_planner_derivation.pdf
- Code: https://github.com/zanzivyr/6dof_ik/blob/main/pathing.py

## Motion Controller
The ultimate goal of this project was to create a motion controller which would pair with the arm allowing simple control of the end effector. It was inspired by this video: https://www.youtube.com/watch?v=xfJCUf1uD3M

Unfortunately I couldn't get my computer to connect with my PSMove controller. So I set out to design my own controller using salvaged components from the PSMove and an MPU6050 9-axis accelerometer. Below is the intended design.
- Motion Controller Design: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/motion_controller.pdf

## Miscellaneous
- Misc. Notes 1: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/jacobian_pseudo_inverse_matlab_notes.pdf
- Motor Limits: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/motor_limits_math.pdf
- Keyboard Test: https://github.com/zanzivyr/6dof_ik/blob/main/keyboard_test.py
- Servo Test: https://github.com/zanzivyr/6dof_ik/blob/main/servo_test.py

If you want to use LX-16A Motors like I did, then you'll need this zip.
https://drive.google.com/file/d/1dBZVQSwtvPDnMTtPonr_dPYOlMWDc_09/view?usp=sharing
