# Robot Arm (6 Degrees of Freedom)

Wrote the Inverse Kinematics Solver for this in python. Number crunching using Matlab.
## Code
 - PS4 Game Pad and Servo Controller: https://github.com/zanzivyr/6dof_ik/blob/main/main.py
 - IK Solver: https://github.com/zanzivyr/6dof_ik/blob/main/ikSolver.py
 - Matlab: https://github.com/zanzivyr/6dof_ik/blob/main/arm_matlab.mat
 - 
## Notes
 - Inverse Kinematics Algorithm: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/ik_calculations_v2.pdf
 - Jacobian Pseudo Inverse: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/jacobian_pseudo_inverse.pdf
## Videos
 - IK Solving Test for Rectangular Path 1: https://youtu.be/0N8iMFF4bWM
 - Test 2: https://youtu.be/SbTDIftg9qU
 - Servo Test: https://youtu.be/0N8iMFF4bWM

Converted my 3D engine for simulating the arm. This helped to sort out some of my problems before getting to the physical arm.
- Simulator Update: https://youtu.be/RmtcbHyHOPE
- Original 3D engine via Quaternions: https://youtube.com/shorts/hPEd5JOKDyQ

## Path Planner
Attempted to write a smooth path planner for the end-effector. The intended useage was the following:
1) The user would use the controller to record multiple set points in the workspace
2) These set points would be stored as a point-to-point path in the order received
3) The Path Planner would extrapolate a smooth trajectory over these points

### Curvature Method (Newest)
- Derivation: https://github.com/zanzivyr/6dof_ik/blob/main/pathing_curvature.pdf
- Code: https://github.com/zanzivyr/6dof_ik/blob/main/pathing_curvature.py

### Original Algorithm (Out of date)
- Derivation: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/path_planner_derivation.pdf
- Code: https://github.com/zanzivyr/6dof_ik/blob/main/pathing.py

## Miscellaneous
- Misc. Notes 1: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/jacobian_pseudo_inverse_matlab_notes.pdf
- Motor Limits: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/motor_limits_math.pdf
- 3D Engine: https://github.com/zanzivyr/6dof_ik/blob/main/graphicsEngine.py
- Keyboard Test: https://github.com/zanzivyr/6dof_ik/blob/main/keyboard_test.py
- Pathing Simulator (Incomplete): https://github.com/zanzivyr/6dof_ik/blob/main/pathing_sim.py
- Servo Test: https://github.com/zanzivyr/6dof_ik/blob/main/servo_test.py
