# Robot Arm (6 Degrees of Freedom)

Wrote the Inverse Kinematics Solver for this in python. Number crunching using Matlab.
## Notes
 - Inverse Kinematics: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/ik_calculations_v2.pdf
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
- 1) The user would use the controller to record multiple set points in the workspace
- 2) These set points would be stored as a point-to-point path in the order received
- 3) The Path Planner would extrapolate a smooth trajectory over these points

Algorithm Notes: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/path_planner_derivation.pdf

## Miscellaneous
- Misc. Notes 1: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/jacobian_pseudo_inverse_matlab_notes.pdf
- Motor Limits: https://raw.githubusercontent.com/zanzivyr/6dof_ik/main/motor_limits_math.pdf
