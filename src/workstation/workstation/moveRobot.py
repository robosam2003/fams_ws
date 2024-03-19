import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
#from roboticstoolbox 
from Mover6 import Mover6
# Define a robot model
#obot = rtb.models.DH.mo()
# robot = rtb.models.Mover6()
#robot = rtb.models.Panda()

robot=Mover6()
print(robot)

Te = robot.fkine(robot.qr)  # forward kinematics
print(Te)
# ets = robot.ets()
# Tep = ets.fkine([0, -0.3, 0, -2.2, 0, 2, 0.7854])

# Define a desired end-effector pose (position and orientation)
#T_desired = rtb.tools.Txyzrpy([0.5, 0.5, 0.5], [0, 0, 0])
Tep = SE3.Trans(0.6, -0.3, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
sol = robot.ik_LM(Tep)         # solve IK
print(sol)
#sol = robot.ik_lm_chan(Tep)         # solve IK
# print(sol)


# T_desired = rtb.SE3([0.5, 0.5, 0.5], [0, 0, 0])
# T_desired = ([0.5, 0.5, 0.5], [0, 0, 0])

# # Solve inverse kinematics
# q_solutions = robot.ikine(T_desired)
 
# # Print the solutions
# print("Found", len(q_solutions), "solutions")
# for q in q_solutions:
#     print("Joint angles:", q)

# # Visualize the robot
# robot.plot(q_solutions)
