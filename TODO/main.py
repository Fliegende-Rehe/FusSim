from .kinematics import RobotArm
from .solver import solve_ik

# Define the desired end-effector position and orientation
target_pos = [0.5, 0.3, 0.2]
target_ori = [0, 0, 0]

# Create a robot arm object
robot = RobotArm()

# Solve the inverse kinematics
joint_angles = solve_ik(robot, target_pos, target_ori)

# Print the resulting joint angles
print("Joint angles:", joint_angles)
