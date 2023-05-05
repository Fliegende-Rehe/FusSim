from resources.ik.robotic_arm import *

def main():
    '''Given a two degree of freedom robotic arm and a desired end position of the end effector,
       calculate the two joint angles (i.e. servo angles).
    '''

    # A 2D array that lists the different axes of rotation (rows) for each joint
    # Here I assume our robotic arm has two joints, but you can add more if you like.
    # k = kx, ky, kz
    k = np.array([[0, 0, 1], [0, 0, 1]])

    # A 2D array that lists the translations from the previous joint to the current joint
    # The first translation is from the base frame to joint 1 (which is equal to the base frame)
    # The second translation is from joint 1 to joint 2
    # t = tx, ty, tz
    # These values are measured with a ruler based on the kinematic diagram
    # This tutorial teaches you how to draw kinematic diagrams:
    # https://automaticaddison.com/how-to-assign-denavit-hartenberg-frames-to-robotic-arms/
    a1 = 4.7
    a2 = 5.9
    a3 = 5.4
    a4 = 6.0
    t = np.array([[0, 0, 0], [a2, 0, a1]])

    # Position of end effector in joint 2 (i.e. the last joint) frame
    p_eff_2 = [a4, 0, a3]

    # Create an object of the RoboticArm class
    k_c = RoboticArm(k, t)

    # Starting joint angles in radians (joint 1, joint 2)
    q_0 = np.array([0, 0])

    # desired end position for the end effector with respect to the base frame of the robotic arm
    endeffector_goal_position = np.array([4.0, 10.0, a1 + a4])

    # Display the starting position of each joint in the global frame
    for i in np.arange(0, k_c.N_joints):
        print(f'joint {i} position = {k_c.position(q_0, index=i)}')

    print(f'end_effector = {k_c.position(q_0, index=-1, p_i=p_eff_2)}')
    print(f'goal = {endeffector_goal_position}')

    # Return joint angles that result in the end effector reaching endeffector_goal_position
    final_q = k_c.pseudo_inverse(q_0, p_eff_N=p_eff_2, goal_position=endeffector_goal_position, max_steps=500)

    # Final Joint Angles in degrees
    print('\n\nFinal Joint Angles in Degrees')
    print(f'Joint 1: {np.degrees(final_q[0])} , Joint 2: {np.degrees(final_q[1])}')


if __name__ == '__main__':
    main()
