"""
A Harvey moves using delta end effector pose control.
This script contains examples of:
    - IK calculations.
    - Joint movement by setting joint target positions.
"""
from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.arms.harvey import Harvey

SCENE_FILE = join(dirname(abspath(__file__)), '../../vrep_scenes/PyRep_harvey.ttt')
DELTA = 0.01
pr = PyRep()
pr.launch(SCENE_FILE, headless=False)
pr.start()
agent = Harvey()

starting_joint_positions = agent.get_joint_positions()
pos, quat = agent.get_tip().get_position(), agent.get_tip().get_quaternion()


def move(index, delta):
    pos[index] += delta
    print (pos[index])
    new_joint_angles = agent.solve_ik(pos, quaternion=quat)
    print (new_joint_angles)
    agent.set_joint_target_positions(new_joint_angles)
    pr.step()
    new_pos, new_quat = agent.get_tip().get_position(), agent.get_tip().get_quaternion()
    print (new_pos)

[move(0, -DELTA) for _ in range(5)]
[move(0, DELTA) for _ in range(5)]
# [move(2, DELTA) for _ in range(5)]
# [move(1, DELTA) for _ in range(5)]

pr.stop()
pr.shutdown()
