"""
Just an example to know how Viriato works using the PyRep abstraction.

Objetives:
[ ] Viriato responds to any actions
[ ] Viriato can replace Panda/PyRep's implemented arm
[ ] Viriato can be used with the other AR's group
"""
from os.path import dirname, join, abspath
import sys
import time

from pyrep import PyRep
from pyrep.objects.dummy import Dummy
from viriato_arm import ViriatoArm

if len(sys.argv) < 2:
    print("[ERROR] NEEDS A SCENE PATH")
    exit(1)

SCENE_FILE = join(dirname(abspath(__file__)), sys.argv[1])
pr = PyRep()
pr.launch(SCENE_FILE, headless=False)
pr.start()

viriato_arm = ViriatoArm()
waypoint = Dummy('waypoint0')

viriato_arm.set_joint_target_velocities([1,1,1,1,1,1,1])
#path = viriato_arm.get_path(position=waypoint.get_position(),
#                            quaternion=waypoint.get_quaternion())
# path.visualize()  # Let's see what the path looks like
# print('Executing plan ...')
# done = False
# while not done:
#     done = path.step()
#     pr.step()
# path.clear_visualization()
while True:
    pr.step()
pr.stop()
pr.shutdown()
