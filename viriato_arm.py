from pyrep.robots.arms.arm import Arm


class ViriatoArm(Arm):

    """
    Class that represents the Viriato arm.
    """

    def __init__(self, count: int = 0):
        super().__init__(count, 'gen3', 7, base_name='gen3')
 