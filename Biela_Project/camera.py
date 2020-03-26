from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.objects.vision_sensor import VisionSensor
import cv2
file = 'biela.ttt'

SCENE_FILE = join(dirname(abspath(file)), 'biela.ttt')
pr = PyRep()

pr.launch(SCENE_FILE, headless=False)
pr.start()
## Código
#robot = biela()
camera = VisionSensor("Vision_sensor")

while True:
    rgb_obs = camera.capture_rgb()
    cv2.imshow("Vision_sensor", camera.capture_rgb())
    #cv2.imshow("camara", rgb_obs)


###
pr.stop()
pr.shutdown()
