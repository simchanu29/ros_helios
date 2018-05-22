import maestro
import time

my_maestro = maestro.Controller("/dev/ttyACM1")

my_maestro.setTarget(target=5000, channel=0)
time.sleep(1)
my_maestro.setTarget(target=7000, channel=0)