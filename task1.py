import servos
import sensors
import time
robot = servos.robot
t1 = time.monotonic()
robot.setSpeedsPWM(1.6, 1.4)
f = open("distances.txt", "w")
while(True):
    t2 = time.monotonic()
    if (t2 - t1 > 0.1):
        # Get a measurement rom each sensor
        lDistance = sensors.lSensor.get_distance()
        fDistance = sensors.fSensor.get_distance()
        rDistance = sensors.rSensor.get_distance()
    
        # Print each measurement
        f.write(str(lDistance))
        f.write(",")
        f.write(str(fDistance))
        f.write(",")
        f.write(str(rDistance))
        f.write("\n")
        t1 = time.monotonic()
        if (fDistance <= 305):
            robot.setSpeedsPWM(1.5, 1.5)
            break

# Stop measurement for all sensors
sensors.lSensor.stop_ranging()
sensors.fSensor.stop_ranging()
sensors.rSensor.stop_ranging()
sensors.GPIO.cleanup()
f.close()
exit()