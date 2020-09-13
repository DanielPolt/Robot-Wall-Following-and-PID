import servos
import sensors
import time
import signal

def ctrlC(signum, frame):
    print("Exiting")
    
    # Stop the servos and sensors
    robot.setSpeedsPWM(1.5, 1.5)
    sensors.lSensor.stop_ranging()
    sensors.fSensor.stop_ranging()
    sensors.rSensor.stop_ranging()
    sensors.GPIO.cleanup()
    f.close()
    exit()

signal.signal(signal.SIGINT, ctrlC)

robot = servos.robot
k = 1.5; #0.1, 0.5, 1.0, 1.5, 2.0, or 5.0
maxS = 3
minS = -3
desired = 305
desiredSide = 178
distance = 0
error = 0
u = 0
satSpeed = 0
t1 = time.monotonic()
f = open("distancesTaskTwo.txt", "w")

while(True):
    t2 = time.monotonic()
    if (t2 - t1 > 0.1):
        # Get a measurement rom each sensor
        lDistance = sensors.lSensor.get_distance()
        fDistance = sensors.fSensor.get_distance()
        rDistance = sensors.rSensor.get_distance()
        
        #print to file
        f.write(str(fDistance))
        f.write("\n")
        
        t1 = time.monotonic()
        
        distance = fDistance
        error = (-1 * desired) + distance
        u = k * error
    
        if(u > (maxS / 0.0393701)):
            satSpeed = maxS
        elif(u < (minS / 0.0393701)):
            satSpeed = minS
        else:
            satSpeed = u * 0.0393701
        
        if (abs(satSpeed) < robot.rightminips):
            robot.setSpeedsPWM(1.5, 1.5)
        else:
            if (abs(rDistance - desiredSide) < 15):
                robot.setSpeedsIPS(satSpeed, satSpeed)
            else:
                if (rDistance - desiredSide < 0):
                    robot.setSpeedsVW(satSpeed, -3.14/12)
                else:
                    robot.setSpeedsVW(satSpeed, 3.14/12)