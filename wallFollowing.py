import servos
import sensors
import time
import signal

def ctrlC(signum, frame):
    print('\n')
    print("Distance Travelled in Inches:" + str((distancetravelled / 32) * servos.CIRCUMFERENCE))
    print("Left Turns: " + str(leftturns))
    print("Right Turns: " + str(rightturns))
    print("U-Turns: " + str(uturns))
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
k = 2; #0.1, 0.5, 1.0, 1.5, 2.0, or 5.0
maxS = 5
minS = -5
desired = 203
desiredSide = 203
turnDistance = 356
distance = 0
error = 0
u = 0
satSpeed = 3
t1 = time.monotonic()
rightturns = 0
leftturns = 0
uturns = 0
distancetravelled = 0
f = open("distancesTaskThree.txt", "w")

rotated = 0;
rotationlength90 = (3.1415 / 2) * servos.DMID
rotationlength180 = (3.1415) * servos.DMID
rotations90 = rotationlength90 / servos.CIRCUMFERENCE
rotations180 = rotationlength180 / servos.CIRCUMFERENCE

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
        if(fDistance - desired > 10) :
            distancetravelled += robot.getCounts()[0]
            print("Moving Straight")
            distance = fDistance
            error = (-1 * desired) + distance
            u = k * error
    
            if(u > (maxS / 0.0393701)):
                satSpeed = maxS
            elif(u < (minS / 0.0393701)):
                satSpeed = minS
            else:
                satSpeed = u * 0.0393701
            if (rDistance - desiredSide < 0):
                robot.setSpeedsVW(satSpeed, (k)*(-3.14/12))
            elif(lDistance - desiredSide < 0):
                robot.setSpeedsVW(satSpeed, (k)*(3.14/12))
            else:
                robot.setSpeedsIPS(satSpeed, satSpeed)
        else:
            robot.setSpeedsPWM(1.5, 1.5)
            if(rDistance > turnDistance) :
                print("Turning Right")
                robot.setSpeedsVW(0, 1)
                while rotated < rotations90:
                    rotated = robot.getCounts()[0] / 32
                robot.setSpeedsPWM(1.5, 1.5)
                rotated = 0
                rightturns += 1
            elif(lDistance > turnDistance) :
                print("Turning Left")
                robot.setSpeedsVW(0, -1)
                while rotated < rotations90:
                    rotated = robot.getCounts()[0] / 32
                robot.setSpeedsPWM(1.5, 1.5)
                rotated = 0
                leftturns += 1
            else :
                print("U-Turn")
                robot.setSpeedsVW(0, 1)
                while rotated < rotations180:
                    rotated = robot.getCounts()[0] / 32
                robot.setSpeedsPWM(1.5, 1.5)
                rotated = 0
                uturns += 1
        robot.resetCounts()
        t1 = time.monotonic()