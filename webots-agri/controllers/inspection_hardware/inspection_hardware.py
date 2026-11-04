"""Hardware checkout only: propellers OFF. F=front, D=down (select drone first).
No flight/autoland, vendor SDK, RoboComp connection, or image streaming is implemented.
"""
import math
from controller import Robot, Keyboard

robot = Robot()
step = int(robot.getBasicTimeStep())
for name in ('front left propeller', 'front right propeller',
             'rear left propeller', 'rear right propeller'):
    motor = robot.getDevice(name)
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)
servo = robot.getDevice('camera pitch')
servo.setVelocity(1.0)
servo.setPosition(math.pi / 2)
encoder = robot.getDevice('camera pitch sensor')
encoder.enable(step)
camera = robot.getDevice('camera')
camera.enable(5 * step)
for name in ('gps', 'inertial unit', 'gyro', 'compass'):
    robot.getDevice(name).enable(5 * step)
keyboard = robot.getKeyboard()
keyboard.enable(step)
print('[inspection] PARKED, propellers OFF. Camera: F=front, D=down. No autonomous flight.', flush=True)
while robot.step(step) != -1:
    key = keyboard.getKey()
    while key != -1:
        if key in (ord('F'), ord('D')):
            target = 0.0 if key == ord('F') else math.pi / 2
            servo.setPosition(target)
            print(f'[inspection] camera target {target:.6f} rad; measured {encoder.getValue():.6f}', flush=True)
        key = keyboard.getKey()
