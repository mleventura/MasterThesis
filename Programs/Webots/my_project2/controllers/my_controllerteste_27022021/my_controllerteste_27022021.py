from controller  import Robot, DistanceSensor, Motor, Camera #, GPS
from math import cos, sin


# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

#variáveis iniciais do epuck
r = 20.5 #raio da roda do epuck
l = 53 #distância entre uma roda e outra do epuck
teta = 0 #angulo de direção do epuck sobre o plano
x = 0
y = 0 


# create the Robot instance.
robot = Robot()

camera = robot.getDevice('camera')
camera.enable(TIME_STEP)
accelerometer = robot.getDevice('accelerometer')
accelerometer.enable(TIME_STEP)
gyro = robot.getDevice('gyro')
gyro.enable(TIME_STEP)
#gps = robot.getDevice('GPS')
#gps.enable(TIME_STEP)

# robot.batterySensorEnable(TIME_STEP)


# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # detect obstacles
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0

    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.85 * MAX_SPEED
    rightSpeed = 0.85 * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  = 0.85 * MAX_SPEED
        rightSpeed = -0.85 * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed  = -0.85 * MAX_SPEED
        rightSpeed = 0.85 * MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    teta +=  1/l*(rightSpeed-leftSpeed)*r*TIME_STEP
    x +=1/2*(rightSpeed+leftSpeed)*r*cos(teta)*TIME_STEP
    y +=1/2*(rightSpeed+leftSpeed)*r*sin(teta)*TIME_STEP
    
    aceleracao = accelerometer.getValues()
    giroscopio = gyro.getValues()
   # posicao = gps.getValues()
   # batteria = robot.batterySensorGetValue()
    
    print('Distancia: ', psValues)
    print('Aceleração: ', aceleracao)
    print('Rotação: ', giroscopio)
   # print('Posição: ', posicao)
    print('Posição direção X: ', x)
    print('Posição direção Y: ', y)
    print('Ângulo teta: ', teta)
    #print(robot.transform)
    
    
  #  print('Translacao: ', robot.translation)