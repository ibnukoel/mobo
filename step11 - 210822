from controller import Robot, Camera, InertialUnit, DistanceSensor, PositionSensor, \
    DistanceSensor, Motor, Gyro, Accelerometer, Display, GPS, Keyboard
import matplotlib.pyplot as plt
import math, numpy, random, cv2
import sys, csv
import pathPlaning
from mapping import map,valnya, grid,grid_scale,delta,movementRobot,printmap
from imager import get_data_from_camera

#initialization
robot = Robot()
timestep = int(robot.getBasicTimeStep())
kb = Keyboard()
kb.enable(timestep)

# activation camera
camera = robot.getDevice('camera')
camera.enable(timestep)
#camera.recognitionEnable(timestep)
width = int(camera.getWidth())
height = int(camera.getHeight())

# getting the position sensors
roki = robot.getDevice('left wheel sensor')
roka = robot.getDevice('right wheel sensor')
roki.enable(timestep)
roka.enable(timestep)

# enable imu
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

#enableGPS
gps = robot.getDevice('gps')
gps.enable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# values for robot in inch
# wheel_radius = 1.6 / 2.0
# wheel_circ = 2 * 3.14 * wheel_radius
# enc_unit = wheel_circ / 6.28

# variable
init = [0, 0]
goal = [3, 3]
RotEnc = [0, 0]
motor_posisi_awal = [0, 0]
#spekRobot
rRoda = 2.05
enc_unit = (2 * 3.18 * rRoda) / 6.28
max_speed = 4
d = 2.28
d_mid = d / 2.0
#Posisi Awal
x_init = 0
y_init = 0
#posisi sekarang
x = 0
y = 0
i = 0 #posisi awal di movementList
pose = [0,0,0]
#posisi_sebelumnya
x_mobo_prev = 0
y_mobo_prev = 0
#exportDataVariable
xname = 1
extension = str('.jpg')


def get_direction():
    # maju = 1, mundur=2, kanan=3, kiri=4
    # jelaskan magsud dari bobot :
    imu_val = (imu.getRollPitchYaw()[2] * 180) / 3.14159
    if (imu_val <= -135 and imu_val >= -180) or (135 <= imu_val <= 180):
        dir = "North"
        bobot = [[25, 75, 100, 75], [1, 4, 3, 2]]
    elif imu_val <= -45 and imu_val > -135:
        dir = "West"
        bobot = [[75, 25, 75, 100], [3, 1, 4, 2]]
    elif 45 <= imu_val <= 135:
        dir = "East"
        bobot = [[75, 100, 75, 25], [4, 2, 3, 1]]
    elif (-45 < imu_val <= 0) or (0 <= imu_val < 45):
        dir = "South"
        bobot = [[100, 75, 25, 75], [2, 3, 1, 4]]
    return imu_val, bobot, dir
def cm(meters):
    return round(meters * 100)
def get_sen_jarak():
    # returns left, front, right sensors in inches
    return cm(depan.getValue()), cm(kanan.getValue()), cm(kiri.getValue())
def get_motor_pos():
    return roki.getValue(), roka.getValue()
def robot_movement(RotEnc, RotEnc_new, enc_unit, pose):
    motor_shift = [0,0]
    jarak = [0,0]
    for i in range(len(RotEnc_new)):  # menghitung jarak pergeseran robot
        motor_shift[i] = RotEnc_new[i] - RotEnc[i]  # menghitung pergeseran rotaty
        jarak[i] = motor_shift[i] * enc_unit  # menghitung jarak pergeseran robot
    v = (jarak[0] + jarak[1]) / 2
    w = (jarak[0] - jarak[1]) / 52
    pose[0] += v * math.cos(pose[2])
    pose[1] += v * math.sin(pose[2])
    pose[2] += w
    return pose

def getMidPoint(camImage):
    #convert to grayscale, gaussian blur filter, and threshold
    camImage = cv2.GaussianBlur(camImage,(9,9),cv2.BORDER_DEFAULT)
    #cv2.imwrite("../thresh.jpg",camImage)
    camImage = cv2.threshold(camImage,100,255,cv2.THRESH_BINARY_INV)      
    #erode for noise elimination,dilate to restore some eroded parts of image

def printdata(saveImage,ImageProc,saveRealPos,saveCSV):
    filename = str(str(xname)+extension)
    gps_pos=[]
    values=[]
    gps_pos.append(filename)
    values.append(filename)
    if saveImage == 1:
        camera.saveImage(filename,10)
    if saveRealPos == 1:
        val_gps = gps.getValues()
        gps_pos.append(val_gps[0])
        gps_pos.append(val_gps[1])
        gps_pos.append(val_gps[2])  
    if ImageProc == 1:
        print(get_data_from_camera(robot,camera))
        values.append(get_data_from_camera(robot,camera))
        print(values)
    if saveCSV == 1:
        file = open('garis256.csv','a')
        writer = csv.writer(file)
        writer.writerow(values)
        file.close()
        file = open('posisi.csv','a')
        writer = csv.writer(file) 
        writer.writerow(gps_pos)
        file.close()

while robot.step(timestep) != -1:
    #print('posisi = ', x, y)
    #print('goal   = ', goal)
    # cek posisi robot terhadap hasil path planing
   
    if kb.getKey() == 315:
        printdata(1,1,1,1)
        print("get data")
        xname=xname+1
        print(xname)
    if kb.getKey() == 317:
        print("automate")
        if x == goal[0] and y == goal[1]:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
        else:
            arah = get_direction()
            arahRobot = arah[0]
            RotEnc_new = get_motor_pos()
            pose = robot_movement(RotEnc, RotEnc_new, enc_unit, pose)
            RotEnc = RotEnc_new
            printdata(1,0,0,0)
            #jika arah sesuai MAJU
            if arahRobot >= movementRobot[i][1] - 0.5 and arahRobot <= movementRobot[i][1] + 0.5:    
                leftMotor.setVelocity(3)
                rightMotor.setVelocity(3)
                # kalkulasi pergerakan robot
                RotEnc_new = get_motor_pos()
                pose = robot_movement(RotEnc, RotEnc_new, enc_unit, pose)
                RotEnc = RotEnc_new
                #update posisi mobo
                x_mobo = math.floor(abs(pose[0]) / grid_scale) - x_mobo_prev
                y_mobo = math.floor(abs(pose[1]) / grid_scale) - y_mobo_prev
    
                #jika robot masih di grid yang sama
                if (x_mobo != 0):
                    print('xmobo ', x_mobo, y_mobo)
                    x_mobo_prev = x_mobo + x_mobo_prev
                    y_mobo_prev = y_mobo + y_mobo_prev
                    x += (x_mobo * movementRobot[i][4]) + (y_mobo * movementRobot[i][4])
                    y += (x_mobo * movementRobot[i][5]) + (y_mobo * movementRobot[i][5])
    
                    print('x dan y = ', x, y)
                    print('xy_mobo = ', x_mobo, y_mobo)
                    print('xy_init = ', x_init, y_init)
                    print('pose = ', pose)
                    print('movement = ', movementRobot[i][2], movementRobot[i][3])
                    print('====================')
    
                #jika robot berpindah grid
                if x_init != x or y_init != y:
                    map[x_init][y_init] = ' '
                    map[x][y] = '#'
                    print('updatemap')
                    x_init = x
                    y_init = y
                    i += 1
                    print('pergerakan ke i= ', i)
                    print(pose)
                    printmap(map)
                    #reset pose untuk grid baru
                    pose = [0, 0, 0]
                    x_mobo_prev = 0
                    y_mobo_prev = 0
    
            #arah tidak sesuai PUTAR sampai arah sesuai
            else:
                possible_left  = 180 + movementRobot[i][1] - arahRobot
                possible_right = 180 - movementRobot[i][1] + arahRobot
                if (possible_right > possible_left): #belokKiri
                    leftMotor.setVelocity(0.5)
                    rightMotor.setVelocity(-0.5)
                if(possible_right < possible_left): #belokKanan
                    leftMotor.setVelocity(-0.5)
                    rightMotor.setVelocity(0.5)
                pose = [0, 0, 0]
                x_mobo_prev = 0
        xname=xname+1
        print(xname)
    
