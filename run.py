#---------------------------------------------------------------
# HEADER STARTS HERE:
#---------------------------------------------------------------
import mpu9250
import time
import sys
import RPi.GPIO as GPIO
import math
import quantizer as quant
import numpy as np

GPIO.setmode(GPIO.BOARD)

# Sensors class initialization
sens = mpu9250.MPU9250()

# A few seconds for initializing the system, about 10 seconds
time.sleep(3)

# Getting gryo biases:
def gyrobias():
    xsum = ysum = zsum = 0
    for i in range(0,1000,1):
        gyro = sens.readGyro()
        xsum = xsum + gyro['x']
        ysum = ysum + gyro['y']
        zsum = zsum + gyro['z']
        time.sleep(0.01)
    gxb = xsum/float(1000)
    gyb = ysum/float(1000)
    gzb = zsum/float(1000)
    gbias = (gxb,gyb,gzb)
    return gbias
gyrob = gyrobias() # gyrob ~ (-0.1018,0.0349,0.4898)
print("Gyro Bias: {:4.2f}, {:4.2f}, {:4.2f}".format(gyrob[0],gyrob[1],gyrob[2]))

# Methods to read measurements from magnetometer, accelerometer and gyroscope:
def getmag(): # with biases
    mag = sens.readMagnet()
    magdata = ( (mag['x']+0*11.68)/1+0*1.0285 , (mag['y']-0*23.61)/1+0*0.9083, (mag['z']+0*47.32)/1+0*1.0791)
    return magdata

def getgyro():
    gyro = sens.readGyro()
    gyrodata = (gyro['x']-gyrob[0],gyro['y']-gyrob[1],gyro['z']-gyrob[2])
    return gyrodata

def getacc():
    acc = sens.readAccel()
    accdata = (acc['x'],acc['y'],acc['z'])
    return accdata


# Complementary Filter:
def getAngleGyro(r,p,y,fgx,fgy,fgz,dt):
    new_r=r+fgx*dt
    new_p=p+fgy*dt
    new_y=y+fgz*dt
    new_r = (new_r + math.pi) % (2 * math.pi) - math.pi
    new_p = (new_p + math.pi) % (2 * math.pi) - math.pi
    new_y = (new_y + math.pi) % (2 * math.pi) - math.pi
    return new_r,new_p,new_y
# Calculating roll and pitch angles from accelerometer:
def getAngleAcc(ax, ay, az):
    phi     = math.atan2(ay,az)
    theta   = math.atan2(ax,math.sqrt(az**2+ay**2))
    return phi,theta
# Calculating roll, pitch and yaw angles using complimentary filter:
def getAngleCompl(r, p, y, ax, ay, az, gx, gy, gz, dt):
    tau=0.00005
    # tau is the time constant in sec
    # for time periods <tau the  gyro takes precedence
    # for time periods > tau the acc takes precedence
    new_r,new_p=getAngleAcc(ax, ay, az)
    a=tau/(tau+dt)
    new_r=a*(new_r+gx*dt)+(1-a)*r
    new_p=a*(new_p+gy*dt)+(1-a)*p
    # note the yaw angle can be calculated only using the
    # gyro that's why a=1
    a=1
    new_y=a*(y+gz*dt)+(1-a)*y
    return new_r,new_p,new_y

#-----------------------------#
# Microcontroller Definitions #
#-----------------------------#
# Motor I/O pins:
STNBY = 37
# Motor A definition
AIN1 = 35
AIN2 = 33
PWMA = 31
# Motor B definition
BIN1 = 36
BIN2 = 38
PWMB = 40
# Setting up pins:
GPIO.setup(STNBY,GPIO.OUT)
GPIO.setup(AIN1,GPIO.OUT)
GPIO.setup(AIN2,GPIO.OUT)
GPIO.setup(PWMA,GPIO.OUT)
GPIO.setup(BIN1,GPIO.OUT)
GPIO.setup(BIN2,GPIO.OUT)
GPIO.setup(PWMB,GPIO.OUT)
# PWM signals for motors:
mypwmA = GPIO.PWM(PWMA,100)
mypwmB = GPIO.PWM(PWMB,100)
# Starting PWM signals:
mypwmA.start(0)
mypwmB.start(0)
# Method to run both motors together:
def RunMotor(x):
        # Motor A
        GPIO.output(STNBY,1)
        GPIO.output(AIN1,0)
        GPIO.output(AIN2,1)
        mypwmA.ChangeDutyCycle(float(50-x))
        # Motor B
        GPIO.output(BIN1,0)
        GPIO.output(BIN2,1)
        mypwmB.ChangeDutyCycle(float(50+x))

#---------------------------------------------------------------
# Controller Design:
#---------------------------------------------------------------
# Diagonal system params:
T       = 5000 # Number of samples
# x is the actual states: [pitch angle,pitch rate]
# xx is the actual states after transformation [xx1, xx2]
# xxhat is the estimate of states after transformation [xxhat1,xxhat2]
x       = np.zeros((T,2))
xx      = np.zeros((T,2))
xxhat   = np.zeros((T,2))
# Control input vector: is the same for all systems!
uu      = np.zeros((1,T))
# Error vector for xx-xxhat
zzz     = np.zeros((1,T))
# initial conditions:
xxhat1  = 0
xxhat2  = 0
# Controller gains:
kk1     = 225.4386
kk2     = 11.1514

AAc     = 7.3198 # Positive eigenvalue of the system
# State-space matrices after transformation and discretization:
AA11    = 1.0222
AA12    = 0
AA21    = 0
AA22    = 0.9783
BB1     = 0.0007653
BB2     = 0.0007487

# Controller Settings and initialization (some variables are defined in the paper):
phi             = 0
theta           = 0
psi             = 0
dt              = 0.0032
angle_offset    = 0.085
m_min           = 0
m_max           = 35
# packet size:
# M comes from experiment from sensors (angle,p) both in rad
M       = np.matrix([[0.01],[0.02]])
p       = np.matrix([[0.1354,-0.1354],[0.9908,0.9908]])
MM      = np.linalg.inv(p)*M

gamma   = 0.015
ro      = 0.01
bg      = 1.00001
J       = (math.exp(AAc*gamma)-1)*MM[0,0] / (AAc*ro) + 0.09
if gamma==0:
    L = 0
else:
    L       = 1 + math.log2((AAc * bg * gamma) / math.log(1 + (ro - MM[0,0] * (math.exp(AAc * gamma) - 1) / (J * AAc)) / math.exp(AAc * gamma)))
g       = math.ceil(max(0.1,L))
# Some variables to store experimental results:
q       = -1*np.ones((T,g))
zbar    = np.zeros((1,T))
trigs   = np.zeros((1,T))
ac_tr   = np.zeros((1,T))

# Iterator reset:
i = 0
t = 0
# Running the control loop:
while i<T-(333*gamma)-1:
    # Sensor reading:
    t1              = time.time()
    gyro            = getgyro()
    acc             = getacc()
    phi,theta,psi   = getAngleCompl(phi,theta,psi,acc[0],acc[1],acc[2],math.radians(gyro[0]),math.radians(gyro[1]),math.radians(gyro[2]),dt)
    angle           = phi-angle_offset
    p               = math.radians(gyro[0])
    # Control input saturation check:
    if abs(angle)<25:
        u = -float(kk1*xxhat[i,0]+kk2*xxhat[i,1])
        # PWM (0-100)
        if u>m_max:
            u = m_max
        elif u<m_min and u>0:
            u = m_min
        elif u>-m_min and u<0:
            u = -m_min
        elif u<-m_max:
            u = -m_max
    else:
        u = 0
    # Upadating/Storing control input variable and iterating:
    uu[0,i]     = u
    i           = i+1
    # Apply control input to actual system:
    RunMotor(u)
    # measurements from actual system[x1,x2]:
    x1          = angle
    x2          = p
    x[i,:]      = [x1,x2]
    # measurement of the unstable state of the diagonal system: xx1
    xx1         = 3.69390*x1 + 0.50464*x2
    xx2         = 0.50464*x2 - 3.69390*x1
    xx[i,:]     = [xx1,xx2]
    # Diagonal system estimate:
    xxhat1      = AA11*xxhat1 + BB1*u
    xxhat2      = AA22*xxhat2 + BB2*u
    # Update initial condition diagonal system:
    # Event triggered:
    zz          = xx1-xxhat1
    zzz[:,i]    = zz
    # Event-triggering with quantization:
    if abs(zzz[:,i])>=J:
        trigs[:,i] = 1
        if trigs[:,i-1]==0:
            # Actual triggerings: 
            ac_tr[:,i-1] = 1
            q[i,:] = quant.encoder([zzz[:,i],i/333,gamma,bg,AAc,ro,MM[0,0],J])
            zdelay = math.floor(333*gamma*np.random.rand())
            zbar[:,i+zdelay] = quant.decoder([q[i,:],(i+zdelay)*0.003,gamma,bg,AAc,J])

    xxhat1 = xxhat1 + zbar[:,i]
    xxhat2    = xxhat2
    # updating estimation:
    xxhat[i,:]    = [xxhat1,xxhat2]
    # printing and next iteration: 
    print("Pendulum Angle: {:7.3f}   ,   Input: {:7.1f}".format(angle,u))
    t = t + (time.time()-t1)
    # Break the loop if absolute value of the pendulum angle is greater than 0.6 radians
    if abs(angle)>0.6:
        break

# Calculating and storing total number of triggerings and transmission rate:
numtrig = np.nonzero(ac_tr)
rate    = len(numtrig[0])*g/t
print("Rate: ", rate)
# Calculating necessary transmission rate according to the paper:
if gamma==0:
    nec_g=1
else:
    nec_g = math.ceil(max(0.1,math.log( (MM[0,0]/(AAc*J)+1)*(math.exp(AAc*gamma)-1) ) ))

# saving all data to csv files:
np.savetxt('x.csv', x, delimiter=',', fmt='%10.5f')
np.savetxt('xx.csv', xx, delimiter=',', fmt='%10.5f')
np.savetxt('xxhat.csv', xxhat, delimiter=',', fmt='%10.5f')
np.savetxt('uu.csv', uu, delimiter=',', fmt='%10.5f')
np.savetxt('zzz.csv', zzz, delimiter=',', fmt='%10.5f')
np.savetxt('trigs.csv', trigs, delimiter=',', fmt='%10.5f')
np.savetxt('actr.csv', ac_tr, delimiter=',', fmt='%10.5f')
np.savetxt('params.csv', [gamma,ro,bg,J,g,nec_g,rate,t], delimiter=',', fmt='%10.5f')
print("J: ", J)
print("g: ", g)
print("T: ", t)

# Cleaning up and Turning off GPIOs
GPIO.output(STNBY,0)
mypwmA.stop()
mypwmB.stop()
GPIO.cleanup()
















