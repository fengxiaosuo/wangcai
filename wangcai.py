#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

#小车电机引脚定义
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

#小车按键定义
key = 8

#超声波引脚定义
EchoPin = 0
TrigPin = 1

#RGB三色灯引脚定义
LED_R = 22
LED_G = 27
LED_B = 24 

#舵机引脚定义
ServoFrontPin = 23
ServoUpDownPin = 9
ServoLeftRightPin = 11

#红外避障引脚定义 
AvoidSensorLeft = 12
AvoidSensorRight = 17

#蜂鸣器引脚定义
buzzer = 8

#灭火电机引脚设置
OutfirePin = 2

#循迹红外引脚定义
#TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
#      3                 5                  4                   18
TrackSensorLeftPin1  =  3   #定义左边第一个循迹红外传感器引脚为3口
TrackSensorLeftPin2  =  5   #定义左边第二个循迹红外传感器引脚为5口
TrackSensorRightPin1 =  4   #定义右边第一个循迹红外传感器引脚为4口
TrackSensorRightPin2 =  18  #定义右边第二个循迹红外传感器引脚为18口

#光敏电阻引脚定义
LdrSensorLeft = 7
LdrSensorRight = 6


class Wangcai:
    name="wangcai"
    ServoFrontPos = 90
    ServoLeftRightPos = 90
    ServoUpDownPos = 90


    # construction function
    def __init__(self, name):
        if(name != ""):
            self.name=name;
        #设置GPIO口为BCM编码方式
        GPIO.setmode(GPIO.BCM)
        #忽略警告信息
        GPIO.setwarnings(False)
        GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(buzzer,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(OutfirePin,GPIO.OUT)
        GPIO.setup(EchoPin,GPIO.IN)
        GPIO.setup(TrigPin,GPIO.OUT)
        GPIO.setup(LED_R, GPIO.OUT)
        GPIO.setup(LED_G, GPIO.OUT)
        GPIO.setup(LED_B, GPIO.OUT)
        GPIO.setup(ServoFrontPin, GPIO.OUT)
        GPIO.setup(ServoUpDownPin, GPIO.OUT)
        GPIO.setup(ServoLeftRightPin, GPIO.OUT)
        GPIO.setup(AvoidSensorLeft,GPIO.IN)
        GPIO.setup(AvoidSensorRight,GPIO.IN)
        GPIO.setup(LdrSensorLeft,GPIO.IN)
        GPIO.setup(LdrSensorRight,GPIO.IN)
        GPIO.setup(TrackSensorLeftPin1,GPIO.IN)
        GPIO.setup(TrackSensorLeftPin2,GPIO.IN)
        GPIO.setup(TrackSensorRightPin1,GPIO.IN)
        GPIO.setup(TrackSensorRightPin2,GPIO.IN)
        #设置pwm引脚和频率为2000hz
        self.pwm_ENA = GPIO.PWM(ENA, 2000)
        self.pwm_ENB = GPIO.PWM(ENB, 2000)
        self.pwm_ENA.start(0)
        self.pwm_ENB.start(0)
        #设置舵机的频率和起始占空比
        self.pwm_FrontServo = GPIO.PWM(ServoFrontPin, 50)
        self.pwm_UpDownServo = GPIO.PWM(ServoUpDownPin, 50)
        self.pwm_LeftRightServo = GPIO.PWM(ServoLeftRightPin, 50)
        self.pwm_FrontServo.start(0)
        self.pwm_UpDownServo.start(0)
        self.pwm_LeftRightServo.start(0)
        #led
        self.pwm_rled = GPIO.PWM(LED_R, 1000)
        self.pwm_gled = GPIO.PWM(LED_G, 1000)
        self.pwm_bled = GPIO.PWM(LED_B, 1000)
        self.pwm_rled.start(0)
        self.pwm_gled.start(0)
        self.pwm_bled.start(0)

        # init servo to init pos
        self.motor_init()

    # deconstruction function
    def __del__(self):
        return

    #电机引脚初始化操作
    def motor_init(self):
        print("motor_init")
        self.frontservo_appointed_detection(self.ServoFrontPos)
        self.pwm_FrontServo.ChangeDutyCycle(0)   #归零信号
        self.updownservo_appointed_detection(self.ServoUpDownPos)
        self.pwm_UpDownServo.ChangeDutyCycle(0)  #归零信号
        self.leftrightservo_appointed_detection(self.ServoLeftRightPos)
        self.pwm_LeftRightServo.ChangeDutyCycle(0)   #归零信号
        time.sleep(0.2)

    #
    def motor_uninit(self):
        return

    #小车前进   
    def advance(self,delaytime=1):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(80)
        self.pwm_ENB.ChangeDutyCycle(80)
        time.sleep(delaytime)

    #小车后退
    def back(self,delaytime=1):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        self.pwm_ENA.ChangeDutyCycle(80)
        self.pwm_ENB.ChangeDutyCycle(80)
        time.sleep(delaytime)

    #小车左转   
    def left(self,delaytime=1):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(80)
        self.pwm_ENB.ChangeDutyCycle(80)
        time.sleep(delaytime)

    #小车右转
    def right(self,delaytime=1):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(80)
        self.pwm_ENB.ChangeDutyCycle(80)
        time.sleep(delaytime)

    #小车原地左转
    def spin_left(self,delaytime=1):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(80)
        self.pwm_ENB.ChangeDutyCycle(80)
        time.sleep(delaytime)

    #小车原地右转
    def spin_right(self,delaytime=1):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        self.pwm_ENA.ChangeDutyCycle(80)
        self.pwm_ENB.ChangeDutyCycle(80)
        time.sleep(delaytime)

    #小车停止   
    def brake(self,delaytime=1):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(80)
        self.pwm_ENB.ChangeDutyCycle(80)
        time.sleep(delaytime)
            
    #舵机来回转动
    def servo_control_color(self):
        for pos in range(181):
            self.pwm_FrontServo.ChangeDutyCycle(2.5 + 10 * pos/180)
        self.corlor_light(pos)
        time.sleep(0.009) 
        for pos in reversed(range(181)):
            self.pwm_FrontServo.ChangeDutyCycle(2.5 + 10 * pos/180)
        self.corlor_light(pos)
        time.sleep(0.009)

    #前舵机旋转到指定角度
    def frontservo_appointed_detection(self,pos): 
        for i in range(18):   
            self.pwm_FrontServo.ChangeDutyCycle(2.5 + 10 * pos/180)
            time.sleep(0.02)                            #等待20ms周期结束
            self.pwm_FrontServo.ChangeDutyCycle(0)  #归零信号
        self.ServoFrontPos = pos

    #摄像头舵机左右旋转到指定角度
    def leftrightservo_appointed_detection(self,pos): 
        for i in range(18):   
            self.pwm_LeftRightServo.ChangeDutyCycle(2.5 + 10 * pos/180)
            time.sleep(0.02)                            #等待20ms周期结束
            self.pwm_LeftRightServo.ChangeDutyCycle(0)  #归零信号
        self.ServoLeftRightPos = pos

    #摄像头舵机上下旋转到指定角度
    def updownservo_appointed_detection(self,pos):  
        for i in range(18):  
            self.pwm_UpDownServo.ChangeDutyCycle(2.5 + 10 * pos/180)
            time.sleep(0.02)                            #等待20ms周期结束
            self.pwm_UpDownServo.ChangeDutyCycle(0) #归零信号
        self.ServoUpDownPos = pos

    #七彩灯亮指定颜色
    def color_led_pwm(self,iRed,iGreen,iBlue):
        v_red = (100*iRed)/255
        v_green = (100*iGreen)/255
        v_blue = (100*iBlue)/255
        self.pwm_rled.ChangeDutyCycle(v_red)
        self.pwm_gled.ChangeDutyCycle(v_green)
        self.pwm_bled.ChangeDutyCycle(v_blue)
        time.sleep(0.02)

    #摄像头舵机向上运动
    def servo_up(self):
        pos = self.ServoUpDownPos
        print "camera up to %s" % pos
        self.updownservo_appointed_detection(pos)
        #time.sleep(0.05)
        pos +=0.7 
        self.ServoUpDownPos = pos
        if self.ServoUpDownPos >= 180:
            self.ServoUpDownPos = 180

    #摄像头舵机向下运动      
    def servo_down(self):
        pos = self.ServoUpDownPos
        print "camera down to %s" % pos
        self.updownservo_appointed_detection(pos)
        #time.sleep(0.05)
        pos = pos - 0.7
        self.ServoUpDownPos = pos
        if self.ServoUpDownPos <= 45:
            self.ServoUpDownPos = 45
        
    #小车鸣笛
    def whistle(self):
        GPIO.output(buzzer, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(buzzer, GPIO.HIGH)
        time.sleep(0.001)

    #超声波测距函数
    def distance(self):
        GPIO.output(TrigPin,GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(TrigPin,GPIO.LOW)
        while not GPIO.input(EchoPin):
            pass
            t1 = time.time()
        while GPIO.input(EchoPin):
            pass
            t2 = time.time()
        #print "distance is %d " % (((t2 - t1)* 340 / 2) * 100)
        time.sleep(0.01)
        return ((t2 - t1)* 340 / 2) * 100

    #巡线测试
    def tracking_sensor(self):
        #检测到黑线时循迹模块相应的指示灯亮，端口电平为LOW
        #未检测到黑线时循迹模块相应的指示灯灭，端口电平为HIGH
        TrackSensorLeftValue1  = GPIO.input(TrackSensorLeftPin1)
        TrackSensorLeftValue2  = GPIO.input(TrackSensorLeftPin2)
        TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
        TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)
        infrared_track_value_list = ['0','0','0','0']
        infrared_track_value_list[0] = str(1 ^TrackSensorLeftValue1)
        infrared_track_value_list[1] = str(1 ^TrackSensorLeftValue2)
        infrared_track_value_list[2] = str(1 ^TrackSensorRightValue1)
        infrared_track_value_list[3] = str(1 ^TrackSensorRightValue2)
        return ''.join(infrared_track_value_list)

    #避障红外引脚测试
    def infrared_avoid_sensor(self):
        #遇到障碍物,红外避障模块的指示灯亮,端口电平为LOW
        #未遇到障碍物,红外避障模块的指示灯灭,端口电平为HIGH
        LeftSensorValue  = GPIO.input(AvoidSensorLeft)
        RightSensorValue = GPIO.input(AvoidSensorRight)
        infrared_avoid_value_list = ['0','0']
        infrared_avoid_value_list[0] = str(1 ^LeftSensorValue)
        infrared_avoid_value_list[1] = str(1 ^RightSensorValue)
        return ''.join(infrared_avoid_value_list)
            
    #寻光引脚测试
    def lighting_sensor(self):
        #遇到光线,寻光模块的指示灯灭,端口电平为HIGH
        #未遇光线,寻光模块的指示灯亮,端口电平为LOW
        LdrSersorLeftValue  = GPIO.input(LdrSensorLeft)
        LdrSersorRightValue = GPIO.input(LdrSensorRight)  
        LDR_value_list = ['0','0']
        LDR_value_list[0] = str(LdrSersorLeftValue)
        LDR_value_list[1] = str(LdrSersorRightValue)    
        return ''.join(LDR_value_list)

# main function
if __name__ == "__main__":

    print('''
*******************************************************
*             wangcai - a speaking dog                *
*******************************************************
''')

    wc = Wangcai("")
    print("my name is %s" % wc.name)

    # test jobs
    if(0) :
        # front servo left, mid, right, mid (0-180)
        print "front servo left, mid, right, mid (0-180)"
        time.sleep(0.5)
        wc.frontservo_appointed_detection(180)
        print "left distance is " % wc.distance()
        time.sleep(0.5)
        wc.frontservo_appointed_detection(90)
        print "front distance is " % wc.distance()
        time.sleep(0.5)
        wc.frontservo_appointed_detection(0)
        print "right distance is " % wc.distance()
        time.sleep(0.5)
        wc.frontservo_appointed_detection(90)
        print "front distance is " % wc.distance()
        time.sleep(0.5)

        # front led color: red, green, blue, mute
        print "front led color: red, green, blue, mute"
        time.sleep(1)
        wc.color_led_pwm(255,0,0)
        time.sleep(1)
        wc.color_led_pwm(0,255,0)
        time.sleep(1)
        wc.color_led_pwm(0,0,255)
        time.sleep(1)
        wc.color_led_pwm(0,0,0)

        # camera servo left, mid, right, mid (between 0 and 180)
        print "camera servo left, mid, right, mid (between 0 and 180)"
        time.sleep(1)
        wc.leftrightservo_appointed_detection(180)
        time.sleep(1)
        wc.leftrightservo_appointed_detection(90)
        time.sleep(1)
        wc.leftrightservo_appointed_detection(0)
        time.sleep(1)
        wc.leftrightservo_appointed_detection(90)
        time.sleep(1)

        # camera servo up, mid, down, mid (between 70 and 180)
        print "camera servo up, mid, down, mid (between 70 and 180)"
        time.sleep(1)
        wc.updownservo_appointed_detection(130)
        time.sleep(1)
        wc.updownservo_appointed_detection(90)
        time.sleep(1)
        wc.updownservo_appointed_detection(180)
        time.sleep(1)
        wc.updownservo_appointed_detection(90)
        time.sleep(1)
        wc.updownservo_appointed_detection(70)
        time.sleep(1)
        wc.updownservo_appointed_detection(90)
        time.sleep(1)

        # car advance, back, left, right, brake
        print "car advance, back, left, right, brake"
        wc.advance()
        time.sleep(1)
        wc.back()
        time.sleep(1)
        wc.left()
        time.sleep(1)
        wc.right()
        time.sleep(1)
        wc.spin_left()
        time.sleep(1)
        wc.spin_right()
        time.sleep(1)
        wc.brake()

        # car whilstle
        time.sleep(1)
        wc.whistle()

        # ultrasonic distance
        i = 0
        while i < 100 :
            print "distance is %s" % wc.distance()
            time.sleep(0.1)
            i += 1

        # 4 bit of tracking sensor
        print "tracking %s" % wc.tracking_sensor()
        # 2 bit
        print "infrared %s" % wc.infrared_avoid_sensor()
        # 2 bit
        print "lighting %s" % wc.lighting_sensor()

        # front fan control
        GPIO.output(OutfirePin,GPIO.HIGH)
        time.sleep(1)
        GPIO.output(OutfirePin,GPIO.LOW)

    else :
        wc.color_led_pwm(255,0,0)
        time.sleep(1)



