#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

#RGB三色灯引脚定义
LED_R = 22
LED_G = 27
LED_B = 24

#小车电机引脚定义
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

class Wangcai:
    name="wangcai"

    def __init__(self, name):
        self.name=name;
        #设置GPIO口为BCM编码方式
        GPIO.setmode(GPIO.BCM)
        #忽略警告信息
        GPIO.setwarnings(False)

    def color(self, r=0, g=0, b=0, duration=1):
        #RGB三色灯设置为输出模式
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LED_R, GPIO.OUT)
        GPIO.setup(LED_G, GPIO.OUT)
        GPIO.setup(LED_B, GPIO.OUT)

        while True:
            if(r):
                GPIO.output(LED_R, GPIO.HIGH)
            if(g):
                GPIO.output(LED_G, GPIO.HIGH)
            if(b):
                GPIO.output(LED_B, GPIO.HIGH)
            time.sleep(duration)
            GPIO.output(LED_R, GPIO.LOW)
            GPIO.output(LED_G, GPIO.LOW)
            GPIO.output(LED_B, GPIO.LOW)
            break

        GPIO.cleanup()

    def colorLed(self, loop=1):
        count = 0
        #RGB三色灯设置为输出模式
        GPIO.setup(LED_R, GPIO.OUT)
        GPIO.setup(LED_G, GPIO.OUT)
        GPIO.setup(LED_B, GPIO.OUT)

        #循环显示7种不同的颜色
        try:
            while (count<loop):
                GPIO.output(LED_R, GPIO.HIGH)
                GPIO.output(LED_G, GPIO.LOW)
                GPIO.output(LED_B, GPIO.LOW)
                time.sleep(1)
                GPIO.output(LED_R, GPIO.LOW)
                GPIO.output(LED_G, GPIO.HIGH)
                GPIO.output(LED_B, GPIO.LOW)
                time.sleep(1)
                GPIO.output(LED_R, GPIO.LOW)
                GPIO.output(LED_G, GPIO.LOW)
                GPIO.output(LED_B, GPIO.HIGH)
                time.sleep(1)
                GPIO.output(LED_R, GPIO.LOW)
                GPIO.output(LED_G, GPIO.LOW)
                GPIO.output(LED_B, GPIO.LOW)
                time.sleep(1)
                count = count+1
        except:
            print "except"
        #使用try except语句，当CTRL+C结束进程时会触发异常后
        #会执行gpio.cleanup()语句清除GPIO管脚的状态
        GPIO.cleanup()

    #电机引脚初始化操作
    def motor_init(self):
        global pwm_ENA
        global pwm_ENB
        global delaytime
        GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
        #设置pwm引脚和频率为2000hz
        pwm_ENA = GPIO.PWM(ENA, 2000)
        pwm_ENB = GPIO.PWM(ENB, 2000)
        pwm_ENA.start(0)
        pwm_ENB.start(0)

    #小车前进	
    def run(self,delaytime):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(80)
        pwm_ENB.ChangeDutyCycle(80)
        time.sleep(delaytime)

    #小车后退
    def back(self,delaytime):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        pwm_ENA.ChangeDutyCycle(80)
        pwm_ENB.ChangeDutyCycle(80)
        time.sleep(delaytime)

    #小车左转	
    def left(self,delaytime):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(80)
        pwm_ENB.ChangeDutyCycle(80)
        time.sleep(delaytime)

    #小车右转
    def right(self,delaytime):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(80)
        pwm_ENB.ChangeDutyCycle(80)
        time.sleep(delaytime)

    #小车原地左转
    def spin_left(self,delaytime):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(80)
        pwm_ENB.ChangeDutyCycle(80)
        time.sleep(delaytime)

    #小车原地右转
    def spin_right(self,delaytime):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        pwm_ENA.ChangeDutyCycle(80)
        pwm_ENB.ChangeDutyCycle(80)
        time.sleep(delaytime)

    #小车停止	
    def brake(self,delaytime=1):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(80)
        pwm_ENB.ChangeDutyCycle(80)
        time.sleep(delaytime)

if __name__ == "__main__":

    print('''
*******************************************************
*             wangcai - a speaking dog                *
*******************************************************
''')

    wc = Wangcai("wangwang")
    print("my name is %s" % wc.name)

    wc.colorLed(1)

    time.sleep(2)

    wc.color(1,0,0,2)
    '''
    #延时2s   
    time.sleep(2)


    #try/except语句用来检测try语句块中的错误，
    #从而让except语句捕获异常信息并处理。
    #小车循环前进1s，后退1s，左转2s，右转2s，原地左转3s
    #原地右转3s，停止1s。
    try:
        wc.motor_init()
        wc.spin_left(1)
        wc.brake(0)
    except KeyboardInterrupt:
        pass
    '''
    pwm_ENA.stop()
    pwm_ENB.stop()
    GPIO.cleanup()
