#!/usr/bin/env python3.8


from typing_extensions import Self
import rospy
from kivymd.app import MDApp
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout
from kivy.properties import ObjectProperty
from kivymd.uix.tab import MDTabsBase
from kivymd.uix.floatlayout import MDFloatLayout
from PyQt5.QtWebEngineWidgets import * 
from kivy.core.window import Window


from autominy_msgs.msg import SpeedPWMCommand
from autominy_msgs.msg import NormalizedSpeedCommand
from autominy_msgs.msg import NormalizedSteeringCommand
from autominy_msgs.msg import SteeringPWMCommand
from os import system

from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import String as Str
##################################################################################
from kivy.uix.image import Image as KivyImage
from kivy.graphics.texture import Texture
from kivy.clock import Clock
import cv2
from sensor_msgs.msg import Image as AutominyImage
from cv_bridge import CvBridge, CvBridgeError
##################################################################################

#Window.size = (300,500)
Window.size = (1200,900)



# class CameraPreview(KivyImage):
    
    
#     def __init__(self):
#         self.sub = rospy.Subscriber("/sensors/camera/color/image_rect_color",AutominyImage,self.callback)

#     def callback(self,data):
#         try:
#             bridge = CvBridge()
#             self.cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
#             #lane_detection(cv_image)
#             cv2.imshow("Original",self.cv_image)
#             cv2.waitKey(3)
        
        
#         except CvBridgeError as e:
#             print(e)
    
    
#     #Drawing method to execute at intervals
#     def update(self, dt):
#         #Load frame
#         ret, self.frame = self.cv_image.read()
#         #Convert to Kivy Texture
#         buf = cv2.flip(self.frame, 0).tostring()
#         texture = Texture.create(size=(self.frame.shape[1], self.frame.shape[0]), colorfmt='bgr') 
#         texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
#         #Change the texture of the instance
#         self.texture = texture
    
     




class ContentNavigationDrawer(BoxLayout):
    screen_manager = ObjectProperty()
    nav_drawer = ObjectProperty()

class Tab(MDFloatLayout, MDTabsBase):
    '''Class implementing content for a tab.'''  
    


class Autominy(MDApp):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.screen = Builder.load_file('/home/aaron/autominy/catkin_ws/src/tutorial/ros_gui.kv')
        

    def build(self):
        return self.screen    
    
    def slider1_Function(self, slider1_value): ## Try Speed PWM function (Implementation)
        #slider1_value = MySlider()
        print(int(slider1_value))
                
        msg_speed1.value=int(slider1_value)
        pub_speed1.publish(msg_speed1)

        
    
    
    def slider2_Function(self,slider2_value): ## Try Steering  PWM function (Implementation)
        
        print(int(slider2_value))
        msg_steer1.value=int(slider2_value)
        pub_steer1.publish(msg_steer1)

    def slider3_Function(self,slider3_value): ## Try Seep Normalized function (Implementation)
        
        print(round(slider3_value, 2))
        #msg_speed.value = round(slider3_value, 2)
        #pub_speed.publish(msg_speed)

    def stop2_funtion(self,*args):
        print('Stooooop')
        msg_speed.value = 0
        pub_speed.publish(msg_speed)


    def stop_function(self,*args):
        print('STOP Button Pressed')
        #msg= True
        #stop_button_pub.publish(msg)
        msg_speed.value = 0
        pub_speed.publish(msg_speed)
        self.root.ids.slider3.value = 0 

        msg_speed1.value = 0
        pub_speed1.publish(msg_speed1)
        self.root.ids.slider1.value = 0 

    def left_function(self,slider3_value):
        print('LEFT Button Pressed')
        msg_steer1.value=int(950)
        pub_steer1.publish(msg_steer1)
        self.root.ids.slider2.value = 950

        msg_speed.value = round(slider3_value, 2)
        pub_speed.publish(msg_speed)
    
    
    def right_function(self,slider3_value):
        print('RIGHT Button Pressed')
        msg_steer1.value=int(2150)
        pub_steer1.publish(msg_steer1)
        self.root.ids.slider2.value = 2150

        msg_speed.value = round(slider3_value, 2)
        pub_speed.publish(msg_speed)

    def backward_function(self, slider3_value):
        print('BACKWARDS Button Pressed')
        if round(slider3_value, 2) < 0:
            msg_speed.value = round(slider3_value, 2)
            pub_speed.publish(msg_speed)
    
    def forward_function(self, slider3_value):
        print('FORWARD Button Pressed')
        if round(slider3_value, 2) > 0:
            msg_speed.value = round(slider3_value, 2)
            pub_speed.publish(msg_speed)
        
    
    
if __name__== '__main__':

    clear = lambda: system("clear")
    pub_steer = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
    msg_steer = NormalizedSteeringCommand()
    pub_speed = rospy.Publisher("/actuators/speed_normalized", NormalizedSpeedCommand, queue_size=10)
    msg_speed = NormalizedSpeedCommand()

    pub_steer1 = rospy.Publisher("/actuators/steering_pwm", SteeringPWMCommand, queue_size = 10)
    msg_steer1 = SteeringPWMCommand()
    pub_speed1 = rospy.Publisher("/actuators/speed_pwm", SpeedPWMCommand, queue_size=10)
    msg_speed1 = SpeedPWMCommand()
    
    
    
    rospy.init_node('gui.py', anonymous=True) 
    Autominy().run()
    