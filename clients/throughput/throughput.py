from tensorflow.keras.datasets import mnist
import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

import time
import struct
import random
import os
import threading
import numpy as np

from tensorflow import keras
from tensorflow.keras.datasets import mnist
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout, Flatten
from tensorflow.keras.layers import Conv2D, MaxPooling2D
from tensorflow.keras import backend as K
from keras.models import model_from_json

import cv2
from cv_bridge import CvBridge
from sorter_msgs.srv import Sort
from std_msgs.msg import UInt32
from sensor_msgs.msg import Image

class SortClient(Node):

    def __init__(self, cycles):
        super().__init__('SortClientNode')

        self.tstart = 0
        self.tstop = 0
        self.cycles = cycles
        timer_period = 0.001  # seconds
        cb_group = ReentrantCallbackGroup()
        self.cli = self.create_client(Sort, 'sorter', callback_group=cb_group)
        while not self.cli.wait_for_service(timeout_sec=4.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Sort.Request()
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=cb_group)

    def send_request(self):

        self.req.unsorted =  random.sample(range(1, 100000), 2048)
        self.tstart = time.time()

        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()

        future = self.cli.call_async(self.req)
        future.add_done_callback(unblock)

        # Check future.done() before waiting on the event.
        # The callback might have been added after the future is completed,
        # resulting in the event never being set.
        if not future.done():
            event.wait()
        if future.exception() is not None:
            raise future.exception()       

        self.future = future.result()

        #self.future = self.cli.call(self.req)
        self.tstop = time.time()
        if self.future.sorted.tolist() == sorted(self.future.sorted):
            self.get_logger().info('{} ms'.format((self.tstop-self.tstart)*1000.0))
        else:
            self.get_logger().info('Data is NOT sorted!')

    def timer_callback(self):
        self.timer.cancel()
        print("send request {}".format(self.cycles))
        for i in range(0,self.cycles):
            self.send_request()                                    
        self.destroy_node()




class InverseClientNode(Node):

    def __init__(self, cycles):
        super().__init__('InverseClientNode')
        
        self.cnt = cycles
        self.publisher_ = self.create_publisher(UInt32, '/angle', 10)
        
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.tstart = 0
        self.tstop = 0       
        self.subscription = self.create_subscription(UInt32,'/legangle',self.listener_callback,  10)
        self.subscription  # prevent unused variable warning
        self.msg = UInt32()
        self.msg.data = ((2*101) << 3) | ((2*152) << 17)  | (2%7)


    def listener_callback(self, msg):
        self.tstop = time.time()
        self.get_logger().info('{} ms'.format((self.tstop-self.tstart)*1000.0))
        if self.cnt > 0:
            self.tstart = time.time()
            self.publisher_.publish(self.msg)
            self.cnt -= 1
            
    def timer_callback(self):
        self.timer.cancel()
        self.tstart = time.time()
        self.publisher_.publish(self.msg)
     

class MnistClientNode(Node):

    def __init__(self, cycles):
        super().__init__('MnistClientNode')
        self.cnt = cycles
        self.bridge = CvBridge()        
        self.publisher_ = self.create_publisher(Image, '/image_classification', 10)
        self.img_rows, self.img_cols = 28, 28
        (self.x_train, self.y_train), (self.x_test, self.y_test) = mnist.load_data()
        
        self.tstart = 0
        self.tstop = 0    
        self.subscription = self.create_subscription(UInt32,'/class',self.listener_callback,  10)
        self.subscription  # prevent unused variable warning
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def listener_callback(self, msg):
        self.tstop = time.time()
        self.get_logger().info('{} ms'.format((self.tstop-self.tstart)*1000.0))
        if self.cnt > 0:
            if K.image_data_format() == 'channels_first':
                self.x_test = self.x_test.reshape(self.x_test.shape[0], 1, self.img_rows, self.img_cols)
            else:
                self.x_test = self.x_test.reshape(self.x_test.shape[0], self.img_rows, self.img_cols , 1)
            self.msg = self.bridge.cv2_to_imgmsg(np.array(self.x_test[random.randint(0,100)]), "mono8")
            self.tstart = time.time()
            self.publisher_.publish(self.msg)
            self.cnt -= 1
        

    def timer_callback(self):
        self.timer.cancel()

        if K.image_data_format() == 'channels_first':
            self.x_test = self.x_test.reshape(self.x_test.shape[0], 1, self.img_rows, self.img_cols)
        else:
            self.x_test = self.x_test.reshape(self.x_test.shape[0], self.img_rows, self.img_cols , 1)
        self.msg = self.bridge.cv2_to_imgmsg(np.array(self.x_test[random.randint(0,100)]), "mono8")
        self.tstart = time.time()
        self.publisher_.publish(self.msg)
        
class SobelClientNode(Node):

    def __init__(self, cycles):
        super().__init__('SobelClientNode')
        self.cnt = cycles     
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.cv_image = cv2.imread('image.jpg') ### an RGB image 
        self.bridge = CvBridge()
        self.msg = self.bridge.cv2_to_imgmsg(np.array(self.cv_image), "bgr8")

        self.tstart = 0
        self.tstop = 0    
        self.subscription = self.create_subscription(Image,'filtered',self.listener_callback,  10)
        self.subscription  # prevent unused variable warning
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        


    def listener_callback(self, msg):
        self.tstop = time.time()
        self.get_logger().info('{} ms'.format((self.tstop-self.tstart)*1000.0))
        if self.cnt > 0:
            self.tstart = time.time()
            print("Sobel")
            self.publisher_.publish(self.msg)
            self.cnt -= 1
        

    def timer_callback(self):
        self.timer.cancel()
        self.tstart = time.time()
        print("Sobel")
        self.publisher_.publish(self.msg)


def main(args=None):

    cycles = 10
    rclpy.init(args=args)
    sort_client = SortClient(cycles)
    inverse_sub = InverseClientNode(cycles)
    mnist_sub = MnistClientNode(cycles)
    sobel_sub = SobelClientNode(cycles)
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(sort_client)
    executor.add_node(inverse_sub)
    executor.add_node(mnist_sub)
    executor.add_node(sobel_sub)
    # Spin in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

   # rate = node.create_rate(2)
    try:
        while rclpy.ok():
            time.sleep(2)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    executor_thread.join()



if __name__ == '__main__':
    main()