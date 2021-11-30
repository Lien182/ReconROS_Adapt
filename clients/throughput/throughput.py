import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import time
import struct
import random
import os
import threading
import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup

from sorter_msgs.srv import Sort

from std_msgs.msg import UInt32


class SortClient(Node):

    def __init__(self):
        super().__init__('sorter_client')

        self.tstart = 0
        self.tstop = 0

        cb_group = ReentrantCallbackGroup()
        self.cli = self.create_client(Sort, 'sorter', callback_group=cb_group)
        while not self.cli.wait_for_service(timeout_sec=4.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Sort.Request()

    def send_request(self):

        self.get_logger().info('send_request!')
        self.req.unsorted =  random.sample(range(1, 100000), 2048)
        self.tstart = time.time()
        self.future = self.cli.call(self.req)
        self.tstop = time.time()
        if self.future.sorted.tolist() == sorted(self.future.sorted):
            self.get_logger().info('Data is sorted! {} ms'.format((self.tstop-self.tstart)*1000.0))
        else:
            self.get_logger().info('Data is NOT sorted!')


    def SortThread(self, cycles):
        for i in range(0,cycles):
            self.send_request()                                    
        self.destroy_node()

    def Run(self,cycles):
        self.x = threading.Thread(target=self.SortThread, args=(cycles,))
        self.x.start()       

    def Join(self):
        self.x.join()


class InverseClientNode(Node):

    def __init__(self, cycles):
        super().__init__('inverseclient')
        
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
        self.get_logger().info('Data has arrived! {} ms'.format((self.tstop-self.tstart)*1000.0))
        if self.cnt > 0:
            self.tstart = time.time()
            self.publisher_.publish(self.msg)
            self.cnt -= 1
            
    def timer_callback(self):
        self.timer.cancel()
        self.tstart = time.time()
        self.publisher_.publish(self.msg)
     

def main(args=None):

    rclpy.init(args=args)
    sort_client = SortClient()
    inverse_sub = InverseClientNode(10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(sort_client)
    executor.add_node(inverse_sub)
    # Spin in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    sort_client.Run(10)
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