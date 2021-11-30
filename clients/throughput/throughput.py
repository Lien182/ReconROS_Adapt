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
from sorter_msgs.srv import Sort




class SortClient(Node):

    def __init__(self):
        super().__init__('sorter_client')

        self.tstart = 0
        self.tstop = 0

        self.cli = self.create_client(Sort, 'sorter')
        while not self.cli.wait_for_service(timeout_sec=4.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Sort.Request()

    def send_request(self):
        self.req.unsorted =  random.sample(range(1, 100000), 2048)
        self.tstart = time.time()
        self.future = self.cli.call_async(self.req)



    def SortThread(self, cycles):
        for i in range(0,cycles):
            
            self.send_request()

            while rclpy.ok():
                rclpy.spin_once(self)
                if self.future.done():
                    try:
                        response = self.future.result()
                    except Exception as e:
                        sort_client.get_logger().info(
                            'Service call failed %r' % (e,))
                    else:
                        self.tstop = time.time()
                        if response.sorted.tolist() == sorted(response.sorted):
                            self.get_logger().info('Data is sorted! {} ms'.format((self.tstop-self.tstart)*1000.0))
                        else:
                            self.get_logger().info('Data is NOT sorted!')
                        
                    break

                        
        self.destroy_node()

    def Run(self,cycles):
        self.x = threading.Thread(target=self.SortThread, args=(cycles,))
        self.x.start()       

    def Join(self):
        self.x.join()


def main(args=None):
    rclpy.init(args=args)
    sort_client = SortClient()
 
    
    sort_client.Run(10)
    sort_client.Join()

    rclpy.shutdown()




if __name__ == '__main__':
    main()



# class ImagePubSub(Node):

#     def __init__(self):
#         super().__init__('Image')
        
#         self.publisher_ = self.create_publisher(UInt32MultiArray, '/unsorted', 10)
        

#         self.tstart = 0
#         self.tend = 0       
#         self.subscription = self.create_subscription(UInt32MultiArray,'/sorted',self.listener_callback,  10)
#         self.subscription  # prevent unused variable warning
#         timer_period = 0.2  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.msg = UInt32MultiArray()
#         self.msg.data = random.sample(range(1, 100000), 2048)
#         self.cnt = 0


#     def listener_callback(self, msg):
#         #cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         #cv2.imshow("Image window", cv_image)
#         #cv2.waitKey(3)
#         self.tend = time.time()
#         print('%f' % (self.tend - self.tstart))
#         self.cnt += 1
#         if self.cnt == 1000:
#             self.timer.cancel()
#             self.destroy_node()
#             exit()
        

#     def timer_callback(self):
#         self.publisher_.publish(self.msg)
#         self.tstart = time.time()
            
# def main(args=None):
#     rclpy.init(args=args)

#     pubsub = ImagePubSub()
   




# if __name__ == '__main__':
#     main()