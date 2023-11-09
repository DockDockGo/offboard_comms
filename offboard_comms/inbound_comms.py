import rclpy
from rclpy.node import Node
import sys
import threading
from fastapi import FastAPI
import uvicorn

from  typing  import  Optional 
from  pydantic  import  BaseModel 

from std_msgs.msg import String

app = FastAPI()

class  Response ( BaseModel ): 
    msg :  str 


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'move', 10)
        self.i = 0

        @app.get( '/fleet_commands', response_model = Response ) 
        async  def  publish ():
            # TODO(sid/sushanth): create mission control action client here and to trigger new mission
            msg = String()
            msg.data = 'Command received: %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)

            response  =  {  
                'msg' :  'Command received: ' + self.i
            }
            self.i += 1 
            return  response 

# Declare ros_node as None initially
ros_node = None

def main(args=None):
    # rclpy.init(args=args)
    # minimal_publisher = MinimalPublisher()
    # spin_thread = threading.Thread(target=rclpy.spin, args=(minimal_publisher,))
    # spin_thread.start()
    # uvicorn.run(app, port=5000, log_level='warning')
    # rclpy.shutdown()

    global ros_node  # Use the global variable so you can assign to it
    rclpy.init(args=args)  # Initialize ROS communication
    ros_node = MinimalPublisher()  # Now you can create your node instance
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    # Start FastAPI server
    uvicorn.run(app, host="0.0.0.0", port=5000, log_level="info")
    
    # Shutdown ROS node on FastAPI server shutdown
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()