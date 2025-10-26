#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid  # noqa: F401

from .map_conversions import MapConversions  # noqa: F401
from .occupancy_grid_map import OccupancyGridMap  # noqa: F401


class OccupancyGridNode(Node):

    def __init__(self):
        super().__init__('occupancy_grid')
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Set up the ROS node (done)
    self.get_logger().info('occupancy_grid node started')

        # TODO Set up the ROS publisher for the occupancy grid map 
        # TODO Set up the ROS publisher for the occupancy grid map
    qos = rclpy.qos.QoSProfile(depth=1)
    qos.reliability = rclpy.qos.QoSReliabilityPolicy.RELIABLE
    qos.durability  = rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.history     = rclpy.qos.QoSHistoryPolicy.KEEP_LAST

    self.grid_map_publisher = self.create_publisher(OccupancyGrid, 'map', qos)
    self.get_logger().info('Publisher /map ready (TRANSIENT_LOCAL)')

    

        # TODO Read in the map information from the ROS parameter server
        # frame_id: frame name for the map
        # resolution: cell size 
        # boundary: world rectangle [xmin, ymin, xmax, ymax]
        # blocks: list of obstacle blocks[xmin, ymin, xmax, ymax,...]
    self.declare_parameter('frame_id','map')
    self.declare_parameter('resolution',0.1)
    self.declare_parameter('boundary', [0.0, 0.0, 1.0, 1.0])
    self.declare_parameter('blocks', [])                   

    #debug line
    self.get_logger().info(f"Params: frame_id={frame_id}, res={resolution}, boundary={boundary}, blocks_len={len(blocks)}")
                         
    frame_id = self.get_parameter('frame_id').value            #string
    resol = float(self.get_parameter('resolution').value)        #float
    boundary = list(self.get_parameter('boundary').value)        #list of floats
    blocks   = list(self.get_parameter('blocks').value)        #list of floats

    self.get_logger().info(f"frame_id={frame_id}, resolution={resolution}")

        # TODO Create an OccupancyGridMap based on the provided data using occupancy_grid_utils
    #short variables
    b = boundary
    bl = blocks
    res = resolution
    grid_map = OccupancyGridMap(b, res, frame_id)  
#AI(per course policy)	
#Tool: ChatGPT 
#What I used it for:
# - Helped me split the flat blocks list into groups of 4 numbers
# - Helped me add each group to the map
# - Helped me save the map in self._grid_map
# - Helped me print how many blocks were added
#Prompt I used:
#I have a flat blocks list, so please split it into groups like [xmin, ymin, xmax, ymax], add each block to the 								map,save the map in self._grid_map, and print after how many blocks were added
    for i in range(0, len(blocks), 4):
        grid_map.add_block(blocks[i:i+4])

        # TODO Create and publish a nav_msgs/OccupancyGrid msg
    current_time = self.get_clock().now()
    map_message  = grid_map.to_msg(current_time)
    self.grid_map_publisher.publish(map_message)


        ##### YOUR CODE ENDS HERE   ##### # noqa: E266


def main(args=None):
    # Start up ROS2
    rclpy.init(args=args)

    # Create the node
    og_node = OccupancyGridNode()

    # Let the node run until it is killed
    rclpy.spin(og_node)

    # Clean up the node and stop ROS2
    og_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
