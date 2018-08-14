# State options:
#  COMPLETING_TASKS        = 1 #moving to waypoints
#  REQUESTING_AUCTION      = 2 #waiting for others to confirm
#  SENDING_ALLOCATIONS     = 3 #we're the auctioneer
#  WAITING_FOR_ALLOCATIONS = 4 #we're not the auctioneer

uint8                     robot_id
uint8                     state
geometry_msgs/PoseStamped pose
Task[]                    tasks
