#!/usr/bin/env python
import rospy
import math
import tf2_ros
import tf.transformations
from nav_msgs.msg          import Odometry,Path
from geometry_msgs.msg     import Twist,PoseStamped
from stay_or_roam.msg      import Task,RobotState,AuctionAnnouncement
from visualization_msgs.msg import Marker,MarkerArray
from auction import performAuction
class stay_or_roam_node:
  COMPLETING_TASKS        = 1 #moving to waypoints
  AUCTIONING              = 2 #waiting for others to confirm
  FINISHED                = 4 #we're done
  
  def __init__(self):
    self.params()
    self.subs()
    self.pubs()
    self.tf_buffer   = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    #rospy.Timer(rospy.Duration(5), self.timerHeartbeat, oneshot=False)

  def params(self):
    self.robot_id       = rospy.get_param('~robot_id', 0)
    self.lin_vel        = rospy.get_param('~lin_vel', 0.5)
    self.ang_vel        = rospy.get_param('~ang_vel', 0.5)
    self.start_delay    = rospy.get_param('~start_delay', 3)
    self.wait_time      = rospy.get_param('~wait_time', 1)
    self.dist_req       = rospy.get_param('~dist_req', 1)
    self.map_frame      = rospy.get_param('~map_frame', "map")
    self.baselink_frame = rospy.get_param('~baselink_frame', "base_link")
    self.auction_time   = rospy.get_param('~auction_time', 3)
    self.detection_dist = rospy.get_param('~detection_dist', 20)
    waypoints = rospy.get_param('~waypoints', -1)
    if len(waypoints) % 2 > 0:
      print("Incorrect waypoint parameter.")
      return
    print("Waypoints are: ")
    print(waypoints)
    self.x = []
    self.y = []
    x    = waypoints[0::2]
    y    = waypoints[1::2]
    self.wp   = 0
    self.robot_state = RobotState()
    self.robot_state.robot_id = self.robot_id
    self.robot_state.state = self.COMPLETING_TASKS
    self.robot_state.pose.header.frame_id = self.baselink_frame
    for i in range(0, len(x)):
      task = Task()
      task.task_id = i
      task.allocated_robot_id = -1
      task.known = False
      task.completed = False
      task.point.x = x[i]
      task.point.y = y[i]
      self.robot_state.tasks.append(task)

  def subs(self):
    rospy.Subscriber("/robot_state", RobotState, self.subscribeRobotState)
    rospy.Subscriber("/auction_announcement", AuctionAnnouncement, self.subscribeAuctionAnnouncement)
    rospy.Subscriber("/auction_allocation", RobotState, self.subscribeAuctionAllocation)
    
  def pubs(self):
    self.pub_vel     = rospy.Publisher('jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    self.pub_path    = rospy.Publisher('path_moved',        Path, queue_size=1)
    self.pub_current = rospy.Publisher('current_waypoints', Path, queue_size=1)
    self.pub_state   = rospy.Publisher('/robot_state', RobotState, queue_size=1)
    self.pub_auction_announcement = rospy.Publisher('/auction_announcement', AuctionAnnouncement, queue_size=1)
    self.pub_auction_allocation = rospy.Publisher('/auction_allocation', RobotState, queue_size=10)
    self.pub_marker = rospy.Publisher('/tasks', MarkerArray, queue_size=1)
    
    self.path_moved        = Path()
    self.current_waypoints = Path()
    self.path_moved.header.frame_id        = self.map_frame
    self.current_waypoints.header.frame_id = self.map_frame

  ###### Waypoint display. ######

  def timerPublishPaths(self, event):
    self.publishPathMoved()
    self.publishCurrentWaypoints()
    self.publishAllTasks()

  def publishPathMoved(self):
    ps = PoseStamped()
    ps.header.stamp = self.robot_state.pose.header.stamp
    ps.pose.position.x   = self.robot_state.pose.pose.position.x
    ps.pose.position.y   = self.robot_state.pose.pose.position.y
    ps.pose.orientation.x = self.robot_state.pose.pose.orientation.x
    ps.pose.orientation.y = self.robot_state.pose.pose.orientation.y
    ps.pose.orientation.z = self.robot_state.pose.pose.orientation.z
    ps.pose.orientation.w = self.robot_state.pose.pose.orientation.w
    self.path_moved.poses.append(ps)
    self.pub_path.publish(self.path_moved)

  def publishCurrentWaypoints(self):
    poses = []
    for i in range(self.wp-1, len(self.x)):
      if i < 0 or i >= len(self.x):
        continue
      pose = PoseStamped()
      pose.header.frame_id = self.map_frame
      pose.header.stamp    = rospy.get_rostime()
      pose.pose.position.x = self.x[i]
      pose.pose.position.y = self.y[i]
      poses.append(pose)
    self.current_waypoints.poses = poses
    self.pub_current.publish(self.current_waypoints)

  def publishAllTasks(self):
    ma = MarkerArray()
    i = 0
    for t in self.robot_state.tasks:
      m = Marker()
      m.id = i
      m.type = 2
      m.header.frame_id = self.map_frame
      m.pose.position = t.point
      m.pose.orientation.w = 1
      m.scale.x = 1
      m.scale.y = 1
      m.scale.z = 1
      m.color.r = 1
      m.color.g = 1
      m.color.b = 1
      m.color.a = 1
      ma.markers.append(m)
      i = i + 1
    self.pub_marker.publish(ma)
    
  ###### Communications between robots #####
  def getDist(self, pose):
    dx = self.robot_state.pose.pose.position.x - pose.position.x
    dy = self.robot_state.pose.pose.position.y - pose.position.y
    dist = math.sqrt( dx*dx + dy*dy )
    return dist
    
  def timerHeartbeat(self, event):
    self.publishRobotState()
    
  def publishRobotState(self):
    print("Publishing State " + str(self.robot_id))
    #print(self.robot_state)
    self.pub_state.publish(self.robot_state)
    
  def subscribeRobotState(self, msg):
    if msg.robot_id == self.robot_id:
      return
    if self.getDist(msg.pose.pose) > self.detection_dist:
      return
    print("Received another robots state " + str(self.robot_id))
    # Extract task information.
    auction = False
    for t, t_new in zip(self.robot_state.tasks,msg.tasks):
      if not t.known and t_new.known:
        t.known = True
        auction = True
      if not t.completed and t_new.completed:
        t.completed = True
      if t.allocated_robot_id == self.robot_id and t_new.allocated_robot_id == msg.robot_id:
        auction = True
    # Maybe start an auction.
    if self.robot_state.state == self.AUCTIONING:
      return
    if auction or msg.state == self.AUCTIONING:
      self.requestAuction()
    
  def requestAuction(self):
    print("Starting an auction! " + str(self.robot_id))
    # Stop moving.
    cmd_vel = Twist()
    self.pub_vel.publish(cmd_vel)
    # Announce desire for auction.
    self.publishRobotState()
    self.auction_ids = [self.robot_id]
    self.auction_poses = [self.robot_state.pose]
    self.publishAuctionAnnouncement()
    self.robot_state.state = self.AUCTIONING
    # Set a time to start auction.
    self.auction_start_time = rospy.get_rostime() + rospy.Duration(self.auction_time)
      
  def publishAuctionAnnouncement(self):
    msg = AuctionAnnouncement()
    msg.robot_id = self.robot_id
    msg.robot_pose = self.robot_state.pose
    msg.auction_ids = self.auction_ids
    msg.poses = self.auction_poses
    self.pub_auction_announcement.publish(msg)
    print("Announcing auction " + str(self.robot_id))
    print(msg)
  
  # Update robots in auction. If any robots are new, re-announce.
  def subscribeAuctionAnnouncement(self, msg):
    if msg.robot_id == self.robot_id:
      return
    if self.getDist(msg.robot_pose.pose) > self.detection_dist:
      return
    print("Received auction announcement " + str(self.robot_id))
    if not self.robot_state.state == self.AUCTIONING:
      self.auction_ids   = [self.robot_id]
      self.auction_poses = [self.robot_state.pose]
    announce = False
    for r,p in zip(msg.auction_ids,msg.poses):
      if r not in self.auction_ids:
        print("Adding ourselves to the auction ids " + str(self.robot_id) + " " + str(r) + " " + str(p))
        self.auction_ids.append(r)
        self.auction_poses.append(p)
        announce = True
    if announce:
      self.auction_start_time = rospy.get_rostime() + rospy.Duration(self.auction_time)    
      self.publishAuctionAnnouncement()

  def subscribeAuctionAllocation(self, msg):
    print("My state is: " + str(self.robot_state.state))
    if not self.robot_state.state == self.AUCTIONING:
      return
    if self.getDist(msg.pose.pose) > self.detection_dist:
      return
    print("Received my allocations! " + str(self.robot_id))
    self.robot_state.state = self.COMPLETING_TASKS
    self.wp = 0
    num_tasks_for_us = 0
    for t in msg.tasks:
      if t.allocated_robot_id == self.robot_id:
        num_tasks_for_us = num_tasks_for_us + 1
    self.x = [0] * num_tasks_for_us
    self.y = [0] * num_tasks_for_us
    self.task_order = [0] * num_tasks_for_us
    for t in msg.tasks:
      if t.allocated_robot_id == self.robot_id:
        self.x[t.order] = t.point.x
        self.y[t.order] = t.point.y 
        self.task_order[t.order] = t.task_id
    print(self.x)
    print(self.y)
    if num_tasks_for_us == 0:
      self.robot_state.state = self.FINISHED
    # Publish allocations to propagate
    self.pub_auction_allocation.publish(self.robot_state)
    
  ###### Task allocation ######

  def allocateTasks(self):
    tasks = performAuction(self.auction_ids, self.auction_poses, self.robot_state.tasks)
    self.robot_state.tasks = tasks
    self.pub_auction_allocation.publish(self.robot_state)
    
  ###### Helper functions ######
  def lookupXYYaw(self):
    try:
      trans = self.tf_buffer.lookup_transform(self.map_frame,self.baselink_frame, rospy.Time())
      quat = (trans.transform.rotation.x, trans.transform.rotation.y,
              trans.transform.rotation.z, trans.transform.rotation.w)
      eul = tf.transformations.euler_from_quaternion(quat)
      return (True, trans.transform.translation.x, trans.transform.translation.y, eul[2], quat)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      return (False, 0, 0, 0)

  def wrapToPi(self, ang):
    pi = 3.1415926538
    while ang <= -pi:
      ang = ang + 2*pi
    while ang > pi:
      ang = ang - 2*pi
    return ang

  ###### Main functions ######
  def loop(self):
    # Update the robot state.
    tup = self.lookupXYYaw()   #(True,x,y,yaw,quat) or (False,0,0,0)
    if not tup[0]:
      return
    self.robot_state.pose.header.seq   = self.robot_state.pose.header.seq + 1
    self.robot_state.pose.header.stamp = rospy.get_rostime()
    self.robot_state.pose.pose.position.x  = tup[1]
    self.robot_state.pose.pose.position.y  = tup[2]
    quat = tup[4]
    self.robot_state.pose.pose.orientation.x = quat[0]
    self.robot_state.pose.pose.orientation.y = quat[1]
    self.robot_state.pose.pose.orientation.z = quat[2]
    self.robot_state.pose.pose.orientation.w = quat[3]
    # If we're within range of a task, we learn about it.
    auction = False
    for t in self.robot_state.tasks:
      if t.known:
        continue
      dx = t.point.x - tup[1]
      dy = t.point.y - tup[2]
      dist = math.sqrt( dx*dx + dy*dy )
      if dist < self.detection_dist:
        t.known = True
        auction = True 
    # If we're waiting for the auction to start, don't do anything else.
    if self.robot_state.state == self.AUCTIONING:
      for ids in self.auction_ids:
        if ids < self.robot_id:
          return
      if rospy.get_rostime() > self.auction_start_time:
        # Start the auction.
        print("I'M THE AUCTIONEER " + str(self.robot_id))
        print(self.robot_id)
        # Allocate the tasks.
        self.allocateTasks()
        #self.robot_state.state = self.SENDING_ALLOCATIONS
    # If we want an auction, start one.
    if auction:
      self.requestAuction()
    # Complete tasks.
    if self.robot_state.state == self.COMPLETING_TASKS:
      # Calculate the distance and angle change needed.
      dx = self.x[self.wp] - tup[1]
      dy = self.y[self.wp] - tup[2]
      dist = math.sqrt( dx*dx + dy*dy )
      dang = math.atan2( dy, dx ) - tup[3]
      dang = self.wrapToPi(dang)
      #print("My location: ")
      #print(tup)
      #print("Waypoint: ")
      #print( (self.x[self.wp], self.y[self.wp], (180/3.1415)*math.atan2( dy, dx)) )
      #print("Target dist,ang: ")
      #print( (dist, (180/3.1415)*dang) )
      # Calculate the command velocity.
      x_vel = self.lin_vel
      dang_abs = math.sqrt(dang*dang)
      if dang_abs > 0.5:
        yaw_vel = self.ang_vel
      else:
        yaw_vel = self.ang_vel*(dang_abs/0.5)
      if dang < 0:
        yaw_vel = -yaw_vel
      # Publish it.
      cmd_vel = Twist()
      cmd_vel.linear.x = x_vel
      cmd_vel.angular.z = yaw_vel
      self.pub_vel.publish(cmd_vel)
      # Check for waypoint completion.
      if dist <= self.dist_req:
        print("Waypoint completed")
        self.robot_state.tasks[self.task_order[self.wp]].completed = True
        self.wp = self.wp + 1
        if self.wp == len(self.x):
          print("Finished")
          self.robot_state.state = self.FINISHED
        else:
          print("Moving to next waypoint")
    #elif self.robot_state.state == self.FINISHED:
      # do nothing.

if __name__ == '__main__':
  rospy.init_node('stay_or_roam_node', anonymous=True)
  obj = stay_or_roam_node()
  rospy.sleep(obj.start_delay)
  rospy.Timer(rospy.Duration(0.5), obj.timerPublishPaths, oneshot=False)
  r = rospy.Rate(30)  #Hz
  while not rospy.is_shutdown():
    obj.loop()
    r.sleep()
  #rospy.spin()
