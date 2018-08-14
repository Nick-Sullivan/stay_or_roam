#!/usr/bin/env python
import rospy
import math
from stay_or_roam.msg      import Task,RobotState,AuctionAnnouncement


def performAuction(robot_ids, poses, tasks):
  print("hello")
  print(robot_ids)
  print(poses)
  print(tasks)
  num_r = len(robot_ids)
  x = []
  y = []
  for p in poses:
    x.append( [p.pose.position.x] )
    y.append( [p.pose.position.y] )
  print('x')
  print(x)
  print('y')
  print(y)
  tasks_left = []
  for t in tasks:
    t.allocated_robot_id = -1
    if not t.known or t.completed:
      continue
    tasks_left.append(t)
  num_t = len(tasks_left)
  
  
  while num_t > 0:
    # Every robot bids on every task.
    bids = []
    x_if_win = []
    y_if_win = []
    for r in range(0, num_r):
      print("r:")
      print(r)
      bids.append([])
      x_if_win.append([])
      y_if_win.append([])
      for t in tasks_left:
        tup = calculateBid(x[r], y[r], t.point.x, t.point.y)
        bids[r].append(tup[0])
        x_if_win[r].append(tup[1])
        y_if_win[r].append(tup[2])
        print('This bid is: ')
        print(tup)
    
    print('bids')
    print(bids)
    # Calculate the winner.
    min_bid = -1
    r_win = -1
    t_win = -1
    for i in range(0, num_r):
      for j in range(0, num_t):
        if bids[i][j] < min_bid or min_bid == -1:
          min_bid = bids[i][j]
          r_win = i
          t_win = j
    print('r win:')
    print(r_win)
    print('t_win:')
    print(t_win)
    # Allocate to the winner.
    x[r_win] = x_if_win[r_win][t_win]
    y[r_win] = y_if_win[r_win][t_win]
    t = tasks_left.pop(t_win)
    t.allocated_robot_id = robot_ids[r_win]
    t.order = len(x[r_win])-2
    num_t = num_t - 1
    # Paths
    print('Paths:')
    print(x)
    print(y)
  print('Tasks after the auction:')
  print(tasks)
  print("DONE")
  return tasks

# Returns a tuple of the bid, along with the new path if the bid is successful.
def calculateBid(x, y, tx, ty):
  print("Calculating bid:")
  print(x)
  print(y)
  print(tx)
  print(ty)
  # Insert the task at each point.
  min_cost = -1
  for i in range(1,len(x)+1):
    newx = list(x)
    newy = list(y)
    newx.insert(i,tx)
    newy.insert(i,ty)
    cost = calculateCost(newx, newy)
    bid = (cost, newx, newy)
    if cost < min_cost or min_cost == -1:
      min_cost = cost
      min_bid = bid
  return min_bid

def calculateCost(xpath, ypath):
  print("Calculating cost")
  print(xpath)
  print(ypath)
  cost = 0
  print(cost)
  for i in range(1,len(xpath)):
    dx = xpath[i] - xpath[i-1]
    dy = ypath[i] - ypath[i-1]
    cost = cost + math.sqrt( dx*dx + dy*dy )
  print(cost)
  return cost
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

