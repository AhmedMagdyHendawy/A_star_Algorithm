import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError

import cv2
from matplotlib import pyplot as plt
import numpy as np

import math
import time



# ROS

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

class Autonomous_Robot():
    def __init__(self,current_pose,robot_lenght,wheel_redius):
        self.current_pose=current_pose
        self.l=robot_lenght
        self.r=wheel_redius


    def GoToGoalController(self,
                            Initial_pose,
                            Final_pose,
                            Map,
                            Ts,
                            tol,  #Matrix of the three coordinate tolerance
                            K,    #Matrix of the three gains of rho, alpha, beta
                            ):
        Done=False
        xg=Final_pose[0]
        yg=Final_pose[1]
        thetag=Final_pose[2]

        Krho=K[0]
        Kalpha=K[1]
        Kbeta=K[2]

        control_input=[]
        states=[[self.current_pose[0]],[self.current_pose[1]]]
        




        self.current_pose=Initial_pose

        while Done == False :
            xc=self.current_pose[0]
            yc=self.current_pose[1]
            thetac=self.current_pose[2]
            

            rho=math.sqrt(math.pow((xg-xc),2)+math.pow((yg-yc),2))
            alpha=(math.atan2(yg-yc,xg-xc)+(yg-yc<0)*2*math.pi)-thetac
            beta=-alpha-thetac+thetag

            if abs(alpha)<=math.pi/2:
                V=Krho*rho
            else:
                V=-Krho*rho

            omega=Kalpha*alpha+Kbeta*beta

            input_tmp=[V,omega]
            control_input.append(input_tmp)

            wr=(2*V+omega*self.l)/(2*self.r)
            wl=((2*V)/self.r)-wr

            xnew=xc+Ts*((self.r*math.cos(thetac)/2)*(wr+wl))
            ynew=yc+Ts*((self.r*math.sin(thetac)/2)*(wl+wr))
            thetanew=thetac+Ts*((self.r/self.l)*(wr-wl))

            self.current_pose[0]=xnew
            self.current_pose[1]=ynew
            self.current_pose[2]=thetanew

            states[0].append(xnew)
            states[1].append(ynew)
            # print(xnew,ynew)

            time.sleep(0.08)

  


            
            if (abs(xg-xc)<tol[0]) & (abs(yg-yc)<tol[1]):
                Done =True

            
        plt.scatter(states[0],states[1])
        plt.show()    
        return control_input

def color_picker(rect,thresh1):
  roi=thresh1[rect[0][1]:rect[1][1],rect[0][0]:rect[1][0]]
  average=cv2.mean(roi)
  if average==(0,0,0,0) or (average[1]<=10 and average[0]==0 and average[2]==0):
    return 0
  return 1


def heuritic_value(grid,final_pose):
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            dist = np.linalg.norm(np.array((grid[i][j])["centroid"])-np.array((grid[final_pose[0]][final_pose[1]])["centroid"]))
            (grid[i][j])["heu"]=dist
    return grid

    




def A_star(grid,start,final_pose):
    grid_new=heuritic_value(grid,final_pose)
    c=start[0]
    r=start[1]
    ROI=[]
    sequence=[start]
    next_state=grid_new[c][r]
    
    while not ((c==final_pose[0]) and (r==final_pose[1])):
      c_tmp=c-1
      r_tmp=r-1
      ROI=[]
      heuristic=[]
      local_min=False
      for i in range(3):
        for j in range(3):
          if not (c_tmp==c and r_tmp==r):
            if not (c_tmp < 0 or r_tmp <0):
              if not ((grid_new[c_tmp][r_tmp])["obstacle"]==1 or ((grid_new[c_tmp][r_tmp])["visited"]==1 and not(local_min))):
                ROI.append([c_tmp,r_tmp])
                heuristic.append((grid[c_tmp][r_tmp])["heu"])
                local_min=False
          c_tmp=c_tmp+1
        r_tmp=r+i
        c_tmp=c-1
      
      heuritic_mat=np.array(heuristic)
      if (heuritic_mat.size == 0):
        print("stuck")
        x=next_state["centroid"][0]
        y=next_state["centroid"][1]
        # plt.imshow(thresh1)
        # plt.show()
        cv2.circle(thresh1, (x,y), 5, (0, 0, 0), -1)
        print(sequence)
        del sequence[-1]
        local_min=True
      else:
        min_index=np.argmin(heuritic_mat)
        c=ROI[min_index][0]
        r=ROI[min_index][1]
        
        (grid_new[c][r])["visited"]=1
        next_state=grid_new[c][r]
        x=next_state["centroid"][0]
        y=next_state["centroid"][1]
        cv2.circle(thresh1, (x,y), 5, (0, 0, 255), -1)
        sequence.append([c,r])
        print(sequence)

    return grid_new,sequence

def callback(msg):
    pass

def gazebo_simulate(control_input):

    speed=control_input[0]
    turn=control_input[1]
    
    twist = Twist()
    twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
    pub.publish(twist)



rospy.init_node("gotogoal")
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
Sub_states=rospy.Subscriber("gazebo/model_states",ModelStates,callback)

Alautonomous_robot=Autonomous_Robot([0,0,math.pi/2],0.4,0.1/2)
image_=cv2.imread("gazebo.jpg")
#image=cv2.cvtColor(image_,cv2.COLOR_BGR2GRAY)

crop_img = image_[0:930, 450:450+470]
ret,thresh1=cv2.threshold(crop_img,160,200,cv2.THRESH_BINARY)
Grid_Map=[]
x=0
y=0
for i in range(470/50):
    grid=[]
    for j in range(950/50):
        cv2.rectangle(crop_img,(x,y),(x+50,y+50),(0,255,0),1,1)
        cell={"centroid": [x+50/2,y+50/2],
              "obstacle": color_picker([[x,y],[x+50,y+50]],thresh1),
              "coordinates": [[x,y],[x+50,y+50]],
              "visited": 0
                }
        grid.append(cell)
        y=y+50
    Grid_Map.append(grid)
    x=x+50
    y=0
Grid_Map=np.array(Grid_Map)
print(Grid_Map[1][1])
ret,thresh1=cv2.threshold(crop_img,160,200,cv2.THRESH_BINARY)
plt.imshow(crop_img)
plt.imshow(thresh1)
plt.show()
Grid_Map, sequence=A_star(Grid_Map,[1,1],[6,17])
plt.imshow(thresh1)
plt.show()
r=rospy.Rate(10)
ratio=float(0.01)
print(ratio)
for z in range(len(sequence)+1):
  x_start=float(((Grid_Map[sequence[z][0]][sequence[z][1]])["centroid"][0])*ratio)
  y_start=float(((Grid_Map[sequence[z][0]][sequence[z][1]])["centroid"][1])*ratio)
  x_finish=float(((Grid_Map[sequence[z+1][0]][sequence[z+1][1]])["centroid"][0])*ratio)
  y_finish=float(((Grid_Map[sequence[z+1][0]][sequence[z+1][1]])["centroid"][1])*ratio)
  print(x_start,y_start,x_finish,y_finish)
  control_inputs=Alautonomous_robot.GoToGoalController(Initial_pose=[x_start,x_start,-math.pi/2],
                                      Final_pose=[x_finish,y_finish,-math.pi/2],
                                      Map=[],
                                      Ts=0.01,
                                      tol=[0.2,0.2,2],
                                     K=[0.5,1.5,-0.5])

  print("Finished")
  for i in range (len(control_inputs)):
      control_input=control_inputs[i]

      gazebo_simulate(control_input)

      r.sleep()


  twist = Twist()
  twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
  twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
  pub.publish(twist)
