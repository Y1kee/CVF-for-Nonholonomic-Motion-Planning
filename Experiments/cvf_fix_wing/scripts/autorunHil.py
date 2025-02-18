import subprocess
import time
import random
import numpy as np
import pickle
import rospy
from geometry_msgs.msg import PoseStamped

class Params:
    def __init__(self):
        self.des_px = 0
        self.des_py = 0
        self.des_angle = 0
        self.defalut_circle_point_x = 0
        self.defalut_circle_point_y = 0



class AutoRun:
    def __init__(self):
        self.cur_pos = None
        self.last_pos = None
        self.subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        
        self.ps = []
        self.has_track = []
        self.close_count = []
        self.away_count = []
        for i in range(9):
            self.ps.append(Params())
            self.has_track.append(False)
            self.close_count.append(0)
            self.away_count.append(0)
        #1
        self.ps[0].des_px = -60.0000*3
        self.ps[0].des_py = -103.9230*3
        self.ps[0].des_angle = 5.7596
        self.ps[0].defalut_circle_point_x = 0.5000*3
        self.ps[0].defalut_circle_point_y = 0.5796*3
        #2
        self.ps[1].des_px = 120.0000*3
        self.ps[1].des_py = 0.0000*3
        self.ps[1].des_angle = 1.5708
        self.ps[1].defalut_circle_point_x = -1.0000*3
        self.ps[1].defalut_circle_point_y = 0.0000*3
        #3
        self.ps[2].des_px = -60.0000*3
        self.ps[2].des_py = 103.9230*3
        self.ps[2].des_angle = 3.6652
        self.ps[2].defalut_circle_point_x = 0.5000*3
        self.ps[2].defalut_circle_point_y = -0.8660*3
        #4
        self.ps[3].des_px = -60.0000*3
        self.ps[3].des_py = -103.9230*3
        self.ps[3].des_angle = 5.7596
        self.ps[3].defalut_circle_point_x = 50.0000*3
        self.ps[3].defalut_circle_point_y = 86.6025*3
        #5
        self.ps[4].des_px = 120.0000*3
        self.ps[4].des_py = 0.0000*3
        self.ps[4].des_angle = 1.5708
        self.ps[4].defalut_circle_point_x = -100.0000*3
        self.ps[4].defalut_circle_point_y = 0.0000*3
        #6
        self.ps[5].des_px = -60.0000*3
        self.ps[5].des_py = 103.9230*3
        self.ps[5].des_angle = 3.6652
        self.ps[5].defalut_circle_point_x = 50.0000*3
        self.ps[5].defalut_circle_point_y = -86.6025*3
        #7
        self.ps[6].des_px = -60.0000*3
        self.ps[6].des_py = -103.9230*3
        self.ps[6].des_angle = 5.7596
        self.ps[6].defalut_circle_point_x = 100.0000*3
        self.ps[6].defalut_circle_point_y = 173.2051*3
        #8
        self.ps[7].des_px = 120.0000*3
        self.ps[7].des_py = 0.0000*3
        self.ps[7].des_angle = 1.5708
        self.ps[7].defalut_circle_point_x = -200.0000*3
        self.ps[7].defalut_circle_point_y = 0.0000*3
        #9
        self.ps[8].des_px = -60.0000*3
        self.ps[8].des_py = 103.9230*3
        self.ps[8].des_angle = 3.6652
        self.ps[8].defalut_circle_point_x = 100.0000*3
        self.ps[8].defalut_circle_point_y = -173.2051*3
        self.id = 0
        self.dist_threshold = 25
        self.pos_list = []
        for i in range(30):
            self.pos_list.append(None)
        self.pos_id = -1

        xcmd = 'des_px:='+str(self.ps[self.id].des_px)+\
        ' des_py:='+str(self.ps[self.id].des_py)+\
        ' des_angle:='+str(self.ps[self.id].des_angle) + \
        ' defalut_circle_point_x:='+str(self.ps[self.id].defalut_circle_point_x) + \
        ' defalut_circle_point_y:='+str(self.ps[self.id].defalut_circle_point_y)
        cmd = 'roslaunch track_controller hil_single_track_spawn.launch '+ xcmd
        self.p1 = subprocess.Popen(cmd, shell=True)
        self.p2 = subprocess.Popen('rosbag record /dbg_msg', shell=True)

        
    def countingReach(self, x, y, lx,ly):
        x1 = self.ps[self.id].des_px
        y1 = self.ps[self.id].des_py
        last_distance = np.sqrt((lx - x1) ** 2 + (ly - y1) ** 2)
        distance = np.sqrt((x - x1) ** 2 + (y - y1) ** 2)
        # print("last_distance: ",last_distance)
        # print("distance: ",distance)
        # print("last_pos: ",lx," ",ly)
        # print("cur_pos: ",x," ",y)
        if(last_distance >= self.dist_threshold and distance < self.dist_threshold and self.away_count[self.id] == self.close_count[self.id]):
            self.close_count[self.id] = self.close_count[self.id] + 1
            print("id: ",self.id, ' close_count: ', self.close_count[self.id])
        if(last_distance <= self.dist_threshold and distance >self.dist_threshold and self.away_count[self.id] == self.close_count[self.id]-1):
            self.away_count[self.id] = self.away_count[self.id] + 1
            print("id: ",self.id, ' away_count: ', self.away_count[self.id])
    def countingStart(self, x, y):
        x1 = self.ps[self.id].defalut_circle_point_x
        y1 = self.ps[self.id].defalut_circle_point_y
        if(self.has_track[self.id]==False):
            if(np.sqrt((  x1 - x) ** 2 + ( y1 - y) ** 2) < self.dist_threshold):
                self.has_track[self.id] = True
                print("start running id: ",self.id)



    def pose_callback(self, msg):
        self.pos_id = self.pos_id + 1
        if(self.pos_id==30):
            self.pos_id = 0
        last_id = self.pos_id + 1
        if(last_id==30):
            last_id = 0
        self.pos_list[self.pos_id] = msg.pose.position

        self.cur_pos = self.pos_list[self.pos_id]
        self.last_pos = self.pos_list[last_id]
        # print("cb-cur_pos: ",self.cur_pos.x," ",self.cur_pos.y)
        # print("cb-last_pos: ",self.last_pos.x," ",self.last_pos.y)

        
        
     

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        rate2 = rospy.Rate(2)  # 0.5s
        while not rospy.is_shutdown():
            if self.cur_pos is not None and self.last_pos is not None:
                self.countingStart(self.cur_pos.x, self.cur_pos.y)
                if(self.has_track[self.id]==True):
                    self.countingReach(self.cur_pos.x, self.cur_pos.y, self.last_pos.x,self.last_pos.y)
                    if (self.close_count[self.id]>=3 and self.away_count[self.id]>=3):
                        self.p1.terminate()
                        self.p2.terminate()
                        self.id = self.id+1
                        if(self.id==9):
                            p3 = subprocess.Popen('pkill mavros', shell=True)
                            p4 = subprocess.Popen('pkill X-Plane-x86_64', shell=True)
                            return
                        else:
                            rate2.sleep()
                            xcmd = 'des_px:='+str(self.ps[self.id].des_px)+\
                                ' des_py:='+str(self.ps[self.id].des_py)+\
                                ' des_angle:='+str(self.ps[self.id].des_angle) + \
                                ' defalut_circle_point_x:='+str(self.ps[self.id].defalut_circle_point_x) + \
                                ' defalut_circle_point_y:='+str(self.ps[self.id].defalut_circle_point_y)

                            cmd = 'roslaunch track_controller hil_single_track_spawn.launch '+ xcmd
                            self.p1 = subprocess.Popen(cmd, shell=True)
                            self.p2 = subprocess.Popen('rosbag record /dbg_msg', shell=True)
            rate2.sleep()
def main():

    rospy.init_node('auto_run', anonymous=True)

    auto_run = AutoRun()

    # 运行主循环
    auto_run.run()

if __name__ == '__main__':
    main()




