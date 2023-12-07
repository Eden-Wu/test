import URBasic
import time
import CXpose as p
import math
import imutils
from imutils.video import VideoStream
import cv2


def move_at_designed_path(C):
    robot.movej(pose=C, a=ACCELERATION, v=VELOCITY, r=0)

# 1. settings and  variables
ROBOT_IP = '127.0.0.1'
# ROBOT_IP = '192.168.1.102'
ACCELERATION = 0.5  # Robot acceleration value
VELOCITY = 0.3  # Robot speed value


C = p.C
C_N = P.C_N

ur3_home = (math.radians(0),
                    math.radians(-90),
                    math.radians(0),
                    math.radians(-90),
                    math.radians(0),
                    math.radians(0))

robot_startposition = (math.radians(-19.21),
                    math.radians(-43.26),
                    math.radians(-137.82),
                    math.radians(-0.92),
                    math.radians(18.66),
                    math.radians(1.59))

robotModel = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP,robotModel=robotModel)

robot.reset_error()
print("robot initialised")
time.sleep(1)

robot.init_realtime_control()  # starts the realtime control loop on the Universal-Robot Controller
time.sleep(1) # just a short wait to make sure everything is initialised

# 2. 开始录制视频
# 设置视频捕获设备，0通常是默认的内置摄像头
cap = cv2.VideoCapture(0)

# 定义视频编码器和创建VideoWriter对象
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('facepose_collection.avi', fourcc, 30.0, (640, 480))

# 检查摄像头是否成功打开
if not cap.isOpened():
    print("摄像头无法打开")
    exit()

while True:
    # 逐帧捕获
    ret, frame = cap.read()
    # 如果正确读取帧，ret为True
    if not ret:
        print("无法读取摄像头画面")
        break
    # 将捕获的帧写入输出文件
    out.write(frame)
    # 显示帧
    cv2.imshow('frame', frame)
    # 按下'q'键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 3.ur3开始走规划路径
tmp = 0
print("开始走规划路径")
for i in C:
    move_at_designed_path(i)
    print("current 到 "+ C_N[tmp])
    tmp = tmp + 1

print("face collectio is done, moving UR to home position")

# 4.程序结束，关闭摄像头，ur3,回到 homeposition

# 释放摄像头资源
cap.release()
out.release()
cv2.destroyAllWindows()

robot.movej(q=ur3_home, a= ACCELERATION, v= VELOCITY)
print("UR is back to home position")

robot.close()

