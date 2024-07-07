#!/usr/bin/env python
# coding:utf-8
import math
import numpy as np
import time
import cv2 as cv
from common import *
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server
from yahboomcar_laser.cfg import laserAvoidPIDConfig
from Rosmaster_Lib import Rosmaster
from std_msgs.msg import String
import heapq
RAD2DEG = 180 / math.pi

class laser_reader():
    def __init__(self):
        rospy.init_node('laser_reader', anonymous=False)
        self.left_flag = 0
        self.right_flag = 0
        self.front_flag = 0
        self.back_flag = 0
        
    def get_message(self):
        sub_laser = rospy.wait_for_message('/scan_anwser', String).data
        self.left_flag = int(sub_laser[1])
        self.right_flag = int(sub_laser[2])
        self.front_flag = int(sub_laser[0])
        self.back_flag = int(sub_laser[3])
        return sub_laser
    
    def check_left(self):
        self.get_message()
        if self.left_flag == 1:
            print("something on the left")
        return self.left_flag

    def check_right(self):
        self.get_message()
        if self.right_flag == 1:
            print("something on the right")
        return self.right_flag

    def check_front(self):
        self.get_message()
        if self.front_flag == 1:
            print("something front of the car")
        return self.front_flag

    def check_back(self):
        self.get_message()
        if self.back_flag == 1:
            print("something back of the car")
        return self.back_flag

    def cancel(self):
        self.ros_ctrl.pub_vel.publish(Twist())
        self.ros_ctrl.cancel()
        self.sub_laser.unregister()
        rospy.loginfo("Shutting down this node.")

def turn_left_motor(bot):
    st  = time.time()
    bot.set_car_motion(0,0,0.6)
    while time.time()-st<4.5:
        a =1
    bot.set_car_motion(0,0,0)


def turn_right_motor(bot):
    st  = time.time()
    bot.set_car_motion(0,0,-0.6)
    while time.time()-st<4.5:
        a =1
    bot.set_car_motion(0,0,0)


def go_forward(bot):
    st  = time.time()
    bot.set_car_motion(0.21,0,0)
    while time.time()-st<3:
        a =1
    bot.set_car_motion(0,0,0)

def go_back_motor(bot):
    st  = time.time()
    bot.set_car_motion(-0.21,0,0)
    while time.time()-st<3:
        a =1
    bot.set_car_motion(0,0,0)

class Graph():
    def __init__(self, length, width):
        self.length = length
        self.width = width
        self.graph = np.zeros((length, width), dtype=int) # 0 means empty, 1 means obstacle
        self.visited = np.zeros((length, width), dtype=int)
        self.checked = np.zeros((length, width), dtype=int)

    def set_obstacle(self, x, y):
        self.graph[x, y] = 1

    def set_visited(self, x, y):
        self.visited[x, y] = 1

    def set_checked(self, x, y):
        self.checked[x, y] = 1

    def if_visited(self, x, y):
        return self.visited[x, y]

    def if_checked(self, x, y):
        return self.checked[x, y]

    def heuristic(self, a, b):
        # Using Manhattan distance as heuristic
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def all_checked(self):
        np.all(self.checked == 1)

    def a_star_search(self, start, goal):
        # Priority queue for the open set
        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {node: float('inf') for node in np.ndindex(self.graph.shape)}
        g_score[start] = 0
        f_score = {node: float('inf') for node in np.ndindex(self.graph.shape)}
        f_score[start] = self.heuristic(start, goal)

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1

                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    if not self.if_checked(neighbor[0], neighbor[1]):
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        self.set_checked(neighbor[0], neighbor[1])

        return []

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def get_neighbors(self, node):
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            x, y = node[0] + dx, node[1] + dy
            if 0 <= x < self.length and 0 <= y < self.width and self.graph[x, y] == 0:
                if self.graph[x, y] == 0:
                    neighbors.append((x, y))
        return neighbors 

    def refresh_check(self):
        self.checked = np.zeros((self.length, self.width), dtype=int)

class Car():
    def __init__(self, current, direction):
        self.current = current
        self.direction = direction
        
    def go(self, graph, bot, cap):
        go_forward_adjust(bot, cap)
        self.current = [self.current[0] + self.direction[0], self.current[1] + self.direction[1]]
        graph.set_visited(self.current[0], self.current[1])
        print(self.current)
        return self.current

    def go_back(self, graph, bot):
        go_back_motor(bot)
        self.current = [self.current[0] - self.direction[0], self.current[1] - self.direction[1]]
        graph.set_visited(self.current[0], self.current[1])
        return self.current

    def turn_left(self, bot):
        turn_left_motor(bot)
        if self.direction == [0, 1]:
            self.direction = [-1, 0]
        elif self.direction == [-1, 0]:
            self.direction = [0, -1]
        elif self.direction == [0, -1]:
            self.direction = [1, 0]
        else:
            self.direction = [0, 1]
        return self.direction
    
    def turn_right(self, bot):
        turn_right_motor(bot)
        if self.direction == [0, 1]:
            self.direction = [1, 0]
        elif self.direction == [1, 0]:
            self.direction = [0, -1]
        elif self.direction == [0, -1]:
            self.direction = [-1, 0]
        else:
            self.direction = [0, 1]
        return self.direction

    def space_front(self):
        return [self.current[0] + self.direction[0], self.current[1] + self.direction[1]]

    def space_left(self):
        if self.direction == [0, 1]:
            return [self.current[0] - 1, self.current[1]]
        elif self.direction == [-1, 0]:
            return [self.current[0], self.current[1] - 1]
        elif self.direction == [0, -1]:
            return [self.current[0] + 1, self.current[1]]
        else:
            return [self.current[0], self.current[1] + 1]
    
    def space_right(self):
        if self.direction == [0, 1]:
            return [self.current[0] + 1, self.current[1]]
        elif self.direction == [-1, 0]:
            return [self.current[0], self.current[1] + 1]
        elif self.direction == [0, -1]:
            return [self.current[0] - 1, self.current[1]]
        else:
            return [self.current[0], self.current[1] - 1]

    def go_path(self, path, bot, cap, pid):
        for coordinate in path:
            delta = [coordinate[0] - self.current[0], coordinate[1] - self.current[1]]
            if delta == self.direction:
                # go forward
                cali_line(cap, pid, bot)
                self.go(graph, bot, cap)
            elif delta == [-self.direction[0], -self.direction[1]]:
                # go back
                cali_line(cap, pid, bot)
                self.go_back(graph, bot)
            elif (delta == [-1, 0] and self.direction == [0, 1]) \
                or (delta == [1, 0] and self.direction == [0, -1]) \
                or (delta == [0, -1] and self.direction == [-1, 0]) \
                or (delta == [0, 1] and self.direction == [1, 0]):
                # turn left
                self.turn_left(bot)
                cali_line(cap, pid, bot)
                self.go(graph, bot, cap)
            elif (delta == [1, 0] and self.direction == [0, 1]) \
                or (delta == [-1, 0] and self.direction == [0, -1]) \
                or (delta == [0, 1] and self.direction == [-1, 0]) \
                or (delta == [0, -1] and self.direction == [1, 0]):
                # turn right
                self.turn_right(bot)
                cali_line(cap, pid, bot)
                self.go(graph, bot, cap)
            else:
                print("Error in path")
                break
                


        
    def check_around(self, graph, laser, bot):
        # time.sleep(1)
        obstacle = []
        empty = []
        if laser.check_front():
            obstacle.append(self.space_front())
        else:
            empty.append(self.space_front())
        if laser.check_left():
            obstacle.append(self.space_left())
        else:
            empty.append(self.space_left())
        if laser.check_right():
            obstacle.append(self.space_right())
        else:
            empty.append(self.space_right())
        # self.turn_left(bot)
        # if laser.check_left():
        #     obstacle.append(self.space_left())
        # else:
        #     empty.append(self.space_left())
        # self.turn_right(bot)

        i = 0
        while i < len(obstacle):
            if obstacle[i][0] < 0 or obstacle[i][1] < 0 or \
                obstacle[i][0] >= graph.length or obstacle[i][1] >= graph.width:
                obstacle.remove(obstacle[i])
                i -= 1
            else:
                print('set obstacle:', obstacle[i][0],' ', obstacle[i][1])
                graph.set_obstacle(obstacle[i][0], obstacle[i][1])
                graph.set_checked(obstacle[i][0], obstacle[i][1])
            i += 1

        i = 0
        while i < len(empty):
            if empty[i][0] < 0 or empty[i][1] < 0 or \
                empty[i][0] >= graph.length or empty[i][1] >= graph.width or \
                graph.if_visited(empty[i][0], empty[i][1]) == 1 or graph.graph[empty[i][0]][empty[i][1]] == 1:
                empty.remove(empty[i])
                i -= 1
            else:
                graph.set_checked(empty[i][0], empty[i][1])
            i += 1
        return empty, obstacle


def DFS(car, graph, laser, bot, cap, pid):
    length = graph.length
    width = graph.width

    cross_node = []
    action_stack = []

    flag = 0
    while not car.current == [- 1,- 1]:
        # time.sleep(1)
        if car.current == [length - 1, width - 1]:
            ret,frame = cap.read()
            print(frame.shape)
            cv.imwrite("1.jpg",frame)
            time.sleep(10)
            flag = 1

        if flag == 1 and graph.all_checked():
            print("all checked")
            return
        empty, obstacle= car.check_around(graph, laser, bot)
        # empty must be near the car with onlt step 1 
        if len(empty) >= 2:
            # at least have two empty space nearby so we should record the cross node
            cross_node.append(car.current)
            print(empty)
            # go the first empty space to go, and record the action
            if empty[0] == car.space_front():
                cali_line(cap, pid, bot)
                car.go(graph, bot, cap)
                action_stack.append("go")
            elif empty[0] == car.space_left():
                car.turn_left(bot)
                cali_line(cap, pid, bot)
                car.go(graph, bot, cap)
                action_stack.append("left")
            elif empty[0] == car.space_right():
                car.turn_right(bot)
                cali_line(cap, pid, bot)
                car.go(graph, bot, cap)
                action_stack.append("right")
        elif len(empty) == 0:
            # no empty space nearby so we should go back to the cross node
            if not cross_node:
                # no place to go
                print("No wat to go to")
                print(car.current)
                return
            target = cross_node.pop()
            if target == car.current:
                print("No wat to go to")
                return


            while car.current != target:
                print("current and target")
                print(car.current)
                print(target)
                if action_stack[-1] == "left":
                    car.go_back(graph, bot)
                    car.turn_right(bot)
                    cali_line(cap, pid, bot)
                    action_stack.pop()
                elif action_stack[-1] == "right":
                    car.go_back(graph, bot)
                    car.turn_left(bot)
                    cali_line(cap, pid, bot)
                    action_stack.pop()
                elif action_stack[-1] == "go":
                    car.go_back(graph, bot)
                    cali_line(cap, pid, bot)
                    action_stack.pop()
        else:
            # only one way to go
            if empty[0] == car.space_front():
                cali_line(cap, pid, bot)
                car.go(graph, bot, cap)
                action_stack.append("go")
            elif empty[0] == car.space_left():
                car.turn_left(bot)
                cali_line(cap, pid, bot)
                car.go(graph, bot, cap)
                action_stack.append("left")
            elif empty[0] == car.space_right():
                car.turn_right(bot)
                cali_line(cap, pid, bot)
                car.go(graph, bot, cap)
                action_stack.append("right")




def get_dir(img):
    # cv.imshow("img",img)
    # 应用高斯模糊以减少噪声
    img = cv.GaussianBlur(img, (5, 5), 0)

    # 创建 img 的副本 img1
    img1 = img.copy()

    # 将上半部分设置为黑色
    img1[:280, :] = 0

    # 截取下半部分
    img_lower = img[280:640, :]

    # 转换为灰度图
    gray_lower = cv.cvtColor(img_lower, cv.COLOR_BGR2GRAY)
    #cv.imshow("Gray Image", gray_lower)

    # 使用大津法进行二值化

    ret, binary_lower = cv.threshold(gray_lower, 40, 255, cv.THRESH_BINARY)
    #cv.imshow("111",binary_lower)
    binary_lower = cv.bitwise_not(binary_lower)

    # 创建核进行膨胀和腐蚀处理
    kernel = np.ones((3,3), np.uint8)

    # 腐蚀处理
    eroded = cv.erode(binary_lower, kernel, iterations=1)
    


    # cv.imshow('bin',binary_lower)
    # 提取轮廓
    _, contours, _ = cv.findContours(eroded, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    if(len(contours)==0):
        print("no line")
        return 320,0
    
    #filter_contours = filter_contours_by_aspect_ratio(contours,0.6,10000)
    
    # 找到最大的轮廓
    max_contour = max(contours, key=cv.contourArea)

    # 创建一个与 img_lower 相同大小的黑色图像
    mask = np.zeros_like(eroded)

    # 绘制最大的轮廓，将其填充为白色
    cv.drawContours(mask, [max_contour], -1, 255, thickness=cv.FILLED)

    # 将掩码应用于原始图像的下半部分
    white_fill = np.full_like(img_lower, 255)
    img_lower_masked = cv.bitwise_and(white_fill, white_fill, mask=mask)

    # 将处理后的结果放回原图像副本的相应位置
    img1[280:640, :] = img_lower_masked

    # 腐蚀处理
    img1 = cv.erode(img1, kernel, iterations=2)

    sorted_contour = sorted(max_contour, key=lambda point: point[0][1])

    points_by_y = {}
    for point in sorted_contour:
        x, y = point[0]
        if y not in points_by_y:
            points_by_y[y] = []
        points_by_y[y].append(x)

    direction = []
    for y, x_values in points_by_y.items():
        x_values.sort()
        for i in range(0, len(x_values), 3):
            segment = x_values[i:i+5]
            avg_x = int(np.mean(segment))
            cv.circle(img1, (avg_x, y+280), 3, (0, 0, 255), -1)
            direction.append(avg_x)

    direction_out = np.mean(direction)
    return direction_out,img1


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def calculate(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
    
        if output>50:
            output = 50
        if output <-50:
            output = -50
        if error > 200:
            output = 60
        if error < -200:
            output = -60
        if output<0:
            output = output - 13
        else:
            output = output + 13
        print("output",output)
        return output


def control_bot(center_x, width, bot, pid, prev_time):
    # print("control_bot")
    # 根据黑线的中心位置调整小车运动方向
    error = center_x - width // 2
    current_time = time.time()
    dt = current_time - prev_time
    correction = pid.calculate(error, dt)

    left_speed = 0 + correction
    right_speed = 0 - correction

    bot.set_motor(left_speed, left_speed, right_speed, right_speed)  # 调整方向

    return current_time

def cali_control_bot(center_x, width, bot, pid, prev_time):
    # 根据黑线的中心位置调整小车运动方向
    error = center_x - width // 2
    
    error = error/60

    if error < 0:
        error -= 0.15
    else:
        error += 0.15
    
    if abs(error)<0.2:
        error = 0
    #print(error)
    if error>0:
        bot.set_car_motion(0,0,-0.4)
        # print("left! adjust!!!")
    elif error<0:
        bot.set_car_motion(0,0,0.4)
        # print("right! adjust!!!")
    if error == 0:
        bot.set_car_motion(0,0,0)
    
    return 0

def cali_line(cap, pid, bot):
    print("start_cali_to_the_line")
    cali_start = time.time()
    count = 0
    while time.time()-cali_start<5:
        ret, frame = cap.read()
        # print(frame.shape)
        #cv.imwrite("1.png", frame)

        if not ret:
            print('break')
            break
        center_x, binary_frame = get_dir(frame)
        cali_time = cali_control_bot(center_x, frame.shape[1], bot, pid, cali_start)
        # cv.imshow('img',frame)
        # cv.imshow('binary', binary_frame)
       
        if cv.waitKey(1) & 0xFF == ord(' '):
            break
    bot.set_car_motion(0,0,0)

def cali_control_bot1(center_x, width, bot, pid, prev_time):
    # 根据黑线的中心位置调整小车运动方向
    error = center_x - width // 2
    
    error = error/60

    if error < 0:
        error -= 0.15
    else:
        error += 0.15
    
    if abs(error)<0.2:
        error = 0
    #print(error)
    if error>0:
        bot.set_car_motion(0.21,0,-0.3)
        # print("left! adjust!!!")
    elif error<0:
        bot.set_car_motion(0.21,0,0.3)
        # print("right! adjust!!!")
    if error == 0:
        bot.set_car_motion(0.21,0,0)
    
    return 0
       
def go_forward_adjust(bot,cap):
    # print("gogogo!!!")
    cali_start = time.time()
    count = 0
    while time.time()-cali_start<3:
        ret, frame = cap.read()
        # print(frame.shape)
        #cv.imwrite("1.png", frame)

        if not ret:
            print('break')
            break
        center_x, binary_frame = get_dir(frame)
        cali_time = cali_control_bot1(center_x, frame.shape[1], bot, pid, cali_start)
        # cv.imshow('img',frame)
        # cv.imshow('binary', binary_frame)
       
        if cv.waitKey(1) & 0xFF == ord(' '):
            break
    bot.set_car_motion(0,0,0)


def filter_contours_by_aspect_ratio(contours, min_aspect_ratio=0.2, max_aspect_ratio=5.0):
    filtered_contours = []
    for contour in contours:
        x, y, w, h = cv.boundingRect(contour)
        aspect_ratio = w / float(h)
        print(aspect_ratio)
        if min_aspect_ratio < aspect_ratio < max_aspect_ratio:
            filtered_contours.append(contour)
    return filtered_contours

def draw_contours(image, contours):
    for contour in contours:
        x, y, w, h = cv.boundingRect(contour)
        cv.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    return image


if __name__ == '__main__':
    tracker = laser_reader()
    bot = Rosmaster()
    pid = PIDController(Kp=0.20, Ki=0.0, Kd=0.0)

    cap = cv.VideoCapture(1)
    ret,frame = cap.read()
    print(frame.shape)
    cv.imwrite('1.jpg', frame)
    # cap.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','J', 'P', 'G'))
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    graph = Graph(4, 4)
    car = Car([0, 0], [0, 1])
    cali_line(cap, pid, bot)
    print('DFS')
    DFS(car, graph, tracker, bot, cap, pid)
    graph.refresh_check()
    print(graph.graph)
    print(graph.checked)
    print('DFS-------------')
    print(car.current[0],car.current[1])
    path = graph.a_star_search((car.current[0],car.current[1]), (0, 0))
    print(path)
    car.go_path(path[1:], bot, cap, pid) 
    print('done')
    cap.release()
    cv.destroyAllWindows()
