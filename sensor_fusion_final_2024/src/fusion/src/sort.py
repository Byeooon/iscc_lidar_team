#! /usr/bin/env python3

import rospy
import numpy as np
from sklearn.cluster import DBSCAN
from fusion.msg import YoloTrafficCone, clustering_points
# from lidar_camera.msg import detect_cone 원래 
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class SortPoints():
    def __init__(self):
        rospy.init_node("sort",anonymous=True)
        self.left = []
        self.right = []
        self.center = []
        self.conePub= rospy.Publisher("/perfect_cone",clustering_points,queue_size= 1)
        # self.yellow_markPub = rospy.Publisher("/rabacon_mark_yellow",MarkerArray,queue_size=1)
        # self.blue_markPub = rospy.Publisher("/rabacon_mark_blue",MarkerArray,queue_size=1)
        rospy.Subscriber("/final_point_clustering_left", clustering_points, self.left_callback)
        # rospy.Subscriber("/final_point_clustering_right", clustering_points, self.right_callback)
        rospy.Subscriber("/final_point_clustering_center", clustering_points, self.center_callback)
        print("초기화")
        
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            # print("-----------------")
            cone_msg = clustering_points()
            cone_msg.x = []
            cone_msg.y = []
            cone_msg.label = []

            # cone_msg = detect_cone()

            yellow_rabacon_x = []
            yellow_rabacon_y = []
            blue_rabacon_x = []
            blue_rabacon_y = []
        
            
            # if left이면
            if self.left != []:
                for i in range(len(self.left.x)):
                    if self.left.label[i] == 0.0:
                        yellow_rabacon_x.append(self.left.x[i])
                        yellow_rabacon_y.append(self.left.y[i])
                    elif self.left.label[i] == 1.0:
                        blue_rabacon_x.append(self.left.x[i]) 
                        blue_rabacon_y.append(self.left.y[i])
            # print("left yellow_rabacon_x : ", yellow_rabacon_x)
            # print("left yellow_rabacon_y : ", yellow_rabacon_y)

            # if center이면
            if self.center != []:
                for i in range(len(self.center.x)):
                    if self.center.label[i] == 0.0:
                        yellow_rabacon_x.append(self.center.x[i])
                        yellow_rabacon_y.append(self.center.y[i])
                    elif self.center.label[i] == 1.0:
                        blue_rabacon_x.append(self.center.x[i]) 
                        blue_rabacon_y.append(self.center.y[i])

            # print("center yellow_rabacon_x : ", yellow_rabacon_x)
            # print("center yellow_rabacon_y : ", yellow_rabacon_y)

            # if right이면     
            if self.right != []:
                for i in range(len(self.right.x)):
                    if self.right.label[i] == 0.0:
                        yellow_rabacon_x.append(self.right.x[i])
                        yellow_rabacon_y.append(self.right.y[i])
                    elif self.right.label[i] == 1.0:
                        blue_rabacon_x.append(self.right.x[i])
                        blue_rabacon_y.append(self.right.y[i])
            # print("right yellow_rabacon_x : ", yellow_rabacon_x)
            # print("right yellow_rabacon_y : ", yellow_rabacon_y)
            
            
        #     #중복 제거
            yellow_rabacon_x = list(set(yellow_rabacon_x))
            yellow_rabacon_y = list(set(yellow_rabacon_y))
            blue_rabacon_x = list(set(blue_rabacon_x))
            blue_rabacon_y = list(set(blue_rabacon_y))
            
            #가까운 점들끼리 묶음 
            if len(yellow_rabacon_x) > 0:
                yellow_rabacon_x, yellow_rabacon_y = self.clustering(yellow_rabacon_x,yellow_rabacon_y)                
            if len(blue_rabacon_x) > 0:
                blue_rabacon_x, blue_rabacon_y = self.clustering(blue_rabacon_x,blue_rabacon_y)    
           
           
            # x 기준으로 정렬해서 쌓음
            # print("yellow_Rabacon_x ", yellow_rabacon_x)
            # print("yellow_Rabacon_y ", yellow_rabacon_y)

            # print("blue_Rabacon_x ", blue_rabacon_x)
            # print("blue_Rabacon_y ", blue_rabacon_y)
            concat_rabacon_x = yellow_rabacon_x + blue_rabacon_x
            concat_rabacon_y = yellow_rabacon_y + blue_rabacon_y
            concat_rabacon_label = []
            for i in range(len(yellow_rabacon_x)):
                concat_rabacon_label.append(0.0)
            for j in range(len(blue_rabacon_x)):
                concat_rabacon_label.append(1.0)

            # print("label !!!!!!! ", concat_rabacon_label)
            # print("concat _x ", concat_rabacon_x)
            # print("concat _y ", concat_rabacon_y)

            # print(concat_rabacon_x)
            # print(concat_rabacon_y)
            # print(concat_rabacon_label)

            for i in range(len(concat_rabacon_x)):
                cone_msg.x.append(concat_rabacon_x[i])
                cone_msg.y.append(concat_rabacon_y[i])
                cone_msg.label.append(concat_rabacon_label[i])

            if cone_msg.x != [] and cone_msg.y != [] and cone_msg.label != []:
                self.conePub.publish(cone_msg)

            # waypoint_marker_arr = MarkerArray()
            # for i in range(len(sorted_yellow_rabacon_x)):
            #     marker = Marker()
            #     marker.header.frame_id = "velodyne"
            #     marker.id = i
            #     marker.type = marker.CYLINDER
            #     marker.action = marker.ADD
            #     marker.scale.x = 0.1
            #     marker.scale.y = 0.1
            #     marker.scale.z = 0.05
            #     marker.color.a = 1.0
            #     marker.color.r = 1.0
            #     marker.color.g = 1.0
            #     marker.color.b = 0.0
            #     marker.pose.orientation.w = 1.0
            #     marker.pose.position.x = sorted_yellow_rabacon_x[i]
            #     marker.pose.position.y = sorted_yellow_rabacon_y[i]
            #     marker.pose.position.z = 0.2
            #     marker.lifetime = rospy.Duration(0.1)
            #     waypoint_marker_arr.markers.append(marker)
            # self.yellow_markPub.publish(waypoint_marker_arr)
            
            # waypoint_marker_arr = MarkerArray()
            # for j in range(len(sorted_blue_rabacon_x)):
            #     marker = Marker()
            #     marker.header.frame_id = "velodyne"
            #     marker.id = j
            #     marker.type = marker.CYLINDER
            #     marker.action = marker.ADD
            #     marker.scale.x = 0.1
            #     marker.scale.y = 0.1
            #     marker.scale.z = 0.05
            #     marker.color.a = 1.0
            #     marker.color.r = 0.0
            #     marker.color.g = 0.0
            #     marker.color.b = 1.0
            #     marker.pose.orientation.w = 1.0
            #     marker.pose.position.x = sorted_blue_rabacon_x[j]
            #     marker.pose.position.y = sorted_blue_rabacon_y[j]
            #     marker.pose.position.z = 0.2
            #     marker.lifetime = rospy.Duration(0.1)
            #     waypoint_marker_arr.markers.append(marker)
            # self.blue_markPub.publish(waypoint_marker_arr)
            
            rate.sleep()
        
            
    def left_callback(self, msg):
        self.left = msg

    def right_callback(self, msg):
        self.right = msg

    def center_callback(self, msg):

        self.center = msg

    def clustering(self,x,y):
        # 비슷하게 인식된 라바콘은 하나로 묶기
        data = np.array(list(zip(x, y)))
        # DBSCAN 클러스터링 모델 생성
        
        dbscan = DBSCAN(eps=0.5, min_samples=1)  # eps: 이웃 거리, min_samples: 최소 이웃 수
        # 데이터를 클러스터링합니다.
        dbscan.fit(data)
        # 클러스터링 결과 확인
        labels = dbscan.labels_  # 각 데이터 포인트의 클러스터 레이블
        # print("label = ",labels)
        # 각 클러스터의 핵심 포인트와 데이터 포인트 출력
        result_x = []  # 클러스터링 결과의 x 좌표를 저장할 리스트
        result_y = []  # 클러스터링 결과의 y 좌표를 저장할 리스트
        
        unique_labels = set(labels)
        for label in unique_labels:
            cluster_x = []  # 각 클러스터의 x 좌표를 저장할 리스트
            cluster_y = []  # 각 클러스터의 y 좌표를 저장할 리스트
            cluster_points = data[labels == label]
            cluster_x.extend(cluster_points[:,0])
            # print("cluster",cluster_x)
            cluster_y.extend(cluster_points[:,1])
            cluster_center_x = np.mean(cluster_x)  # 클러스터의 x 평균
            # print("mean = ",cluster_center_x)
            cluster_center_y = np.mean(cluster_y)  # 클러스터의 y 평균
            result_x.append(cluster_center_x)
            result_y.append(cluster_center_y)
        # print("result _ x : " , result_x)
        # print("result _ y : " , result_y)
        return result_x, result_y
        
if __name__ == '__main__':
    try:
        sorter = SortPoints()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

        