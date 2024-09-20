#! /usr/bin/env python3

import rospy
import time
import math
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from fusion.msg import YoloTrafficCone, clustering_points

numpy_int64 = np.int64(42)

class create_matrix:
    def __init__(self,params_cam,params_lidar):

        self.params_cam = params_cam
        self.params_lidar = params_lidar
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        
        global bridge
        bridge = CvBridge()
        global RT
        RT = self.transformMTX_lidar2cam(self.params_lidar, self.params_cam)
        global proj_mtx
        proj_mtx = self.project2img_mtx(self.params_cam)

    def transformMTX_lidar2cam(self, params_lidar, params_cam):

        lidar_pos = [params_lidar.get(i) for i in (["X","Y","Z"])]
        cam_pos = [params_cam.get(i) for i in (["X","Y","Z"])]

        x_rel = cam_pos[0] - lidar_pos[0]
        y_rel = cam_pos[1] - lidar_pos[1]
        z_rel = cam_pos[2] - lidar_pos[2]

         # 기존값
        R_T = np.matmul(self.rotationMtx(0., 0., 0.),self.translationMtx(x_rel, y_rel, z_rel))
        R_T = np.matmul(self.rotationMtx(np.deg2rad(90.), 0., 0.), R_T )
        R_T = np.matmul(self.rotationMtx(0., 0., np.deg2rad(90.)), R_T )
        R_T = np.matmul(self.rotationMtx(0., np.deg2rad(145.), 0.), R_T ) # 왼쪽 방향으로 카메라 틈
        R_T = np.matmul(self.rotationMtx(0.,0., np.deg2rad(29.)), R_T ) # 아래 방향으로 카메라 틈
        

        # R_T = np.linalg.inv(R_T) #역행렬

        return R_T 


    def translationMtx(self,x, y, z):
        M = np.array([[1,   0,  0,  -x],
                      [0,   1,  0,  -y],
                      [0,   0,  1,  -z],
                      [0,   0,  0,  1],
                      ])
        return M

    def rotationMtx(self,yaw, pitch, roll):
        R_x = np.array([[1,     0,              0,                  0],
                        [0,     math.cos(roll), -math.sin(roll),    0],
                        [0,     math.sin(roll),  math.cos(roll),    0],
                        [0,     0,              0,                  1], 
                        ])
        
        R_y = np.array([[math.cos(pitch),    0,  math.sin(pitch),     0],
                        [0,                 1,  0,                  0],
                        [-math.sin(pitch),   0,  math.cos(pitch),     0],
                        [0,                 0,  0,                  1], 
                        ])
        
        R_z = np.array([[math.cos(yaw),     -math.sin(yaw),  0,  0],
                        [math.sin(yaw),     math.cos(yaw),  0,  0],
                        [0,                 0,              1,  0],
                        [0,                 0,              0,  1], 
                        ])
        R = np.matmul(R_x, np.matmul(R_y, R_z))

        return R
    
    def project2img_mtx(self, params_cam):
 
        #1280 X 720 
        #카메라 3
        # fc_x = 950
        # fc_y = 950
        # cx = 610
        # cy = 330
        
        #카메라 1 왼쪽 바탕
        # fc_x = 880
        # fc_y = 880
        # cx = 600
        # cy = 340
        
        
        
        #640 X 480
        #카메라 2
        # fc_x = 640
        # fc_y = 640
        # cx = 311
        # cy = 232
        
        #카메라 1
        # fc_x = 630l
        # fc_y = 630
        # cx = 288
        # cy = 232
        
        #카메라 3
        # fc_x = 630
        # fc_y = 630
        # cx = 311
        # cy = 220.5

        # # morai
        # fc_x = 320
        # fc_y = 320
        # cx = 320
        # cy = 240
        
        # kcity 
        # fc_x = 719.1834
        # fc_y = 706.52499
        # cx = 199.1
        # cy = 294.2903

        # fmtc
        fc_x = 629.67368
        fc_y = 630.7475
        cx = 316.75903
        cy = 246.84972

        R_f = np.array([[fc_x, 0,    cx],
                        [0,    fc_y, cy]])

        return R_f




class LIDAR2CAMTransform:
    def __init__(self,params_cam,params_lidar):
        self.params_cam = params_cam
        self.params_lidar = params_lidar
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        
        self.image_pub = rospy.Publisher("/sensor_fusion_result_right",Image,queue_size= 1)
        self.clustering_point_pub_right = rospy.Publisher("/final_point_clustering_right",clustering_points,queue_size= 1)
        self.yolo_sub = rospy.Subscriber("/traffic_cone_yolo_right_result",Image,self.callback_image)
        self.cluster_sub = rospy.Subscriber("/cluster",PointCloud2,self.callback_lidar)
        self.bbox_sub = rospy.Subscriber("/traffic_cone_bbox_class_right",YoloTrafficCone,self.callback_bbox)
        
        
    def callback_image(self,msg):        
        start_image = time.time()
        self.frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if 'point_x' not in globals():
            print("right, global variable 'center : point_x' is not defined")
            return
        
        self.camera_frame = self.draw_pts_img(self.frame, point_x, point_y)
        self.image_pub.publish(bridge.cv2_to_imgmsg(self.camera_frame, "bgr8"))

        end_image = time.time()
        # print(f"callback_image : {end_image - start_image:.5f} sec")
        

    def callback_bbox(self,msg):
        start_bbox = time.time()
        
        if 'point_x' not in globals():
            print("right, global variable 'center_bbox : point_x' is not defined")
            return

        cluster_msg = clustering_points()
        if msg.boxes == []:
            cluster_msg.x = []
            cluster_msg.y = []
            cluster_msg.label = []
        
        elif len(point_x) == 0:
            
            cluster_msg.x = []
            cluster_msg.y = []
            cluster_msg.label = []
            
        else:
            
            point_check = []
            
            data = np.array(msg.boxes)
            bounding_boxes = []

            for i in range(0, len(data), 5):
                bounding_box = data[i:i+5].tolist()
                bounding_boxes.append(bounding_box)
            
            #바운딩 박스를 하나씩 탐색
            for box in bounding_boxes:
                # box = [int(x1), int(y1), int(x2), int(y2), class] 이 형태로 들어옴
                min_distance = float('inf')
                min_idx = -1
                # print("class : ", box[-1])
                # clustering point를 하나씩 탐색
                for j in range(len(point_x)):
                    if ((j not in point_check) and (box[0] <= point_x[j]) and (point_x[j] <= box[2]) and (box[1] <= point_y[j]) and (point_y[j] <= box[3])):
                    # if(j not in point_check):
                        centerx = (box[0] + box[2]) / 2
                        centery = (box[1] + box[3]) / 2
                        img2bbox_distance = math.sqrt(pow(centerx - point_x[j],2) + pow(centery - point_y[j],2))
                        if(min_distance > img2bbox_distance):
                                min_distance = img2bbox_distance
                                min_idx = j
                
                # 최적의 point 탐색 했다면
                if min_idx != -1: 

                    if len(np.where(xyi_compare[0] == point_x[min_idx])[0]) == 0:
                        print("center : 역변환 실패")
                    else:
                        point_check.append(min_idx)
                        
                        cluster_msg.label.append(box[4])                
                        reverse_index = np.where(xyi_compare == point_x[min_idx])[1][0]
                        reverse_zc = zc[0][reverse_index]
                        reverse_xc = xc[0][reverse_index]
                        reverse_yc = yc[0][reverse_index]
                        reverse_xyz_c = np.array([[reverse_xc , reverse_yc, reverse_zc, 1.]])
                        reverse_xyz_p = np.matmul(reverse_xyz_c,np.linalg.inv(RT.T))
                        cluster_msg.x.append(reverse_xyz_p[0][0])
                        cluster_msg.y.append(reverse_xyz_p[0][1])
                # cam_frame = self.draw_pts_img1(self.global_frame, point_x[min_idx], point_y[min_idx])
                # self.image_pub.publish(bridge.cv2_to_imgmsg(cam_frame, "bgr8"))
            # bounding box하고 point는 있지만, 아무것도 일치가 되지 않았을 경우            
            if (len(cluster_msg.x) == 0):
                cluster_msg.x = []
                cluster_msg.y = []
                cluster_msg.label = []                
        self.clustering_point_pub_right.publish(cluster_msg)
            
        end_bbox = time.time()
        # print(f"callback_bbox : {end_bbox - start_bbox:.5f} sec")
        
    def callback_lidar(self, msg):
        global point_x, point_y
        if msg.data == b'':
            point_x = []
            point_y = []
            return
    

        self.point_lidar = self.pointxyz_to_xyz(msg) #pcl -> numpy
        # print("msg : ", self.point_lidar)

        self.point_camera = self.transform_lidar2cam(self.point_lidar) #extrinsic parameter를 이용한 좌표계 변환
        
        self.point_image = self.project_pts2img(self.point_camera) #intrinsic parameter를 이용한 좌표계 변환 + 이미지 이외의 점들 crop하기
        # self.camera_frame = self.camera(self.point_image)
        # self.image_pub.publish(bridge.cv2_to_imgmsg(self.point_image, "bgr8"))

    def pointxyz_to_xyz(self,cloud_msg):
        
        point_cloud = np.array(list(pc2.read_points(cloud_msg, skip_nans=True)))
        point_cloud = point_cloud[:, :3]
        return point_cloud
    
    def transform_lidar2cam(self, xyz_p):
        
        xyz_c = np.matmul(np.concatenate([xyz_p, np.ones((xyz_p.shape[0], 1))], axis = 1), RT.T)
        xyz_c = xyz_c[xyz_c[:, 2] >= 0]
        return xyz_c

    def project_pts2img(self, xyz_c):
        
        #카메라 좌표계(3차원)
        xyz_c = xyz_c.T
        global xc
        global yc
        global zc
        xc, yc, zc = xyz_c[0, :].reshape([1,-1]), xyz_c[1,:].reshape([1,-1]), xyz_c[2,:].reshape([1,-1])
        
        #정규좌표계
        xn, yn = xc/(zc+0.000001), yc/(zc+0.000001)
        #카메라 이미자상 좌표계(2차원)
        xyi = np.matmul(proj_mtx, np.concatenate([xn,yn, np.ones_like(xn)], axis = 0))
        
        ## 비교할 대상
        global xyi_compare
        xyi_compare = xyi
        xyi = xyi.T 
        
        xyi = self.crop_pts(xyi) # 이미지 안에만 있는 점들로 짜르기
        
        xyz_i = xyi
        global point_x
        global point_y
        xi, yi = xyz_i[:, 0].reshape([1,-1]), xyz_i[:,1].reshape([1,-1])
        ## xyi_compare 과 비교
        point_x = xi.flatten()
        point_y = yi.flatten()
        return xyi


    def crop_pts(self, xyi):
        
        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:, 0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:, 1]<self.height), :]
        return xyi

    # def camera(self, xyz_i):
    #     global point_x
    #     global point_y
    #     xi, yi = xyz_i[:, 0].reshape([1,-1]), xyz_i[:,1].reshape([1,-1])
    #     ## xyi_compare 과 비교
    #     point_x = xi.flatten()
    #     point_y = yi.flatten()
        
    def draw_pts_img(self, img, xi, yi):

        frame = img
        # cv2.imshow("frame ", frame)
        
        for x,y in zip(xi,yi):
            center = (int(x), int(y))
            # print("coordi : ",sum(xi)/len(xi), ", ", sum(yi)/len(yi))
            frame = cv2.circle(frame, center, 1, (0,0,255),-1)

        return frame

if __name__ == '__main__':
    rospy.init_node("webcam_pub" , anonymous = True)
    params_cam = {"WIDTH": 640, "HEIGHT": 480, "FOV": 78, "X": -1.16, "Y": -0.21, "Z": 0.525}
    # params_lidar = {"X": 0, "Y": 0, "Z": 0.91}
    params_lidar = {"X": 0, "Y": 0, "Z": 0}
    # params_cam = {"WIDTH": 640, "HEIGHT": 480, "FOV": 78, "X": -1.88, "Y": -0.27, "Z": 0.3} # morai
    prepare_matrix = create_matrix(params_cam,params_lidar)
    lidar_camera_calibration = LIDAR2CAMTransform(params_cam,params_lidar)
    
    rospy.spin()