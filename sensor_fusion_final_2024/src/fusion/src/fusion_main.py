#!/usr/bin/python3

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
        
        global RT
        RT = self.transformMTX_lidar2cam(self.params_lidar, self.params_cam)

        global proj_mtx
        proj_mtx = self.project2img_mtx()


    def transformMTX_lidar2cam(self, params_lidar, params_cam):

        lidar_pos = [params_lidar.get(i) for i in (["X","Y","Z"])]
        cam_pos = [params_cam.get(i) for i in (["X","Y","Z"])]

        x_rel = cam_pos[0] - lidar_pos[0]
        y_rel = cam_pos[1] - lidar_pos[1]
        z_rel = cam_pos[2] - lidar_pos[2]

        # 잘된 케이스 : 라이다 오른쪽 뒤에 카메라를 설치하고 카메라는 왼쪽 아래를 봄 , 완벽함. 
        R_T = np.matmul( self.translationMtx(x_rel,y_rel, z_rel),self.rotationMtx(np.deg2rad(-90.), 0., 0.)) # 기본값
        R_T = np.matmul(R_T, self.rotationMtx(0., 0., np.deg2rad(-90.))) # 위아래 각도값 정면 

        # R_T = np.matmul(R_T, self.rotationMtx(0., np.deg2rad(-30.), 0.)) #좌측으로 틈  
        # R_T = np.matmul(R_T, self.rotationMtx(0., 0., np.deg2rad(-30.))) # 아래로 최대 내림 
        R_T = np.linalg.inv(R_T)

        return R_T


    def translationMtx(self,x, y, z):
        M = np.array([[1,   0,  0,  x],
                      [0,   1,  0,  y],
                      [0,   0,  1,  z],
                      [0,   0,  0,  1],
                      ])
        return M
    

    def rotationMtx(self, yaw, pitch, roll):
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
    

    def project2img_mtx(self):
        fc_x = 700.302102
        fc_y = 698.865457
        cx = 315.048841
        cy = 198.505301
        R_f = np.array([[fc_x, 0,    cx],
                        [0,    fc_y, cy]])

        return R_f
    

class LIDAR2CAMTransform:
    def __init__(self,params_cam,params_lidar):
        self.params_cam = params_cam
        self.params_lidar = params_lidar
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]

        self.image_pub = rospy.Publisher("/fusion_result_center",Image,queue_size= 1)
        self.clustering_point_pub_center = rospy.Publisher("/final_point_clustering_center",clustering_points,queue_size= 1)
        self.cluster_sub = rospy.Subscriber("/cluster",PointCloud2,self.lidar_callback)
        self.yolo_sub = rospy.Subscriber("/traffic_cone_yolo_result",Image,self.image_callback)
        self.bbox_sub = rospy.Subscriber("/traffic_cone_bbox_class",YoloTrafficCone,self.bbox_callback)
        self.frame = None
        self.bridge = CvBridge()

        # camera 좌표
        self.point_x = []
        self.point_y = []
        self.xyi_compare, self.xc, self.yc, self.zc = None, None, None, None


    def image_callback(self, msg):  # 10H
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


    def bbox_callback(self, msg): 
        cluster_msg = clustering_points()
        if msg.boxes == []:
            cluster_msg.x = []
            cluster_msg.y = []
            cluster_msg.label = []
        
        # print(">??????????????????? ", self.point_x)
        if len(self.point_x) == 0:
            # print("here")
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
            
            point_x_in_bbox = []
            point_y_in_bbox = []

            #바운딩 박스를 하나씩 탐색
            for box in bounding_boxes:
                # print("!!!!!!!!!!!!!!! ", box)
                # box = [int(x1), int(y1), int(x2), int(y2), class] 이 형태로 들어옴
                min_distance = float('inf')
                min_idx = -1
                # print("class : ", box[-1])
                # clustering point를 하나씩 탐색

                for j in range(len(self.point_x)):
                    # print(self.point_x)

                    if ((j not in point_check) and (box[0] <= self.point_x[j]) and (self.point_x[j] <= box[2]) and (box[1] <= self.point_y[j]) and (self.point_y[j] <= box[3])):
                    # if(j not in point_check):
                        
                        point_x_in_bbox.append(self.point_x[j])
                        point_y_in_bbox.append(self.point_y[j])
                        # print("point x in bbox", point_x_in_bbox)

                        centerx = (box[0] + box[2]) / 2
                        centery = (box[1] + box[3]) / 2
                        img2bbox_distance = math.sqrt(pow(centerx - self.point_x[j],2) + pow(centery - self.point_y[j],2))
                        if(min_distance > img2bbox_distance):
                                min_distance = img2bbox_distance
                                min_idx = j
                
                # 최적의 point 탐색 했다면
                if min_idx != -1: 
                    if len(np.where(self.xyi_compare[0] == self.point_x[min_idx])[0]) == 0:
                        print("center : 역변환 실패")
                    else:
                        point_check.append(min_idx)
                        cluster_msg.label.append(box[4])                
                        reverse_index = np.where(self.xyi_compare == self.point_x[min_idx])[1][0]
                        reverse_zc = self.zc[0][reverse_index]
                        reverse_xc = self.xc[0][reverse_index]
                        reverse_yc = self.yc[0][reverse_index]
                        reverse_xyz_c = np.array([[reverse_xc , reverse_yc, reverse_zc, 1.]])
                        reverse_xyz_p = np.matmul(reverse_xyz_c,np.linalg.inv(RT.T))
                        cluster_msg.x.append(reverse_xyz_p[0][0])
                        cluster_msg.y.append(reverse_xyz_p[0][1])
            
            try:
            
                cam_frame = self.draw_pts_img(self.frame, point_x_in_bbox, point_y_in_bbox)
                tmp_frame = self.bridge.cv2_to_imgmsg(cam_frame, "bgr8")
                self.image_pub.publish(tmp_frame)
            except:
                pass

            # bounding box하고 point는 있지만, 아무것도 일치가 되지 않았을 경우            
            if (len(cluster_msg.x) == 0):
                cluster_msg.x = []
                cluster_msg.y = []
                cluster_msg.label = []                
        self.clustering_point_pub_center.publish(cluster_msg)
            
    
    def lidar_callback(self, msg):
        if msg.data == b'':
            self.point_x = []
            self.point_y = []
            return
        self.point_lidar = self.pointxyz_to_xyz(msg)
        self.point_camera = self.transform_lidar2cam(self.point_lidar)
        self.project_pts2img(self.point_camera)


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
        self.xc, self.yc, self.zc = xyz_c[0, :].reshape([1,-1]), xyz_c[1,:].reshape([1,-1]), xyz_c[2,:].reshape([1,-1])
        #정규좌표계
        xn, yn = self.xc/(self.zc+0.000001), self.yc/(self.zc+0.000001)
        #카메라 이미자상 좌표계(2차원)
        xyi = np.matmul(proj_mtx, np.concatenate([xn,yn, np.ones_like(xn)], axis = 0))
        self.xyi_compare = xyi
        xyi = xyi.T 
        # print("xyi ", xyi)
        # xyi = self.crop_pts(xyi)
        xi, yi = xyi[:, 0].reshape([1,-1]), xyi[:,1].reshape([1,-1])
        self.point_x = xi.flatten()
        # print("hereoreroeor", self.point_x)
        self.point_y = yi.flatten()


    def crop_pts(self, xyi):
        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:, 0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:, 1]<self.height), :]
        return xyi


    def draw_pts_img(self, img, xi, yi):
        frame = img
        for ctr in zip(xi,yi):
            center = (int(ctr[0]), int(ctr[1]))
            frame = cv2.circle(frame, center, 3, (255,0,0),-1)
        return frame


if __name__ == "__main__" :
    rospy.init_node("webcam_pub", anonymous=True)
    params_cam = {"WIDTH": 640, "HEIGHT":480, "FOV":78, "X": 0.0, "Y": 0.0, "Z": 0.063}
    # "X": -0.515, "Y": 0.34, "Z": -0.14
    params_lidar = {"X" : 0, "Y": 0, "Z" : 0}
    prepare_matrix = create_matrix(params_cam, params_lidar)
    lidar_camera_calibration = LIDAR2CAMTransform(params_cam, params_lidar)
    rospy.spin()