import sys
sys.path.append('/home/ubuntu/.conda/envs/py36/lib/python3.6/site-packages')
import torch
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class camLane(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("---车道线检测模块初始化---")
        self.shortImagesub = self.create_subscription(Image, "short_focal_image", self.shortImagecallback, 10)
        self.bridge = CvBridge()

        # 原图数据
        self.ori_img_w = 640
        self.ori_img_h = 480

        self.train_height = 320
        self.train_width = 1600
        self.crop_ratio = 0.6
        self.cut_height = int(self.train_height * (1 - self.crop_ratio))

        self.input_width = self.train_width
        self.input_height = self.train_height

        self.num_row = 72
        self.num_col = 81
        self.row_anchor = np.linspace(0.42, 1, self.num_row)
        self.col_anchor = np.linspace(0, 1, self.num_col)

        # cuda
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = torch.jit.load('/home/ubuntu/JiaqiZhang/ROS_WS/model/culane_model.pt', map_location=self.device)

    
    def shortImagecallback(self, image):

        image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
    
        self.forward(image)

        return None
    
    def forward(self, img):
        im0 = img.copy()
        img = img[self.cut_height:, :, :]
        img = cv2.resize(img, (self.input_width, self.input_height), cv2.INTER_CUBIC)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(np.float32(img[:, :, :, np.newaxis]), (3, 2, 0, 1))
        # img = np.ascontiguousarray(img)
        img = torch.tensor(img).to(self.device)
        preds = self.model(img)
        print(preds)
        coords = self.pred2coords(preds, self.row_anchor, self.col_anchor, original_image_width=self.ori_img_w, original_image_height=self.ori_img_h)
        for lane in coords:
            for coord in lane:
                cv2.circle(im0, coord, 2, (0, 255, 0), -1)
        cv2.imshow("result", im0)
        cv2.waitKey(2)
    
    def pred2coords(self, pred, row_anchor, col_anchor, local_width = 1, original_image_width = 1640, original_image_height = 590):
        batch_size, num_grid_row, num_cls_row, num_lane_row = pred['loc_row'].shape
        batch_size, num_grid_col, num_cls_col, num_lane_col = pred['loc_col'].shape

        max_indices_row = pred['loc_row'].argmax(1).cpu()
        # n , num_cls, num_lanes
        valid_row = pred['exist_row'].argmax(1).cpu()
        # n, num_cls, num_lanes

        max_indices_col = pred['loc_col'].argmax(1).cpu()
        # n , num_cls, num_lanes
        valid_col = pred['exist_col'].argmax(1).cpu()
        # n, num_cls, num_lanes

        pred['loc_row'] = pred['loc_row'].cpu()
        pred['loc_col'] = pred['loc_col'].cpu()

        coords = []

        row_lane_idx = [1,2]
        col_lane_idx = [0,3]

        for i in row_lane_idx:
            tmp = []
            if valid_row[0,:,i].sum() > num_cls_row / 2:
                for k in range(valid_row.shape[1]):
                    if valid_row[0,k,i]:
                        all_ind = torch.tensor(list(range(max(0,max_indices_row[0,k,i] - local_width), min(num_grid_row-1, max_indices_row[0,k,i] + local_width) + 1)))
                        
                        out_tmp = (pred['loc_row'][0,all_ind,k,i].softmax(0) * all_ind.float()).sum() + 0.5
                        out_tmp = out_tmp / (num_grid_row-1) * original_image_width
                        tmp.append((int(out_tmp), int(row_anchor[k] * original_image_height)))
                coords.append(tmp)

        for i in col_lane_idx:
            tmp = []
            if valid_col[0,:,i].sum() > num_cls_col / 4:
                for k in range(valid_col.shape[1]):
                    if valid_col[0,k,i]:
                        all_ind = torch.tensor(list(range(max(0,max_indices_col[0,k,i] - local_width), min(num_grid_col-1, max_indices_col[0,k,i] + local_width) + 1)))
                        
                        out_tmp = (pred['loc_col'][0,all_ind,k,i].softmax(0) * all_ind.float()).sum() + 0.5

                        out_tmp = out_tmp / (num_grid_col-1) * original_image_height
                        tmp.append((int(col_anchor[k] * original_image_width), int(out_tmp)))
                coords.append(tmp)

        return coords


def main(args=None):
    
    rclpy.init(args=args)
    node = camLane("camLane")
    rclpy.spin(node)
    rclpy.shutdown()
