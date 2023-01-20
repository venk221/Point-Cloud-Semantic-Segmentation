
import numpy as np
import open3d as o3d
import os
import calibration as ca
import utils as ut
import cv2
from distutils.util import strtobool

def main():
    calib = ca.CalibrationData('./calibration.txt')
    R = calib.R0
    P = calib.P
    K = calib.K
    D = calib.D
    Tr_cam_to_lidar = calib.Tr_cam_to_lidar
    Tr_lidar_to_cam = ut.transform_velo_to_cam(R, Tr_cam_to_lidar)
    P_lidar_to_cam = ut.projection_velo_to_cam(R, Tr_lidar_to_cam,P)
    pcd_data = os.listdir('./pcd/')
    pcd_data.sort()
    data=os.listdir('./images')
    data.sort()
    data1=os.listdir('./images_segmented')
    data1.sort()
    for idx in range(len(pcd_data)):
        rgb_image=cv2.imread('./images/'+data[idx])
        fused_image=rgb_image.copy()
        rgb_image_segmented=cv2.imread('./images_segmented/'+data1[idx])
        pcd=np.asarray(o3d.io.read_point_cloud('./pcd/'+pcd_data[idx]).points)
        point_cloud=pcd[pcd[:,0]>=0]
        pts_2D,_, pts_3D_img = ut.project_lidar_on_image(P_lidar_to_cam, point_cloud, (rgb_image.shape[1], rgb_image.shape[0]))
        N = pts_3D_img.shape[0]
        semantic = np.zeros((N,3), dtype=np.float32)
        print(idx)
        for i in range(pts_2D.shape[0]):
            x = np.int32(pts_2D[i, 0])
            y = np.int32(pts_2D[i, 1])
            colour = np.float64(rgb_image_segmented[y, x]) 
            pt = (x,y)
            cv2.circle(fused_image, pt, 2, color=colour, thickness=1)
            semantic[i] = rgb_image_segmented[y,x]
        img = np.vstack((rgb_image, rgb_image_segmented,fused_image))
        fname=data[idx].split('.')[0]
        cv2.imwrite(f'./fused/{fname}_fused.png',img)
        rgb_pointcloud = np.hstack((pts_3D_img[:,:3], semantic))
        xyz = rgb_pointcloud[:, 0:3]
        visualizer = o3d.visualization.Visualizer()
        pcd = o3d.geometry.PointCloud()
        visualizer.add_geometry(pcd)
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(semantic)
        o3d.io.write_point_cloud(f'./points_painted/{fname}_painted.pcd',pcd)
                
            #Help taken from https://github.com/naitri/PointPainting    
if __name__ == '__main__':
    main()
