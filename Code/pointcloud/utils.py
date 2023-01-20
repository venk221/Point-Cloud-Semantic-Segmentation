from scipy.spatial.transform import Rotation
import numpy as np
import open3d as o3d
import struct
import cv2
import matplotlib.pyplot as plt
import os
def label_to_color(semantic_map,palette):
    color_seg = np.zeros((semantic_map.shape[0], semantic_map.shape[1], 3), dtype=np.uint8)
    for label, color in enumerate(palette):
        color_seg[semantic_map  == label] = color
    color_seg = color_seg[..., ::-1]
    color_seg = color_seg.astype(np.uint8)
    return color_seg




def semantics_to_colors(semantics:np.array,palette:np.array) -> np.array:
        colors = np.ones((semantics.shape[0], 3))
        for label,color in enumerate(palette): 
            colors[semantics == label] = (color[0]/255,color[1]/255,color[2]/255)    
        return colors



def visuallize_pointcloud(pointcloud: np.array,path:str,filename:str,palette:np.array) -> None:
        semantics  = pointcloud[:, 3]
        xyz = pointcloud[:, 0:3]
        visualizer = o3d.visualization.Visualizer()
        pcd = o3d.geometry.PointCloud()
        visualizer.add_geometry(pcd)
        colors = semantics_to_colors(semantics,palette)
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.io.write_point_cloud(os.path.join(path,"results","painted_cloud",filename+".pcd"),pcd)
        


def transform_velo_to_cam(R0, Tr_cam_to_lidar):
    R_ref2rect = np.eye(4)
    R0_rect = R0.reshape(3, 3) 
    R_ref2rect[:3, :3] = R0_rect
    R_ref2rect_inv = np.linalg.inv(R_ref2rect)  
    cam2velo_ref = np.vstack((Tr_cam_to_lidar.reshape(3, 4), np.array([0., 0., 0., 1.])))  
    P_cam2velo_ref = np.linalg.inv(cam2velo_ref)
    proj_mat = P_cam2velo_ref @ R_ref2rect_inv
    return proj_mat

def projection_velo_to_cam(R0, Tr_lidar_to_cam,P):
    R_rect = np.eye(4)
    R0 = R0.reshape(3, 3)
    R_rect[:3, :3] = R0
    P_ = P.reshape((3, 4))
    proj_mat = P_ @ R_rect @ Tr_lidar_to_cam
    return proj_mat





def convert_3D_to_2D(P,lidar_pts):
    pts_3d = convert_3d_to_hom(lidar_pts)
    pts_2d= np.dot(pts_3d,P.T)
    depth = pts_2d[:, 2]
    depth[depth==0] = -1e-6
    pts_2d[:, 0] /= pts_2d[:, 2]
    pts_2d[:, 1] /= pts_2d[:, 2]
    pts_2d = pts_2d[:, :2]
    return pts_2d,depth


def remove_lidar_points_beyond_img(P,lidar_pts, xmin, ymin, xmax, ymax):
    pts_2d,depth = convert_3D_to_2D(P,lidar_pts) 
    inside_pts_indices = ((pts_2d[:, 0] >= xmin) & (pts_2d[:, 0] < xmax) & (pts_2d[:, 1] >= ymin) & (pts_2d[:, 1] < ymax))
    return  pts_2d, inside_pts_indices,depth


def project_lidar_on_image(P, lidar_pts, size):
    all_pts_2d, fov_inds, depth = remove_lidar_points_beyond_img(P,lidar_pts, 0, 0,size[0], size[1])
    return all_pts_2d[fov_inds],depth[fov_inds], lidar_pts[fov_inds]

def convert_3d_to_hom(pts_3d):  
    n = pts_3d.shape[0]
    pts_3d_hom = np.hstack((pts_3d, np.ones((n, 1))))
    return pts_3d_hom
