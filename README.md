"# Point-Cloud-Semantic-Segmentation" 
1) Download the KITTI-360 on which this project is performed.
2) Convert the .bin to .pcd using the bin2pcd.py which is present in the Code folder.
3) Use ICP by running the icp.py to reduce the difference between two point clouds that is present in the Code/pcd_folder folder.
4) Use semantic_segmentation.py to segment the images semantically in DeepLabV3Plus-Pytorch folder *.
5) Again generate the map using icp_final.py

* NOTE: Download the pretrained weights for the network from https://www.dropbox.com/sh/w3z9z8lqpi8b2w7/AAABmvcqWdVtTJCFQ75OmK0va/best_deeplabv3_resnet101_voc_os16.pth?dl=0 and store them in the Code folder. Keep the first few frames from the dowloaded sequence for our project.
