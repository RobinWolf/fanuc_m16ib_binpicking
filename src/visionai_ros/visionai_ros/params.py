# -*- coding: utf-8 -*-
"""
Created on Fri Aug  4 09:26:25 2023
@author: st34616, contributor rw39401
Set _debug_pointcloud to False in release environment. It basically slows down the process by doing the double operation.
"""
from pathlib import Path
import numpy as np

cwd = Path.cwd()

##################### Use Simulation environment #####################
_simulate = False


##################### Debug + Visualisation #####################
_logfile = cwd / "logs" / "300924_timeit_log_jetson_16gb_real.txt"
_debug =  False  # enable prints
_visMask = False # viz selected instance to grip
_visPcloud = False  # viz cropped ROI PCD in interactive mode
_visBeforePcloud = False # viz source pcd and target (cropped ROI) before registration
_debug_Viz = True  # save captured RDB image


##################### Zivid Capture Settings #####################
_pathZividSettings = cwd / "Data" / "capture_settings" / "settings_new_11122023.yml"   # camera configuration
_export_full_scene = False  # export captured pcd to file
_export_full_scene_path = cwd / "PathPlanner" / "Captured_Scan.stl" # full captured scene stl (no cropped areas)
_PCtoRobotbase = True   # apply Hand-Eye Calibration
_ImagePath = 'Scans/1.png'  # image to save captured rgb if _debug_Viz == True
_simScanPath = "Scans/scan2.zdf"    # Scan used for Simulation if _simulate = True
_caibrationMatrix = np.array([[ -9.75352466e-01, 1.67853869e-02, -2.20013127e-01, 1.98877747e+02],  
                            [6.24175072e-02, 9.77365434e-01, -2.02140734e-01, 1.36782410e+03],
                            [ 2.11640209e-01, -2.10891142e-01, -9.54323530e-01, 8.75918091e+02],
                            [ 0., 0., 0., 1. ] ], dtype=np.float32)    #Calibration JS33039 12.09.2023     


##################### AI Settings #####################
_config_yaml_path = cwd / "Data" / "detectron2_cfg" / "config.yaml"
_viz_detectron2_results = False
_confidence = 0.4
_class = 100 # 100 is used to return all classes (no multi class prediction)
_path2Pth = cwd / "model" /"model_final.pth"


##################### Filters #####################
_exportJSON = False # export the propeties of picked instance to a json file
_pickInstanceJsonPath = cwd / "json" / "filteres_instance.json"
_idx_forced = None
_area = False
_probability = True 


##################### Mesh Exports #####################
_saveCroppedPly = False # save cropped ROI PCD to file
_croppedPLYPath = cwd / "mesh" / "cropped_ROI.ply" 
_collisionMeshPath = cwd / "PathPlanner" / "Scan.stl"   # Path where the collision mesh is saved
_downsample_voxel_size_full_scan = 3    # voxel size of collision mesh in mm


##################### Registration Settings #####################
_pathClass0 =  cwd / "Scans" /  "top.ply" 
_pathClass1 =  cwd / "Scans" /  "bottom.ply"

# FPFH Parameters
_voxel_size_source=0.6  # params to calculate fpfh features of source pcd
_radius_feature_source=1.4
_max_nn_source=100
_voxel_size_target=0.5  # params to calculate fpfh features of source pcd
_radius_feature_target=1.8
_max_nn_target=100

# RANSAC Parameters
_distance_threshold=0.8 # Maximum distance between corresponding points in the two clouds for them to be considered a valid match
_r_k = 3    # A parameter for RANSAC, usually the number of iterations or the percentage of inliers ???
_r_s = 0.05 # parameter related to edge length checking at validating the correspondence of the two clouds
_ransac_iterations = 400000 # RANCAC convergence criterias
_ransac_confidence = 1

# ICP Paramsters
_ICPradius = 0.4    # ICP point-to-point sampling radius
_ICPiter_=50000 # ICP convergence criteria


##################### CIT Pathplanning Settings #####################
_planner_xml = cwd / "PathPlanner" / "planner.xml"
_path_LS_Script = cwd / "PathPlanner" / "results" / "TEST.ls"
_pathPlannerexe = "C:\\Program Files\\binpicking-3.1.0-win64\\bin\\onlinePlanner.exe"
_pathPlannerAP = cwd / "PathPlanner" /  "Picking.ap"
_pathPlannerOutAP = cwd / "PathPlanner" /  "results" / "JobResult.ap"
_pathPlannerJobStatus = cwd / "PathPlanner" /  "results" / "JobStatus.txt"


##################### Robot Connection Settings #####################
# TCP/IP Server Parameters
_host_ip = '127.0.0.1'  # use for simulation
##_host_ip = '192.168.1.250'    # use for real robot
_host_port = 60000

# FTP Parameters
_ftp_ip = '192.168.1.105'
_ftp_id = 'admin'
_ftp_pwd = 'fanuc'
_remote_ls_file_path='MD:\\'




#------------------------------- NOT USED ?????????????

# Detectron2 settings
_pathBrown =  cwd.parent / "projectdata" / "Parts" / "PaintkitBrownCondensator.ply" 
_pathBlack =  cwd.parent / "projectdata" / "Parts" / "PaintkitBlackCondensator.ply" 
_pathCoil =  cwd / "Scans" /  "stitched_10k.ply" 
_readXML = cwd / "Data" / "robot" / "readjob.xml"
_writeXML = cwd  / "Data" / "robot" / "writejob.xml"
_pathONNX = cwd  / "model" / "best.pt"
_littleEndian = False
_server_ip = '10.130.100.252'
_server_port = 59002




