import numpy as np
import open3d as o3d
import cv2
import os
import time
import Source as s
import Action as a
import Datasources as ds
import os
import yaml

def save_pointcloud(pcd_tensor, filename, output_folder):
    """Konvertiert eine Tensor-PointCloud zu legacy und speichert als PLY."""
    # Punkte extrahieren
    points = pcd_tensor.point.positions.cpu().numpy()
    
    # Farben extrahieren, falls vorhanden
    colors = None
    if "colors" in pcd_tensor.point:
        colors = pcd_tensor.point.colors.cpu().numpy()

    # Legacy PointCloud erstellen
    pcd_legacy = o3d.geometry.PointCloud()
    pcd_legacy.points = o3d.utility.Vector3dVector(points)
    if colors is not None:
        pcd_legacy.colors = o3d.utility.Vector3dVector(colors)
    
    # Speichern
    path = os.path.join(output_folder, filename)
    o3d.io.write_point_cloud(path, pcd_legacy)
    print(f"Saved {path}")

# Extrinsics
with open("/home/ibaelias/Documents/paper_enes_elias/fast_mail/configs/cameras/calibration/2025-09-13.yaml") as f:
    data = yaml.safe_load(f)

T_upper = np.array(data["upper_cam"]["extrinsics"])
T_down = np.array(data["down_cam"]["extrinsics"])

flip = np.array([
    [-1, 0, 0, 0],
    [ 0, 1, 0, 0],
    [ 0, 0, 1, 0],
    [ 0, 0, 0, 1]
], dtype=float)

T_down_flipped = flip @ T_down @ flip
T_upper_flipped = flip @ T_upper @ flip

T_upper2down = np.linalg.inv(T_down_flipped) @ T_upper_flipped

# manual corrction of pointcloud, here a translation
T_manual = np.eye(4)
T_manual[:3, 3] = [-0.048509 , 0.011509, 0.000726683]
T_upper2down = T_manual @ T_upper2down

# --- To save pointlcoud ---
dir = "ply_output"
os.makedirs(dir, exist_ok=True)
counter = 0

# --- prepare camera ---
streamer = s.Receiver("10.42.0.27", 35555)

# upper = s.VirtualCamera("upper_cam", streamer, 400, 480, use_imgs=False, use_segmentation=False, use_pc_creation=True)
# down = s.VirtualCamera("down_cam", streamer, 400, 480, use_imgs=False, use_segmentation=False, use_pc_creation=True)

# ori_widht/height, for RealSense configuration
# res_width/height, for resizing

QUALITY = "LOW"
USE_SEGMENTATION = False # For Segmentation the YOLO server must running (see documentation)
MAX_POINTS = 1000000

upper_id = "007522061936"
left_id  = "243322073029"
right_id = "944622073668"

if QUALITY == "HIGH":
    width = 1280
    height = 720
    fps = 30
elif QUALITY == "MEDIUM":
    width = 848
    height = 480
    fps = 60
elif QUALITY == "LOW":
    width = 640
    height = 480
    fps = 60

upper = s.RealSenseCamera(id=upper_id, org_width=width, org_height=height, res_width=width, res_height=height, fps=fps, use_imgs=False, use_segmentation=USE_SEGMENTATION, use_pc_creation=True)
left  = s.RealSenseCamera(id=left_id, org_width=width, org_height=height, res_width=width, res_height=height, fps=fps, use_imgs=False, use_segmentation=USE_SEGMENTATION, use_pc_creation=True)
right = s.RealSenseCamera(id=right_id, org_width=width, org_height=height, res_width=width, res_height=height, fps=fps, use_imgs=False, use_segmentation=USE_SEGMENTATION, use_pc_creation=True)

sending = a.ZMQPublishPointCloudAction()
timedebugging = False

try:
    while True:
        dev = "CUDA:0"  # oder "CPU:0"
        pcd_right = None
        pcd_left = None
        pcd_upper = None

        # Upper to Down
        while True:
            if pcd_upper is None:
                pcd_upper  = upper.get_pcd(dev)
            if pcd_left is None:
                pcd_left = left.get_pcd(dev)
            if pcd_right is None:
                pcd_right = right.get_pcd(dev)
            if pcd_upper is not None and pcd_left is not None and pcd_right is not None:
                break
            time.sleep(0.005)

        # --- UPPER -> DOWN transform (o3d.t: in-place) ---
        pc_upper_in_down = pcd_upper.clone()
        pc_upper_in_down.transform(T_upper2down)

        # --- Merge über Tensor-Konkatenation ---
        posU = pcd_upper.point["positions"]
        posL = pcd_left.point["positions"]
        posR = pcd_right.point["positions"]

        posM = o3d.core.concatenate((posU, posL, posR), axis=0)

        colU = pcd_upper.point["colors"]
        colL = pcd_left.point["colors"]
        colR = pcd_right.point["colors"]
        
        colM = o3d.core.concatenate((colU, colL, colR), axis=0)

        pcd_merged = o3d.t.geometry.PointCloud(posM.device)
        pcd_merged.point["positions"] = posM
        pcd_merged.point["colors"] = colM

        # For current Cam not necessary, makes cropping easier but here it is easy enough 
        # without this Transformation
        # T_cam_to_base = np.array([
        #     [+0.81, -0.37, +0.46, -0.39],
        #     [+0.13, +0.87, +0.47, -0.38],
        #     [-0.58, -0.32, +0.75, -0.53],
        #     [0, 0, 0, 1]
        # ], dtype=np.float32)

        # # Get tensor of positions directly (stays on GPU if your device is GPU)
        # pcd_transformed = pcd_merged.clone()
        # pcd_transformed.transform(T_cam_to_base)

        pcd_transformed = pcd_merged # remove if transformed is used

        # RealSense
        x_min, x_max = -0.4, 0.38
        y_min, y_max = -1, 1
        z_min, z_max = 0.5, +1.15

        # Get tensor of positions directly (stays on GPU if your device is GPU)
        pos = pcd_transformed.point["positions"]   # shape (N, 3), device = CUDA:0 if GPU

        # Create boolean mask on GPU
        mask = (pos[:, 0] >= x_min) & (pos[:, 0] <= x_max)
        mask &= (pos[:, 1] >= y_min) & (pos[:, 1] <= y_max)
        mask &= (pos[:, 2] >= z_min) & (pos[:, 2] <= z_max)

        # Apply mask to points
        xyz_cropped = pos[mask]

        # Apply mask to colors if available
        pcd_cropped = o3d.t.geometry.PointCloud(pos.device)
        pcd_cropped.point["positions"] = xyz_cropped

        if "colors" in pcd_transformed.point:
            col = pcd_transformed.point["colors"]
            pcd_cropped.point["colors"] = col[mask]

        pcd_transformed = pcd_cropped

        # --- Downsampling auf dem aktuellen Device ---
        voxel = 0.007 # 0.007 
        pcd_downsampled = pcd_transformed.voxel_down_sample(voxel_size=voxel) # important: only fast if you use CUDA:0, which requires GPU

        # --- NumPy holen (bei GPU vorher .cpu()) ---
        pos = pcd_downsampled.point["positions"]
        if pos.device.get_type() != o3d.core.Device.DeviceType.CPU:
            pos = pos.cpu()
        xyz = pos.numpy()

        rgb = None
        if "colors" in pcd_downsampled.point:
            col = pcd_downsampled.point["colors"]
            if col.device.get_type() != o3d.core.Device.DeviceType.CPU:
                col = col.cpu()
            rgb = col.numpy()
            rgb = rgb[:, [0, 1, 2]] # RealSense: 0,1,2; Zed: 2,1,0

        sending.set_pointcloud(xyz, rgb)
        sending.execute()


except KeyboardInterrupt:
    print("Beendet durch Nutzer.")
    streamer.stop_receiver_thread()
