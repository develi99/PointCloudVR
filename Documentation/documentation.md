# Documentation

The Project consits of 2 main components. 
- The Render which runs inside of Meta Quest and is responsibel for the real time visualization of the point cloud.  
- The Streamer, which provides a merged and post processed point cloud out of rgbd data from several cameras in real time to the Meta Quest 

## Streamer

The Streamer has 2 main components:
- [Source](docu_source.md): is responsibel for receveing and providing the data (e.g. point clouds) to the main loop
- [Main Loop](docu_mainloop.md): reads all pointclouds out of the cameras and processes them at the end it sends them to Meta Quest

For the Renderer that receives RGBD you only have to send the images, there exists no example at the moment. For the Renderer that receives Pointclouds you can look [here](../Streamer/Streamer.py). You should adapt the code to work on your system. Also the extrinsic calibartion must be provided.

## Meta Quest

- [RGBD](../Documentation/docu): Located in [Scene](../Unity/Assets/Scenes/RGBD.unity)
- [PointCloud](../Documentation/docu_renderer) Located in [Scene](../Unity/Assets/Scenes/Points.unity)

## Insallation & Usage

### Build Project
You can either deploy the apk in the git root or build it on your own.
- Open the Unity Folder as a Unity Project (We used Unity version 6000.0.24f1)
- Go to Build Profiles
- Switch to Android
- And then select the scene you want either Points or RGBD (you can find them in scenes)
- Deploy apk on your device

### Run
- start the server code first (streamer)
- do this by running Streamer.py (you have to adapt this to your needs, the calibration and camera config will not work for you)
- then start the Unity Code
