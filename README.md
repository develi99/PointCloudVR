# PointCloudVR
Live visualization of point clouds from a camera stream in VR using Meta Quest.

## 🚀 Features
- Loading, processing and streaming of RGB and depth data
- Real-time computation and visualization of Pointclouds on Meta Quest (Unity XR)
- Modular architecture for extension

## 🎥 Video
We used PointCloudVR for making Teleoperated Demonstrations. 
You can find a video of it in [video](video/).
In this video we used a downsampled rgb and depth image (214x160 = ~34K). Each pixel is represented as a cube, so ~34K Cubes are rendered.
Here we reached ~70 fps rendering. The datastreaming is here the bottleneck with ~30 fps. 

## 🧱 Project Structure
```bash
PointCloudVR/
├── python/ The Python Code of the Streamer
├── unity/ The Unity Project for the Meta Quest Application
├── example/ Some examples of setting a the streamer
├── docs/ Documentation
├── video/ videos
├── LICENSE
└── README.md
```

## Installation




## 📄 License

This project is licensed under the [MIT License](LICENSE).