# PointCloudVR
Live visualization of point clouds from a camera stream in VR using Meta Quest.

## 🚀 Features
- Loading, processing and streaming of RGB and depth data
- Real-time computation and visualization of Pointclouds on Meta Quest (Unity XR)
- Modular architecture for extension

## Video

We used PointCloudVR for making Teleoperated Demonstrations.

<video width="600" controls>
  <source src="video/video_with_T.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

In this video we used a downsampled rgb and depth image (214x160 = ~34K). Each pixel is represented as a cube, so ~34K Cubes are rendered.
Here we reached ~70 fps rendering. The datastreaming is here the bottleneck with ~30 fps. 

## 🧱 Project Structure

```bash
PointCloudVR/
├── python/ 
├── unity/
├── example/
├── docs/
├── video/
├── LICENSE
└── README.md
```

## Installation



## 📄 License

This project is licensed under the [MIT License](LICENSE).