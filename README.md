# 📡 PointCloudVR

**PointCloudVR** enables live visualization of RGB-D point clouds streamed from an external camera into a VR environment using Meta Quest. It combines real-time depth processing, GPU-accelerated rendering, and XR controller interaction for immersive exploration of spatial data.

---

## 🚀 Features

- 🔄 Real-time streaming of RGB and depth data via ZeroMQ
- 🧠 GPU-based computation and rendering of 3D point clouds
- 🎮 Intuitive VR interaction using Meta Quest controllers (Unity XR)
- 🧩 Modular architecture – easy to extend or integrate into other XR workflows

---

## 🎥 Demo Video

PointCloudVR was used for **teleoperated robot demonstrations**.

A demo video is available in the [`video/`](video/) folder.

- Input resolution: **214 × 160** (~34,000 points)
- Each pixel is visualized as a **3D cube**
- Achieved:
  - **~70 FPS rendering**
  - **~30 FPS data streaming** (main bottleneck)

---

## 🧱 Project Structure

```bash
PointCloudVR/
├── python/      # Python-based streamer sending RGBD data
├── unity/       # Unity XR application for Meta Quest
├── example/     # Example configurations for the streamer
├── docs/        # Project documentation
├── video/       # Demo videos
├── LICENSE
└── README.md
```

## 📄 License

This project is licensed under the [MIT License](LICENSE).