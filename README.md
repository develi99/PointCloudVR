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

## Installation

### 1. Clone the Repository

First, clone this repository to your local machine:

```bash
git clone https://github.com/develi99/PointCloudVR.git
```

### 2. Install the Python Package

Navigate into the project directory and install the Python module in editable mode:

```bash
cd PointCloudVR
pip install -e .
```

### 3. Open the Unity Project

1. Open the Unity project located in the `unity/` directory using Unity (2020.3 or later is recommended).
2. Unity may prompt you to install additional packages or apply automatic fixes — go ahead and accept those.
3. If errors still appear, close and reopen Unity. This often resolves temporary import issues.

### 4. Load the Correct Scene

Manually open the following scene in Unity:

```
Assets/scenes/Cubes_advanced.unity
```

You can confirm the scene is loaded if its name appears in the top-left corner of the Unity Editor.

### 5. Running and Testing

1. Run one of the Python example scripts located in the `examples/` folder.
2. Make sure a depth camera or compatible input source is connected.
3. To use your own custom source, add it under:

```
/streamer/Source
```

4. Once the Python server is running, open the Unity scene.
   - If nothing appears at first, try restarting Unity. It should begin visualizing the stream once the server is active.

> ⚠️ **Note**: Your PC and headset must be on the same Wi-Fi or LAN network. `localhost` is currently **not supported** for server-headset communication.


### 6. Build and Deploy to Meta Quest

Once everything is working:

1. Build the Unity project for Android (for the Meta Quest).
2. Deploy the build to your Meta Quest headset.
3. Ensure both the Python server (on your PC) and the Unity app (on the Quest) are running **within the same subnet** — otherwise, the connection will fail.

---

## Limitations

The application on Meta Quest requires RGB-D data, with the RGB component provided as a JPEG image and the depth data as a raw array.
A key limitation is that it is currently not possible to generate a point cloud using data from more than one camera directly on the Meta Quest.
Due to hardware constraints of the Meta Quest, it would be more efficient to compute the point cloud externally—ideally within the Python script—and then transmit the resulting data to the headset.

i am wokring on a version that implements these things. I will commit it when i finished it.

---

## Documentation

You can find a documentation [here](/docs/docu.md)

---

## 📄 License

This project is licensed under the [MIT License](LICENSE).
