# 📡 PointCloudVR – Real-Time Point Cloud Streaming for VR

PointCloudVR is a modular system for streaming and rendering 3D point cloud data in VR using Meta Quest headsets. The system supports two different pipelines depending on where the point cloud is generated: **on-device GPU reconstruction from RGB-D streams** or **precomputed point cloud streaming**.

<table align="center">
  <tr>
    <td align="center">
      <img src="video/Latency.gif" width="100%"/><br/>
      <b>Latency</b>
    </td>
    <td align="center">
      <img src="video/PushT.gif" width="100%"/><br/>
      <b>Push-T Task</b>
    </td>
  </tr>
</table>
<!-- <img src="video/Latency.gif" width="100%"/> -->
<!-- <img src="video/PushT.gif" width="100%"/> -->

---

## 🧠 System Overview

The architecture consists of two independent modes:

### **Mode A: RGB-D Streaming → GPU Reconstruction (Quest-side)**

* The streamer sends **RGB + Depth images** to the headset.
* The Quest reconstructs the point cloud in real time using **GPU compute shaders**.
* Rendering is fully GPU-accelerated inside Unity.

**Pipeline:**

```
Camera → Python Streamer → (RGB + Depth) → Quest → Compute Shader → Point Cloud Rendering
```

**Key characteristics:**

* Lightweight streamer (only image resizing + transmission)
* Flexible resolution control via downsampling
* High performance rendering on Quest GPU
* No multi-view fusion on streamer side

---

### **Mode B: Precomputed Point Cloud Streaming (CPU/Server-side reconstruction)**

* The streamer generates the **complete point cloud on the host machine**.
* The Quest only receives already-processed 3D points.
* Rendering is done via optimized GPU shaders in Unity.

**Pipeline:**

```
Camera → Python Streamer → Point Cloud Generation → Stream → Quest → GPU Point Cloud Renderer
```

**Key characteristics:**

* More computational flexibility on host machine
* Supports:

  * segmentation
  * filtering & denoising
  * point cloud fusion (multi-view support)
* More efficient runtime on Quest
* Enables multi-camera setups (MV possible)

---

## ⚙️ Core Design Goals

* Real-time streaming with minimal latency
* GPU-accelerated rendering in Unity (compute shaders + instancing)
* Modular pipeline separation (capture → process → render)
* Support for scalable point cloud resolution

---

## 🎮 Rendering on Quest

Both modes use a shared rendering approach:

* GPU instancing / compute shaders
* Efficient point cloud buffers
* Real-time updates via streaming interface

---

## ⚡ Performance Considerations

| Mode                  | Bottleneck                      | Strength                             |
| --------------------- | ------------------------------- | ------------------------------------ |
| RGB-D Streaming       | Network + reconstruction shader | Low bandwidth, simple pipeline       |
| Point Cloud Streaming | CPU generation on host          | High flexibility, multi-view capable |

---

## 📌 Key Difference Between Modes

* **Mode A (RGB-D):**

  * Minimal streamer complexity
  * No multi-view fusion on host
  * Reconstruction happens on Quest GPU

* **Mode B (Point Cloud):**

  * Full control over point cloud generation
  * Supports segmentation, filtering, merging
  * More efficient rendering pipeline on Quest

---

## 📄 Documentation

Further details can be found in [Docu](Documentation/documentation.md)

Currently there is an error in the code i have to fix, so without adaptions you want be able to use it

---

## 🪪 License

MIT License
