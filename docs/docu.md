# Documentation

## Streamer
The streamer is implemented in python (see [python](../python)) and responsible for reading the data out of a source and send it to the Meta Quest. There a 3 important Abstract Concepts: Source, ProcessingStep and Action, which are represented as ABCs in the python code. These 3 classes can be seen as a processing pipeline. First data is read with Source, then proceessed with ProcessingStep and then different Actions are made with this data. You can find a example how a streamer can be defined in [examples](../examples).

### Source
With the Class Source sources are defined, which can be for example different Cameras or a other Streamer in the Internet.

![Source Classdiagram](Source.drawio.png)

### ProcessingStep
With ProcessingStep several processingsteps can be connected. These ProcessingSteps are always executed in order, so the output of the first is the input for the second, ...

![Processing Step Classdiagram](ProcessingStep.drawio.png)

### Actions
Actions are the last Piece of the Processing Pipeline. Here you can see a ShowImage, which opens a window to see depth and rgb data. Additionally there is the ZMQPublisher, that class is responsible for sending the data to the Meta Quest.

![Action Classdiagram](Action.drawio.png)


## Meta Quest
On Meta Quest the pointcloud is generation via compute shader based on the rgb and depth image. Instanced Rendering and an own shader are used to render the cubes efficient. The Code consists of three parts, [C# Code](../unity/Assets/Scripts/PointCloudVR/CubeRendering.cs), [Compute Buffer](../unity/Assets/Scripts/PointCloudVR/CubeRendering.copmute) and [Rendering Shader](../unity/Assets/Scripts/PointCloudVR/CubeRendering.shader).

### [C# Code](../unity/Assets/Scripts/PointCloudVR/CubeRendering.cs).

The c# script is responsible for rendering a real-time point cloud from depth and RGB image data, received over the network using ZeroMQ. It handles data parsing, controller inputs, compute shader dispatching, and rendering of colored cubes via GPU instancing.

#### 🔌 Networking (ZeroMQ)

- Uses a **`PullSocket`** to receive image data from a remote server.
- The connection is dynamically discovered via **UDP broadcast** (`FindServer()`).
- A dedicated **listener thread** (`ZmqListener`) receives and parses messages containing:
  - Image resolution
  - RGB image bytes
  - Depth data
  - Camera intrinsics
  - Culling parameters
- Shared data is safely exchanged with the Unity main thread using a `lock`.

#### 🧠 Main Data Flow

##### `Start()`
- Initializes rendering resources and network listener.
- Requests camera permission on Android (required for certain AR devices).
- Calls `InitBuffers()` to set up compute/render buffers.

##### `Update()`
1. Handles controller input (`controllerHandling()`).
2. Safely reads the latest shared data from the network thread.
3. Updates compute and render buffers if new data is available.
4. Renders the point cloud via `Graphics.DrawMeshInstancedIndirect`.
5. Optionally logs performance (FPS).

#### 🎮 Controller Input (XR)

##### Right Controller:
- **Right Stick**: Yaw and pitch rotation
- **Primary/Secondary Button**: Roll rotation

##### Left Controller:
- **Left Stick**: Translate point cloud in X/Z
- **Trigger/Grip**: Move up/down (Y-axis)
- **Buttons**: Adjust cube size

All input values are frame-rate independent, scaled using `Time.deltaTime`.

#### 🧮 Compute Shader Pipeline

- A compute shader (`pointCloudCompute`) transforms depth values into world-space positions.
- Camera intrinsics (`fx`, `fy`, `cx`, `cy`) and the current pose are passed into the shader.
- Resulting transformation matrices are written into a GPU `ComputeBuffer` (`matrixBuffer`).
- The point cloud is rendered as instanced cubes using the **indirect instancing** method.

#### 🗂️ Buffer Overview

| Buffer         | Purpose                                      |
|----------------|----------------------------------------------|
| `matrixBuffer` | Holds transformation matrices for each cube  |
| `depthBuffer`  | Stores raw depth data (converted from bytes) |
| `argsBuffer`   | Parameters for indirect drawing              |
| `rgbTexture`   | RGB image used as input for the material     |

Buffers are reallocated on resolution changes or app focus.

#### 📷 Camera Intrinsics & Pose

- Intrinsics (`fx`, `fy`, `cx`, `cy`) are provided at runtime by the ZMQ server.
- A **pose matrix** is constructed from controller-driven translation & rotation.
- Used in the compute shader to reconstruct correct 3D positions.

#### 📊 Performance Logging

- If `logPerformance` is enabled, the component logs **frames per second (FPS)** once per second.

#### 🧽 Cleanup & App State

- `OnDestroy()` releases GPU buffers, stops the listener thread, and disposes of the ZMQ socket.
- `OnApplicationPause()` and `OnApplicationFocus()` reinitialize buffers to prevent graphical issues on mobile devices.

#### 🧵 Thread-Safety

- All incoming data is written to shared variables using a `lock`.
- Thread-safe versions (suffixed with `_ts`) are copied to the main variables each `Update()` frame.

#### 🔁 Network Message Format

Each message received from the server has the following structure (in order):

| Field                 | Type      | Description                     |
|----------------------|-----------|---------------------------------|
| Width, Height        | `int` x2  | Resolution of the frame         |
| RGB Byte Length      | `int`     | Length of the RGB byte array    |
| RGB Data             | `byte[]`  | JPEG or raw RGB image           |
| Depth Byte Length    | `int`     | Length of the depth byte array  |
| Depth Data           | `byte[]`  | 16-bit ushort depth values      |
| Intrinsics (fx, fy, cx, cy) | `float` x4 | Camera parameters        |
| Cull Params (minZ, maxZ, x, y) | `float` x4 | Limits for visibility |

#### 🧩 Key Features

- Real-time rendering of point cloud from RGBD data
- GPU-accelerated instancing for thousands of cubes
- Dynamic server discovery (no IP hardcoding)
- XR controller input for intuitive interaction
- Modular structure for integration into XR or AR pipelines

