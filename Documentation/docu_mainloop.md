## Main Loop

The [Streamer](../Streamer/Streamer.py) represents the main loop of the system. It orchestrates multiple `Source` instances, processes the acquired 3D data, and transmits it to the Meta Quest headset or other clients via ZeroMQ.


### Overview

- Central runtime component of the pipeline
- Connects multiple `Source` instances
- Applies spatial alignment and filtering
- Performs voxel downsampling
- Publishes the final point cloud via ZeroMQ
- All point cloud processing is GPU-accelerated when available


### Processing Pipeline

| Step                    | Description                                                                 |
|-------------------------|-----------------------------------------------------------------------------|
| **Read**                | Reads the Pointclouds from the cameras, waits until every camera has  returned a pointcloud |
| **Transform**           | Align point clouds using calibration extrinsics                             |
| **Merge**               | Concatenate point clouds into one unified representation                    |
| **Crop**                | Apply 3D bounding box to remove irrelevant regions                          |
| **Downsample**          | Reduce number of points via voxel-based downsampling                        |
| **Publish**             | Send via ZMQ using a PUB socket                                             |

 All these Steps are clearly marked in the [Streamer.py](/Streamer.py), which contains the mainloop


 ### Publish

To send point cloud data from the streaming system to the Meta Quest, the class **`ZMQPublishPointCloudAction`** from *[Action.py](../Action.py)* is used.  
In the code, this publisher is created and used as follows:

```python
sending = a.ZMQPublishPointCloudAction()

# Inside the main loop:
sending.set_pointcloud(xyz, rgb)
sending.execute()
```

The method execute() is responsible for transmitting the point cloud to the headset.
It has the following signature:



To publish the data, which means to send the data to the Meta Quest, the ZMQPublishCloudAction from [Action.py](../Action.py) is used.
In the Code its creates with this line
sending = a.ZMQPublishPointCloudAction()

and at the end of the while loop it publishes the date

sending.set_pointcloud(xyz, rgb)
sending.execute()

The Method execute has a parameter    
def execute(self, num_points = 100000):
which is extremly important. This number tells the Meta Quest how big the array should be. So how many points do i render max.
It shoukd represent the max number that you send. For example you have to cameras with 640x480 pixels, then you would first have about 1 million points.
After Cropping and Downsampling there might be about 50000 left. The Problem is, that this number will vary a lot, because of noise and interations in the environment.
So for example put it to 60 thousand and it should work. If you send more points it has undefined behaviour, because this case wasn't implemented, it probably would case a failure inside of Meta Quest and the 
pointcloud would not be shown until you send less or equal the specified number.