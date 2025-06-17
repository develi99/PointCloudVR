import streamer.Source as s
import streamer.ProcessingStep as p
import streamer.Actions as a
import streamer.Datasources as ds
import cv2


"""
In this example a Internet Streamer is used, as source. The Data is then send to the Meta Quest
You can also use this package as a streamer and just set the ZMQPublishAction to stream the data.
"""


# Setup Source Internet example
# adapt the ip address to the real streamer, here the ZMQServer.py is the streamer
strategy = s.InternetStrategy("10.42.0.27", 35555, 640, 480)
camera = s.CameraContext(strategy)
camera.init()


# Setup Processing
processing = p.DownSampling(blocksize=3)
processing.set_next(p.EncodeRGBAsJPEG())


# Setup Aktionen

# Culling, for the rendering in the MetaQuest
culling = ds.Culling(
    zcullmin = 0.01,
    zcullmax = 2.0,
    x_cull = 1.0,
    y_cull = 1.0
)

actions = a.ActionPipeline()
a1 = a.ShowImageAction()
actions.add_action(a1)
a2 = a.ZMQPublishAction(culling, config_scaling=0.33) # Scaling because image is downsampled

# Width and Height not always the same as the read picture from the camera, because of the processingssteps
# At the moment must be set manually
a2.set_width_height(214, 160)
actions.add_action(a2)


# Run
try:
    while True:
        rgb, depth, config = camera.get_frame()
        rgb, depth = processing.process(rgb, depth)
        a2.set_config(config)
        actions.execute_all(rgb, depth)
except KeyboardInterrupt:
    pass
finally:
    camera.close()
    cv2.destroyAllWindows()
