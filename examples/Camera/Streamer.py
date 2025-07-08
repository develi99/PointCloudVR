import Source as s
import ProcessingStep as p
import Actions as a
import Datasources as ds
import cv2

# Setup Source Camera example
strategy = s.LuxonisCameraStrategy(640, 480)
camera = s.CameraContext(strategy)
camera.init()

# Setup Processing
processing = p.DownSampling(blocksize=3)
#processing.set_next(p.EncodeRGBAsJPEG())


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

a2 = a.ZMQPublishPointCloudAction(culling, config_scaling=0.33)
#a2 = a.ZMQPublishAction(culling, config_scaling=0.33) # Scaling because image is downsampled

# Not always the same as the read picture from the camera, because of the processingssteps
# At the moment must be set manually
a2.set_width_height(214, 160)
actions.add_action(a2)



# Run
try:
    while True:
        #rgb, depth = camera.get_frame() # If Camera is used
        rgb, depth, config = camera.get_frame() # if internet is used as source
        rgb, depth = processing.process(rgb, depth)
        a2.set_config(config)
        actions.execute_all(rgb, depth)
except KeyboardInterrupt:
    pass
finally:
    camera.close()
    cv2.destroyAllWindows()
