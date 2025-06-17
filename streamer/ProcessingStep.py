from abc import ABC, abstractmethod
import numpy as np
from skimage.measure import block_reduce
import cv2



class ProcessingStep(ABC):
    def __init__(self, next_step=None):
        self.next_step = next_step

    def set_next(self, next_step):
        self.next_step = next_step
        return next_step  # Für einfaches Verkettung

    def process(self, rgb_frame, depth_frame):
        # Prozessiere den aktuellen Schritt
        rgb_out, depth_out = self._process(rgb_frame, depth_frame)

        # Übergib weiter, falls noch weitere Steps
        if self.next_step:
            return self.next_step.process(rgb_out, depth_out)
        else:
            return rgb_out, depth_out

    @abstractmethod
    def _process(self, rgb_frame, depth_frame):
        """Implementiere die Verarbeitung des aktuellen Schritts"""
        pass


class EncodeRGBAsJPEG(ProcessingStep):
    def _process(self, rgb_frame, depth_frame):
        ret_rgb, rgb_buf = cv2.imencode('.jpg', rgb_frame)
        return rgb_buf, depth_frame


class DownSampling(ProcessingStep):
    def __init__(self, blocksize):
        super().__init__()
        self.block_size = blocksize

    def _process(self, rgb_frame, depth_frame):
        depth = self.downsample(depth_frame, mode='min')
        rgb = self.downsample(rgb_frame, mode='avg')

        return rgb, depth

    def downsample(self, img, mode='avg'):
        func = np.mean if mode == 'avg' else np.min

        if img.ndim == 3:  # RGB oder multi-channel
            # block_reduce erwartet tuple mit Länge ndim, hier z.B. (block_size, block_size, 1)
            down = block_reduce(img, block_size=(self.block_size, self.block_size, 1), func=func)
        else:
            down = block_reduce(img, block_size=(self.block_size, self.block_size), func=func)

        return down
