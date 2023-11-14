import numpy as np
import imageio.v3 as iio

class PointCloudGen:
    
    # Depth camera parameters
    __FX_DEPTH = None
    __FY_DEPTH = None
    __CX_DEPTH = None
    __CY_DEPTH = None
    
    # RGB camera parameters
    __FX_RGB = None
    __FY_RGB = None
    __CX_RGB = None
    __CY_RGB = None
    
    # Rotation matrix
    __R = -np.array([[9.9997798940829263e-01, 5.0518419386157446e-03, 4.3011152014118693e-03],
                    [-5.0359919480810989e-03, 9.9998051861143999e-01, -3.6879781309514218e-03],
                    [- 4.3196624923060242e-03, 3.6662365748484798e-03, 9.9998394948385538e-01]])
    # Translation vector
    __T = np.array([2.5031875059141302e-02, -2.9342312935846411e-04, 6.6238747008330102e-04])

    # Image files and locations
    __depth_image_path = None
    __rgb_image_path = None
    __depth_image = None
    __rgb_image = None
    
    
    def __init__(self):
        pass
        
    def set_depth_cam_settings(self, fx, fy, ppx, ppy):
        self.__FX_DEPTH = fx
        self.__FY_DEPTH = fy
        self.__CX_DEPTH = ppx
        self.__CY_DEPTH = ppy
    
    def set_rgb_cam_settings(self, fx, fy, ppx, ppy):
        self.__FX_RGB = fx
        self.__FY_RGB = fy
        self.__CX_RGB = ppx
        self.__CY_RGB = ppy
        
    def set_depth_image_path(self, path):
        self.__depth_image_path = path
        self.__depth_image = iio.imread(path)
        
    def set_rgb_image_path(self, path):
        self.__rgb_image_path = path
        self.__rgb_image = iio.imread(path)
