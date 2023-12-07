import numpy as np
import imageio.v3 as iio
import open3d as o3d
import os

class PointCloudGen:
    
    # Point cloud object
    __pcd_o3d = None
    
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

    # Distance range
    __distance_min = 10
    __distance_max = 50
    
    
    # Constructor
    def __init__(self):
        pass
    
    """
    Set depth camera parameters
    ATTRIBUTES
    ----------
    fx : float
        Focal length in x
    fy : float
        Focal length in y
    ppx : float
        Principal point in x
    ppy : float
        Principal point in y
    """
    def set_depth_cam_settings(self, fx, fy, ppx, ppy):
        self.__FX_DEPTH = fx
        self.__FY_DEPTH = fy
        self.__CX_DEPTH = ppx
        self.__CY_DEPTH = ppy
    
    """
    Set RGB camera parameters
    ATTRIBUTES
    ----------
    fx : float
        Focal length in x
    fy : float
        Focal length in y
    ppx : float
        Principal point in x
    ppy : float
        Principal point in y
    """
    def set_rgb_cam_settings(self, fx, fy, ppx, ppy):
        self.__FX_RGB = fx
        self.__FY_RGB = fy
        self.__CX_RGB = ppx
        self.__CY_RGB = ppy

    """
    Set rotation matrix of the camera
    ATTRIBUTES
    ----------
    rotation_matrix : numpy array
        Rotation matrix
    """
    def set_rotation_matrix(self, rotation_matrix):
        self.rotation_matrix = rotation_matrix

    """
    Set translation vector of the camera
    ATTRIBUTES
    ----------
    translation_vector : numpy array
        Translation vector
    """
    def set_translation_vector(self, translation_vector):
        self.translation_vector = translation_vector
        
    """
    Set depth image path
    ATTRIBUTES
    ----------
    path : string
        Path to the depth image
    """
    def set_depth_image_path(self, path):
        self.__depth_image_path = path
        self.__depth_image = iio.imread(path)
        
    """
    Set RGB image path
    ATTRIBUTES
    ----------
    path : string
        Path to the RGB image
    """
    def set_rgb_image_path(self, path):
        self.__rgb_image_path = path
        self.__rgb_image = iio.imread(path)
    
    """
    Get depth image
    RETURNS
    -------
    image : numpy array
        Depth image
    """
    def get_depth_image(self):
        if self.__depth_image is None:
            print("Error: Depth image is not set")
            return None
        return self.__depth_image
    
    """
    Get RGB image
    RETURNS
    -------
    image : numpy array
        RGB image
    """
    def get_rgb_image(self):
        if self.__rgb_image is None:
            print("Error: RGB image is not set")
            return None
        return self.__rgb_image

    """
    Calculate points for the pointcloud
    """
    def calculate_points(self, depth_image = None, rgb_image = None, distance_min = None, distance_max = None):
        
        if depth_image != None and rgb_image != None and __distance_min != None and __distance_max != None:
            __depth_image = depth_image
            __rgb_image = rgb_image
            __distance_min = distance_min
            __distance_max = distance_max
        else:
            __depth_image = self.__depth_image
            __rgb_image = self.__rgb_image
            __distance_min = self.__distance_min
            __distance_max = self.__distance_max

        height, width, index = __depth_image.shape
    
        __pcd_depth = []
        __pcd_colors = []
        
        for i in range(height):
            for j in range(width):
                """
                    Convert the pixel from depth coordinate system
                    to depth sensor 3D coordinate system
                """
                z = __depth_image[i][j]
                x = (j - self.__CX_DEPTH) * z / self.__FX_DEPTH
                y = (i - self.__CY_DEPTH) * z / self.__FY_DEPTH

                """
                    Convert the point from depth sensor 3D coordinate system
                    to rgb camera coordinate system:
                """
                [x_RGB, y_RGB, z_RGB] = np.linalg.inv(self.__R).dot([x, y, z]) - np.linalg.inv(self.__R).dot(self.__T)

                """
                    Convert from rgb camera coordinates system
                    to rgb image coordinates system:
                """
                j_rgb = int((x_RGB[0] * self.__FX_RGB) / z_RGB[0] + self.__CX_RGB + width / 2)
                i_rgb = int((y_RGB[0] * self.__FY_RGB) / z_RGB[0] + self.__CY_RGB)

                # Add point to point cloud:
                __pcd_depth.append([x[0], y[0], z[0]])

                # Add the color of the pixel if it exists:
                if 0 <= j_rgb < width and 0 <= i_rgb < height:
                    __pcd_colors.append(__rgb_image[i_rgb][j_rgb] / 255)
                else:
                    __pcd_colors.append([0., 0., 0.])

        # remove the background:
        # Convert __pcd_depth to a numpy array
        __pcd_depth_np = np.array(__pcd_depth)
        __pcd_colors_np = np.array(__pcd_colors)
        
        # Assuming pcd_depth is your point cloud as a numpy array
        z = __pcd_depth_np[:, 2]  # get the z-coordinates

        # Create a mask of points that are within the distance range
        mask = (z > __distance_min) & (z < __distance_max)

        # Apply the mask to the point cloud
        filtered_pcd_depth = __pcd_depth_np[mask]
        filtered_pcd_colors = __pcd_colors_np[mask]

        # mirror the point cloud:
        filtered_pcd_depth_mirrored, filtered_pcd_colors_mirrored = self.mirror_point_cloud(filtered_pcd_depth, filtered_pcd_colors)        

        return filtered_pcd_depth_mirrored, filtered_pcd_colors_mirrored
    
    """
    Mirror point cloud
    ATTRIBUTES
    ----------
    pcd_depth : numpy array
        Point cloud depth
    pcd_colors : numpy array
        Point cloud colors
    RETURNS
    -------
    pcd_depth_mirrored : numpy array
        Point cloud depth mirrored
    pcd_colors_mirrored : numpy array
        Point cloud colors mirrored
    """
    def mirror_point_cloud(self, pcd_depth, pcd_colors):
        
        __pcd_depth_mirrored = []
        __pcd_colors_mirrored = []
        
        for i in range(len(pcd_depth)):
            __pcd_depth_mirrored.append([pcd_depth[i][0], -pcd_depth[i][1], pcd_depth[i][2]])
            __pcd_colors_mirrored.append([pcd_colors[i][0], pcd_colors[i][1], pcd_colors[i][2]])
        
        return __pcd_depth_mirrored, __pcd_colors_mirrored

    """
    Generate point cloud
    ATTRIBUTES
    ----------
    pcd_depth : numpy array
        Point cloud depth
    pcd_colors : numpy array
        Point cloud colors
    RETURNS
    -------
    pcd_o3d : open3d.geometry.PointCloud
        Point cloud object
    """
    def generate_point_cloud(self, pcd_depth, pcd_colors):
        
        __pcd_o3d = o3d.geometry.PointCloud()  # create point cloud object
        __pcd_o3d.points = o3d.utility.Vector3dVector(pcd_depth)  # set pcd_np as the point cloud points
        __pcd_o3d.colors = o3d.utility.Vector3dVector(pcd_colors)
        
        return __pcd_o3d
    
    """
    Export point cloud to .ply file
    ATTRIBUTES
    ----------
    pcd_o3d : open3d.geometry.PointCloud
        Point cloud object
    export_path : string
        Path to the .ply file
    RETURNS
    -------
    export_path : string
        Path to the .ply file
    """
    def export_point_cloud(self, pcd_o3d, export_path):
        
        if pcd_o3d is None or export_path is None:
            print("Error: point cloud or export path is None")
            return None

        if not export_path.endswith('.ply'):
            export_path += '.ply'
    
        # Check if the directory exists and create it if it doesn't
        directory = os.path.dirname(export_path)
        if not os.path.exists(directory):
            os.makedirs(directory)
            
        success = o3d.io.write_point_cloud(export_path, pcd_o3d)
        
        if (success):
            return export_path
        else:
            return None
    
    """
    Display point cloud in a open3d window
    ATTRIBUTES
    ----------
    pcd_o3d : open3d.geometry.PointCloud
        Point cloud object
    """
    def display_point_cloud(self, pcd_o3d):
        o3d.visualization.draw_geometries([pcd_o3d])