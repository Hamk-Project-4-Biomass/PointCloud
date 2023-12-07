# PointCloudGen Class

This class represents a Point Cloud Generator that takes depth and rgb images and converts them to pointclouds.

## Description

The `PointCloudGen` class provides methods for setting camera parameters, loading depth and RGB images, calculating 3D points for a point cloud, generating an Open3D point cloud object, and exporting the point cloud to a .ply file.

## Attributes

**Note:** Attributes with a double underscore prefix are private and cannot not be accessed directly.

- `__pcd_o3d`: Open3D PointCloud object.
- `__FX_DEPTH`, `__FY_DEPTH`, `__CX_DEPTH`, `__CY_DEPTH`: Depth camera parameters.
- `__FX_RGB`, `__FY_RGB`, `__CX_RGB`, `__CY_RGB`: RGB camera parameters.
- `__R`: Rotation matrix.
- `__T`: Translation vector.
- `__depth_image_path`: Path to the depth image.
- `__rgb_image_path`: Path to the RGB image.
- `__depth_image`: Loaded depth image.
- `__rgb_image`: Loaded RGB image.

## Methods

### `__init__(self)`

Constructor method for the `PointCloudGen` class.

### `set_depth_cam_settings(self, fx: float, fy: float, ppx: float, ppy: float)`

Set depth camera parameters.

### `set_rgb_cam_settings(self, fx: float, fy: float, ppx: float, ppy: float)`

Set RGB camera parameters.

### `set_depth_image_path(self, path: str)`

Set the path to the depth image.

### `set_rgb_image_path(self, path: str)`

Set the path to the RGB image.

### `get_depth_image(self) -> np.ndarray`

Get the loaded depth image.  
Returns the depth image as a numpy array.

### `get_rgb_image(self) -> np.ndarray`

Get the loaded RGB image.  
Returns the RGB image as a numpy array.

### `calculate_points(self, depth_image: Optional[np.ndarray] = None, rgb_image: Optional[np.ndarray] = None) -> list, list`

Calculate 3D points for the point cloud based on depth and RGB images.  
Returns a list of 3D points and a list of RGB colors for thoses points.

### `generate_point_cloud(self, pcd_depth: List[List[float]], pcd_colors: List[List[float]]) -> o3d.geometry.PointCloud`

Generate an Open3D point cloud object.  
Returns the Open3D point cloud object.

### `export_point_cloud(self, pcd_o3d: o3d.geometry.PointCloud, export_path: str) -> string|None`

Export the Open3D point cloud to a .ply file.  
Returns the path to the exported file on success, `None` otherwise.

### `display_point_cloud(self, pcd_o3d)`

Display the Open3D point cloud object.

## Examples

### Generating a Point Cloud and Exporting to .ply File

```python
# Creating the point cloud generator object
pcg = PointCloudGen()

# Setting the camera parameters
pcg.set_depth_cam_settings(FX_DEPTH, FY_DEPTH, CX_DEPTH, CY_DEPTH)
pcg.set_rgb_cam_settings(FX_RGB, FY_RGB, CX_RGB, CY_RGB)

# Setting the image paths to the object. The methods will automatically read the images
pcg.set_depth_image_path("path_to_depth_image.png")
pcg.set_rgb_image_path("path_to_rgb_image.png")

# Calculating the 3D points and RGB colors for the point cloud
pcd_depth, pcd_rgb = pcg.calculate_points()
# Generating the Open3D point cloud object
pcd_o3d = pcg.generate_point_cloud(pcd_depth, pcd_rgb)

# Exporting the point cloud to a point_cloud.ply file
print(pcg.export_point_cloud(pcd_o3d, "point_cloud"))
```