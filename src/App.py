from Class.PointCloudGen import PointCloudGen
from datetime import datetime

# Depth camera parameters
FX_DEPTH = 5.8262448167737955e+02
FY_DEPTH = 5.8269103270988637e+02
CX_DEPTH = 3.1304475870804731e+02
CY_DEPTH = 2.3844389626620386e+02

# RGB camera parameters
FX_RGB = 5.1885790117450188e+02
FY_RGB = 5.1946961112127485e+02
CX_RGB = 3.2558244941119034e+0
CY_RGB = 2.5373616633400465e+02

if __name__ == "__main__":
    pcg = PointCloudGen()
    
    pcg.set_depth_cam_settings(FX_DEPTH, FY_DEPTH, CX_DEPTH, CY_DEPTH)
    pcg.set_rgb_cam_settings(FX_RGB, FY_RGB, CX_RGB, CY_RGB)
    
    pcg.set_depth_image_path("./images/png/depth/2023-11-09.16.25.23_depth_image.png")
    pcg.set_rgb_image_path("./images/png/color/2023-11-09.16.25.23_color_image.png")
    
    pcd_depth, pcd_rgb = pcg.calculate_points()
    pcd_o3d = pcg.generate_point_cloud(pcd_depth, pcd_rgb)
    
    print(pcg.export_point_cloud(pcd_o3d, f"output/pointcloud/pointcloud_{datetime.now().strftime('%Y%m%d_%H_%M_%S.%f')[:-3]}"))
