import open3d as o3d
import numpy as np
# Load two point clouds in PLY format
source = './Shiba_lego_model/no_tail_NeRF/base.obj' #'./up/extracted_chair_down_reality_align.obj' Shiba_lego_model/
source = o3d.io.read_triangle_mesh(source)
#source_cloud = read_point_cloud("./down/extracted_chair_down_reality_align.obj")
target = o3d.io.read_triangle_mesh("./Shiba_lego_model/with_tail_NeRF/base(1).obj")  #./Shiba_lego_model/with_tail_NeRF/base(1).obj
source_point_cloud = o3d.geometry.PointCloud()
source_point_cloud.points = source.vertices
target_point_cloud = o3d.geometry.PointCloud()
target_point_cloud.points = target.vertices

# Apply voxel downsampling to reduce the number of points (optional but recommended for efficiency)
voxel_size = 0.05  # Voxel size for downsampling (adjust as needed)
source_down = source_point_cloud.voxel_down_sample(voxel_size)
target_down = target_point_cloud.voxel_down_sample(voxel_size)
# Estimate normals (required for some registration methods)
def filter(point_cloud, height = -0.5):
    points = np.asarray(point_cloud.points)
    # Filter points where y > -0.5
    filtered_points = points[points[:, 1] > height]
    # Create a new Open3D point cloud from the filtered points
    filtered_point_cloud = o3d.geometry.PointCloud()
    filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)
    return filtered_point_cloud

source_down = filter(source_down, -0.5)
target_down = filter(target_down, -0.45)
source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
# Apply ICP registration
threshold = 0.2  # Distance threshold (adjust based on your data)
icp_result = o3d.pipelines.registration.registration_icp(
    source_down, target_down, threshold, np.eye(4),
    o3d.pipelines.registration.TransformationEstimationPointToPlane())
# Print the transformation matrix (from source to target)
print(icp_result.transformation)
coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])
points = [[0, 0, 0], [0, -1, 0]]  # Line along the X-axis indicating 1 unit
lines = [[0, 1]]  # Connect the first and second points
colors = [[1, 0, 0]]  # Color the line red
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(lines)
line_set.colors = o3d.utility.Vector3dVector(colors)

# Optionally visualize the aligned point clouds
source.transform(icp_result.transformation)
source_point_cloud.transform(icp_result.transformation)
target_point_cloud_filter = filter(target_point_cloud, -0.45)
# target_point_cloud_filter.paint_uniform_color([.5,.5,.5])
source_point_cloud_filter = filter(source_point_cloud, -0.40)
color_red = [0, 1, 0]  # RGB values for red
num_points_1 = np.asarray(source_point_cloud_filter.points).shape[0]
source_point_cloud_filter.colors = o3d.utility.Vector3dVector(np.tile(color_red, (num_points_1, 1)))
# Set color for point_cloud_2 to blue
color_blue = [0, 0, 1]  # RGB values for blue
num_points_2 = np.asarray(target_point_cloud_filter.points).shape[0]
target_point_cloud_filter.colors = o3d.utility.Vector3dVector(np.tile(color_blue, (num_points_2, 1)))
o3d.visualization.draw_geometries([target_point_cloud_filter, source_point_cloud_filter], "Aligned Point Clouds")

threshold = 0.09
pcd1 = source_point_cloud_filter
pcd2 = target_point_cloud_filter
dists = pcd2.compute_point_cloud_distance(pcd1)
dists = np.asarray(dists)
ind = np.where(dists > threshold)[0]
inverseind = np.where(dists <= threshold)[0]
#Create a version with all the points that are different than the original
mesh2_without_mesh1 = pcd2.select_by_index(ind)
mesh2_with_mesh1 = pcd2.select_by_index(inverseind)
#Make everything the same color and change the color of different points
mesh2_without_mesh1.paint_uniform_color([1,0,0])
mesh2_with_mesh1.paint_uniform_color([.5,.5,.5])
#Combine the two point clouds
mesh2withdifferencesred = mesh2_without_mesh1 + mesh2_with_mesh1
o3d.visualization.draw_geometries([mesh2withdifferencesred])



# o3d.visualization.draw_geometries([target_point_cloud, source_point_cloud], "Aligned Point Clouds")
# o3d.visualization.draw_geometries([source_point_cloud_filter], "Aligned Point Clouds")
# o3d.visualization.draw_geometries([target_point_cloud_filter], "Aligned Point Clouds")
# o3d.visualization.draw_geometries([target_down, coordinate_frame, line_set], "Aligned Point Clouds")
# # o3d.visualization.draw_geometries([filtered_point_cloud], "Aligned Point Clouds")

# vis = o3d.visualization.Visualizer()
# vis.create_window("Aligned Point Clouds", width=800, height=600)
# # Add geometry to the visualizer
# vis.add_geometry(source_point_cloud_filter)
# # Set up the viewpoint
# view_ctl = vis.get_view_control()
# camera_params = view_ctl.convert_to_pinhole_camera_parameters()
# # You can modify the following parameters according to your needs
# camera_params.extrinsic = np.array([
#     [1, 0, 0, 0],  # Rotation part
#     [0, 1, 0, 0],
#     [0, 0, 1, 0],
#     [0, 0, -5, 1]  # Translation part, adjust the Z coordinate as needed
# ])
# camera_params.intrinsic.set_intrinsics(800, 600, 500, 500, 400, 300)  # Example intrinsic parameters
# view_ctl.convert_from_pinhole_camera_parameters(camera_params)
# # Run the visualizer
# vis.run()

# mesh1 = source
# mesh2 = target
# # Directly add the two meshes
# combined_mesh = mesh1 + mesh2
# # Optional: Compute normals for the new mesh
# combined_mesh.compute_vertex_normals()
# # Save the combined mesh to a new OBJ file
# o3d.io.write_triangle_mesh("combined_mesh_after_icp.obj", combined_mesh)

# combined_pcd = target_point_cloud_filter + source_point_cloud_filter
# # Save the combined point cloud
# o3d.io.write_point_cloud("with_tail_point_cloud_after_icp.pcd", combined_pcd)