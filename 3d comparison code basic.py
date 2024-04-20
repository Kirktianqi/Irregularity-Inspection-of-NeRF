import numpy as np
import open3d as o3d
import copy

if __name__ == "__main__":

    #define a threshold for differences
    threshold = 0.1

    print("Testing mesh in open3d ...")
    mesh1 = o3d.io.read_triangle_mesh("C:/Users/kirkding1/Downloads/Door_closed.obj")
    mesh2 = o3d.io.read_triangle_mesh("C:/Users/kirkding1/Downloads/Door_open.obj")
    #print(mesh1)
    #print(mesh2)
    #print(np.asarray(mesh1.triangles))
    #print(np.asarray(mesh2.triangles))
    #print("")

    #print("Computing normal and rendering it.")
    mesh1.compute_vertex_normals()
    mesh1.paint_uniform_color([.5,.5,.5])
    mesh2.compute_vertex_normals()
    mesh2.paint_uniform_color([1,0,0])
    #print(np.asarray(mesh1.triangle_normals))
    #print(np.asarray(mesh2.triangle_normals))
    #o3d.visualization.draw_geometries([mesh1, mesh2])

    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = mesh1.vertices
    pcd1.colors = mesh1.vertex_colors
    pcd1.normals = mesh1.vertex_normals
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = mesh2.vertices
    pcd2.colors = mesh2.vertex_colors
    pcd2.normals = mesh2.vertex_normals
    o3d.visualization.draw_geometries([pcd1, pcd2])
    #mountaincloud1 = vol1.crop_point_cloud(pcd)

    #Get the distance between all the points in the mesh and pick out all points that are
    #a given distance apart
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
    
    #pcd2.colors = o3d.utility.Vector3dVector(pointcolors)

    #Change the color
    o3d.visualization.draw_geometries([mesh2withdifferencesred])

    #Transform point cloud to mesh
    raidia = [0.005, 0.01, 0.02, 0.04]
    #final_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(mesh2withdifferencesred, o3d.utility.DoubleVector(raidia))

    #Present Final mesh
    #o3d.visualization.draw_geometries([final_mesh])
