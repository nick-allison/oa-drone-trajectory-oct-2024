"""Utility functions for the camera model.
"""
import numpy as np

from src.data_model import Camera

def compute_focal_length_in_mm(camera: Camera) -> np.ndarray:
    """Computes the focal length in mm for the given camera

    Args:
        camera (Camera): the camera model.

    Returns:
        np.ndarray: [fx, fy] in mm.
    """
    raise NotImplementedError() 
    # Note(Ayush): Solution provided by project leader.
    # pixel_to_mm_x = camera.sensor_size_x_mm / camera.image_size_x_px
    # pixel_to_mm_y = camera.sensor_size_y_mm / camera.image_size_y_px

    # return np.array([camera.fx * pixel_to_mm_x, camera.fy * pixel_to_mm_y])

def project_world_point_to_image(camera: Camera, point: np.ndarray) -> np.ndarray:
    """Project a 3D world point into the image coordinates.

    Args:
        camera (Camera): the camera model
        point (np.ndarray): the 3D world point

    Returns:
        np.ndarray: [u, v] pixel coordinates corresponding to the point.
    """
    x = camera.fx * (point[0] / point[2])
    y = camera.fy * (point[1] / point[2])

    u = x + camera.cx
    v = y + camera.cy
    
    return np.array([u, v], dtype=np.float32)


def compute_image_footprint_on_surface(camera: Camera, distance_from_surface: float) -> np.ndarray:
    """Compute the footprint of the image captured by the camera at a given distance from the surface.

    Args:
        camera (Camera): the camera model.
        distance_from_surface (float): distance from the surface (in m).

    Returns:
        np.ndarray: [footprint_x, footprint_y] in meters.
    """
    X = (camera.image_size_x_px * distance_from_surface) / camera.fx
    Y = (camera.image_size_y_px * distance_from_surface) / camera.fy

    return np.array([X, Y], dtype=np.float32) 


#Doesn't work - changing strategies
def compute_footprint_non_nadir_x_only(camera: Camera, distance_from_surface: float, x_angle:float=0):
    """Compute the footprint of the image captured by the camera at a given distance from the surface
    where the camera is rotated in the x direction by the x_angle parameter in radians..

    Args:
        camera (Camera): the camera model.
        distance_from_surface (float): distance from the surface (in m).
        x_angle: angle that the camera is rotated in the x direction in degrees.  The default is 0.

    Returns:
    the four corner of the trapezoid shaped footprint in 
        np.ndarray: [[x, y]], 
                    [x, y], 
                    [x, y], 
                    [x, y]] in meters.

    in cartesian coordinates with origin at the point on the ground with the same
    x and y of the center of the camera.
    """

    x_angle = np.deg2rad(x_angle)

    X = (camera.image_size_x_px * distance_from_surface) / camera.fx
    Y = (camera.image_size_y_px * distance_from_surface) / camera.fy

    #Angle between line from middle of camera to ground 
    # and line from middle of camera to the right or left
    # side of the image 
    orig_x_angle = np.arctan((X / 2) / distance_from_surface)
    #length of the center of the camera to the horizontal edge of the image
    orig_hyp = (1 / np.cos(orig_x_angle)) * distance_from_surface
    #adjust one of the angles
    try:
        x_angle_1 = orig_x_angle + x_angle
        if x_angle_1 > np.pi / 2 or x_angle_1 < -np.pi / 2:
            raise
    except:
        print("Error: Invalid angle (rotates the camera too much)")
        return
    #the new hypotenuse
    hyp_1 = (1 / np.cos(x_angle_1)) * distance_from_surface
    #adjust the other angle
    try:
        x_angle_2 = orig_x_angle - x_angle
        if x_angle_2 > np.pi / 2 or x_angle_2 < -np.pi / 2:
            raise
    except:
        print("Error: Invalid angle (rotates the camera too much)")
        return
    #and the hypotenuse
    hyp_2 = (1 / np.cos(x_angle_2)) * distance_from_surface

    #distance of one of the edges of the image to the 
    # point directly below the center of the camera
    dist1 = np.cos(x_angle_1) * distance_from_surface
    #distance of the other edge of the image to the 
    # point directly below the center of the camera
    dist2 = np.cos(x_angle_2) * distance_from_surface

    #the ratio of the new hypotenuse to the old is 
    # the same as the ratio of the new edge length to the old
    side_len_1 = (hyp_1 / orig_hyp) * Y
    side_len_2 = (hyp_2 / orig_hyp) * Y

    #The result is a trapezoid an edge dist1 away from the center of the camera
    #and side_len_1 long and an edge dist2 away from the centerof the camera
    #connected together.
    return np.array([[-side_len_1 / 2, dist1], 
                    [side_len_1 / 2, dist1],
                    [-side_len_2 / 2, dist2],
                    [side_len_2 / 2, dist2]], 
                    dtype=np.float32) 



def compute_footprint_non_nadir(camera: Camera, distance_from_surface: float, x_angle:float=0, y_angle:float=0):
    """Compute the footprint of the image captured by the camera at a given distance from the surface
        in th general case where the x_angle y_angle rotation of the camera are provided.

    Args:
        camera (Camera): the camera model.
        distance_from_surface (float): distance from the surface (in m).
        x_angle: angle that the camera is rotated in the x direction in radians.  The default is 0.
        y_angle: angle that the camera is rotated in the y direction in radians.  The default is 0.

    Returns:
    the four corner of the  footprint 
        np.ndarray: [[x, y]], 
                    [x, y], 
                    [x, y], 
                    [x, y]] in meters.

    in cartesian coordinates with origin at the point on the ground with the same
    x and y of the center of the camera.
    """

    x_angle = np.deg2rad(x_angle) #it was going backwards somewhere, so this corrects it
    y_angle = -np.deg2rad(y_angle)

    X = (camera.image_size_x_px * distance_from_surface) / camera.fx
    Y = (camera.image_size_y_px * distance_from_surface) / camera.fy

    corners = np.array([
        [-X / 2, Y / 2, distance_from_surface],
        [X / 2, Y / 2, distance_from_surface],
        [-X / 2, -Y / 2, distance_from_surface],
        [X / 2, -Y / 2, distance_from_surface]
    ])

    y_rot = np.array([
        [1, 0, 0],
        [0, np.cos(y_angle), -np.sin(y_angle)],
        [0, np.sin(y_angle), np.cos(y_angle)]
    ])

    x_rot = np.array([
        [np.cos(x_angle), 0, np.sin(x_angle)],
        [0, 1, 0],
        [-np.sin(x_angle), 0, np.cos(x_angle)]
    ])

    R = y_rot @ x_rot

    rotated_corners = np.dot(corners, R.T)

    footprint = []
    for corner in rotated_corners:
        x, y, z = corner
        scale = distance_from_surface / z if z != 0 else 0 
        footprint.append([x * scale, y * scale])

    return np.array(footprint)[[0, 1, 3, 2]]

def compute_ground_sampling_distance(camera: Camera, distance_from_surface: float) -> float:
    """Compute the ground sampling distance (GSD) at a given distance from the surface.

    Args:
        camera (Camera): the camera model.
        distance_from_surface (float): distance from the surface (in m).
    
    Returns:
        float: the GSD in meters (smaller among x and y directions).
    """
    X = (camera.image_size_x_px * distance_from_surface) / camera.fx
    Y  = (camera.image_size_y_px * distance_from_surface) / camera.fy
    return min((X / camera.image_size_x_px), (Y / camera.image_size_y_px))

def reproject_image_point_to_world(camera: Camera, point: np.ndarray, distance: float) -> np.ndarray:
    """Compute the reprojected point a 2d projected image point and the distance from the surface.

    Args:
        camera (Camera): the camera model.
        point (np.ndarray): the 2D projected image point
        distance (float): distance from the surface (in m).
    
    Returns:
        np.ndarray: [X, Y, Z] reprojected 3D point.
    """
    x = point[0] - camera.cx
    y = point[1] - camera.cy

    X = (x * distance) / camera.fx
    Y = (y * distance) / camera.fy
    Z = distance

    return np.array([X, Y, Z], dtype=np.float32) 