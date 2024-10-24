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
    X = (camera.image_size_x * distance_from_surface) / camera.fx
    Y = (camera.image_size_y * distance_from_surface) / camera.fy

    return np.array([X, Y], dtype=np.float32) 

def compute_ground_sampling_distance(camera: Camera, distance_from_surface: float) -> float:
    """Compute the ground sampling distance (GSD) at a given distance from the surface.

    Args:
        camera (Camera): the camera model.
        distance_from_surface (float): distance from the surface (in m).
    
    Returns:
        float: the GSD in meters (smaller among x and y directions).
    """
    X = (camera.image_size_x * distance_from_surface) / camera.fx
    Y  = (camera.image_size_y * distance_from_surface) / camera.fy
    return min((X / camera.image_size_x), (Y / camera.image_size_y))

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