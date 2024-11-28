import typing as T
import math

import numpy as np

from src.data_model import Camera, DatasetSpec, Waypoint
from src.camera_utils import compute_image_footprint_on_surface, compute_ground_sampling_distance


def compute_distance_between_images(camera: Camera, dataset_spec: DatasetSpec) -> np.ndarray:
    """Compute the distance between images in the horizontal and vertical directions for specified overlap and sidelap.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.

    Returns:
        float: The distance between images in the horizontal direction.
        float: The distance between images in the vertical direction.
    """
    
    try:
        footprint = compute_image_footprint_on_surface(camera, dataset_spec.height)

        if dataset_spec.overlap >= 0 and dataset_spec.overlap <= 1:
            x = (1 - dataset_spec.overlap) * footprint[0]
        else:
            raise Exception("Invalid Camera Overlap")

        if dataset_spec.sidelap >= 0 and dataset_spec.sidelap <= 1:
            y = (1 - dataset_spec.sidelap) * footprint[1]
        else:
            raise Exception("Invalid Camera Sidelap")

        return np.array([x, y] , dtype=np.float32)
    
    except Exception as e:
        print("Error: " + str(e))

def compute_speed_during_photo_capture(camera: Camera, dataset_spec: DatasetSpec, allowed_movement_px: float = 1) -> float:
    """Compute the speed of drone during an active photo capture to prevent more than 1px of motion blur.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.
        allowed_movement_px (float, optional): The maximum allowed movement in pixels. Defaults to 1 px.

    Returns:
        float: The speed at which the drone should move during photo capture.
    """
    gsd = compute_ground_sampling_distance(camera, dataset_spec.height)
    ms = dataset_spec.exposure_time_ms
    return ((gsd * 1000) / ms) * allowed_movement_px #coorect would be to divide ms by 1000 to get seconds, 
    #but this is equivalent and I want to make it hard for the floats to get really small.


def generate_photo_plan_on_grid(camera: Camera, dataset_spec: DatasetSpec) -> T.List[Waypoint]:
    """Generate the complete photo plan as a list of waypoints in a lawn-mower pattern.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.

    Returns:
        List[Waypoint]: scan plan as a list of waypoints.

    """
    way_point_list = []

    max_speed = compute_speed_during_photo_capture(camera=camera, dataset_spec=dataset_spec)
    dist_x, dist_y = compute_distance_between_images(camera=camera, dataset_spec=dataset_spec)
    
    if dataset_spec.scan_dimension_x % dist_x == 0:
        x_points = int(dataset_spec.scan_dimension_x //dist_x)
    else:
        x_points = int(dataset_spec.scan_dimension_x //dist_x) + 1

    if dataset_spec.scan_dimension_y % dist_y == 0:
        y_points = int(dataset_spec.scan_dimension_y //dist_y)
    else:
        y_points = int(dataset_spec.scan_dimension_y // dist_y) + 1

    x_len = dataset_spec.scan_dimension_x / x_points
    y_len = dataset_spec.scan_dimension_y / y_points

    for j in range(y_points + 1):
        if j % 2 == 0:
            for i in range(x_points + 1):
                way_point_list.append(Waypoint(x_pos = i * x_len, 
                                               y_pos = j * y_len,
                                               z_pos=dataset_spec.height,
                                               max_speed=max_speed))
        else:
            for i in range(x_points, -1, -1):
                way_point_list.append(Waypoint(x_pos = i * x_len, 
                                               y_pos = j * y_len,
                                               z_pos=dataset_spec.height,
                                               max_speed=max_speed))
                
    return(way_point_list)


def compute_time(waypoint_list, max_acc, max_vel):
    total_time = 0
    for i in range(len(waypoint_list) - 1):
        y_dist = waypoint_list[i + 1].y_pos - waypoint_list[i].y_pos
        x_dist = waypoint_list[i + 1].x_pos - waypoint_list[i].x_pos
        euc_dist = np.sqrt(y_dist**2 + x_dist**2)

        initial_vel = waypoint_list[i].max_speed
        final_vel = min(np.sqrt(initial_vel**2 + 2 * max_acc * euc_dist), max_vel)

        delta_vel = final_vel - initial_vel
        accel_time = delta_vel / max_acc
        accel_dist = (initial_vel + final_vel) / 2 * accel_time

        if accel_dist < euc_dist:
            cruise_dist = euc_dist - accel_dist
            cruise_time = cruise_dist / final_vel
        else:
            accel_time = np.sqrt(2 * euc_dist / max_acc)
            cruise_time = 0

        segment_time = accel_time + cruise_time
        total_time += segment_time
        print(f"Time for segment {i}: {segment_time:.2f} seconds")

    return total_time