"""Utility to visualize photo plans.
"""
import numpy as np

import typing as T

import plotly.graph_objects as go

from src.data_model import Waypoint

def compute_segment_time(
    start_wp: Waypoint,
    end_wp: Waypoint,
    max_acc: float,
    max_speed: float,
    photo_speed: float,
) -> float:
    """Compute the time for a segment, incorporating motion blur constraints.

    Args:
        start_wp (Waypoint): Starting waypoint.
        end_wp (Waypoint): Ending waypoint.
        max_acc (float): Maximum acceleration (m/s^2).
        max_speed (float): Maximum speed (m/s).
        photo_speed (float): Speed limit during photo capture to prevent motion blur (m/s).
        exposure_time (float): Time required for photo capture (s).

    Returns:
        float: Total time for the segment (s).
    """
    x_dist = end_wp.x_pos - start_wp.x_pos
    y_dist = end_wp.y_pos - start_wp.y_pos
    z_dist = end_wp.z_pos - start_wp.z_pos
    euc_dist = np.sqrt(x_dist**2 + y_dist**2 + z_dist**2)

    initial_speed = start_wp.max_speed
    cruise_speed = min(max_speed, photo_speed)  # Constrain speed by photo capture requirements
    final_speed = min(np.sqrt(initial_speed**2 + 2 * max_acc * euc_dist), cruise_speed)

    accel_time = (final_speed - initial_speed) / max_acc
    accel_dist = (initial_speed + final_speed) / 2 * accel_time

    if accel_dist < euc_dist:
        cruise_dist = euc_dist - accel_dist
        cruise_time = cruise_dist / final_speed
    else:
        accel_time = np.sqrt(2 * euc_dist / max_acc)
        cruise_time = 0

    return accel_time + cruise_time


def plot_photo_plan(
    waypoints: T.List[Waypoint],
    image_footprint_size: np.array,
    max_acc: float,
    max_speed: float,
    photo_speed: float,
):
    """Plot the photo plan on a 2D grid.

    Args:
        waypoints (T.List[Waypoint]): List of waypoints for the photo plan.
        image_footprint_size (np.array): The size of the image footprints in meters.
        max_acc (float): Maximum acceleration of the drone in m/s^2.
        max_speed (float): Maximum speed of the drone in m/s.
        photo_speed (float): Speed limit during photo capture to prevent motion blur (m/s).
        exposure_time (float): Time in seconds required for each image capture.
        show_waypoints_only (T.Optional[T.List[int]]): List of waypoint indices whose footprints should be shown.
                                                       If None, show all footprints.

    Returns:
        T.Any: Plotly figure object.
    """
    x_positions = [wp.x_pos for wp in waypoints]
    y_positions = [wp.y_pos for wp in waypoints]
    z_positions = [wp.z_pos for wp in waypoints]

    cumulative_time = 0
    times = []
    time_labels = []

    for i in range(len(waypoints) - 1):
        segment_time = compute_segment_time(
            waypoints[i], waypoints[i + 1], max_acc, max_speed, photo_speed
        )
        cumulative_time += segment_time
        times.append(segment_time)
        time_labels.append(f"{cumulative_time:.2f}s")

    flight_path_trace = go.Scatter3d(
        x=x_positions,
        y=y_positions,
        z=z_positions,
        mode='lines+markers',
        marker=dict(size=6, color=cumulative_time, colorscale='Viridis', showscale=True),
        line=dict(color='blue', width=2),
        name='Flight Path'
    )

    footprint_traces = []
    for i, wp in enumerate(waypoints):
        corners = [
            [wp.x_pos - image_footprint_size[0] / 2, wp.y_pos - image_footprint_size[1] / 2, 0],
            [wp.x_pos + image_footprint_size[0] / 2, wp.y_pos - image_footprint_size[1] / 2, 0],
            [wp.x_pos + image_footprint_size[0] / 2, wp.y_pos + image_footprint_size[1] / 2, 0],
            [wp.x_pos - image_footprint_size[0] / 2, wp.y_pos + image_footprint_size[1] / 2, 0],
            [wp.x_pos - image_footprint_size[0] / 2, wp.y_pos - image_footprint_size[1] / 2, 0]
        ]
        x_fp, y_fp, z_fp = zip(*corners)
        footprint_trace = go.Scatter3d(
            x=x_fp, y=y_fp, z=z_fp,
            mode='lines',
            line=dict(color='red', width=2),
            name=f'Footprint {i}'
        )
        footprint_traces.append(footprint_trace)

    traces = [flight_path_trace] + footprint_traces

    for i, (x, y, z, label) in enumerate(zip(x_positions, y_positions, z_positions, time_labels)):
        traces.append(go.Scatter3d(
            x=[x], y=[y], z=[z],
            mode='text',
            text=[label],
            textposition='top center',
            name=f'Time {i}'
        ))

    fig = go.Figure(data=traces)
    fig.update_layout(
        title="Drone Flight Path with Image Footprints",
        width=900,
        height=900,
        scene=dict(
            xaxis_title="X (meters)",
            yaxis_title="Y (meters)",
            zaxis_title="Z (meters)"
        )
    )
    return fig