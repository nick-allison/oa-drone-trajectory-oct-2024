"""Utility to visualize photo plans.
"""

import typing as T

import plotly.graph_objects as go

from src.data_model import Waypoint


def plot_photo_plan(photo_plans: T.List[Waypoint]) -> go.Figure:
    """Plot the photo plan on a 2D grid.

    Args:
        photo_plans (T.List[Waypoint]): List of waypoints for the photo plan.

    Returns:
        T.Any: Plotly figure object.
    """
    raise NotImplementedError()
