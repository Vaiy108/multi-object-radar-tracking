import numpy as np
from .models import ObjectState

def create_intersection_scenario() -> list[ObjectState]:
    """
    Creates a traffic-like scene:
    - Two vehicles crossing
    - One bicycle/pedestrian-like slow target
    - One vehicle moving away
    """
    #ground truth
    return [
        ObjectState(x=-60.0, y=10.0, vx=8.0, vy=0.0, object_id=1, label="car"),
        ObjectState(x=45.0, y=-35.0, vx=-5.0, vy=4.0, object_id=2, label="car"),
        ObjectState(x=-20.0, y=-20.0, vx=1.5, vy=1.0, object_id=3, label="bicycle"),
        ObjectState(x=10.0, y=60.0, vx=0.0, vy=-6.0, object_id=4, label="truck"),
    ]

def step_objects(objects: list[ObjectState], dt: float) -> None:
    for obj in objects:
        obj.step(dt)
