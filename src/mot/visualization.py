from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

def plot_scene(truth_history, track_history, output_path: str | Path):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    plt.figure(figsize=(9, 7))

    for obj_id, states in truth_history.items():
        arr = np.array(states)
        plt.plot(arr[:, 0], arr[:, 1], linewidth=2, label=f"truth {obj_id}")

    for track_id, positions in track_history.items():
        if len(positions) < 2:
            continue

        # positions now contains: [(step, np.array([x, y])), ...]
        xy = [pos for step, pos in positions]
        arr = np.array(xy)

        plt.plot(
            arr[:, 0],
            arr[:, 1],
            linestyle="--",
            label=f"track {track_id}"
        )

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Multi-Object Radar Tracking: Truth vs Estimated Tracks")
    plt.axis("equal")
    plt.grid(True)
    plt.legend(loc="best", fontsize=8)
    plt.tight_layout()
    plt.savefig(output_path, dpi=160)
    plt.close()
