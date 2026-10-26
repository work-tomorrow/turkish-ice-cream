"""Offline IK viewer and CSV exporter. Run from the repository root; see README."""

import argparse
import csv
import math

import numpy as np

from core.app.kinematics_helper import ArmIKConfig, ArmIKHelper


# Length: cm, Angle: degrees
CONFIG = ArmIKConfig()


def create_visualizer(ik: ArmIKHelper):
    """Create the figure and sliders; the caller retains sliders until show ends."""
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider

    cfg = ik.config
    view_limit = max(
        cfg.target_good_radius + 5.0,
        cfg.extra_full_radius + 5.0,
        math.ceil((cfg.extra_trigger_radius + ik.approximate_extra_reach_cm() + 10) / 5) * 5,
    )
    fig = plt.figure(figsize=(14, 8))
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax2d = fig.add_subplot(1, 2, 2)
    fig.subplots_adjust(top=0.70, bottom=0.22)
    slider_y = Slider(fig.add_axes([0.20, 0.11, 0.60, 0.03]), "target_y cm",
                      -view_limit, view_limit, valinit=0)
    slider_z = Slider(fig.add_axes([0.20, 0.06, 0.60, 0.03]), "target_z cm",
                      -view_limit, view_limit, valinit=0)
    limits = " | ".join(
        f"{joint}[{limit.min_deg:g},{limit.max_deg:g}] "
        f"sign={cfg.joint_signs[joint]:+g}, zero={cfg.joint_zero_degs[joint]:g}"
        for joint, limit in cfg.joint_limits.items()
    )

    def update(_=None):
        result = ik.solve_yz(slider_y.val, slider_z.val)
        points, _, rod_end, end_forward = ik.forward_kinematics(result.angles_deg)
        # Keep the user's camera angle while redrawing the arm.
        elevation, azimuth = ax3d.elev, ax3d.azim
        ax3d.clear()
        ax3d.view_init(elevation, azimuth)
        ax3d.plot(*points.T, marker="o", linewidth=3)
        for point, label in zip(points, (
            "Base", "J1", "J2 offset", "L_SHORT end", "Rod start", "Ice cream",
        )):
            ax3d.text(*point, label)
        arrow_end = rod_end + end_forward * 20
        ax3d.quiver(*rod_end, *end_forward, length=20, normalize=True)
        axis_ends = np.eye(3) * 30
        for end, label in zip(axis_ends, ("X forward", "Y left/right", "Z up/down")):
            ax3d.quiver(0, 0, 0, *end, arrow_length_ratio=0.1)
            ax3d.text(*(end * 1.1), label)

        # Include the forward arrow and world axes when fitting the view.
        visible = np.vstack((points, arrow_end, axis_ends * 1.1))
        center = (visible.min(axis=0) + visible.max(axis=0)) / 2
        half = max(float(np.ptp(visible, axis=0).max()) / 2 + 10, 25)
        ax3d.set_xlim(center[0] - half, center[0] + half)
        ax3d.set_ylim(center[1] - half, center[1] + half)
        ax3d.set_zlim(center[2] - half, center[2] + half)
        ax3d.set_box_aspect((1, 1, 1))
        ax3d.set(xlabel="X cm", ylabel="Y cm", zlabel="Z cm", title="3D Arm")

        ax2d.clear()
        for radius, style, label in (
            (cfg.extra_trigger_radius, "--", "pitch extra trigger radius"),
            (cfg.extra_full_radius, ":", "pitch extra full radius"),
            (cfg.target_good_radius, "-.", "target reference radius"),
        ):
            ax2d.add_patch(plt.Circle((0, 0), radius, fill=False, linestyle=style, label=label))
        ax2d.scatter([result.target_y_cm], [result.target_z_cm], s=80, label="Target y/z")
        ax2d.scatter([rod_end[1]], [rod_end[2]], s=80, label="Reached y/z")
        ax2d.plot([result.target_y_cm, rod_end[1]], [result.target_z_cm, rod_end[2]],
                  linewidth=1, label="error")
        ax2d.axhline(0, linewidth=1)
        ax2d.axvline(0, linewidth=1)
        ax2d.set(xlim=(-view_limit, view_limit), ylim=(-view_limit, view_limit),
                 xlabel="Y cm, left/right", ylabel="Z cm, up/down", title="Y-Z View")
        ax2d.set_aspect("equal", adjustable="box")
        ax2d.legend(loc="upper right", fontsize="small")

        model = ", ".join(f"{joint}={angle:.1f}" for joint, angle in result.angles_deg.items())
        motor = ", ".join(f"{joint}={angle:.1f}" for joint, angle in result.motor_angles_deg.items())
        fig.suptitle(
            f"IK Visualizer | L_SHORT={cfg.l_short:g} cm, L_ROD={cfg.l_rod:g} cm\n"
            f"model: {model}\nmotor: {motor}\n"
            f"extra pitch/yaw={result.extra_pitch_deg:.1f}/{result.extra_yaw_deg:.1f} deg | "
            f"pitch activation={result.activation:.2f} "
            f"(radial={result.radial_activation:.2f}, limit={result.limit_activation:.2f})\n"
            f"End=({rod_end[0]:.1f}, {rod_end[1]:.1f}, {rod_end[2]:.1f}) cm | "
            f"error={result.error_cm:.3f} cm | reachable={result.reachable}\n"
            f"{result.mode}\n{limits}",
            fontsize=10,
        )
        fig.canvas.draw_idle()

    ik.reset()
    slider_y.on_changed(update)
    slider_z.on_changed(update)
    update()
    return fig, (slider_y, slider_z)


def generate_ik_angle_table(
    y_min=-40,
    y_max=40,
    z_min=-40,
    z_max=40,
    output_path="ik_angle_table.csv",
    reachable_error_threshold_cm=0.5,
    *,
    config=CONFIG,
):
    """Export integer Y/Z points in snake order, using warm starts between points."""
    if y_min > y_max or z_min > z_max:
        raise ValueError("Grid minimum must not exceed maximum")
    if not math.isfinite(reachable_error_threshold_cm) or reachable_error_threshold_cm < 0:
        raise ValueError("Reachable error threshold must be finite and nonnegative")

    ik = ArmIKHelper(config)
    result_fields = [
        "extra_pitch_deg", "extra_yaw_deg", "activation", "radial_activation",
        "limit_activation", "reached_y_cm", "reached_z_cm", "error_cm", "reachable", "mode",
    ]
    fieldnames = [
        "scan_index", "target_y_cm", "target_z_cm",
        *(f"J{i}_model_deg" for i in range(1, 5)),
        *(f"J{i}_motor_deg" for i in range(1, 5)),
        *result_fields,
    ]
    total = reachable_count = 0
    with open(output_path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for y in range(y_min, y_max + 1):
            z_values = (range(z_min, z_max + 1) if (y - y_min) % 2 == 0
                        else range(z_max, z_min - 1, -1))
            for z in z_values:
                result = ik.solve_yz(y, z, reachable_error_threshold_cm=reachable_error_threshold_cm)
                row = {
                    "scan_index": total, "target_y_cm": y, "target_z_cm": z,
                    **{f"{joint}_model_deg": angle for joint, angle in result.angles_deg.items()},
                    **{f"{joint}_motor_deg": angle for joint, angle in result.motor_angles_deg.items()},
                    **{key: getattr(result, key) for key in result_fields},
                }
                writer.writerow({key: round(value, 6) if isinstance(value, float) else value
                                 for key, value in row.items()})
                total += 1
                reachable_count += result.reachable
    print(f"Saved table to: {output_path}")
    print(f"Total points: {total}")
    print(f"Reachable points: {reachable_count}")
    print(f"Unreachable points: {total - reachable_count}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", metavar="PATH", help="Export CSV instead of opening the viewer")
    parser.add_argument("--y-min", type=int, default=-40)
    parser.add_argument("--y-max", type=int, default=40)
    parser.add_argument("--z-min", type=int, default=-40)
    parser.add_argument("--z-max", type=int, default=40)
    args = parser.parse_args()
    if args.y_min > args.y_max or args.z_min > args.z_max:
        parser.error("Grid minimum must not exceed maximum")
    if args.csv:
        generate_ik_angle_table(args.y_min, args.y_max, args.z_min, args.z_max, args.csv)
    else:
        import matplotlib.pyplot as plt

        fig, sliders = create_visualizer(ArmIKHelper(CONFIG))
        plt.show()


if __name__ == "__main__":
    main()
