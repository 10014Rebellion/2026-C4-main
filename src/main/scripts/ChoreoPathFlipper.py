import json
import math
import sys
from pathlib import Path

FIELD_HEIGHT = 8.069326

SIDE_SWAP_REPLACEMENTS = (
    ("Left", "Right"),
    ("TL", "TR"),
    ("BL", "BR"),
    ("BSL", "BSR"),
    ("CSL", "CSR"),
    ("TSL", "TSR"),
)

RIGHT_TOKENS = ("Right", "TR", "BR", "BSR", "CSR", "TSR")


def norm_angle(rad):
    while rad > math.pi:
        rad -= 2 * math.pi
    while rad <= -math.pi:
        rad += 2 * math.pi
    return rad


def swap_side_names(value):
    if isinstance(value, str):
        placeholders = []

        for i, (left, right) in enumerate(SIDE_SWAP_REPLACEMENTS):
            left_token = f"__SIDE_SWAP_LEFT_{i}__"
            right_token = f"__SIDE_SWAP_RIGHT_{i}__"

            value = value.replace(left, left_token)
            value = value.replace(right, right_token)

            placeholders.append((left_token, right_token, left, right))

        for left_token, right_token, left, right in placeholders:
            value = value.replace(left_token, right)
            value = value.replace(right_token, left)

        return value

    if isinstance(value, list):
        return [swap_side_names(v) for v in value]

    if isinstance(value, dict):
        return {k: swap_side_names(v) for k, v in value.items()}

    return value


def get_right_equivalent_path(input_path):
    output_stem = swap_side_names(input_path.stem)
    return input_path.with_name(output_stem + ".traj")


def is_right_path(path):
    return any(token in path.stem for token in RIGHT_TOKENS)


def flip_number_field(obj, key, flip_func):
    if key not in obj:
        return

    field = obj[key]

    if isinstance(field, dict) and "val" in field:
        field["val"] = flip_func(field["val"])

        exp = field.get("exp", "")
        if isinstance(exp, str):
            if exp.endswith(" m"):
                field["exp"] = f"{field['val']} m"
            elif exp.endswith(" rad"):
                field["exp"] = f"{field['val']} rad"

    elif isinstance(field, (int, float)):
        obj[key] = flip_func(field)


def flip_heading_field(wp):
    if "heading" not in wp:
        return

    h = wp["heading"]

    if isinstance(h, dict) and "val" in h:
        new_val = norm_angle(-h["val"])

        if abs(new_val) < 1e-9:
            new_val = 0.0

        h["val"] = new_val

        exp = h.get("exp", "")
        if isinstance(exp, str):
            if "deg" in exp:
                h["exp"] = f"{math.degrees(new_val)} deg"
            elif "rad" in exp:
                h["exp"] = f"{new_val} rad"
            else:
                # DO NOT reswap here
                h["exp"] = exp

    elif isinstance(h, (int, float)):
        new_val = norm_angle(-h)
        if abs(new_val) < 1e-9:
            new_val = 0.0
        wp["heading"] = new_val


def flip_waypoint(wp):
    wp = swap_side_names(wp)
    flip_number_field(wp, "y", lambda y: FIELD_HEIGHT - y)
    flip_heading_field(wp)
    return wp


def flip_constraint(constraint):
    constraint = swap_side_names(constraint)

    data = constraint.get("data", {})
    props = data.get("props", {})

    if data.get("type") == "PointAt":
        flip_number_field(props, "y", lambda y: FIELD_HEIGHT - y)

    return constraint


def make_ungenerated_flipped_traj(input_path):
    input_path = Path(input_path)

    output_path = get_right_equivalent_path(input_path)

    if output_path == input_path:
        print(f"Skipped non-left path: {input_path}")
        return False

    if output_path.exists():
        print(f"Skipped, already exists: {output_path}")
        return False

    with input_path.open("r", encoding="utf-8") as f:
        traj = json.load(f)

    # Only swap name once
    if "name" in traj:
        traj["name"] = swap_side_names(traj["name"])

    if "params" in traj:
        traj["params"]["waypoints"] = [
            flip_waypoint(wp) for wp in traj["params"].get("waypoints", [])
        ]

        traj["params"]["constraints"] = [
            flip_constraint(c) for c in traj["params"].get("constraints", [])
        ]

    # Force ungenerated state
    traj["snapshot"] = {
        "waypoints": [],
        "constraints": [],
        "targetDt": traj.get("params", {}).get("targetDt", {}).get("val", 0.05)
    }

    traj["trajectory"] = {
        "config": None,
        "sampleType": None,
        "waypoints": [],
        "samples": [],
        "splits": []
    }

    traj["events"] = traj.get("events", [])

    with output_path.open("w", encoding="utf-8") as f:
        json.dump(traj, f, indent=1)

    print(f"Created: {output_path}")
    return True


def process_all(directory):
    directory = Path(directory)

    if not directory.exists() or not directory.is_dir():
        print(f"Invalid directory: {directory}")
        sys.exit(1)

    created = 0
    skipped = 0

    for path in sorted(directory.glob("*.traj")):
        if is_right_path(path):
            skipped += 1
            continue

        result = make_ungenerated_flipped_traj(path)
        if result:
            created += 1
        else:
            skipped += 1

    print()
    print(f"Done. Created: {created}, skipped: {skipped}")


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1].lower() == "all":
        process_all(Path.cwd())

    elif len(sys.argv) == 3 and sys.argv[1].lower() == "all":
        process_all(sys.argv[2])

    elif len(sys.argv) == 2:
        make_ungenerated_flipped_traj(sys.argv[1])

    else:
        print("Usage:")
        print("  python main.py path/to/file.traj")
        print("  python main.py all")
        print("  python main.py all path/to/dir")
        sys.exit(1)