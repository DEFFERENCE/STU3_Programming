from matplotlib.textpath import TextPath
from matplotlib.font_manager import FontProperties
from matplotlib.path import Path
import numpy as np
import matplotlib.pyplot as plt

# === Interpolation for smoother path ===
def interpolate_path(path, num_points=6):
    interpolated = []
    current_pos = np.array([0.0, 0.0])
    i = 0
    while i < len(path.codes):
        code = path.codes[i]
        if code == Path.MOVETO:
            current_pos = path.vertices[i]
            interpolated.append(current_pos.copy())
            i += 1
        elif code == Path.LINETO:
            start = current_pos
            end = path.vertices[i]
            line = np.linspace(start, end, num_points)
            interpolated.extend(line[1:])
            current_pos = end
            i += 1
        elif code == Path.CURVE3:
            p0 = current_pos
            p1 = path.vertices[i]
            p2 = path.vertices[i + 1]
            t = np.linspace(0, 1, num_points)
            curve = (1 - t)[:, None]**2 * p0 + \
                    2 * (1 - t)[:, None] * t[:, None] * p1 + \
                    t[:, None]**2 * p2
            interpolated.extend(curve[1:])
            current_pos = p2
            i += 2
        elif code == Path.CURVE4:
            p0 = current_pos
            p1 = path.vertices[i]
            p2 = path.vertices[i + 1]
            p3 = path.vertices[i + 2]
            t = np.linspace(0, 1, num_points)
            curve = (1 - t)[:, None]**3 * p0 + \
                    3 * (1 - t)[:, None]**2 * t[:, None] * p1 + \
                    3 * (1 - t)[:, None] * t[:, None]**2 * p2 + \
                    t[:, None]**3 * p3
            interpolated.extend(curve[1:])
            current_pos = p3
            i += 3
        else:
            i += 1
    return np.array(interpolated)

# === Generate the word path ===
def get_smooth_word_path(word, font='OCR A Extended', height_mm=80, interp_points=3):
    mm_per_pt = 0.3528
    size_pt = height_mm / mm_per_pt
    font_prop = FontProperties(family=font, size=size_pt)
    text_path = TextPath((0, 0), word, prop=font_prop)
    vertices = text_path.vertices.copy()
    min_x, min_y = np.min(vertices, axis=0)
    vertices -= [min_x, min_y]
    path = Path(vertices, text_path.codes)
    return interpolate_path(path, num_points=interp_points)

# === Convert Cartesian to Polar ===
def cartesian_to_polar(path, revolute_deg_range=(10, 130)):
    x_min, x_max = np.min(path[:, 0]), np.max(path[:, 0])
    theta_range = np.radians(revolute_deg_range[1] - revolute_deg_range[0])
    theta_start = np.radians(revolute_deg_range[0])
    norm_x = (path[:, 0] - x_min) / (x_max - x_min)
    angles = theta_start + norm_x * theta_range  # NO mirroring
    radius = path[:, 1]
    return np.column_stack((angles, radius))

# === Export C arrays ===
def export_to_c_array_cartesian(path, var_name="cartesian_path"):
    c_array = f"float {var_name}[][2] = {{\n"
    for x, y in path:
        c_array += f"    {{{x:.3f}f, {y:.3f}f}},\n"
    c_array += "};"
    return c_array

def export_to_c_array_polar(polar_path, var_name="polar_path"):
    c_array = f"float {var_name}[][2] = {{\n"
    for theta, r in polar_path:
        c_array += f"    {{{theta:.6f}f, {r:.3f}f}},\n"
    c_array += "};"
    return c_array

# === CONFIGURATION ===
word = "FIB0_G|0"
height_mm = 80
interp_points = 4

angle_range = (40, 95)  # Revolute joint angle range in degrees
offset_x = 0           # mm to the right
offset_y = 300           # mm upward

# === MAIN WORKFLOW ===
# 1. Generate word path in Cartesian
path = get_smooth_word_path(word, height_mm=height_mm, interp_points=interp_points)

# 2. Offset in X and Y
path[:, 0] += offset_x  # right
path[:, 1] += offset_y  # up

# 3. Mirror left-right (horizontal flip across Y-axis)
path[:, 0] = -path[:, 0] + (np.max(path[:, 0]) + np.min(path[:, 0]))

# 4. Convert to Polar (angle [10°, 130°], radius = Y)
polar_path = cartesian_to_polar(path, revolute_deg_range=angle_range)

# 5. Visualization
plt.figure(figsize=(12, 5))
plt.plot(path[:, 0], path[:, 1], 'b-', label="Cartesian Path")
plt.axis('equal')
plt.title(f"'{word}' Path (offset + horizontal mirror)")
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")
plt.grid(True)
plt.legend()
plt.show()

# 6. Export to C
cartesian_c = export_to_c_array_cartesian(path, "cartesian_path")
polar_c = export_to_c_array_polar(polar_path, "polar_path")

with open("cartesian_path.c", "w") as f:
    f.write(cartesian_c)

with open("polar_path.c", "w") as f:
    f.write(polar_c)

print("✅ Cartesian and Polar C arrays exported.")
