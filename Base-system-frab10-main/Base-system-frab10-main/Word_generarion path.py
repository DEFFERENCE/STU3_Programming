import numpy as np
import matplotlib.pyplot as plt
from matplotlib.textpath import TextPath
from matplotlib.font_manager import FontProperties
from matplotlib.path import Path


# STEP 1: Interpolate smooth path
def interpolate_path(path, num_points=10):
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


# STEP 2: Generate Cartesian path from text
def get_smooth_word_path(word, font='Press Start 2P', size=1.0, spacing=1.0, interp_points=10):
    font_prop = FontProperties(family=font, size=size)
    text_path = TextPath((0, 0), word, prop=font_prop)
    vertices = text_path.vertices.copy()

    # Normalize and scale
    min_x, min_y = np.min(vertices, axis=0)
    vertices -= [min_x, min_y]
    scale = 1.0 / np.max(vertices)
    vertices *= scale * spacing

    path = Path(vertices, text_path.codes)
    return interpolate_path(path, num_points=interp_points)


# STEP 3: Convert Cartesian to Polar coordinates
def cartesian_to_polar(coords, degrees=False):
    x = coords[:, 0]
    y = coords[:, 1]
    theta = np.arctan2(y, x)
    r = np.hypot(x, y)

    if degrees:
        theta = np.degrees(theta)

    return np.stack((theta, r), axis=-1)


# STEP 4: Export to C array format
def export_to_c_array(data, var_name="path"):
    c_array = f"float {var_name}[][2] = {{\n"
    for a, b in data:
        c_array += f"    {{{a:.3f}f, {b:.3f}f}},\n"
    c_array += "};\n"
    return c_array


# STEP 5: MAIN SCRIPT
if __name__ == "__main__":
    word = "FIBO_G10"
    spacing = 100
    interp_points = 20

    # Generate Cartesian path
    cartesian_path = get_smooth_word_path(word, spacing=spacing, interp_points=interp_points)

    # Convert to Polar coordinates (in radians)
    polar_path = cartesian_to_polar(cartesian_path, degrees=False)

    # Export to C arrays
    cartesian_c_code = export_to_c_array(cartesian_path, var_name="smooth_path_cartesian")
    polar_c_code = export_to_c_array(polar_path, var_name="smooth_path_polar")

    # Save to C source files
    with open("smooth_path_cartesian.c", "w") as f:
        f.write(cartesian_c_code)

    with open("smooth_path_polar.c", "w") as f:
        f.write(polar_c_code)

    print("[OK] Cartesian and Polar C arrays exported!")

    # Optional plot
    plt.plot(cartesian_path[:, 0], cartesian_path[:, 1], 'b-')
    plt.axis('equal')
    plt.title(f"Smooth path for '{word}'")
    plt.show()
