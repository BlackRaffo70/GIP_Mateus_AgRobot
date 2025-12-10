import json
import numpy as np
import open3d as o3d

# Load metadata and annotations
with open('Vineyard Pointcloud/meta.json', 'r') as f:
    metadata = json.load(f)

with open('Vineyard Pointcloud/dataset 2025-10-03 09-46-48/ann/pc_color_filtered.pcd.json', 'r') as f:
    annotations = json.load(f)

# Load original point cloud (.pcd format)
pcd = o3d.io.read_point_cloud('Vineyard Pointcloud/dataset 2025-10-03 09-46-48/pointcloud/pc_color_filtered.pcd')
points = np.asarray(pcd.points)
print(f"Loaded {len(points)} points from PCD")

# Create color map from metadata
color_map = {}
for cls in metadata.get('classes', []):
    hex_color = cls.get('color', '#808080')
    hex_color = hex_color.lstrip('#')
    rgb = tuple(int(hex_color[i:i+2], 16) / 255.0 for i in (0, 2, 4))
    color_map[cls.get('id')] = rgb
    print(f"Class {cls.get('title')}: {rgb}")

# Map object keys to class IDs
objs = annotations.get('objects', [])
obj_key_to_class_id = {}
for o in objs:
    k = o.get('key')
    if k:
        title = o.get('classTitle')
        # Find matching class ID from metadata by title
        for cls in metadata.get('classes', []):
            if cls.get('title') == title:
                obj_key_to_class_id[k] = cls.get('id')
                break

# Initialize colors array (default gray)
colors = np.full((len(points), 3), [0.5, 0.5, 0.5])

# Apply class colors based on figures/annotations
colored_count = 0
figs = annotations.get('figures', [])
for fig in figs:
    owner_key = fig.get('objectKey')
    if owner_key in obj_key_to_class_id:
        class_id = obj_key_to_class_id[owner_key]
        # Extract indices from nested geometry
        geometry = fig.get('geometry', {})
        indices = geometry.get('indices', [])
        
        if indices and isinstance(indices, list):
            for idx in indices:
                if 0 <= idx < len(colors):
                    colors[idx] = color_map.get(class_id, [0.5, 0.5, 0.5])
                    colored_count += 1

print(f"Colored {colored_count} points")

# Set colors on point cloud
pcd.colors = o3d.utility.Vector3dVector(colors)

# Visualize
o3d.visualization.draw_geometries([pcd])
