import json, os, numpy as np, open3d as o3d

def _extract_points_from_obj(obj):
    """Return list of (x,y,z) tuples found in obj or None."""
    if isinstance(obj, list):
        pts = []
        for it in obj:
            if isinstance(it, (list, tuple)) and len(it) >= 3 and all(isinstance(x, (int, float)) for x in it[:3]):
                pts.append(tuple(it[:3]))
            elif isinstance(it, dict):
                if all(k in it for k in ("x","y","z")):
                    pts.append((it["x"], it["y"], it["z"]))
                else:
                    for k in ("point","position","coords","coordinates","xyz","vertices","verts"):
                        if k in it:
                            v = it[k]
                            if isinstance(v, (list, tuple)) and len(v) >= 3:
                                pts.append(tuple(v[:3])); break
                            if isinstance(v, dict) and all(a in v for a in ("x","y","z")):
                                pts.append((v["x"], v["y"], v["z"])); break
        return pts if pts else None
    if isinstance(obj, dict):
        if all(k in obj for k in ("x","y","z")):
            return [(obj["x"], obj["y"], obj["z"])]
        for k in ("points","geometry","coordinates","positions","verts","vertices","data"):
            if k in obj:
                pts = _extract_points_from_obj(obj[k])
                if pts:
                    return pts
    return None

def _recursive_values(root):
    stack = [root]
    while stack:
        cur = stack.pop()
        if isinstance(cur, dict):
            for v in cur.values():
                stack.append(v)
        elif isinstance(cur, list):
            for v in cur:
                stack.append(v)
        else:
            yield cur

def load_point_cloud_with_annotations(json_path, meta_path):
    """Load points and class ids by joining objects <-> figures (common LabelStudio/Cloud format)."""
    with open(json_path, 'r', encoding='utf-8') as f:
        annotations = json.load(f)
    with open(meta_path, 'r', encoding='utf-8') as f:
        metadata = json.load(f)

    objs = annotations.get('objects', [])
    figs = annotations.get('figures', [])

    if not objs and not figs:
        raise ValueError("Annotations JSON has no 'objects' or 'figures' lists. Run inspector to see structure.")

    # map object key -> class title (or class id if present)
    obj_key_to_class_title = {}
    for o in objs:
        k = o.get('key') or o.get('id')
        if k:
            obj_key_to_class_title[k] = o.get('classTitle') or o.get('class') or o.get('classId') or o.get('label') or None

    # try to load key_id_map (helps matching numeric ids)
    kid_paths = [
        os.path.join(os.path.dirname(json_path), 'key_id_map.json'),
        os.path.join(os.path.dirname(os.path.dirname(json_path)), 'key_id_map.json'),
        os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(json_path))), 'key_id_map.json'),
    ]
    key_id_map = {}
    for p in kid_paths:
        if os.path.exists(p):
            try:
                key_id_map = json.load(open(p, 'r', encoding='utf-8'))
                break
            except Exception:
                key_id_map = {}
    # invert objects map for numeric id -> object key (if present)
    numid_to_objkey = {}
    for grp in ('objects','figures','tags','videos'):
        for sk, nv in key_id_map.get(grp, {}).items():
            try:
                numid_to_objkey[int(nv)] = sk
            except Exception:
                pass

    # metadata title -> class id map (for coloring)
    title_to_metaid = {}
    for cls in metadata.get('classes', []):
        title_to_metaid[cls.get('title')] = cls.get('id')

    points = []
    class_ids = []

    # for each figure, try to find which object it belongs to then extract its points
    object_keys = set(obj_key_to_class_title.keys())
    for fig in figs:
        # 1) try common linking fields
        owner_key = fig.get('objectKey') or fig.get('object_id') or fig.get('objectId') or fig.get('parent') or None

        # 2) fallback: search any primitive value in fig that equals an object key
        if owner_key is None:
            for val in _recursive_values(fig):
                if isinstance(val, str) and val in object_keys:
                    owner_key = val; break
                if isinstance(val, (int, float)):
                    # try numeric id -> obj key
                    if int(val) in numid_to_objkey:
                        owner_key = numid_to_objkey[int(val)]; break

        # 3) if still None, try to find figure.key matching an object key (rare)
        if owner_key is None and (fig.get('key') in object_keys):
            owner_key = fig.get('key')

        # extract points from figure
        pts = _extract_points_from_obj(fig)
        if not pts:
            # try nested geometry fields more aggressively
            for fld in ('geometry','data','points','coordinates','positions','vertices','verts'):
                if fld in fig:
                    pts = _extract_points_from_obj(fig[fld])
                    if pts:
                        break

        if not pts:
            # skip if no coords in this figure
            continue

        # determine class id using owner_key -> classTitle -> metadata id
        meta_id = -1
        if owner_key:
            title = obj_key_to_class_title.get(owner_key)
            if title:
                meta_id = title_to_metaid.get(title, -1)

        # append all pts
        for p in pts:
            points.append(p)
            class_ids.append(meta_id)

    if not points:
        raise ValueError("No points extracted from figures/objects. If annotations reference indices into the original .pcd, load the .pcd and use those indices.")

    points = np.asarray(points, dtype=float)
    class_ids = np.asarray(class_ids, dtype=int)
    return points, class_ids, metadata

def visualize_colored_cloud(points, class_ids, metadata):
    """Visualize point cloud with class colors (class_ids are meta ids)."""
    color_map = {}
    for cls in metadata.get('classes', []):
        try:
            color_map[cls.get('id')] = tuple(int(cls.get('color', '#808080').lstrip('#')[i:i+2], 16)/255.0 for i in (0,2,4))
        except Exception:
            color_map[cls.get('id')] = (0.5,0.5,0.5)

    colors = np.array([color_map.get(int(cid), (0.5,0.5,0.5)) for cid in class_ids], dtype=float)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd])