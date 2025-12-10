import json
import numpy as np
import open3d as o3d
from scipy.spatial.distance import cdist
from scipy.spatial import cKDTree
from sklearn.decomposition import PCA
from sklearn.cluster import KMeans

# Load data
with open('Vineyard Pointcloud/meta.json', 'r') as f:
    metadata = json.load(f)

with open('Vineyard Pointcloud/dataset 2025-10-03 09-46-48/ann/pc_color_filtered.pcd.json', 'r') as f:
    annotations = json.load(f)

pcd = o3d.io.read_point_cloud('Vineyard Pointcloud/dataset 2025-10-03 09-46-48/pointcloud/pc_color_filtered.pcd')
points = np.asarray(pcd.points)

# Create color map
color_map = {}
for cls in metadata.get('classes', []):
    hex_color = cls.get('color', '#808080')
    hex_color = hex_color.lstrip('#')
    rgb = tuple(int(hex_color[i:i+2], 16) / 255.0 for i in (0, 2, 4))
    color_map[cls.get('id')] = rgb

# Map objects to class IDs
objs = annotations.get('objects', [])
obj_key_to_class_id = {}
for o in objs:
    k = o.get('key')
    if k:
        title = o.get('classTitle')
        for cls in metadata.get('classes', []):
            if cls.get('title') == title:
                obj_key_to_class_id[k] = cls.get('id')
                break

# Extract segments
segments = {}
figs = annotations.get('figures', [])
for fig in figs:
    owner_key = fig.get('objectKey')
    if owner_key in obj_key_to_class_id:
        class_id = obj_key_to_class_id[owner_key]
        geometry = fig.get('geometry', {})
        indices = geometry.get('indices', [])
        
        if indices:
            fig_key = fig.get('key')
            segments[fig_key] = {
                'class_id': class_id,
                'indices': np.array(indices, dtype=int),
                'points': points[np.array(indices, dtype=int)],
                'object_key': owner_key
            }

print(f"Found {len(segments)} segments")

# --- NEW: remove isolated annotated points (not connected to anything)
def remove_isolated_points(segments, points, radius=0.03, min_neighbors=1):
    """
    Remove annotated points that have no neighbors within `radius`.
    - radius: meters; neighbor search radius
    - min_neighbors: minimum other points within radius required to consider point 'connected'
      (default 1 means at least one other nearby point; points with only themselves are removed)
    Updates segments in-place and returns number of removed global points and removed segments.
    """
    if not segments:
        return 0, 0
    # collect all annotated indices and local mapping
    annotated_indices = np.unique(np.concatenate([seg['indices'] for seg in segments.values()]))
    subset_points = points[annotated_indices]
    tree = cKDTree(subset_points)
    # neighbors list for each annotated point (includes self)
    neighbors = tree.query_ball_point(subset_points, r=radius)
    # isolated if neighbors count <= min_neighbors (include self in count)
    isolated_mask = np.array([len(nb) <= min_neighbors for nb in neighbors])
    removed_global = annotated_indices[isolated_mask]
    if removed_global.size == 0:
        return 0, 0
    # remove isolated global indices from each segment
    removed_segments = []
    removed_count = 0
    removed_set = set(removed_global.tolist())
    for seg_key in list(segments.keys()):
        seg = segments[seg_key]
        mask_keep = np.array([idx not in removed_set for idx in seg['indices']])
        if not np.any(mask_keep):
            # whole segment becomes empty -> remove segment
            removed_segments.append(seg_key)
            removed_count += seg['indices'].size
            del segments[seg_key]
            continue
        new_indices = seg['indices'][mask_keep]
        seg['indices'] = new_indices
        seg['points'] = points[new_indices]
        removed_count += np.sum(~mask_keep)
    return removed_count, len(removed_segments)

# call cleaner (tune radius/min_neighbors if needed)
removed_count, removed_segs = remove_isolated_points(segments, points, radius=0.03, min_neighbors=1)
if removed_count:
    print(f"Removed {removed_count} isolated annotated points; removed {removed_segs} empty segments.")
else:
    print("No isolated annotated points found.")

# === BRANCH DIRECTION ANALYSIS ===

def get_branch_direction(segment, sample_distance=0.05):
    pts = segment['points']
    base_idx = np.argmin(pts[:, 2])
    base_point = pts[base_idx]
    distances = np.linalg.norm(pts - base_point, axis=1)
    nearby_mask = distances < sample_distance
    if np.sum(nearby_mask) < 3:
        nearby_pts = pts
    else:
        nearby_pts = pts[nearby_mask]
    if len(nearby_pts) > 2:
        pca = PCA(n_components=1)
        pca.fit(nearby_pts)
        direction = pca.components_[0]
    else:
        direction = np.array([0.0, 0.0, 1.0])
    direction = direction / np.linalg.norm(direction)
    # Ensure direction points from base toward tip (positive Z when upward)
    if direction[2] < 0:
        direction = -direction
    z_component = direction[2]
    return direction, z_component, base_point

def should_cut_branch(z_component):
    """
    Cut branch completely when its principal direction has negative z (pointing down).
    Here we interpret 'negative z direction' strictly: z_component <= 0 -> cut.
    """
    return z_component <= 0.0

def prune_upward_branch(segment, gems_to_keep=3):
    """
    Produce a Y-like pruning: keep points around up to `gems_to_keep` top clusters (gems).
    Strategy:
     - select the top fraction of points by Z (top 30% of branch by height)
     - cluster them with KMeans into up to `gems_to_keep` clusters
     - keep all points that belong to those clusters (absolute indices returned)
    Returns absolute indices (subset of segment['indices']) to KEEP.
    """
    pts = segment['points']
    abs_idx = segment['indices']  # absolute indices into global point cloud

    z_min, z_max = pts[:, 2].min(), pts[:, 2].max()
    z_range = z_max - z_min
    if z_range <= 0:
        return abs_idx.copy()

    # Start with top fraction; increase fraction if not enough points
    fraction = 0.5
    selected_local_idx = np.where(pts[:, 2] >= z_min + z_range * (1 - fraction))[0]

    # Expand selection until at least gems_to_keep * 5 points or selection covers >= 50% branch
    while len(selected_local_idx) < (gems_to_keep * 5) and fraction < 0.9:
        fraction += 0.1
        selected_local_idx = np.where(pts[:, 2] >= z_min + z_range * (1 - fraction))[0]

    if len(selected_local_idx) < gems_to_keep:
        # fallback: pick top-gem single points (the highest points)
        top_order = np.argsort(pts[:, 2])[::-1]
        keep_local = top_order[:min(len(top_order), gems_to_keep)]
        return abs_idx[keep_local]

    selected_pts = pts[selected_local_idx]

    # cluster into up to gems_to_keep clusters
    n_clusters = min(gems_to_keep, len(selected_pts))
    try:
        kmeans = KMeans(n_clusters=n_clusters, random_state=0, n_init=10)
        labels = kmeans.fit_predict(selected_pts)
    except Exception:
        # fallback: single cluster
        labels = np.zeros(len(selected_pts), dtype=int)
        n_clusters = 1

    # for each cluster, take cluster members as gem region
    keep_local_idx = []
    for cl in range(n_clusters):
        members = selected_local_idx[labels == cl]
        # expand cluster by including nearby points within a small radius (based on branch size)
        if len(members) == 0:
            continue
        cluster_pts = pts[members]
        centroid = cluster_pts.mean(axis=0)
        # radius = max(0.02, 0.1 * characteristic size)
        char_size = max(0.02, np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)) * 0.05)
        dists = np.linalg.norm(pts - centroid, axis=1)
        expanded = np.where(dists <= max(char_size, 0.03))[0]
        keep_local_idx.append(expanded)

    if not keep_local_idx:
        # fallback to selected local idx
        keep_local = selected_local_idx
    else:
        keep_local = np.unique(np.concatenate(keep_local_idx))

    # Ensure we return absolute indices (subset of segment['indices'])
    return abs_idx[keep_local]

# === DECISION TREE ===

cutting_decisions = {}
prune_decisions = {}

print("\n=== BRANCH CUTTING ANALYSIS ===\n")

for seg_id, seg in segments.items():
    class_name = [c['title'] for c in metadata['classes'] if c['id'] == seg['class_id']][0]
    if class_name != 'Branch 1':
        continue
    direction, z_comp, base = get_branch_direction(seg, sample_distance=0.1)
    print(f"Branch {seg_id[:8]}... Z-component: {z_comp:.3f}")
    if should_cut_branch(z_comp):
        # cut entire branch (all points removed)
        cutting_decisions[seg_id] = {
            'action': 'CUT_ENTIRELY',
            'reason': f'Branch principal direction points downwards (Z={z_comp:.3f})',
            'base_point': base.tolist(),
            'direction': direction.tolist(),
            'indices_to_remove': seg['indices'].tolist()
        }
        print(f"  → ACTION: CUT ENTIRELY (all points removed)\n")
    else:
        # prune branch to keep 3 gems (Y-like)
        keep_abs = prune_upward_branch(seg, gems_to_keep=3)
        remove_abs = np.setdiff1d(seg['indices'], keep_abs, assume_unique=False)
        prune_decisions[seg_id] = {
            'action': 'PRUNE',
            'reason': f'Upward branch (Z={z_comp:.3f}), keep 3 gems',
            'base_point': base.tolist(),
            'direction': direction.tolist(),
            'indices_to_keep': keep_abs.tolist(),
            'indices_to_remove': remove_abs.tolist(),
            'keep_count': int(len(keep_abs)),
            'remove_count': int(len(remove_abs))
        }
        print(f"  → ACTION: PRUNE to 3 gems (keep {len(keep_abs)}/{len(seg['indices'])} points)\n")

# Apply branch dependency rule
print("\n=== APPLYING BRANCH DEPENDENCY RULE ===")
def apply_branch_dependency_rule(segments, cutting_decisions, prune_decisions, attach_tol=0.15):
    """
    If a branch base is cut, any other branch attached at that base is cut entirely as well.
    We look for pruned branches whose base is within attach_tol of any cut branch base;
    those pruned branches are moved to cutting_decisions (entire branch removed).
    """
    moved = []
    cut_bases = [np.array(info['base_point']) for info in cutting_decisions.values()]
    if not cut_bases:
        return
    for prune_seg_id, prune_data in list(prune_decisions.items()):
        prune_base = np.array(prune_data['base_point'])
        dists = np.linalg.norm(np.stack(cut_bases) - prune_base, axis=1)
        if np.any(dists < attach_tol):
            # move entire pruned branch to cut
            seg = segments[prune_seg_id]
            cutting_decisions[prune_seg_id] = {
                'action': 'CUT_ENTIRELY',
                'reason': f'Attached to cut base (dist {float(dists.min()):.3f} m)',
                'base_point': prune_data['base_point'],
                'direction': prune_data['direction'],
                'indices_to_remove': seg['indices'].tolist()
            }
            del prune_decisions[prune_seg_id]
            moved.append(prune_seg_id)
            print(f"Branch {prune_seg_id[:8]}... → MOVED TO CUT (attached base)")

apply_branch_dependency_rule(segments, cutting_decisions, prune_decisions, attach_tol=0.15)

# after apply_branch_dependency_rule(...) call add/enforce connectivity to trunk:
def _segment_base_point(seg):
    pts = seg['points']
    return pts[np.argmin(pts[:, 2])]

def build_adjacency(segments, tol=0.12):
    """Build adjacency dict: two segments adjacent if their base is within tol or one base close to other's points."""
    keys = list(segments.keys())
    bases = np.array([_segment_base_point(segments[k]) for k in keys])
    adj = {k: set() for k in keys}
    if len(keys) == 0:
        return adj
    # base-to-base distances
    D = cdist(bases, bases)
    for i, ki in enumerate(keys):
        for j, kj in enumerate(keys):
            if i == j:
                continue
            if D[i, j] < tol:
                adj[ki].add(kj)
                adj[kj].add(ki)
                continue
            # also test base i to any point of segment j (coarse test using segment bbox center)
            bbox_center_j = segments[kj]['points'].mean(axis=0)
            if np.linalg.norm(bases[i] - bbox_center_j) < tol:
                adj[ki].add(kj)
                adj[kj].add(ki)
    return adj

def enforce_connectivity_to_trunk(segments, cutting_decisions, prune_decisions, attach_tol=0.15):
    """
    Ensure kept parts are attached to the trunk via a chain of non-cut segments.
    Any pruned segment that is not reachable from a trunk segment becomes CUT entirely.
    """
    # Identify trunk segment keys (class title == 'Tree')
    trunk_keys = []
    id_to_title = {c['id']: c['title'] for c in metadata.get('classes', [])}
    for k, seg in segments.items():
        title = id_to_title.get(seg['class_id'], None)
        if title == 'Tree':
            trunk_keys.append(k)

    if not trunk_keys:
        # no trunk segments found; nothing to attach to
        print("WARNING: no 'Tree' segments found — connectivity enforcement skipped.")
        return

    adj = build_adjacency(segments, tol=attach_tol)

    # define which segments are considered "present" in the structure (not fully cut)
    def is_present(seg_key):
        # if in cutting_decisions -> removed
        if seg_key in cutting_decisions:
            return False
        # if pruned but no keep indices -> treat as removed
        if seg_key in prune_decisions:
            if len(prune_decisions[seg_key]['indices_to_keep']) == 0:
                return False
            return True
        # otherwise segment is present (not explicitly decided -> consider present)
        return True

    # BFS from each trunk key through present segments
    reachable = set()
    from collections import deque
    q = deque()
    for t in trunk_keys:
        if t in segments and is_present(t):
            q.append(t); reachable.add(t)
    while q:
        cur = q.popleft()
        for nb in adj.get(cur, ()):
            if nb not in reachable and is_present(nb):
                reachable.add(nb); q.append(nb)

    # any pruned segment not in reachable should be moved to cutting_decisions (entire removal)
    moved = []
    for prune_k in list(prune_decisions.keys()):
        if prune_k not in reachable:
            seg = segments[prune_k]
            cutting_decisions[prune_k] = {
                'action': 'CUT_ENTIRELY',
                'reason': 'Not connected to trunk after other base cuts',
                'base_point': prune_decisions[prune_k]['base_point'],
                'direction': prune_decisions[prune_k]['direction'],
                'indices_to_remove': seg['indices'].tolist()
            }
            del prune_decisions[prune_k]
            moved.append(prune_k)
            print(f"ENFORCE: pruned branch {prune_k[:8]} moved to CUT (not reachable from trunk)")

    print(f"Connectivity: {len(reachable)} segments reachable from trunk, {len(moved)} pruned->cut moved.")

# call enforcement after dependency rule
apply_branch_dependency_rule(segments, cutting_decisions, prune_decisions, attach_tol=0.15)
enforce_connectivity_to_trunk(segments, cutting_decisions, prune_decisions, attach_tol=0.15)

# DEBUG: coverage and per-class counts
def diagnostics(segments, points, cutting_decisions, prune_decisions):
    total_points = len(points)
    colored_mask = np.zeros(total_points, dtype=bool)
    for seg_id, seg in segments.items():
        colored_mask[seg['indices']] = True
    covered_pct = colored_mask.sum() / total_points * 100.0
    # count segments per class title
    counts = {}
    id_to_title = {c['id']: c['title'] for c in metadata.get('classes', [])}
    for seg_id, seg in segments.items():
        title = id_to_title.get(seg['class_id'], 'UNKNOWN')
        counts[title] = counts.get(title, 0) + 1
    print(f"ANNOTATION COVERAGE: {colored_mask.sum()} / {total_points} points ({covered_pct:.2f}%) are part of any segment.")
    print("SEGMENTS BY CLASS:", counts)
diagnostics(segments, points, cutting_decisions, prune_decisions)

# === VISUALIZATION WITH CUTTING RESULTS ===

def visualize_with_cutting_plan(segments, points, cutting_decisions, prune_decisions):
    """
    Visualizzazione: mostra SOLO i punti annotati (su cui è possibile operare).
    Colori:
     - azzurro (cyan) = punti che restano
     - arancione = punti rimossi (cut + prune removed)
    Camera allineata con Z up.
    """
    # raccolgo tutti gli indici annotati (tutti i segmenti)
    if len(segments) == 0:
        print("No segments to visualize.")
        return

    annotated_indices = np.unique(np.concatenate([seg['indices'] for seg in segments.values()])) if segments else np.array([], dtype=int)
    if annotated_indices.size == 0:
        print("Nessun punto annotato trovato — niente da visualizzare.")
        return

    # mappa indici globali -> indici locali nella nuvola ridotta
    index_map = np.full(len(points), -1, dtype=int)
    index_map[annotated_indices] = np.arange(annotated_indices.size)

    # punti ridotti (solo annotati)
    subset_points = points[annotated_indices]

    # inizializza colori: default kept (cyan)
    colors = np.tile(np.array([0.0, 0.76, 1.0], dtype=float), (len(subset_points), 1))

    # costruisci mask locale dei punti rimossi (cut + prune)
    removed_local_mask = np.zeros(len(subset_points), dtype=bool)
    # cut decisions
    for info in cutting_decisions.values():
        for gi in info.get('indices_to_remove', []):
            li = index_map[gi]
            if li >= 0:
                removed_local_mask[li] = True
    # prune decisions (removed parts)
    for info in prune_decisions.values():
        for gi in info.get('indices_to_remove', []):
            li = index_map[gi]
            if li >= 0:
                removed_local_mask[li] = True

    # colorazione: arancione per rimosso
    colors[removed_local_mask] = np.array([1.0, 0.5, 0.0], dtype=float)

    # costruisco pointcloud ridotta e visualizzo
    pcd_vis = o3d.geometry.PointCloud()
    pcd_vis.points = o3d.utility.Vector3dVector(subset_points)
    pcd_vis.colors = o3d.utility.Vector3dVector(colors)

    # frame di riferimento al centro della nuvola ridotta
    center = subset_points.mean(axis=0)
    axis_length = max(0.2, np.linalg.norm(subset_points.max(axis=0) - subset_points.min(axis=0)) * 0.2)
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=axis_length, origin=center)

    # visualizzatore con camera "standing" (Z up)
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Cutting Plan (cyan=kept, orange=removed)', width=1280, height=720)
    vis.add_geometry(pcd_vis)
    vis.add_geometry(axis)

    ctr = vis.get_view_control()
    front = np.array([0.0, -1.0, 0.0])
    up = np.array([0.0, 0.0, 1.0])
    ctr.set_front(front.tolist())
    ctr.set_up(up.tolist())
    ctr.set_lookat(center.tolist())
    ctr.set_zoom(0.7)

    vis.run()
    vis.destroy_window()

# === EXPORT CUTTING PLAN ===

def export_cutting_plan(cutting_decisions, prune_decisions, output_path='cutting_plan.json'):
    plan = {
        'cut_entirely': cutting_decisions,
        'prune': prune_decisions,
        'summary': {
            'total_branches_to_cut': len(cutting_decisions),
            'total_branches_to_prune': len(prune_decisions),
            'total_points_to_remove': sum(len(v['indices_to_remove']) for v in cutting_decisions.values()) +
                                      sum(len(v['indices_to_remove']) for v in prune_decisions.values())
        }
    }
    with open(output_path, 'w') as f:
        json.dump(plan, f, indent=2)
    print(f"Cutting plan saved to {output_path}")

# Run visualization and export
print("\nLaunching Open3D viewer (orange=removed, green=kept gems)...\n")
visualize_with_cutting_plan(segments, points, cutting_decisions, prune_decisions)
export_cutting_plan(cutting_decisions, prune_decisions)

def apply_rules_until_stable(segments, cutting_decisions, prune_decisions, attach_tol=0.15, max_iter=10):
    """
    Repeatedly apply dependency and connectivity rules until no further segments change.
    Ensures: if a base is cut, all attached parts (and any segments disconnected from trunk) are moved to CUT.
    """
    for it in range(max_iter):
        before_cut = set(cutting_decisions.keys())
        before_prune = set(prune_decisions.keys())

        # dependency: move pruned branches attached to a cut base -> cut
        apply_branch_dependency_rule(segments, cutting_decisions, prune_decisions, attach_tol=attach_tol)
        # connectivity: any pruned segment not reachable from trunk -> cut
        enforce_connectivity_to_trunk(segments, cutting_decisions, prune_decisions, attach_tol=attach_tol)

        after_cut = set(cutting_decisions.keys())
        after_prune = set(prune_decisions.keys())

        if before_cut == after_cut and before_prune == after_prune:
            print(f"Rules converged after {it+1} iterations.")
            return
    print(f"Reached max iterations ({max_iter}) - rules may not be fully converged.")

# === APPLYING BRANCH DEPENDENCY & CONNECTIVITY RULES ===
apply_rules_until_stable(segments, cutting_decisions, prune_decisions, attach_tol=0.15, max_iter=10)