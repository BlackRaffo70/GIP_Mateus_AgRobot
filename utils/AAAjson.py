import json, itertools

p = r"Vineyard Pointcloud/dataset 2025-10-03 09-46-48/ann/pc_color_filtered.pcd.json"
j = json.load(open(p, 'r'))
print("ROOT TYPE:", type(j))
if isinstance(j, dict):
    print("TOP KEYS:", list(j.keys())[:50])
for k,v in itertools.islice(j.items(), 20):
    print("KEY:", k, "TYPE:", type(v), "LEN(if list):", len(v) if isinstance(v, list) else "-")
    if isinstance(v, list) and v:
        print(" first item (trunc):", str(v[0])[:400])
        break
# optional: search for a known annotation key sample
sample = "d4bcc4e7401e42da9d86134d317f79af"
def find(root):
    stack=[root]
    while stack:
        cur=stack.pop()
        if isinstance(cur, dict):
            if cur.get("key")==sample or sample in cur:
                print("FOUND candidate dict (trunc):", str(cur)[:600]); return
            for val in cur.values(): stack.append(val)
        elif isinstance(cur, list):
            for v in cur: stack.append(v)
    print("NOT FOUND")
find(j)
