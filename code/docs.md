# Documentazione tecnica — branches.py

Breve: script per analisi e simulazione di potatura su point cloud di vite. Individua segmenti (figure), decide se tagliare o potare e visualizza risultato con Open3D rispettando vincoli di connettività alla struttura principale (tronco).

## Requisiti
- Python 3.8+
- Pacchetti: numpy, open3d, scipy, scikit-learn
- Struttura dati attesa:
  - `Vineyard Pointcloud/meta.json` — definizione classi (id, title, color)
  - `Vineyard Pointcloud/.../ann/pc_color_filtered.pcd.json` — annotazioni con `objects` e `figures`; ogni figura con `geometry.indices`
  - file PCD originale: `.../pointcloud/pc_color_filtered.pcd`

## Flusso principale
1. Caricamento metadata, annotazioni e point cloud (.pcd).
2. Costruzione dizionario `segments` — per ogni figura valida salvo:
   - `class_id` (metadato)
   - `indices` (numpy array indici globali)
   - `points` (coordinare XYZ corrispondenti)
3. Per ogni segmento classificato come ramo (`Branch 1`):
   - calcolo direzione principale locale (PCA sul vicino alla base)
   - decisione operativa:
     - se direzione z ≤ 0 → CUT_ENTIRELY (rimuovere tutto)
     - altrimenti → PRUNE a Y con 3 "gems" (conservare punti clusterizzati in cima)
4. Applicazione regole di dipendenza e connettività:
   - se la base di un ramo pruned è molto vicina alla base di un ramo tagliato → quel ramo diventa tagliato (base condivisa)
   - costruzione grafo di adiacenza tra segmenti (distanza tra basi / centroidi)
   - qualsiasi segmento non raggiungibile dai tronchi (`Tree`) attraverso segmenti non-cut → viene markato cut (evita "pezzi sospesi")
5. Output: visualizzazione Open3D con legenda colori e esportazione `cutting_plan.json`.

## Funzioni chiave (firma e comportamento sintetico)

- get_branch_direction(segment, sample_distance=0.05) -> (direction, z_component, base_point)
  - PCA sui punti vicini alla base del segmento per stimare la direzione principale. Normalizza e orienta la direzione da base verso apice. Restituisce componente z per decisioni.

- should_cut_branch(z_component) -> bool
  - Regola: `z_component <= 0.0` → cut (configurabile).

- prune_upward_branch(segment, gems_to_keep=3) -> np.array (indici assoluti da mantenere)
  - Seleziona la porzione superiore del ramo (top fraction), applica KMeans per ottenere fino a `gems_to_keep` cluster; espande cluster per includere punti adiacenti; restituisce gli indici assoluti dei punti da mantenere (gli altri sono rimossi).

- apply_branch_dependency_rule(segments, cutting_decisions, prune_decisions, attach_tol=0.15)
  - Se la base di un ramo pruned è entro `attach_tol` da una base cut, promuove l'intero ramo a CUT_ENTIRELY.

- _segment_base_point(seg) -> point
  - Punto base definito come punto con minima Z all'interno del segmento.

- build_adjacency(segments, tol=0.12) -> dict
  - Costruisce grafo di adiacenza tra segmenti usando distanza base↔base e test su centroide; ritorna mapping seg -> set(segmenti adiacenti).

- enforce_connectivity_to_trunk(segments, cutting_decisions, prune_decisions, attach_tol=0.15)
  - Esegue BFS dal/nei segmenti `Tree`. I segmenti non raggiungibili (e pruned) vengono convertiti in CUT_ENTIRELY (evita parti fluttuanti).

- apply_rules_until_stable(segments, cutting_decisions, prune_decisions, attach_tol=0.15, max_iter=10)
  - Esegue iterativamente dependency + connectivity fino a convergenza o max iterazioni.

- visualize_with_cutting_plan(segments, points, cutting_decisions, prune_decisions)
  - Costruisce colori e apre viewer Open3D; posiziona frame di riferimento e imposta camera "standing" (Z up, front = -Y).  
  - Legend (colori nello script):
    - Tree trunk: cyan
    - Entirely cut branches: ORANGE (intero segmento rimosso)
    - Pruned branches: ORANGE = parti rimosse, GREEN = parti mantenute (gems)
    - Other branches: pink
  - Camera: front = [0,-1,0], up = [0,0,1], lookat = cloud center

- export_cutting_plan(cutting_decisions, prune_decisions, output_path='cutting_plan.json')
  - Salva piano dettagliato (indici, motivazioni, conteggi).

- diagnostics(segments, points, cutting_decisions, prune_decisions)
  - Stampa copertura annotazioni e conteggio segmenti per classe (utile per capire perché colori sono sparsi).

## Parametri di tuning e consigli
- sample_distance (get_branch_direction): dimensione locale per PCA — aumentare se annotazioni sono sparse.
- should_cut threshold: oggi `<=0.0`; per atteggiamenti meno stringenti usare `z < -0.05`.
- attach_tol / build_adjacency tol: distanza (m) usata per considerare due basi collegate; tipico range 0.10–0.20.
- prune_upward_branch:
  - fraction iniziale (top fraction) e dimensione caratteristica `char_size` definiscono estensione dei gem; testare su campioni per evitare "floating gems".
- KMeans n_init e random_state possono essere cambiati per stabilità.

## Limitazioni note / assunzioni
- Segmenti sono considerati unità atomiche: i punti tenuti per un ramo sono subset dello stesso segmento. Lo script non costruisce un grafo punto-a-punto (livello fine), quindi l'inferenza di collegamento è per-segmento (base-based).
- Se molte figure non contengono `geometry.indices`, quelle parti del cloud non saranno colorate né considerate segmenti.
- Pruning Y-shape è un'approssimazione geometrica: identifica cluster apicali e mantiene volumi locali; non garantisce sempre una forma a Y perfetta ma produce 3 "gems" spatialmente coerenti.
- Risultati demo: per presentazione UI, assicurarsi alta copertura annotativa e/o regolare `attach_tol` e `sample_distance`.

## Raccomandazioni pratiche per demo coerente con realtà
1. Aumentare copertura annotazioni nelle zone periferiche dei rami (figure con indices).
2. Usare attach_tol conservativo (es. 0.12–0.18 m) per evitare pezzi sospesi.
3. Eseguire `diagnostics()` prima della presentazione per dimostrare copertura e numero di segmenti.
4. Se necessario, estendere il modello con grafi punto‑a‑punto (KDTree) per garantire che ogni punto mantenuto abbia una catena di connessione fino al tronco.

---  
File: `branches.py` — entry point; leggere le stampe di debug/diagnostics per parametri e copertura.