<img width="1015" height="198" alt="image" src="https://github.com/user-attachments/assets/6a37401e-d0d2-4583-952a-ac8964c9bc45" />


<div align="center">

![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![Open3D](https://img.shields.io/badge/Open3D-0.18+-green.svg)
![Status](https://img.shields.io/badge/Status-Completed-success.svg)

**Università di Bologna** • *Gestione dell'Innovazione dei Progetti* • **Prof. Laura Toschi**

---

### 👥 Team di Sviluppo

**Raffaele Neri** • **Matteo Melotti** • **Sebastiano Giannitti** • **Enrico Borsetti** • **Marco Crisafulli** • **Edoardo Buttazzi**

---

</div>

---

## 🎯 Panoramica

Sistema di computer vision per l'analisi automatizzata di vigneti tramite nuvole di punti 3D. Il sistema analizza la struttura tridimensionale delle viti, classifica automaticamente rami e germogli, e genera piani di potatura ottimali basati su regole agronomiche.

### 🎓 Contesto Accademico

**Corso:** Gestione dell'Innovazione dei Progetti  
**Docente:** Prof. Laura Toschi  
**Università:** Alma Mater Studiorum - Università di Bologna  
**Anno Accademico:** 2024/2025

---

## ✨ Caratteristiche Principali

- 🤖 **Automazione completa** - Pipeline end-to-end dalla nuvola di punti al piano di potatura
- 🎯 **Analisi geometrica 3D** - PCA per determinazione direzione rami
- 🧠 **Regole agronomiche** - Preservazione germogli vitali, verifica connettività al tronco
- 📊 **Visualizzazione interattiva** - Rendering 3D con codifica colore (cyan=mantieni, arancione=rimuovi)
- 📄 **Export strutturato** - Piano di potatura in formato JSON con statistiche dettagliate

---

## 🛠️ Tecnologie

| Libreria | Utilizzo |
|----------|----------|
| **Open3D** | Elaborazione nuvole punti 3D, visualizzazione |
| **NumPy** | Calcolo matriciale, manipolazione array |
| **SciPy** | Strutture spaziali (KDTree), calcolo distanze |
| **scikit-learn** | PCA, K-Means clustering |

---

## 🧠 Algoritmi Implementati

### 1️⃣ Rimozione Punti Isolati
Utilizza KDTree per identificare ed eliminare punti non connessi (rumore).

### 2️⃣ Analisi Direzione (PCA)
Calcola la direzione principale di crescita per ogni ramo, determinando se è orientato verso l'alto o il basso.

### 3️⃣ Clustering Germogli (K-Means)
Identifica i 3 cluster superiori di germogli da preservare nella potatura (configurazione a "Y").

### 4️⃣ Verifica Connettività (BFS)
Assicura che ogni struttura mantenuta sia collegata al tronco principale tramite graph traversal.

### 5️⃣ Convergenza Iterativa
Applica ripetutamente le regole di dipendenza e connettività fino a raggiungere uno stato stabile.

---

## 📐 Logica di Potatura

### 🎯 Regole Decisionali

**🔴 Taglio Completo** quando:
- Rami con direzione verso il basso (componente Z ≤ 0)
- Rami collegati a una base già tagliata
- Rami non raggiungibili dal tronco principale

**🟡 Potatura Selettiva** quando:
- Rami con direzione verso l'alto (componente Z > 0)
- Preservazione dei 3 cluster superiori di germogli
- Mantenimento configurazione a "Y"

### ⚙️ Parametri

| Parametro | Valore | Descrizione |
|-----------|--------|-------------|
| `radius` | 0.03 m | Tolleranza vicinanza punti |
| `attach_tol` | 0.15 m | Tolleranza attacco base |
| `gems_to_keep` | 3 | Germogli da preservare |

---

## 🚀 Installazione e Utilizzo

### 💻 Installazione

```bash
# Installa dipendenze
pip install open3d numpy scipy scikit-learn

# Esegui il programma
python vineyard_pruning.py
```

### 📁 Struttura File Richiesti

```
Vineyard Pointcloud/
├── meta.json                          # Definizioni classi
└── dataset 2025-10-03 09-46-48/
    ├── ann/
    │   └── pc_color_filtered.pcd.json # Annotazioni
    └── pointcloud/
        └── pc_color_filtered.pcd      # Nuvola punti 3D
```

### 📤 Output

Il sistema genera:
- **cutting_plan.json** - Piano di potatura completo con indici punti e statistiche
- **Visualizzazione 3D** - Finestra interattiva Open3D (cyan=mantieni, arancione=rimuovi)

---

## 📊 Visualizzazione

Il sistema mostra i risultati in una finestra 3D interattiva:

- 🔵 **Cyan** → Punti da mantenere
- 🟠 **Arancione** → Punti da rimuovere

**Controlli:**
- Mouse: Ruota vista
- Scroll: Zoom
- Frecce: Sposta camera

---

## 🔮 Sviluppi Futuri

- 🤖 Segmentazione automatica con deep learning (PointNet++)
- ⚙️ Auto-tuning parametri con reinforcement learning
- 🔗 API REST per integrazione con sistemi esterni
- 🤖 Export traiettorie per bracci robotici (ROS)
- 📱 Applicazione mobile per agricoltori

---

## 💡 Conclusioni

Il progetto dimostra l'applicabilità di tecniche di computer vision e machine learning per automatizzare processi agricoli tradizionalmente manuali. L'approccio rule-based garantisce interpretabilità delle decisioni, fondamentale per l'adozione nel settore agricolo.

---

<div align="center">

*Realizzato per l'esame di Gestione dell'Innovazione dei Progetti*  
*Università di Bologna - A.A. 2024/2025*

</div>
