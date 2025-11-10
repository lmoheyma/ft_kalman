# 🚀 ft_kalman - Filtre de Kalman pour Navigation Inertielle

Implémentation d'un **filtre de Kalman** en C++ pour estimer la position d'un véhicule en 3D à partir de capteurs inertiels (IMU) bruités.

---

## 📖 Table des Matières

1. [Introduction au Projet](#-introduction-au-projet)
2. [Contexte et Objectifs](#-contexte-et-objectifs)
3. [Théorie du Filtre de Kalman](#-théorie-du-filtre-de-kalman)
4. [Explications Détaillées des Matrices](#-explications-détaillées-des-matrices)
5. [Algorithme Complet](#-algorithme-complet)
6. [Architecture du Code](#-architecture-du-code)
7. [Compilation et Utilisation](#-compilation-et-utilisation)
8. [Ressources et Références](#-ressources-et-références)

---

## 🎯 Introduction au Projet

### Qu'est-ce qu'un Filtre de Kalman ?

Le **filtre de Kalman** est un algorithme mathématique qui permet d'estimer l'état d'un système dynamique (comme la position et la vitesse d'un véhicule) à partir de **mesures bruitées et incomplètes**.

#### Utilisations dans le Monde Réel :
- 🛩️ **Navigation aérienne et spatiale** (GPS + accéléromètres)
- 🚗 **Voitures autonomes** (fusion de capteurs)
- 📱 **Smartphones** (localisation indoor)
- 💹 **Finance** (prédiction de tendances)
- 🎥 **Vision par ordinateur** (tracking d'objets)

### Le Problème à Résoudre

Imaginez un véhicule dans l'espace (sans gravité ni air) qui possède :
- Un **accéléromètre** : mesure l'accélération très souvent (100 Hz) mais avec un petit bruit
- Un **GPS** : mesure la position rarement (0.33 Hz) mais avec un gros bruit
- Un **gyroscope** : mesure l'orientation avec un bruit moyen

**Question** : Comment combiner ces mesures imparfaites pour obtenir la meilleure estimation possible de la position ?

**Réponse** : Le filtre de Kalman !

---

## 🎮 Contexte et Objectifs

### Scénario du Projet

Vous êtes la **centrale inertielle (IMU)** d'un véhicule générique se déplaçant dans un environnement sans air ni gravité. Le véhicule se déplace **uniquement selon son axe longitudinal** (vers l'avant).

### Capteurs Disponibles

| Capteur | Fréquence | Bruit (σ) | Mesure |
|---------|-----------|-----------|--------|
| **Accéléromètre** | 100 Hz (0.01s) | 10⁻³ | Accélération (ax, ay, az) en m/s² |
| **Gyroscope** | 100 Hz (0.01s) | 10⁻² | Direction (angles d'Euler) |
| **GPS** | 0.33 Hz (3s) | 10⁻¹ | Position (x, y, z) en mètres |

> **Note** : Le bruit est un **bruit blanc gaussien** avec moyenne υ = 0 et écart-type σ

### Contraintes

- ❌ **Erreur maximale** : Votre estimation ne doit **JAMAIS** dépasser 5 mètres de la position réelle
- ⏱️ **Timeout** : Votre filtre doit répondre en moins d'1 seconde
- 🎯 **Précision** : Plus votre estimation est proche de la réalité, mieux c'est
- ⏳ **Durée** : Le système doit fonctionner jusqu'à 90 minutes de trajectoire

### Objectif

Créer un filtre de Kalman qui **fusionne intelligemment** les mesures de l'accéléromètre (fréquent mais bruité) et du GPS (rare mais moins bruité) pour obtenir une estimation optimale de la position.

---

## 🧮 Théorie du Filtre de Kalman

### Le Principe Fondamental

Le filtre de Kalman résout le problème suivant :

> **"Comment estimer optimalement l'état d'un système dynamique à partir de mesures bruitées ?"**

Il trouve un **compromis optimal** (au sens statistique des moindres carrés) entre :
1. **Le modèle physique** : "Je connais les lois de la physique (cinématique)"
2. **Les mesures bruitées** : "Mes capteurs me disent quelque chose, mais avec du bruit"

### Les Deux Phases du Filtre

Le filtre de Kalman fonctionne en **deux étapes** répétées en boucle :

#### 1️⃣ **PRÉDICTION** (toutes les 0.01s avec l'accéléromètre)

On utilise le **modèle physique** pour prédire où devrait être le véhicule :

```
"Si j'étais à la position X avec une vitesse V,
et que j'accélère de A pendant Δt secondes,
où devrais-je être maintenant ?"
```

**Équations** :
```
x̂(k|k-1) = F × x̂(k-1) + B × u(k)  ← Prédire l'état
P(k|k-1) = F × P(k-1) × Fᵀ + Q      ← Incertitude augmente
```

#### 2️⃣ **MISE À JOUR** (toutes les 3s quand le GPS arrive)

On utilise la **mesure GPS** pour corriger la prédiction :

```
"Le GPS me dit que je suis à la position Z,
mais ma prédiction disait position X,
combien dois-je corriger ?"
```

**Équations** :
```
y = z_GPS - H × x̂(k|k-1)           ← Innovation (erreur)
K = P × Hᵀ × (H×P×Hᵀ + R)⁻¹         ← Gain optimal
x̂(k|k) = x̂(k|k-1) + K × y          ← Correction
P(k|k) = (I - K×H) × P(k|k-1)       ← Incertitude diminue
```

### Illustration du Processus

```
Temps 0.00s: 
  Initialisation avec position et vitesse vraies
  
Temps 0.01s:
  PRÉDICTION avec accéléromètre
  → Position estimée (incertitude +)
  
Temps 0.02s:
  PRÉDICTION avec accéléromètre
  → Position estimée (incertitude ++)
  
...

Temps 3.00s:
  MISE À JOUR avec GPS !
  → Correction de la position (incertitude -)
  
Temps 3.01s:
  PRÉDICTION avec accéléromètre
  → Position estimée (incertitude +)
  
... et ainsi de suite
```

### Le Gain de Kalman : Le Cœur du Filtre

Le **gain de Kalman K** détermine **à quel point on fait confiance** au GPS vs. à notre prédiction.

**Cas 1** : Prédiction incertaine (P grand) + GPS précis (R petit)
```
K → grand → on corrige beaucoup
x_new ≈ z_GPS (on fait confiance au GPS)
```

**Cas 2** : Prédiction précise (P petit) + GPS bruité (R grand)
```
K → petit → on corrige peu
x_new ≈ x_pred (on ignore le GPS)
```

**Le filtre calcule automatiquement le meilleur compromis !**

---

## 📊 Explications Détaillées des Matrices

### Vue d'Ensemble

| Symbole | Nom | Dimension | Type | Rôle |
|---------|-----|-----------|------|------|
| **x** | Vecteur d'état | 6×1 | Vecteur | Ce qu'on estime |
| **F** | Transition d'état | 6×6 | Matrice | Lois de la physique |
| **B** | Contrôle | 6×3 | Matrice | Effet de l'accélération |
| **H** | Observation | 3×6 | Matrice | Lien état ↔ mesure |
| **P** | Covariance état | 6×6 | Matrice | Incertitude estimation |
| **Q** | Bruit processus | 6×6 | Matrice | Bruit accéléromètre |
| **R** | Bruit mesure | 3×3 | Matrice | Bruit GPS |
| **K** | Gain de Kalman | 6×3 | Matrice | Pondération optimale |

---

### 1️⃣ Vecteur d'État (x) - 6×1

**Définition** : Tout ce qu'on veut estimer sur le système.

```
x = [x,  y,  z,  vx, vy, vz]ᵀ
     ↑   ↑   ↑   ↑   ↑   ↑
     Position 3D   Vitesse 3D
     (mètres)      (m/s)
```

**Exemple** :
```
x = [100.5    ← Le véhicule est à X = 100.5m
     -50.2    ← Y = -50.2m
      30.0    ← Z = 30.0m
      15.3    ← Se déplace à vx = 15.3 m/s en X
       0.0    ← vy = 0 m/s (pas de mouvement latéral)
       0.0]   ← vz = 0 m/s (pas de mouvement vertical)
```

**Pourquoi 6 dimensions ?**
- Position seule ne suffit pas → on a besoin de la vitesse pour prédire le futur
- Vitesse seule ne suffit pas → on veut connaître la position

---

### 2️⃣ Matrice de Transition d'État (F) - 6×6

**Rôle** : Modélise comment l'état évolue naturellement avec le temps (sans accélération externe).

**Équations physiques** :
```
Position nouvelle = Position ancienne + Vitesse × Temps
Vitesse nouvelle  = Vitesse ancienne (si pas d'accélération)
```

**Matrice** :
```
F = [1  0  0  Δt  0   0 ]
    [0  1  0  0   Δt  0 ]    Avec Δt = 0.01s
    [0  0  1  0   0   Δt]
    [0  0  0  1   0   0 ]
    [0  0  0  0   1   0 ]
    [0  0  0  0   0   1 ]
```

**Explication ligne par ligne** :

- **Ligne 1** : `x_new = 1×x_old + Δt×vx`
  - Position X évolue selon sa vitesse
  
- **Ligne 4** : `vx_new = 1×vx_old`
  - Vitesse reste constante (principe d'inertie)

**Code** :
```cpp
void KalmanFilter::initStateTransitionMatrix(void) {
    // Matrice identité 6×6
    for (size_t i = 0; i < 6; i++)
        F[i][i] = 1.0;
    
    // Ajout des termes de couplage position-vitesse
    for (size_t i = 0; i < 3; i++)
        F[i][i+3] = DELTA_T;  // F[0][3] = F[1][4] = F[2][5] = 0.01
}
```

**Exemple numérique** :

Si `x = [100, 50, 30, 10, 5, 2]` (position + vitesse)

Alors `F×x` après 0.01s :
```
[1  0  0  0.01  0     0   ] [100]   [100.1]  ← x += vx×0.01 = 100 + 10×0.01
[0  1  0  0     0.01  0   ] [50 ] = [50.05]  ← y += vy×0.01 = 50 + 5×0.01
[0  0  1  0     0     0.01] [30 ]   [30.02]  ← z += vz×0.01
[0  0  0  1     0     0   ] [10 ]   [10   ]  ← vx inchangé
[0  0  0  0     1     0   ] [5  ]   [5    ]  ← vy inchangé
[0  0  0  0     0     1   ] [2  ]   [2    ]  ← vz inchangé
```

---

### 3️⃣ Matrice de Contrôle (B) - 6×3

**Rôle** : Traduit l'accélération mesurée en changement d'état.

**Équations physiques** :
```
Δposition = ½ × accélération × temps²
Δvitesse  = accélération × temps
```

**Matrice** :
```
B = [Δt²/2   0      0    ]
    [0       Δt²/2  0    ]
    [0       0      Δt²/2]
    [Δt      0      0    ]
    [0       Δt     0    ]
    [0       0      Δt   ]

Avec Δt = 0.01s :

B = [0.00005   0        0      ]
    [0         0.00005  0      ]
    [0         0        0.00005]
    [0.01      0        0      ]
    [0         0.01     0      ]
    [0         0        0.01   ]
```

**Utilisation** :
```cpp
// u = [ax, ay, az] = accélération mesurée
Vector Bu = B × u;  // Contribution de l'accélération
```

**Exemple numérique** :

Si accélération `u = [2.0, 0.0, 0.0]` m/s² :
```
B × u = [0.00005×2.0]   [0.0001]  ← Position X augmente de 0.1mm
        [0.00005×0.0] = [0.0   ]  ← Position Y inchangée
        [0.00005×0.0]   [0.0   ]  ← Position Z inchangée
        [0.01   ×2.0]   [0.02  ]  ← Vitesse X augmente de 2 cm/s
        [0.01   ×0.0]   [0.0   ]
        [0.01   ×0.0]   [0.0   ]
```

**Code** :
```cpp
void KalmanFilter::initControlMatrix(void) {
    Matrix upB = matrixScalar(identityMatrix(3), (DELTA_T * DELTA_T) * 0.5);
    Matrix lowB = matrixScalar(identityMatrix(3), DELTA_T);
    this->B = mergeMatrixVertical(upB, lowB);
}
```

---

### 4️⃣ Matrice d'Observation (H) - 3×6

**Rôle** : Extrait ce que le GPS peut mesurer depuis l'état complet.

**Signification** : "Le GPS ne mesure que la **position**, pas la vitesse"

**Matrice** :
```
H = [1  0  0  0  0  0]
    [0  1  0  0  0  0]
    [0  0  1  0  0  0]
     ↑           ↑
     Position    Vitesse (ignorée)
```

**Utilisation** :
```cpp
Vector position_estimée = H × x;  // Extrait [x, y, z] de [x,y,z,vx,vy,vz]
```

**Exemple** :
```
Si x = [100, -50, 30, 15, 0, 0]ᵀ

H×x = [1  0  0  0  0  0] [100]   [100]
      [0  1  0  0  0  0] [-50] = [-50]
      [0  0  1  0  0  0] [30 ]   [30 ]
                         [15 ]
                         [0  ]
                         [0  ]
```

Le GPS devrait mesurer environ `[100, -50, 30]` (+ bruit)

**Code** :
```cpp
void KalmanFilter::initMeasurementMatrix(void) {
    // [I₃×₃ | 0₃×₃]ᵀ
    this->H = mergeMatrixVertical(identityMatrix(3), Matrix(3, std::vector<double>(3, 0.0)));
    this->H = transpose(this->H);
}
```

---

### 5️⃣ Matrice de Covariance de l'État (P) - 6×6

**Rôle** : Quantifie **l'incertitude** sur chaque composante de l'état.

**Matrice** (simplifiée, diagonale) :
```
P = [σx²    0      0      0      0      0    ]
    [0      σy²    0      0      0      0    ]
    [0      0      σz²    0      0      0    ]
    [0      0      0      σvx²   0      0    ]
    [0      0      0      0      σvy²   0    ]
    [0      0      0      0      0      σvz² ]
```

**Interprétation** :

- `P[0][0] = σx²` = variance de l'erreur sur X
  - Si P[0][0] = 0.01 → écart-type σx = √0.01 = 0.1m
  - L'erreur sur X est de ±10cm avec 68% de probabilité

- `P[3][3] = σvx²` = variance de l'erreur sur vx
  - Si P[3][3] = 0.1 → écart-type σvx = 0.316 m/s

**Initialisation** :
```cpp
Vector diagonal = {0.01, 0.01, 0.01, 0.1, 0.1, 0.1};
P = diag(diagonal);
```

```
P(0) = [0.01  0     0     0    0    0  ]  ← Incertitude position ±10cm
       [0     0.01  0     0    0    0  ]
       [0     0     0.01  0    0    0  ]
       [0     0     0     0.1  0    0  ]  ← Incertitude vitesse ±0.3 m/s
       [0     0     0     0    0.1  0  ]
       [0     0     0     0    0    0.1]
```

**Évolution** :

- **Prédiction** : P **augmente** (l'incertitude croît avec le temps)
  ```
  P(k|k-1) = F × P × Fᵀ + Q
  ```

- **Mise à jour GPS** : P **diminue** (la mesure réduit l'incertitude)
  ```
  P(k|k) = (I - K×H) × P
  ```

**Analogie** :

Imaginez marcher les yeux fermés :
- Plus vous marchez, plus vous êtes incertain de votre position → **P augmente**
- Vous ouvrez les yeux (GPS) → vous savez où vous êtes → **P diminue**

---

### 6️⃣ Matrice de Bruit du Processus (Q) - 6×6

**Rôle** : Modélise l'incertitude introduite par le **bruit de l'accéléromètre**.

**Calcul** :
```
Q = G × Gᵀ × σ_acc²
```

Où G est la matrice de propagation du bruit :
```
G = [Δt²/2 × I₃]
    [Δt    × I₃]
```

**Code** :
```cpp
void KalmanFilter::initProcessNoiseMatrix(void) {
    Matrix G = propagationMatrix();
    this->Q = multiply(G, transpose(G));
    this->Q = matrixScalar(this->Q, ACCELEROMETER_NOISE * ACCELEROMETER_NOISE);
    // σ² = (10⁻³)² = 10⁻⁶
}
```

**Signification physique** :

Plus l'accéléromètre est bruité (grand σ_acc) :
- Plus Q est grande
- Plus l'incertitude augmente lors de la prédiction
- Plus le filtre fera confiance au GPS lors de la mise à jour

**Ordre de grandeur** :

Avec σ_acc = 10⁻³ et Δt = 0.01s :
```
Q ≈ 10⁻⁶ × [...] → très petit
```

L'accéléromètre introduit très peu d'incertitude (c'est le capteur le plus précis).

---

### 7️⃣ Matrice de Bruit de Mesure (R) - 3×3

**Rôle** : Quantifie le **bruit du GPS**.

**Matrice** :
```
R = [σ_GPS²   0        0      ]
    [0        σ_GPS²   0      ]
    [0        0        σ_GPS² ]

  = [0.01  0     0   ]
    [0     0.01  0   ]
    [0     0     0.01]
```

**Avec** : σ_GPS = 10⁻¹ = 0.1m

**Code** :
```cpp
void KalmanFilter::initUncertaintyMatrix(void) {
    this->R = matrixScalar(identityMatrix(3), GPS_NOISE * GPS_NOISE);
}
```

**Interprétation** :

- R[0][0] = 0.01 = variance du bruit GPS en X
- √0.01 = 0.1m → le GPS a une précision de ±10cm

**Impact sur le filtre** :

- R **grand** → GPS peu fiable → le filtre lui fait **moins confiance**
- R **petit** → GPS précis → le filtre lui fait **plus confiance**

---

### 8️⃣ Gain de Kalman (K) - 6×3

**Rôle** : Détermine le **poids optimal** entre prédiction et mesure GPS.

**Formule** :
```
K = P(k|k-1) × Hᵀ × [H × P(k|k-1) × Hᵀ + R]⁻¹
```

**Interprétation** :

**Si K ≈ 0 (petit)** :
```
x = x_pred + 0 × innovation
  ≈ x_pred
```
→ On **ignore le GPS**, on garde la prédiction

**Si K ≈ I (grand)** :
```
x = x_pred + I × [z_GPS - x_pred]
  ≈ z_GPS
```
→ On **fait totalement confiance au GPS**

**Calcul dans le code** :
```cpp
void KalmanFilter::update(const Vector& gps_measurement) {
    // Innovation
    Vector y = z_GPS - H×x_pred;
    
    // Covariance de l'innovation
    Matrix S = H×P×Hᵀ + R;
    
    // Gain de Kalman
    Matrix K = P×Hᵀ×S⁻¹;
    
    // Mise à jour
    x = x_pred + K×y;
    P = (I - K×H)×P;
}
```

**Exemple numérique** :

**Situation 1** : Prédiction incertaine
```
P = grande matrice (grosse incertitude)
R = petite (GPS précis)
→ K ≈ 1 → on corrige beaucoup
```

**Situation 2** : Prédiction certaine
```
P = petite matrice (bonne estimation)
R = grande (GPS bruité)
→ K ≈ 0 → on ignore le GPS
```

---

### 9️⃣ Vecteur d'Innovation (y) - 3×1

**Rôle** : Mesure l'**écart** entre ce que le GPS observe et ce qu'on prédisait.

**Formule** :
```
y = z_GPS - H × x_pred
```

**Exemple** :
```
GPS mesure     : z = [100.2, -50.1, 30.0]
On prédisait   : H×x = [100.0, -50.0, 30.0]
Innovation     : y = [0.2, -0.1, 0.0]
```

**Interprétation** :
- Le GPS dit qu'on est **20cm trop à droite** en X
- **10cm trop en avant** en Y
- Parfait en Z

Le filtre va corriger proportionnellement à cette innovation, pondérée par K.

---

### 🔟 Matrice de Covariance de l'Innovation (S) - 3×3

**Rôle** : Variance de l'innovation = **incertitude totale**.

**Formule** :
```
S = H × P × Hᵀ + R
  = [incertitude prédiction] + [incertitude GPS]
```

**Utilisation** :

Sert à calculer le gain de Kalman :
```
K = P×Hᵀ × S⁻¹
```

S normalise l'innovation pour tenir compte de toutes les sources d'incertitude.

---

## 🔄 Algorithme Complet

### Pseudo-Code Simplifié

```
1. INITIALISATION
   ├─ Recevoir position initiale (x₀, y₀, z₀)
   ├─ Recevoir vitesse initiale (vx₀, vy₀, vz₀)
   ├─ Créer vecteur d'état : x = [x₀, y₀, z₀, vx₀, vy₀, vz₀]
   ├─ Initialiser P (incertitude initiale)
   ├─ Initialiser Q (bruit accéléromètre)
   ├─ Initialiser R (bruit GPS)
   ├─ Initialiser F, B, H (matrices du système)
   └─ Envoyer première estimation

2. BOUCLE PRINCIPALE (toutes les 0.01s)
   
   ┌──────────────────────────────────────┐
   │ PHASE 1 : PRÉDICTION (toujours)     │
   ├──────────────────────────────────────┤
   │ a) Recevoir accélération u          │
   │ b) x_pred = F×x + B×u               │
   │ c) P_pred = F×P×Fᵀ + Q              │
   │ d) Envoyer estimation [x,y,z]       │
   └──────────────────────────────────────┘
   
   ┌──────────────────────────────────────┐
   │ PHASE 2 : MISE À JOUR (si GPS)      │
   ├──────────────────────────────────────┤
   │ a) Recevoir position GPS z          │
   │ b) y = z - H×x_pred                 │
   │ c) S = H×P×Hᵀ + R                   │
   │ d) K = P×Hᵀ×S⁻¹                     │
   │ e) x = x_pred + K×y                 │
   │ f) P = (I - K×H)×P                  │
   └──────────────────────────────────────┘
   
   ┌──────────────────────────────────────┐
   │ VÉRIFICATION                         │
   ├──────────────────────────────────────┤
   │ Erreur = distance(estimation, vrai) │
   │ Si erreur > 5m → ARRÊT              │
   └──────────────────────────────────────┘

3. FIN (quand MSG_END reçu)
```

### Flux de Données Détaillé

```
ENTRÉES :
  Accéléromètre : u (3×1) toutes les 0.01s
  GPS           : z (3×1) toutes les 3s
  
PRÉDICTION (100 Hz) :
  u (3×1) ──→ B (6×3) ──→ B×u (6×1) ─┐
                                      ├─→ x_pred = F×x + B×u
  x (6×1) ──→ F (6×6) ──→ F×x (6×1) ─┘
  
  P (6×6) ──→ F×P×Fᵀ + Q ──→ P_pred (6×6)
  
MISE À JOUR (0.33 Hz) :
  z_GPS (3×1), x_pred (6×1) ──→ y = z - H×x_pred (3×1)
  P, H, R ──→ S = H×P×Hᵀ + R (3×3)
  P, H, S⁻¹ ──→ K = P×Hᵀ×S⁻¹ (6×3)
  K×y + x_pred ──→ x_new (6×1)
  (I - K×H)×P ──→ P_new (6×6)
  
SORTIE :
  Estimation position : [x, y, z] toutes les 0.01s
```

---

## 🏗️ Architecture du Code

### Structure des Fichiers

```
ft_kalman/
├── inc/
│   ├── Client.hpp          → Communication UDP avec le serveur
│   ├── Parser.hpp          → Parsing des messages reçus
│   ├── KalmanFilter.hpp    → Implémentation du filtre
│   └── maths.hpp           → Opérations matricielles
├── src/
│   ├── Client.cpp
│   ├── Parser.cpp
│   ├── KalmanFilter.cpp
│   ├── maths.cpp
│   └── main.cpp            → Boucle principale
├── tests/
│   └── plot_logs.py        → Visualisation des résultats
├── Makefile
└── README.md
```

### Classes Principales

#### 1. **KalmanFilter**

```cpp
class KalmanFilter {
private:
    Vector _stateVector;     // x (6×1)
    Vector _acceleration;    // u (3×1)
    Matrix P;                // Covariance (6×6)
    Matrix Q;                // Bruit processus (6×6)
    Matrix R;                // Bruit mesure (3×3)
    Matrix F;                // Transition (6×6)
    Matrix B;                // Contrôle (6×3)
    Matrix H;                // Observation (3×6)
    
public:
    void predictStateVector();           // Phase 1
    void update(const Vector& gps);      // Phase 2
    void initCovarianceMatrix();         // Initialisation P
    void initProcessNoiseMatrix();       // Initialisation Q
    void initStateTransitionMatrix();    // Initialisation F
    // ...
};
```

#### 2. **Client**

Gère la communication UDP avec le serveur :
```cpp
class Client {
public:
    void init();                                  // Connexion
    void receive();                               // Recevoir mesures
    void sendEstimation(const Vector& est);       // Envoyer estimation
};
```

#### 3. **Parser**

Parse les messages du serveur :
```cpp
class Parser {
public:
    map<string, vector<double>> parseMessage(const string& msg);
    vector<double> createInitialState(map<...> data);
};
```

### Boucle Principale (main.cpp)

```cpp
int main() {
    // 1. Initialisation
    Client client;
    KalmanFilter kalman;
    client.init();
    
    // 2. Recevoir données initiales
    auto data = parser.parseMessage(client.getBuffer());
    kalman.setStateVector(parser.createInitialState(data));
    kalman.setAcceleration(data["ACCELERATION"]);
    
    // 3. Initialiser matrices
    kalman.initProcessNoiseMatrix();
    kalman.initCovarianceMatrix();
    kalman.initMeasurementMatrix();
    kalman.initStateTransitionMatrix();
    kalman.initControlMatrix();
    kalman.initUncertaintyMatrix();
    
    // 4. Envoyer première estimation
    client.sendEstimation(estimation);
    
    // 5. Boucle principale
    while (true) {
        // Recevoir nouvelles données
        client.receive();
        auto data = parser.parseMessage(client.getBuffer());
        
        // Mettre à jour accélération
        if (data.count("ACCELERATION"))
            kalman.setAcceleration(data["ACCELERATION"]);
        
        // PRÉDICTION
        kalman.predictStateVector();
        
        // MISE À JOUR (si GPS disponible)
        if (data.count("POSITION"))
            kalman.update(data["POSITION"]);
        
        // Envoyer estimation
        Vector state = kalman.getStateVector();
        Vector estimation = {state[0], state[1], state[2]};
        client.sendEstimation(estimation);
        
        // Vérifier erreur
        if (data.count("TRUE_POSITION")) {
            double error = calculateDistance(estimation, data["TRUE_POSITION"]);
            if (error > 5.0) break;  // ÉCHEC
        }
    }
    
    return 0;
}
```

---

## 🛠️ Compilation et Utilisation

### Prérequis

- **Compilateur C++** : g++ ou clang++ avec support C++17
- **Make** : pour la compilation
- **Python 3** (optionnel) : pour la visualisation

### Compilation

```bash
make
```

Compile avec les flags : `-Wall -Wextra -Werror -std=c++17`

### Utilisation

#### 1. Lancer le serveur IMU

```bash
# Linux
./imu-sensor-stream-linux -s 42 -d 42 -p 4242

# macOS
./imu-sensor-stream-macos -s 42 -d 42 -p 4242
```

Options :
- `-s` : seed de génération de trajectoire
- `-d` : seed de bruit
- `-p` : port UDP (default 4242)

#### 2. Lancer le client Kalman

```bash
./KalmanClient
```

#### 3. Visualiser les résultats (optionnel)

```bash
python3 tests/plot_logs.py
```

Génère un graphique avec :
- Position estimée vs position GPS
- Évolution de l'erreur
- Moments de mise à jour GPS

---

## 📈 Résultats et Performance

### Critères de Succès

✅ **Erreur < 5m** en permanence  
✅ **Temps de réponse < 1s** par estimation  
✅ **Pas de crash** (segfault, memory leak)  
✅ **Fonctionne jusqu'à 90 minutes** de trajectoire  

### Exemple de Logs

```
t=0.00 POS_EST=1.732 VEL_X=5.000 POS_GPS=1.732 GPS=1
t=0.01 POS_EST=1.782 VEL_X=5.020 POS_GPS=0.000 GPS=0
t=0.02 POS_EST=1.832 VEL_X=5.040 POS_GPS=0.000 GPS=0
...
t=3.00 POS_EST=16.234 VEL_X=5.100 POS_GPS=16.180 GPS=1  ← Correction GPS
t=3.01 POS_EST=16.185 VEL_X=5.102 POS_GPS=0.000 GPS=0   ← État corrigé
```

### Optimisations Possibles

1. **Filtrage adaptatif** : Ajuster Q et R dynamiquement
2. **Filtre de Kalman étendu (EKF)** : Pour des modèles non-linéaires
3. **Prédiction multi-pas** : Anticiper plusieurs mesures
4. **Détection d'outliers** : Rejeter les mesures GPS aberrantes

---

## 📚 Ressources et Références

### Théorie

- **Livre de référence** : *"Kalman Filtering: Theory and Practice Using MATLAB"* - Grewal & Andrews
- **Tutoriel interactif** : [Understanding the Kalman Filter](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)
- **Vidéo** : [Kalman Filter Explained Simply](https://www.youtube.com/watch?v=mwn8xhgNpFY)

### Mathématiques

- **Algèbre linéaire** : Matrices, multiplication, inversion
- **Probabilités** : Gaussiennes, variance, covariance
- **Optimisation** : Moindres carrés, estimation au sens du maximum de vraisemblance

### Applications

- **Navigation GPS** : Fusion GPS + accéléromètre + gyroscope
- **Véhicules autonomes** : Localisation précise
- **Robotique** : SLAM (Simultaneous Localization And Mapping)
- **Finance** : Prédiction de prix

---

## ❓ FAQ

### Pourquoi le GPS est-il si bruité ?

Le GPS a un bruit de **±10cm** (σ = 0.1m) car :
- Réflexions des signaux satellites
- Erreurs atmosphériques
- Précision limitée du récepteur

### Pourquoi ne pas utiliser uniquement le GPS ?

Le GPS arrive seulement **toutes les 3 secondes**. Entre deux mesures GPS, le véhicule parcourt une distance significative (ex: 15 m/s × 3s = 45m). L'accéléromètre permet de prédire la position entre deux GPS.

### Pourquoi ne pas utiliser uniquement l'accéléromètre ?

L'accéléromètre a un bruit faible mais **s'accumule dans le temps** :
- Erreur sur accélération → erreur sur vitesse (intégration)
- Erreur sur vitesse → erreur sur position (double intégration)
- Au bout de 3s, l'erreur peut devenir importante

Le GPS permet de **recalibrer régulièrement** l'estimation.

### Qu'est-ce qu'un bruit blanc gaussien ?

Un bruit :
- **Blanc** : décorrélé dans le temps (le bruit à t=0 est indépendant du bruit à t=1)
- **Gaussien** : suit une loi normale N(υ, σ²)
  - υ = 0 : moyenne nulle
  - σ : écart-type (10⁻³ pour l'accéléromètre, 10⁻¹ pour le GPS)

### Pourquoi P diminue-t-il lors de la mise à jour ?

Quand on reçoit une nouvelle mesure GPS, on **ajoute de l'information** au système. Plus on a d'information, moins on est incertain → P diminue.

### Que se passe-t-il si l'erreur dépasse 5m ?

Le programme s'arrête avec un message d'erreur. Cela signifie que le filtre a divergé, probablement à cause de :
- Mauvaise initialisation des matrices
- Erreur dans les calculs matriciels
- Bug dans le code

---

## 🎓 Conclusion

Le filtre de Kalman est un outil **puissant et élégant** qui combine :
- **Physique** (lois de la cinématique)
- **Mathématiques** (algèbre linéaire, probabilités)
- **Informatique** (implémentation efficace)

Il résout un problème fondamental : **estimer optimalement un état caché à partir de mesures bruitées**.
