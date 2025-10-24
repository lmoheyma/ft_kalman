import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import re

# Lecture du fichier de logs
data = []
with open("logs.txt", "r") as f:
    for line in f:
        match = re.search(r't=(\S+) POS_EST=(\S+) VEL_X=(\S+) POS_GPS=(\S+) POS_TRUE=(\S+) GPS=(\S+)', line)
        if match:
            data.append({
                'time': float(match.group(1)),
                'pos_est': float(match.group(2)),
                'vel_x': float(match.group(3)),
                'pos_gps': float(match.group(4)),
                'pos_true': float(match.group(5)),
                'has_gps': int(match.group(6))
            })

df = pd.DataFrame(data)

# Remplacer les 0 par NaN pour mieux visualiser
df['pos_gps'] = df['pos_gps'].replace(0, np.nan)
df['pos_true'] = df['pos_true'].replace(0, np.nan)

# Calcul de l'erreur
df['error'] = np.abs(df['pos_est'] - df['pos_true'])

# Création des graphiques
fig, axes = plt.subplots(3, 1, figsize=(14, 10))

# Graphique 1 : Position
axes[0].plot(df['time'], df['pos_est'], label='Position X estimée', color='blue', linewidth=2)
axes[0].scatter(df[df['has_gps']==1]['time'], df[df['has_gps']==1]['pos_gps'], 
                label='Mesures GPS', color='green', marker='o', s=30, alpha=0.7)
axes[0].plot(df['time'], df['pos_true'], label='Position X vraie', 
             linestyle='--', color='red', linewidth=1, alpha=0.7)
axes[0].set_ylabel('Position X (m)')
axes[0].set_title('Estimation de position vs GPS vs Réalité')
axes[0].legend()
axes[0].grid(True, alpha=0.3)

# Graphique 2 : Vitesse
axes[1].plot(df['time'], df['vel_x'], label='Vitesse X estimée', color='orange', linewidth=2)
axes[1].set_ylabel('Vitesse X (m/s)')
axes[1].set_title('Estimation de vitesse')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

# Graphique 3 : Erreur
df_with_true = df[df['pos_true'].notna()]
axes[2].plot(df_with_true['time'], df_with_true['error'], 
             label='Erreur absolue', color='red', linewidth=2)
axes[2].axhline(y=5.0, color='darkred', linestyle='--', linewidth=2, label='Limite (5m)')
axes[2].set_xlabel('Temps (s)')
axes[2].set_ylabel('Erreur (m)')
axes[2].set_title('Erreur de position')
axes[2].legend()
axes[2].grid(True, alpha=0.3)
axes[2].set_ylim(bottom=0)

plt.tight_layout()
plt.savefig('kalman_results.png', dpi=150)
print("Graphique sauvegardé : kalman_results.png")
plt.show()

# Statistiques
print("\n=== STATISTIQUES ===")
print(f"Erreur moyenne: {df_with_true['error'].mean():.4f} m")
print(f"Erreur max: {df_with_true['error'].max():.4f} m")
print(f"Erreur finale: {df_with_true['error'].iloc[-1]:.4f} m")
print(f"Nombre de mesures GPS: {df['has_gps'].sum()}")
print(f"Durée totale: {df['time'].iloc[-1]:.2f} s")
