import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Chargement des logs
df = pd.read_csv("logs.txt", names=["time", "pos_est", "vel_x", "pos_true"])

# Conversion explicite des NaN (au cas où CSV contient 'nan' en string)
df["pos_true"] = pd.to_numeric(df["pos_true"], errors="coerce")

# Calcul de l'erreur uniquement quand pos_true est valide
df["error"] = df["pos_est"] - df["pos_true"]
df["error_abs"] = df["error"].abs()

# Supprimer les lignes sans pos_true pour la courbe d’erreur
df_error = df[df["pos_true"].notna()]

# Trace
plt.figure(figsize=(12, 6))

# Position
plt.subplot(2, 1, 1)
plt.plot(df["time"], df["pos_est"], label="Position X estimée", color="blue")
plt.plot(df["time"], df["pos_true"], label="Position X vraie (GPS)", linestyle="--", color="green")
plt.ylabel("Position X (m)")
plt.legend()
plt.grid()

# Vitesse et erreur
plt.subplot(2, 1, 2)
plt.plot(df["time"], df["vel_x"], label="Vitesse X estimée", color="orange")
plt.plot(df_error["time"], df_error["error_abs"], label="Erreur |X_est - X_true|", color="red", linestyle="--")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse / Erreur (m ou m/s)")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
