import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import glob
import os
import numpy as np

# Fonction pour charger tous les fichiers CSV d'un certain type
def load_csv_files(file_pattern):
    all_files = glob.glob(file_pattern)
    df_list = []
    for file in all_files:
        df = pd.read_csv(file)
        df_list.append(df)
    if df_list:
        return pd.concat(df_list, ignore_index=True)
    else:
        return pd.DataFrame()  # Retourne un DataFrame vide si aucun fichier n'est trouvé

# Charger les données
warning_data = load_csv_files("collision_data_*.csv")
alert_data = load_csv_files("collision_alert_*.csv")

# 1. Analyse générale du scénario
total_vehicles = warning_data['VehicleID'].nunique() if not warning_data.empty else 0
simulation_duration = warning_data['Time'].max() if not warning_data.empty else 0
total_warnings = warning_data[warning_data['SubCauseCode'] > 0].shape[0] if not warning_data.empty else 0
total_alerts_received = alert_data.shape[0] if not alert_data.empty else 0

print("Analyse générale du scénario:")
print(f"Nombre total de véhicules: {total_vehicles}")
print(f"Durée totale de la simulation: {simulation_duration:.2f} secondes")
print(f"Nombre total d'alertes émises: {total_warnings}")
print(f"Nombre total d'alertes reçues: {total_alerts_received}")

# 2. Analyse des émissions d'alertes (CollisionWarningService)

if not warning_data.empty:
    # a. Fréquence des alertes émises au fil du temps
    plt.figure(figsize=(12, 6))
    warning_data[warning_data['SubCauseCode'] > 0]['Time'].hist(bins=50)
    plt.title("Fréquence des alertes émises au fil du temps")
    plt.xlabel("Temps (secondes)")
    plt.ylabel("Nombre d'alertes")
    plt.savefig("frequence_alertes.png")
    plt.show()
    plt.close()

    # b. Distribution des niveaux de danger (SubCauseCode)
    danger_levels = warning_data['SubCauseCode'].value_counts().sort_index()
    labels = ['Pas de danger', 'Avertissement', 'Critique']
    sizes = [danger_levels.get(i, 0) for i in range(3)]  # Assure que nous avons 3 valeurs
    plt.figure(figsize=(8, 8))
    plt.pie(sizes, labels=labels, autopct='%1.1f%%', startangle=90)
    plt.title("Distribution des niveaux de danger")
    plt.savefig("distribution_danger.png")
    plt.show()
    plt.close()

    # c. Évolution du TTC moyen au fil du temps
    warning_data['TTC'] = pd.to_numeric(warning_data['TTC'], errors='coerce')
    ttc_evolution = warning_data.groupby('Time')['TTC'].mean().rolling(window=10).mean()
    plt.figure(figsize=(12, 6))
    ttc_evolution.plot()
    plt.title("Évolution du TTC moyen au fil du temps")
    plt.xlabel("Temps (secondes)")
    plt.ylabel("TTC moyen (secondes)")
    plt.savefig("evolution_ttc.png")
    plt.show()

    # d. Carte des positions où les alertes sont émises
    plt.figure(figsize=(12, 8))
    alerts = warning_data[warning_data['SubCauseCode'] > 0]
    if len(alerts) > 1:
        sns.kdeplot(data=alerts, x='PositionX', y='PositionY', cmap="YlOrRd", fill=True)
    else:
        plt.scatter(alerts['PositionX'], alerts['PositionY'], color='red', s=100)
    plt.title("Carte des positions d'alertes")
    plt.xlabel("Position X")
    plt.ylabel("Position Y")
    plt.savefig("map_alertes.png")
    plt.close()

    print("Les graphiques ont été sauvegardés dans le répertoire courant.")
else:
    print("Aucune donnée d'alerte disponible pour l'analyse.")