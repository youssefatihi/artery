import os
import subprocess

def read_file_content(file_path):
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            return file.read()
    except Exception as e:
        return f"Erreur lors de la lecture du fichier {file_path}: {str(e)}"

def create_prompt(directory):
    prompt = """Résumé des actions : 1. Nous avons créé un scénario de simulation pour un système d'alerte de collision arrière en utilisant le framework Artery. 2. Nous avons développé deux services principaux : CollisionWarningService et CollisionAlertReceiverService. 3. Nous avons configuré l'environnement de simulation, y compris l'intégration avec SUMO pour la simulation de trafic. 4. Nous avons travaillé sur l'intégration du LocalEnvironmentModel pour la détection des objets environnants. 5. Nous avons résolu plusieurs problèmes de compilation et de configuration liés à l'utilisation d'Artery et OMNeT++. Présentation du projet : Nous sommes en train de développer une simulation de véhicules connectés utilisant la communication V2X (Vehicle-to-Everything) pour améliorer la sécurité routière. Le scénario spécifique que nous simulons est un système d'alerte de collision arrière. Modules principaux : 1. CollisionWarningService : Ce service est responsable de la détection des risques de collision et de l'émission des alertes. 2. CollisionAlertReceiverService : Ce service reçoit les alertes de collision et réagit en conséquence, par exemple en réduisant la vitesse du véhicule. 3. LocalEnvironmentModel : Ce module gère la perception de l'environnement local autour de chaque véhicule, y compris la détection des autres véhicules à proximité. 4. SUMO Integration : Nous utilisons SUMO (Simulation of Urban MObility) pour simuler le trafic routier réaliste sur lequel notre système V2X opère. 5. Artery Framework : Nous utilisons Artery comme base pour notre simulation, qui fournit une plateforme pour la simulation de communications V2X basée sur OMNeT++. Le but de ce projet est de démontrer comment la communication V2X peut être utilisée pour prévenir les collisions arrière en alertant les conducteurs des dangers potentiels et en permettant aux véhicules de réagir automatiquement aux situations dangereuses. Voici le code . """

    # Liste des fichiers à exclure
    excluded_files = ['ssm_vehA.xml', 'ssm_vehB.xml', 'prompt_output.txt', 'highway_fcd.xml', 'highway.net.xml', 'prompt.py', 'script.sh']
    excluded_dirs = ['results','logs']

    for root, dirs, files in os.walk(directory):
        # Exclure les répertoires non désirés
        dirs[:] = [d for d in dirs if d not in excluded_dirs]
        
        for file in files:
            if file not in excluded_files:
                file_path = os.path.join(root, file)
                relative_path = os.path.relpath(file_path, directory)
                prompt += f"{relative_path} : \"{read_file_content(file_path)}\"\n\n"

    return prompt

def open_in_vscode(file_path):
    try:
        subprocess.run(['code', file_path])
        print(f"Le fichier {file_path} a été ouvert dans Visual Studio Code.")
    except FileNotFoundError:
        print("Visual Studio Code n'a pas été trouvé. Assurez-vous qu'il est installé et accessible via la commande 'code'.")
    except Exception as e:
        print(f"Une erreur s'est produite lors de l'ouverture du fichier dans Visual Studio Code : {str(e)}")

def main():
    directory = input("Entrez le chemin du répertoire contenant les fichiers du projet : ")
    output_file = "prompt_output.txt"
    
    prompt = create_prompt(directory)
    
    with open(output_file, 'w', encoding='utf-8') as file:
        file.write(prompt)
    
    print(f"Le prompt a été généré et enregistré dans {output_file}")
    
    # Ouvrir le fichier dans Visual Studio Code
    open_in_vscode(output_file)

if __name__ == "__main__":
    main()