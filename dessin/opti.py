import cv2
import numpy as np
import os

# ==============================================================================
# RÉGLAGES
# ==============================================================================
NOM_IMAGE = input("Nom de l'image (avec extension) : ").strip()

# --- LEVIERS D'OPTIMISATION POUR ARDUINO ---
# Augmente cette valeur pour réduire le nombre de points.
# 1.0 = Très fidèle mais beaucoup de points.
# 3.0 à 5.0 = Excellente simplification, idéal pour les moteurs pas-à-pas / servos.
TOLERANCE_SIMPLIFICATION = 0.6

# Nombre de pixels minimum pour qu'une forme soit acceptée.
# Permet d'éliminer les poussières ou imperfections de l'image.
SEUIL_PIXELS_MIN = 15  
# ==============================================================================

DOSSIER_DU_SCRIPT = os.path.dirname(os.path.abspath(__file__))
CHEMIN_IMAGE = os.path.join(DOSSIER_DU_SCRIPT, NOM_IMAGE)

if not os.path.exists(CHEMIN_IMAGE):
    print(f"ERREUR : Place '{NOM_IMAGE}' dans : {DOSSIER_DU_SCRIPT}")
    exit()

# 1. CHARGEMENT ET BORDURE DE SÉCURITÉ
img_gris = cv2.imread(CHEMIN_IMAGE, cv2.IMREAD_GRAYSCALE)
BORDURE = 10
img_blanche = cv2.copyMakeBorder(img_gris, BORDURE, BORDURE, BORDURE, BORDURE, cv2.BORDER_CONSTANT, value=255)
hauteur_image, _ = img_blanche.shape

# 2. SEUILLAGE
_, img_binaire = cv2.threshold(img_blanche, 220, 255, cv2.THRESH_BINARY_INV)

# 3. EXTRACTION DES CONTOURS
contours, _ = cv2.findContours(img_binaire, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

points_arduino = []
points_geogebra = []
lignes_index = []
index_courant = 0

# 4. SIMPLIFICATION FORME PAR FORME
for i, c in enumerate(contours):
    # Filtre 1 : On ignore les formes trop petites en surface (bruit)
    if cv2.contourArea(c) < SEUIL_PIXELS_MIN:
        continue
        
    # Algorithme de Douglas-Peucker
    points_simplifies = cv2.approxPolyDP(c, TOLERANCE_SIMPLIFICATION, closed=True)
    
    # Filtre 2 : Une forme valide doit avoir au moins 3 sommets initiaux
    if len(points_simplifies) < 3:
        continue
        
    # Extraction des points de la forme actuelle
    points_forme_courante = []
    for pt in points_simplifies:
        x_reel = int(pt[0][0] - BORDURE)
        y_reel = int(pt[0][1] - BORDURE)
        points_forme_courante.append((x_reel, y_reel))
    
    # Fermeture de la forme (copie du premier point à la fin)
    points_forme_courante.append(points_forme_courante[0])
    
    nb_pts_forme = len(points_forme_courante)
    
    # Stockage de l'index
    virgule = "," if i < len(contours) - 1 else ""
    lignes_index.append(f"  {{ {index_courant}, {nb_pts_forme} }}{virgule} // Forme {i}\n")
    
    # Remplissage des listes globales
    for pt in points_forme_courante:
        x_reel, y_reel = pt
        points_arduino.append((x_reel, y_reel))
        
        # Points inversés (GeoGebra)
        y_inverse = (hauteur_image - 2 * BORDURE) - y_reel
        points_geogebra.append((x_reel, y_inverse))
        
    index_courant += nb_pts_forme

# Nettoyage de la virgule sur la dernière forme
if lignes_index:
    lignes_index[-1] = lignes_index[-1].replace("}, //", "}  //").replace("},\n", "}\n")

# ==============================================================================
# 5. ÉCRITURE DU FICHIER ARDUINO (C++)
# ==============================================================================
code_c = "// --- CONFIGURATION POUR TRACEUR ARDUINO ---\n"
code_c += f"const int NOMBRE_FORMES = {len(lignes_index)};\n"
code_c += "const int indexFormes[NOMBRE_FORMES][2] = {\n"
code_c += "".join(lignes_index)
code_c += "};\n\n"

code_c += f"// Liste des coordonnées réduites à envoyer aux moteurs\n"
code_c += f"const int TOTAL_POINTS = {len(points_arduino)};\n"
code_c += "const int listePoints[TOTAL_POINTS][2] = {\n"
for k, pt in enumerate(points_arduino):
    virgule = "," if k < len(points_arduino) - 1 else ""
    code_c += f"  {{ {pt[0]}, {pt[1]} }}{virgule}\n"
code_c += "};\n"

with open(os.path.join(DOSSIER_DU_SCRIPT, "code_arduino.txt"), "w", encoding="utf-8") as f_cpp:
    f_cpp.write(code_c)

# ==============================================================================
# 6. ÉCRITURE DU FICHIER GEOGEBRA
# ==============================================================================
texte_geogebra = ""
for pt in points_geogebra:
    texte_geogebra += f"{pt[0]}, {pt[1]}\n"

with open(os.path.join(DOSSIER_DU_SCRIPT, "points_geogebra.txt"), "w", encoding="utf-8") as f_geo:
    f_geo.write(texte_geogebra)

print("==========================================================")
print("             OPTIMISATION ARDUINO TERMINÉE !              ")
print("==========================================================")
print(f"-> Nombre de points générés : {len(points_arduino)}")
print(f"-> Nombre de formes : {len(lignes_index)}")
print("\nVérifie le rendu dans GeoGebra pour ajuster 'TOLERANCE_SIMPLIFICATION' si besoin.")
