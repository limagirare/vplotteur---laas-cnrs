#include <TMCStepper.h>
#include <AccelStepper.h>
#include <MultiStepper.h> 
#include <Servo.h> // Ajout de la bibliothèque pour le servomoteur

using namespace TMC2130_n;

// ════════════════════════════════════════════════════════════
//  PARAMÈTRES UTILISATEUR — à ajuster selon montage
// ════════════════════════════════════════════════════════════

// Broche et angles pour le servomoteur de levée de stylo
#define SERVO_PIN 0
#define SERVO_ANGLE_BAS 180    // Angle où le stylo TOUCHE le tableau (à ajuster)
#define SERVO_ANGLE_HAUT 30   // Angle où le stylo est LEVÉ (à ajuster)
#define DELAY_SERVO_MS 700    // Temps d'attente pour que le servo bouge (ms)

// Diamètre de la poulie / bobine d'enroulement (mm)
#define PULLEY_DIAM_MM 30.0f

// Dimensions physiques du tableau (mm)
#define TABLEAU_W_MM 1741.0f  
#define TABLEAU_H_MM 900.0f   

// Configuration position initiale maximale au lancement
#define INIT_LENGTH_W_MM 1625.0f 
#define INIT_LENGTH_X_MM 1735.0f 

// ── Géométrie du gondola ─────────────────────────────────────
#define GONDOLA_OFFSET_MM 50.0f
#define PEN_TIP_OFFSET_MM 45.0f

// Paramètres des moteurs
#define MOTOR_STEPS_PER_REV 200  
#define MICROSTEPS 16            

// mm parcourus par micro-pas
const float MM_PER_STEP = (PI * PULLEY_DIAM_MM) / (float)(MOTOR_STEPS_PER_REV * MICROSTEPS);

// Vitesse et accélération par défaut pour le tracé (steps/s)
#define DRAW_SPEED 1200.0f
#define DRAW_ACCEL 1200.0f

// Instance du Servo
Servo penServo;

// ════════════════════════════════════════════════════════════
//  DONNÉES DU DESSIN (GÉNÉRÉES PAR LE SCRIPT PYTHON)
// ════════════════════════════════════════════════════════════

typedef struct {
  float x;
  float y;
} Point2D;

// --- REGLAGES DE TAILLE ET DE POSITIONNEMENT ---
#define ECHELLE_DESSIN 2.0f 

// Permet de centrer automatiquement le dessin sur le tableau
#define OFFSET_X_MM ((TABLEAU_W_MM / 2.0f) - (311.0f * ECHELLE_DESSIN / 2.0f))
#define OFFSET_Y_MM ((TABLEAU_H_MM / 2.0f) - (160.0f * ECHELLE_DESSIN / 2.0f))

/// --- CONFIGURATION POUR TRACEUR ARDUINO ---
const int NOMBRE_FORMES = 31;
const int indexFormes[NOMBRE_FORMES][2] = {
  { 0, 18 }, // Forme 1
  { 18, 7 }, // Forme 3
  { 25, 17 }, // Forme 5
  { 42, 39 }, // Forme 6
  { 81, 58 }, // Forme 8
  { 139, 54 }, // Forme 9
  { 193, 18 }, // Forme 11
  { 211, 9 }, // Forme 12
  { 220, 8 }, // Forme 13
  { 228, 13 }, // Forme 14
  { 241, 79 }, // Forme 16
  { 320, 8 }, // Forme 17
  { 328, 12 }, // Forme 18
  { 340, 18 }, // Forme 19
  { 358, 23 }, // Forme 20
  { 381, 22 }, // Forme 21
  { 403, 25 }, // Forme 22
  { 428, 20 }, // Forme 23
  { 448, 12 }, // Forme 24
  { 460, 13 }, // Forme 25
  { 473, 8 }, // Forme 27
  { 481, 109 }, // Forme 28
  { 590, 14 }, // Forme 30
  { 604, 6 }, // Forme 31
  { 610, 108 }, // Forme 32
  { 718, 22 }, // Forme 33
  { 740, 7 }, // Forme 36
  { 747, 80 }, // Forme 37
  { 827, 32 }, // Forme 38
  { 859, 33 }, // Forme 39
  { 892, 255 } // Forme 40
};

// Liste des coordonnées réduites à envoyer aux moteurs
const int TOTAL_POINTS = 1147;
const int listePoints[TOTAL_POINTS][2] = {
  { 129, 266 },
  { 121, 275 },
  { 107, 285 },
  { 99, 289 },
  { 97, 289 },
  { 96, 287 },
  { 95, 288 },
  { 93, 288 },
  { 89, 286 },
  { 92, 282 },
  { 96, 281 },
  { 106, 276 },
  { 108, 276 },
  { 115, 272 },
  { 117, 272 },
  { 124, 268 },
  { 127, 265 },
  { 129, 266 },
  { 104, 248 },
  { 105, 250 },
  { 93, 264 },
  { 88, 259 },
  { 88, 257 },
  { 102, 250 },
  { 104, 248 },
  { 146, 238 },
  { 152, 241 },
  { 155, 245 },
  { 151, 250 },
  { 149, 255 },
  { 144, 260 },
  { 139, 270 },
  { 137, 272 },
  { 136, 271 },
  { 134, 271 },
  { 130, 266 },
  { 130, 263 },
  { 129, 262 },
  { 130, 253 },
  { 134, 243 },
  { 139, 238 },
  { 146, 238 },
  { 187, 233 },
  { 190, 236 },
  { 190, 244 },
  { 187, 250 },
  { 187, 252 },
  { 185, 254 },
  { 182, 260 },
  { 165, 281 },
  { 163, 285 },
  { 164, 285 },
  { 169, 279 },
  { 170, 281 },
  { 157, 307 },
  { 149, 316 },
  { 144, 317 },
  { 138, 310 },
  { 137, 300 },
  { 136, 299 },
  { 137, 285 },
  { 138, 284 },
  { 138, 280 },
  { 139, 279 },
  { 139, 276 },
  { 140, 275 },
  { 142, 267 },
  { 145, 263 },
  { 146, 264 },
  { 146, 277 },
  { 147, 278 },
  { 147, 268 },
  { 148, 267 },
  { 148, 263 },
  { 149, 262 },
  { 150, 257 },
  { 154, 250 },
  { 163, 241 },
  { 169, 237 },
  { 179, 233 },
  { 187, 233 },
  { 188, 217 },
  { 193, 223 },
  { 198, 232 },
  { 199, 236 },
  { 203, 242 },
  { 213, 252 },
  { 224, 259 },
  { 227, 262 },
  { 227, 265 },
  { 225, 270 },
  { 217, 279 },
  { 210, 284 },
  { 200, 289 },
  { 194, 290 },
  { 190, 292 },
  { 184, 292 },
  { 183, 293 },
  { 168, 293 },
  { 167, 292 },
  { 176, 272 },
  { 185, 261 },
  { 191, 249 },
  { 191, 246 },
  { 192, 245 },
  { 192, 238 },
  { 191, 237 },
  { 191, 235 },
  { 188, 232 },
  { 185, 232 },
  { 184, 231 },
  { 176, 232 },
  { 166, 236 },
  { 157, 243 },
  { 150, 237 },
  { 148, 237 },
  { 147, 236 },
  { 141, 236 },
  { 140, 235 },
  { 140, 229 },
  { 139, 228 },
  { 146, 222 },
  { 150, 220 },
  { 151, 221 },
  { 151, 223 },
  { 148, 229 },
  { 148, 231 },
  { 150, 233 },
  { 153, 233 },
  { 154, 234 },
  { 162, 234 },
  { 163, 233 },
  { 167, 233 },
  { 168, 232 },
  { 174, 231 },
  { 182, 227 },
  { 185, 224 },
  { 185, 219 },
  { 188, 217 },
  { 130, 218 },
  { 134, 222 },
  { 137, 228 },
  { 138, 237 },
  { 132, 242 },
  { 129, 248 },
  { 129, 250 },
  { 127, 252 },
  { 119, 255 },
  { 112, 260 },
  { 109, 264 },
  { 119, 257 },
  { 121, 257 },
  { 123, 255 },
  { 125, 255 },
  { 126, 254 },
  { 128, 255 },
  { 127, 256 },
  { 127, 261 },
  { 128, 263 },
  { 123, 267 },
  { 115, 271 },
  { 113, 271 },
  { 110, 273 },
  { 108, 273 },
  { 105, 275 },
  { 103, 275 },
  { 92, 280 },
  { 88, 284 },
  { 88, 287 },
  { 89, 288 },
  { 96, 288 },
  { 97, 289 },
  { 92, 291 },
  { 87, 290 },
  { 85, 288 },
  { 85, 282 },
  { 88, 276 },
  { 95, 266 },
  { 107, 252 },
  { 116, 239 },
  { 116, 238 },
  { 111, 244 },
  { 110, 243 },
  { 110, 241 },
  { 112, 238 },
  { 112, 235 },
  { 114, 232 },
  { 114, 230 },
  { 117, 223 },
  { 119, 220 },
  { 123, 217 },
  { 128, 217 },
  { 130, 218 },
  { 187, 209 },
  { 184, 215 },
  { 184, 217 },
  { 181, 222 },
  { 181, 224 },
  { 179, 226 },
  { 173, 229 },
  { 163, 231 },
  { 162, 232 },
  { 153, 232 },
  { 150, 230 },
  { 156, 215 },
  { 162, 212 },
  { 166, 212 },
  { 167, 211 },
  { 180, 210 },
  { 185, 208 },
  { 187, 209 },
  { 152, 201 },
  { 154, 203 },
  { 155, 206 },
  { 151, 209 },
  { 146, 209 },
  { 145, 208 },
  { 147, 205 },
  { 151, 201 },
  { 152, 201 },
  { 147, 197 },
  { 150, 199 },
  { 146, 204 },
  { 145, 203 },
  { 146, 201 },
  { 146, 199 },
  { 145, 198 },
  { 147, 197 },
  { 142, 194 },
  { 142, 195 },
  { 144, 196 },
  { 144, 205 },
  { 142, 208 },
  { 145, 210 },
  { 152, 210 },
  { 154, 209 },
  { 157, 206 },
  { 154, 200 },
  { 151, 197 },
  { 147, 195 },
  { 142, 194 },
  { 158, 168 },
  { 158, 173 },
  { 157, 174 },
  { 157, 176 },
  { 153, 180 },
  { 150, 180 },
  { 149, 181 },
  { 146, 181 },
  { 141, 183 },
  { 137, 183 },
  { 132, 185 },
  { 128, 185 },
  { 123, 187 },
  { 119, 187 },
  { 118, 188 },
  { 114, 188 },
  { 113, 189 },
  { 109, 189 },
  { 108, 190 },
  { 103, 190 },
  { 102, 191 },
  { 98, 191 },
  { 97, 192 },
  { 92, 192 },
  { 91, 193 },
  { 87, 193 },
  { 86, 194 },
  { 81, 194 },
  { 80, 195 },
  { 76, 195 },
  { 75, 196 },
  { 69, 196 },
  { 68, 197 },
  { 56, 198 },
  { 55, 199 },
  { 49, 199 },
  { 48, 200 },
  { 40, 200 },
  { 39, 201 },
  { 19, 201 },
  { 13, 197 },
  { 13, 193 },
  { 16, 190 },
  { 21, 188 },
  { 29, 187 },
  { 34, 185 },
  { 38, 185 },
  { 39, 184 },
  { 42, 184 },
  { 47, 182 },
  { 51, 182 },
  { 56, 180 },
  { 60, 180 },
  { 64, 178 },
  { 68, 178 },
  { 73, 176 },
  { 77, 176 },
  { 78, 175 },
  { 81, 175 },
  { 82, 174 },
  { 85, 174 },
  { 90, 172 },
  { 94, 172 },
  { 99, 170 },
  { 103, 170 },
  { 104, 169 },
  { 107, 169 },
  { 112, 167 },
  { 116, 167 },
  { 117, 166 },
  { 125, 165 },
  { 130, 163 },
  { 134, 163 },
  { 139, 161 },
  { 143, 161 },
  { 148, 159 },
  { 153, 161 },
  { 156, 164 },
  { 158, 168 },
  { 251, 144 },
  { 246, 145 },
  { 242, 149 },
  { 239, 154 },
  { 238, 158 },
  { 240, 156 },
  { 241, 153 },
  { 251, 144 },
  { 139, 137 },
  { 141, 139 },
  { 143, 143 },
  { 143, 145 },
  { 142, 146 },
  { 140, 144 },
  { 138, 144 },
  { 136, 149 },
  { 135, 148 },
  { 135, 139 },
  { 137, 137 },
  { 139, 137 },
  { 170, 136 },
  { 175, 137 },
  { 179, 142 },
  { 180, 148 },
  { 181, 149 },
  { 181, 154 },
  { 179, 156 },
  { 177, 156 },
  { 176, 155 },
  { 177, 152 },
  { 176, 151 },
  { 176, 148 },
  { 174, 145 },
  { 170, 145 },
  { 169, 147 },
  { 168, 146 },
  { 167, 140 },
  { 170, 136 },
  { 136, 120 },
  { 140, 125 },
  { 140, 127 },
  { 142, 131 },
  { 142, 134 },
  { 143, 135 },
  { 143, 138 },
  { 142, 139 },
  { 141, 137 },
  { 136, 136 },
  { 134, 139 },
  { 134, 146 },
  { 133, 147 },
  { 132, 146 },
  { 132, 144 },
  { 130, 140 },
  { 130, 136 },
  { 129, 135 },
  { 129, 125 },
  { 130, 124 },
  { 130, 122 },
  { 132, 120 },
  { 136, 120 },
  { 177, 117 },
  { 183, 117 },
  { 188, 122 },
  { 191, 130 },
  { 191, 139 },
  { 190, 140 },
  { 190, 144 },
  { 186, 151 },
  { 183, 154 },
  { 182, 153 },
  { 181, 144 },
  { 178, 138 },
  { 174, 135 },
  { 170, 135 },
  { 169, 136 },
  { 167, 135 },
  { 168, 134 },
  { 168, 129 },
  { 170, 126 },
  { 170, 124 },
  { 172, 121 },
  { 177, 117 },
  { 133, 116 },
  { 129, 119 },
  { 129, 121 },
  { 128, 122 },
  { 128, 135 },
  { 129, 136 },
  { 129, 139 },
  { 130, 140 },
  { 130, 143 },
  { 132, 146 },
  { 132, 148 },
  { 136, 153 },
  { 140, 154 },
  { 143, 151 },
  { 143, 149 },
  { 144, 148 },
  { 144, 136 },
  { 143, 135 },
  { 143, 131 },
  { 142, 130 },
  { 142, 127 },
  { 140, 124 },
  { 140, 122 },
  { 136, 117 },
  { 133, 116 },
  { 177, 114 },
  { 171, 119 },
  { 168, 125 },
  { 167, 132 },
  { 166, 133 },
  { 166, 144 },
  { 167, 145 },
  { 167, 149 },
  { 169, 153 },
  { 172, 156 },
  { 174, 157 },
  { 181, 157 },
  { 187, 152 },
  { 191, 145 },
  { 191, 142 },
  { 192, 141 },
  { 192, 127 },
  { 189, 119 },
  { 184, 114 },
  { 177, 114 },
  { 131, 96 },
  { 127, 96 },
  { 124, 99 },
  { 124, 101 },
  { 123, 102 },
  { 123, 107 },
  { 124, 103 },
  { 127, 99 },
  { 130, 98 },
  { 133, 100 },
  { 133, 98 },
  { 131, 96 },
  { 180, 92 },
  { 179, 94 },
  { 181, 94 },
  { 185, 91 },
  { 190, 92 },
  { 196, 98 },
  { 199, 104 },
  { 200, 104 },
  { 196, 94 },
  { 192, 90 },
  { 190, 89 },
  { 184, 89 },
  { 180, 92 },
  { 285, 83 },
  { 286, 84 },
  { 285, 85 },
  { 285, 91 },
  { 284, 92 },
  { 283, 91 },
  { 282, 85 },
  { 285, 83 },
  { 228, 73 },
  { 230, 77 },
  { 230, 91 },
  { 229, 92 },
  { 228, 101 },
  { 227, 102 },
  { 226, 110 },
  { 225, 111 },
  { 224, 122 },
  { 223, 123 },
  { 223, 133 },
  { 222, 134 },
  { 223, 135 },
  { 223, 142 },
  { 224, 143 },
  { 224, 146 },
  { 228, 152 },
  { 228, 155 },
  { 229, 155 },
  { 229, 152 },
  { 231, 150 },
  { 232, 147 },
  { 241, 138 },
  { 248, 135 },
  { 251, 135 },
  { 253, 136 },
  { 257, 141 },
  { 257, 150 },
  { 253, 160 },
  { 251, 163 },
  { 242, 171 },
  { 237, 173 },
  { 236, 171 },
  { 231, 177 },
  { 227, 184 },
  { 217, 194 },
  { 206, 200 },
  { 204, 200 },
  { 198, 203 },
  { 192, 204 },
  { 188, 206 },
  { 185, 206 },
  { 180, 208 },
  { 176, 208 },
  { 175, 209 },
  { 171, 209 },
  { 170, 210 },
  { 166, 210 },
  { 165, 209 },
  { 167, 208 },
  { 167, 207 },
  { 166, 207 },
  { 164, 209 },
  { 156, 213 },
  { 150, 214 },
  { 149, 215 },
  { 140, 214 },
  { 137, 211 },
  { 138, 210 },
  { 138, 203 },
  { 136, 204 },
  { 133, 202 },
  { 125, 192 },
  { 124, 189 },
  { 125, 188 },
  { 134, 187 },
  { 135, 186 },
  { 138, 186 },
  { 139, 185 },
  { 154, 182 },
  { 158, 178 },
  { 160, 170 },
  { 159, 169 },
  { 158, 164 },
  { 154, 160 },
  { 147, 157 },
  { 146, 158 },
  { 142, 158 },
  { 141, 159 },
  { 133, 160 },
  { 132, 161 },
  { 124, 162 },
  { 123, 163 },
  { 121, 162 },
  { 126, 150 },
  { 126, 145 },
  { 123, 137 },
  { 123, 134 },
  { 122, 133 },
  { 122, 130 },
  { 120, 125 },
  { 120, 121 },
  { 119, 120 },
  { 118, 106 },
  { 117, 105 },
  { 117, 98 },
  { 130, 90 },
  { 132, 90 },
  { 138, 87 },
  { 145, 86 },
  { 146, 85 },
  { 151, 85 },
  { 152, 84 },
  { 200, 85 },
  { 201, 84 },
  { 205, 84 },
  { 206, 83 },
  { 212, 82 },
  { 228, 73 },
  { 311, 37 },
  { 312, 39 },
  { 311, 40 },
  { 310, 47 },
  { 300, 56 },
  { 299, 56 },
  { 279, 76 },
  { 277, 72 },
  { 279, 70 },
  { 284, 61 },
  { 294, 50 },
  { 295, 50 },
  { 302, 43 },
  { 311, 37 },
  { 317, 33 },
  { 318, 35 },
  { 313, 44 },
  { 312, 43 },
  { 313, 36 },
  { 317, 33 },
  { 90, 59 },
  { 101, 47 },
  { 108, 42 },
  { 120, 36 },
  { 122, 36 },
  { 126, 34 },
  { 129, 34 },
  { 130, 33 },
  { 134, 33 },
  { 135, 32 },
  { 153, 32 },
  { 154, 33 },
  { 160, 33 },
  { 161, 34 },
  { 168, 35 },
  { 169, 36 },
  { 183, 40 },
  { 188, 43 },
  { 190, 43 },
  { 208, 52 },
  { 225, 63 },
  { 235, 72 },
  { 236, 72 },
  { 256, 93 },
  { 262, 102 },
  { 263, 105 },
  { 265, 107 },
  { 270, 119 },
  { 271, 127 },
  { 272, 128 },
  { 272, 140 },
  { 271, 141 },
  { 271, 143 },
  { 264, 156 },
  { 252, 170 },
  { 244, 176 },
  { 238, 179 },
  { 233, 180 },
  { 232, 179 },
  { 234, 176 },
  { 244, 172 },
  { 253, 164 },
  { 258, 155 },
  { 259, 149 },
  { 260, 148 },
  { 260, 143 },
  { 256, 135 },
  { 252, 133 },
  { 247, 133 },
  { 240, 136 },
  { 236, 140 },
  { 235, 140 },
  { 229, 149 },
  { 226, 145 },
  { 226, 142 },
  { 225, 141 },
  { 225, 126 },
  { 226, 125 },
  { 226, 118 },
  { 227, 117 },
  { 227, 113 },
  { 228, 112 },
  { 228, 108 },
  { 229, 107 },
  { 229, 103 },
  { 230, 102 },
  { 230, 97 },
  { 231, 96 },
  { 231, 91 },
  { 232, 90 },
  { 232, 80 },
  { 231, 79 },
  { 230, 73 },
  { 228, 71 },
  { 212, 80 },
  { 210, 80 },
  { 206, 82 },
  { 202, 82 },
  { 201, 83 },
  { 180, 83 },
  { 179, 82 },
  { 170, 82 },
  { 169, 81 },
  { 157, 81 },
  { 156, 82 },
  { 149, 82 },
  { 148, 83 },
  { 144, 83 },
  { 143, 84 },
  { 136, 85 },
  { 128, 89 },
  { 126, 89 },
  { 124, 91 },
  { 118, 94 },
  { 110, 101 },
  { 104, 109 },
  { 103, 112 },
  { 101, 113 },
  { 91, 102 },
  { 88, 96 },
  { 88, 94 },
  { 86, 90 },
  { 86, 87 },
  { 85, 86 },
  { 85, 71 },
  { 86, 70 },
  { 86, 67 },
  { 90, 59 },
  { 336, 31 },
  { 332, 36 },
  { 331, 39 },
  { 328, 42 },
  { 326, 42 },
  { 316, 48 },
  { 316, 49 },
  { 325, 49 },
  { 326, 51 },
  { 312, 65 },
  { 299, 74 },
  { 285, 81 },
  { 283, 81 },
  { 282, 82 },
  { 280, 81 },
  { 280, 77 },
  { 307, 51 },
  { 308, 51 },
  { 314, 45 },
  { 315, 45 },
  { 334, 30 },
  { 336, 31 },
  { 338, 26 },
  { 326, 34 },
  { 318, 41 },
  { 316, 40 },
  { 321, 31 },
  { 336, 25 },
  { 338, 26 },
  { 145, 28 },
  { 152, 24 },
  { 159, 23 },
  { 160, 24 },
  { 167, 25 },
  { 185, 34 },
  { 198, 38 },
  { 176, 27 },
  { 180, 25 },
  { 190, 25 },
  { 191, 24 },
  { 195, 24 },
  { 196, 25 },
  { 205, 25 },
  { 206, 26 },
  { 210, 26 },
  { 211, 27 },
  { 216, 28 },
  { 226, 33 },
  { 236, 40 },
  { 250, 54 },
  { 256, 62 },
  { 265, 78 },
  { 267, 85 },
  { 266, 86 },
  { 264, 85 },
  { 260, 81 },
  { 248, 72 },
  { 244, 70 },
  { 232, 60 },
  { 233, 63 },
  { 241, 70 },
  { 258, 82 },
  { 265, 89 },
  { 266, 89 },
  { 274, 97 },
  { 274, 98 },
  { 281, 106 },
  { 285, 112 },
  { 292, 128 },
  { 292, 131 },
  { 293, 132 },
  { 293, 145 },
  { 292, 147 },
  { 285, 154 },
  { 282, 155 },
  { 280, 157 },
  { 267, 162 },
  { 264, 162 },
  { 263, 161 },
  { 267, 155 },
  { 273, 142 },
  { 273, 138 },
  { 274, 137 },
  { 274, 129 },
  { 273, 128 },
  { 272, 117 },
  { 269, 111 },
  { 269, 109 },
  { 266, 103 },
  { 257, 90 },
  { 236, 69 },
  { 235, 69 },
  { 226, 61 },
  { 211, 51 },
  { 206, 49 },
  { 204, 47 },
  { 192, 41 },
  { 190, 41 },
  { 185, 38 },
  { 183, 38 },
  { 180, 36 },
  { 178, 36 },
  { 174, 34 },
  { 171, 34 },
  { 167, 32 },
  { 159, 31 },
  { 158, 30 },
  { 146, 29 },
  { 145, 28 },
  { 198, 22 },
  { 199, 21 },
  { 209, 21 },
  { 210, 22 },
  { 218, 23 },
  { 219, 24 },
  { 227, 26 },
  { 230, 28 },
  { 232, 28 },
  { 243, 34 },
  { 250, 39 },
  { 262, 51 },
  { 271, 64 },
  { 279, 83 },
  { 279, 86 },
  { 281, 91 },
  { 282, 101 },
  { 281, 102 },
  { 272, 92 },
  { 268, 79 },
  { 262, 68 },
  { 253, 55 },
  { 241, 42 },
  { 240, 42 },
  { 231, 34 },
  { 219, 27 },
  { 217, 27 },
  { 214, 25 },
  { 211, 25 },
  { 210, 24 },
  { 199, 23 },
  { 198, 22 },
  { 218, 21 },
  { 220, 19 },
  { 231, 16 },
  { 234, 14 },
  { 236, 14 },
  { 239, 12 },
  { 241, 12 },
  { 251, 8 },
  { 259, 8 },
  { 260, 9 },
  { 262, 9 },
  { 271, 14 },
  { 276, 19 },
  { 282, 29 },
  { 282, 31 },
  { 284, 34 },
  { 284, 37 },
  { 286, 42 },
  { 286, 50 },
  { 287, 51 },
  { 287, 54 },
  { 281, 61 },
  { 276, 70 },
  { 272, 62 },
  { 265, 52 },
  { 259, 46 },
  { 259, 45 },
  { 258, 45 },
  { 257, 43 },
  { 256, 43 },
  { 250, 37 },
  { 238, 29 },
  { 218, 21 },
  { 344, 23 },
  { 331, 25 },
  { 328, 27 },
  { 318, 30 },
  { 316, 32 },
  { 311, 34 },
  { 302, 40 },
  { 290, 51 },
  { 289, 50 },
  { 289, 45 },
  { 288, 44 },
  { 287, 36 },
  { 286, 35 },
  { 285, 30 },
  { 280, 21 },
  { 270, 11 },
  { 267, 9 },
  { 265, 9 },
  { 262, 7 },
  { 259, 7 },
  { 258, 6 },
  { 253, 6 },
  { 252, 7 },
  { 245, 8 },
  { 241, 10 },
  { 238, 10 },
  { 232, 13 },
  { 229, 13 },
  { 226, 15 },
  { 224, 15 },
  { 220, 17 },
  { 217, 17 },
  { 213, 19 },
  { 196, 19 },
  { 190, 22 },
  { 177, 23 },
  { 172, 25 },
  { 162, 21 },
  { 154, 21 },
  { 153, 22 },
  { 147, 23 },
  { 145, 25 },
  { 142, 26 },
  { 138, 30 },
  { 132, 30 },
  { 131, 31 },
  { 124, 32 },
  { 108, 39 },
  { 97, 47 },
  { 93, 51 },
  { 86, 61 },
  { 85, 65 },
  { 83, 68 },
  { 82, 81 },
  { 83, 82 },
  { 83, 88 },
  { 89, 103 },
  { 99, 114 },
  { 103, 114 },
  { 107, 108 },
  { 114, 101 },
  { 115, 102 },
  { 115, 110 },
  { 116, 111 },
  { 117, 121 },
  { 118, 122 },
  { 119, 129 },
  { 120, 130 },
  { 120, 132 },
  { 124, 142 },
  { 124, 145 },
  { 125, 146 },
  { 125, 149 },
  { 118, 162 },
  { 116, 164 },
  { 111, 166 },
  { 107, 166 },
  { 102, 168 },
  { 98, 168 },
  { 97, 169 },
  { 94, 169 },
  { 93, 170 },
  { 90, 170 },
  { 85, 172 },
  { 81, 172 },
  { 76, 174 },
  { 67, 175 },
  { 63, 177 },
  { 59, 177 },
  { 54, 179 },
  { 50, 179 },
  { 49, 180 },
  { 46, 180 },
  { 45, 181 },
  { 42, 181 },
  { 41, 182 },
  { 33, 183 },
  { 32, 184 },
  { 24, 185 },
  { 23, 186 },
  { 18, 187 },
  { 12, 191 },
  { 11, 193 },
  { 12, 198 },
  { 15, 201 },
  { 21, 202 },
  { 22, 203 },
  { 41, 203 },
  { 42, 202 },
  { 49, 202 },
  { 50, 201 },
  { 70, 199 },
  { 71, 198 },
  { 75, 198 },
  { 76, 197 },
  { 82, 197 },
  { 83, 196 },
  { 93, 195 },
  { 94, 194 },
  { 98, 194 },
  { 99, 193 },
  { 104, 193 },
  { 105, 192 },
  { 109, 192 },
  { 110, 191 },
  { 114, 191 },
  { 115, 190 },
  { 119, 190 },
  { 120, 189 },
  { 124, 195 },
  { 135, 206 },
  { 137, 207 },
  { 135, 210 },
  { 135, 213 },
  { 137, 215 },
  { 142, 217 },
  { 149, 217 },
  { 150, 218 },
  { 142, 222 },
  { 139, 225 },
  { 133, 218 },
  { 127, 215 },
  { 121, 216 },
  { 116, 220 },
  { 112, 228 },
  { 112, 230 },
  { 109, 237 },
  { 109, 240 },
  { 107, 244 },
  { 103, 247 },
  { 98, 249 },
  { 96, 251 },
  { 89, 254 },
  { 86, 257 },
  { 87, 262 },
  { 91, 266 },
  { 91, 267 },
  { 85, 276 },
  { 85, 278 },
  { 83, 281 },
  { 83, 287 },
  { 87, 292 },
  { 89, 292 },
  { 90, 293 },
  { 101, 291 },
  { 117, 282 },
  { 127, 272 },
  { 127, 271 },
  { 130, 269 },
  { 131, 271 },
  { 137, 274 },
  { 137, 276 },
  { 135, 280 },
  { 135, 285 },
  { 134, 286 },
  { 134, 302 },
  { 135, 303 },
  { 135, 307 },
  { 137, 312 },
  { 139, 315 },
  { 143, 318 },
  { 145, 318 },
  { 146, 319 },
  { 147, 318 },
  { 150, 318 },
  { 153, 316 },
  { 160, 307 },
  { 164, 298 },
  { 166, 296 },
  { 185, 296 },
  { 186, 295 },
  { 191, 295 },
  { 195, 293 },
  { 198, 293 },
  { 209, 288 },
  { 215, 284 },
  { 224, 275 },
  { 228, 267 },
  { 228, 261 },
  { 216, 251 },
  { 215, 251 },
  { 208, 244 },
  { 202, 236 },
  { 195, 222 },
  { 189, 216 },
  { 188, 216 },
  { 187, 214 },
  { 192, 207 },
  { 206, 203 },
  { 209, 201 },
  { 211, 201 },
  { 221, 195 },
  { 231, 183 },
  { 235, 183 },
  { 241, 180 },
  { 243, 180 },
  { 249, 176 },
  { 259, 166 },
  { 261, 165 },
  { 266, 165 },
  { 267, 164 },
  { 275, 162 },
  { 283, 158 },
  { 291, 151 },
  { 294, 146 },
  { 294, 142 },
  { 295, 141 },
  { 295, 132 },
  { 294, 131 },
  { 294, 127 },
  { 293, 126 },
  { 292, 121 },
  { 286, 109 },
  { 284, 107 },
  { 285, 105 },
  { 285, 101 },
  { 286, 100 },
  { 286, 96 },
  { 287, 95 },
  { 288, 82 },
  { 291, 80 },
  { 293, 80 },
  { 297, 78 },
  { 299, 76 },
  { 308, 71 },
  { 323, 57 },
  { 328, 49 },
  { 327, 48 },
  { 321, 48 },
  { 320, 47 },
  { 323, 45 },
  { 330, 43 },
  { 333, 40 },
  { 334, 37 },
  { 344, 23 }
};



// ════════════════════════════════════════════════════════════
//  CÂBLAGE ET CONFIGURATION DES INSTANCES (TMC2130)
// ════════════════════════════════════════════════════════════
#define SW_MOSI 19
#define SW_MISO 16
#define SW_SCK 18
#define FEEDBACK_PERIOD_MS 100
#define R_SENSE 0.11f

#define W_CS_PIN 27
#define W_EN_PIN 3
#define W_DIR_PIN 4
#define W_STEP_PIN 5
#define W_DIAG0_PIN 2
#define W_MICROSTEPS 16
#define W_CURRENT_MA 600
#define W_STALL_VALUE 3

#define X_CS_PIN 14
#define X_EN_PIN 7
#define X_DIR_PIN 8
#define X_STEP_PIN 9
#define X_DIAG0_PIN 6
#define X_MICROSTEPS 16
#define X_CURRENT_MA 600
#define X_STALL_VALUE 3

#define Z_CS_PIN 17
#define Z_EN_PIN 22
#define Z_DIR_PIN 21
#define Z_STEP_PIN 20
#define Z_DIAG0_PIN 26
#define Z_MICROSTEPS 16
#define Z_CURRENT_MA 600
#define Z_STALL_VALUE 13

#define UV_EN_PIN 28

#define W_MAX_POS_STEPS 17000
#define W_MAX_VEL_STEPS 10000
#define W_MAX_ACC_STEPS 200000

#define X_MAX_POS_STEPS 15000
#define X_MAX_VEL_STEPS 10000
#define X_MAX_ACC_STEPS 200000

#define Z_MIN_POS_STEPS 0
#define Z_MAX_POS_STEPS 72000
#define Z_MAX_VEL_STEPS 5000
#define Z_MAX_ACC_STEPS 100000

typedef struct {
  char name;
  uint8_t cs; uint8_t en; uint8_t dir; uint8_t step; int8_t diag0;
  uint32_t microsteps; uint16_t current_mA; int16_t stall_value;
  int32_t min_pos; int32_t max_pos;
  uint32_t max_vel; uint32_t max_acc;
} AxisConfig;

typedef struct { TMC2130Stepper *driver; AccelStepper *stepper; } AxisData;

TMC2130Stepper driverX(X_CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
TMC2130Stepper driverY(W_CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
TMC2130Stepper driverZ(Z_CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

AccelStepper stepperW(AccelStepper::DRIVER, W_STEP_PIN, W_DIR_PIN);
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

MultiStepper steppers;

AxisConfig axisConfigs[] = {
  { 'W', W_CS_PIN, W_EN_PIN, W_DIR_PIN, W_STEP_PIN, W_DIAG0_PIN, W_MICROSTEPS, W_CURRENT_MA, W_STALL_VALUE, 0, W_MAX_POS_STEPS, W_MAX_VEL_STEPS, W_MAX_ACC_STEPS },
  { 'X', X_CS_PIN, X_EN_PIN, X_DIR_PIN, X_STEP_PIN, X_DIAG0_PIN, X_MICROSTEPS, X_CURRENT_MA, X_STALL_VALUE, 0, X_MAX_POS_STEPS, X_MAX_VEL_STEPS, W_MAX_ACC_STEPS },
  { 'Z', Z_CS_PIN, Z_EN_PIN, Z_DIR_PIN, Z_STEP_PIN, Z_DIAG0_PIN, Z_MICROSTEPS, Z_CURRENT_MA, Z_STALL_VALUE, Z_MIN_POS_STEPS, Z_MAX_POS_STEPS, Z_MAX_VEL_STEPS, Z_MAX_ACC_STEPS }
};

AxisData axes[3];

float pen_x_mm;
float pen_y_mm;
unsigned long lastFeedback = 0;
bool traceDone = false;

// ════════════════════════════════════════════════════════════
//  FONCTIONS INTERNES DU SERVO
// ════════════════════════════════════════════════════════════

void leverStylo() {
  penServo.write(SERVO_ANGLE_HAUT);
  delay(DELAY_SERVO_MS); // Laisse le temps mécanique au bras de monter
}

void baisserStylo() {
  penServo.write(SERVO_ANGLE_BAS);
  delay(DELAY_SERVO_MS); // Laisse le stylo se poser proprement
}

// ════════════════════════════════════════════════════════════
//  GÉOMÉTRIE ET CINÉMATIQUE
// ════════════════════════════════════════════════════════════

float calcL_left(float px, float py) {
  float ax = px - GONDOLA_OFFSET_MM / 2.0f;
  float ay = py - PEN_TIP_OFFSET_MM;
  return sqrtf(ax * ax + ay * ay);
}

float calcL_right(float px, float py) {
  float ax = px + GONDOLA_OFFSET_MM / 2.0f;
  float ay = py - PEN_TIP_OFFSET_MM;
  float dx = TABLEAU_W_MM - ax;
  return sqrtf(dx * dx + ay * ay);
}

long mmToSteps(float mm) {
  return (long)(mm / MM_PER_STEP);
}

// ────────────────────────────────────────────────────────────
//  CONTRÔLE SIMPLIFIÉ (Mouvement direct d'un point à l'autre)
// ────────────────────────────────────────────────────────────
void penMoveTo(float x_mm, float y_mm) {
  // On garde la sécurité des bords physiques du tableau
  x_mm = constrain(x_mm, 10.0f, TABLEAU_W_MM - 10.0f);
  y_mm = constrain(y_mm, 10.0f, TABLEAU_H_MM - 10.0f);

  // Mouvement direct vers les coordonnées calculées sans subdivision
  long positions[2] = {
    -mmToSteps(INIT_LENGTH_W_MM - calcL_left(x_mm, y_mm)),
     mmToSteps(INIT_LENGTH_X_MM - calcL_right(x_mm, y_mm))
  };
  
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();

  pen_x_mm = x_mm;
  pen_y_mm = y_mm;
}

// ════════════════════════════════════════════════════════════
//  TRACÉ PAR FORMES EN CORRÉLATION AVEC LES LEVÉES DE STYLO
// ════════════════════════════════════════════════════════════

void tracePointsList() {
  Serial.printf("Debut du tracé ordonné par formes (%d formes détectées)...\n", NOMBRE_FORMES);
  
  for (int f = 0; f < NOMBRE_FORMES; f++) {
    int indexDepart = indexFormes[f][0];
    int nbPointsForme = indexFormes[f][1];
    
    Serial.printf("--- Tracé Forme %d : Déplacement aérien vers le premier point ---\n", f);
    
    // 1. Lever le stylo avant d'aller au point de départ de la nouvelle forme
    leverStylo();
    
    // 2. Aller en l'air jusqu'au premier point
    float debut_x = ((float)listePoints[indexDepart][0] * ECHELLE_DESSIN) + OFFSET_X_MM;
    float debut_y = ((float)listePoints[indexDepart][1] * ECHELLE_DESSIN) + OFFSET_Y_MM;
    penMoveTo(debut_x, debut_y); 
    
    // 3. Poser le stylo sur le tableau
    baisserStylo();
    
    // 4. Tracer tous les points internes de cette forme spécifique
    for (int p = 0; p < nbPointsForme; p++) {
      int i = indexDepart + p; // Index réel global dans listePoints
      
      float x = ((float)listePoints[i][0] * ECHELLE_DESSIN) + OFFSET_X_MM;
      float y = ((float)listePoints[i][1] * ECHELLE_DESSIN) + OFFSET_Y_MM;

      penMoveTo(x, y);

      if (millis() - lastFeedback >= FEEDBACK_PERIOD_MS) {
        lastFeedback = millis();
        Serial.printf("Forme %d/%d | Point %d/%d | X=%.1f, Y=%.1f\n", f+1, NOMBRE_FORMES, p+1, nbPointsForme, x, y);
      }
    }
  }
  
  // Tracé fini, on sécurise le stylo en l'air
  leverStylo();
  Serial.println("Dessin terminé avec levées de stylo gérées !");
}

// ════════════════════════════════════════════════════════════
//  INITIALISATIONS ET SETUP
// ════════════════════════════════════════════════════════════

void axis_init(int idx) {
  AxisConfig *cfg = &axisConfigs[idx];
  pinMode(cfg->cs, OUTPUT);   digitalWrite(cfg->cs, HIGH);
  pinMode(cfg->en, OUTPUT);   digitalWrite(cfg->en, LOW);
  if (cfg->diag0 >= 0) pinMode(cfg->diag0, INPUT);
  
  TMC2130Stepper *drv = (idx == 0 ? &driverY : idx == 1 ? &driverX : &driverZ);
  axes[idx].driver = drv;
  drv->begin();
  drv->rms_current(cfg->current_mA);
  drv->microsteps(cfg->microsteps);
  drv->en_pwm_mode(true);
  drv->pwm_autoscale(true);

  AccelStepper *stp = (idx == 0 ? &stepperW : idx == 1 ? &stepperX : &stepperZ);
  axes[idx].stepper = stp;
  stp->setEnablePin(cfg->en);
  stp->setPinsInverted(true, false, true);
  stp->enableOutputs();
  stp->setMaxSpeed(cfg->max_vel);
  stp->setAcceleration(cfg->max_acc);
}

void setup() {
  analogWriteFreq(1000);
  Serial.begin(9600);
  while (!Serial) ;

  // Initialisation mécanique du Servo de levée de stylo
  penServo.attach(SERVO_PIN);
  leverStylo(); // On commence obligatoirement en l'air pour la sécurité

  SPI.begin();
  axis_init(0);
  axis_init(1);
  axis_init(2);

  steppers.addStepper(stepperW);
  steppers.addStepper(stepperX);

  stepperW.setMaxSpeed(DRAW_SPEED);
  stepperX.setMaxSpeed(DRAW_SPEED);

  stepperW.setCurrentPosition(0);
  stepperX.setCurrentPosition(0);
  
  float premier_x = ((float)listePoints[0][0] * ECHELLE_DESSIN) + OFFSET_X_MM;
  float premier_y = ((float)listePoints[0][1] * ECHELLE_DESSIN) + OFFSET_Y_MM;
  pen_x_mm = premier_x;
  pen_y_mm = premier_y;

  Serial.println("Machine prete. Fils consideres au maximum.");
  Serial.println("Appuyez sur Entree pour rembobiner automatiquement (en l'air) vers le PREMIER point...");
  while (!Serial.available()) {}
  Serial.read();

  Serial.println("Rembobinage automatique vers le point de depart du dessin...");
  penMoveTo(premier_x, premier_y);
  
  delay(2000); 

  // Lancement du dessin séquencé par le tableau indexFormes
  tracePointsList();
  traceDone = true;
}

void loop() {
  // Vide
}