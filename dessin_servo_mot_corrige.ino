#include <TMCStepper.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

// ── Données de la police (généré par font_extractor.html)
#include "font_data.h"

using namespace TMC2130_n;

// ════════════════════════════════════════════════════════════
//  PARAMÈTRES UTILISATEUR — à ajuster selon montage
// ════════════════════════════════════════════════════════════

#define SERVO_PIN 1
#define SERVO_ANGLE_BAS 90
#define SERVO_ANGLE_HAUT 30
#define DELAY_SERVO_MS 700

#define PULLEY_DIAM_MM 30.0f
#define TABLEAU_W_MM 1741.0f
#define TABLEAU_H_MM 900.0f

#define INIT_LENGTH_W_MM 1625.0f
#define INIT_LENGTH_X_MM 1735.0f

#define GONDOLA_OFFSET_MM 50.0f
#define PEN_TIP_OFFSET_MM 45.0f

#define MOTOR_STEPS_PER_REV 200
#define MICROSTEPS 16

const float MM_PER_STEP = (PI * PULLEY_DIAM_MM) / (float)(MOTOR_STEPS_PER_REV * MICROSTEPS);

#define DRAW_SPEED 1200.0f
#define DRAW_ACCEL 1200.0f

// ════════════════════════════════════════════════════════════
//  PARAMÈTRES D'ÉCRITURE — à ajuster
// ════════════════════════════════════════════════════════════

// Hauteur du texte sur le tableau en mm
// FONT_REF_HEIGHT_PX correspond à cette hauteur
#define TEXT_HEIGHT_MM 120.0f

// Position du coin supérieur-gauche du texte sur le tableau
#define TEXT_ORIGIN_X_MM 200.0f
#define TEXT_ORIGIN_Y_MM 350.0f

// Echelle pixel→mm : TEXT_HEIGHT_MM / FONT_REF_HEIGHT_PX
// (calculé dynamiquement, voir wordScale())

// ════════════════════════════════════════════════════════════
//  CÂBLAGE ET CONFIGURATION (inchangé)
// ════════════════════════════════════════════════════════════
#define SW_MOSI 19
#define SW_MISO 16
#define SW_SCK  18
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
  uint8_t cs, en, dir, step;
  int8_t  diag0;
  uint32_t microsteps;
  uint16_t current_mA;
  int16_t  stall_value;
  int32_t  min_pos, max_pos;
  uint32_t max_vel, max_acc;
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
  {'W', W_CS_PIN, W_EN_PIN, W_DIR_PIN, W_STEP_PIN, W_DIAG0_PIN, W_MICROSTEPS, W_CURRENT_MA, W_STALL_VALUE, 0, W_MAX_POS_STEPS, W_MAX_VEL_STEPS, W_MAX_ACC_STEPS},
  {'X', X_CS_PIN, X_EN_PIN, X_DIR_PIN, X_STEP_PIN, X_DIAG0_PIN, X_MICROSTEPS, X_CURRENT_MA, X_STALL_VALUE, 0, X_MAX_POS_STEPS, X_MAX_VEL_STEPS, W_MAX_ACC_STEPS},
  {'Z', Z_CS_PIN, Z_EN_PIN, Z_DIR_PIN, Z_STEP_PIN, Z_DIAG0_PIN, Z_MICROSTEPS, Z_CURRENT_MA, Z_STALL_VALUE, Z_MIN_POS_STEPS, Z_MAX_POS_STEPS, Z_MAX_VEL_STEPS, Z_MAX_ACC_STEPS}
};

AxisData axes[3];

float pen_x_mm = TEXT_ORIGIN_X_MM;
float pen_y_mm = TEXT_ORIGIN_Y_MM;
unsigned long lastFeedback = 0;

Servo penServo;

// ════════════════════════════════════════════════════════════
//  SERVO
// ════════════════════════════════════════════════════════════
void leverStylo() {
  if (!penServo.attached()) penServo.attach(SERVO_PIN, 500, 2500);
  penServo.write(SERVO_ANGLE_HAUT);
  delay(DELAY_SERVO_MS);
  penServo.detach();
}

void baisserStylo() {
  if (!penServo.attached()) penServo.attach(SERVO_PIN, 500, 2500);
  penServo.write(SERVO_ANGLE_BAS);
  delay(DELAY_SERVO_MS);
  penServo.detach();
}

// ════════════════════════════════════════════════════════════
//  CINÉMATIQUE (identique à l'original)
// ════════════════════════════════════════════════════════════
float calcL_left(float px, float py) {
  float ax = px - GONDOLA_OFFSET_MM / 2.0f;
  float ay = py - PEN_TIP_OFFSET_MM;
  return sqrtf(ax*ax + ay*ay);
}

float calcL_right(float px, float py) {
  float ax = px + GONDOLA_OFFSET_MM / 2.0f;
  float ay = py - PEN_TIP_OFFSET_MM;
  float dx = TABLEAU_W_MM - ax;
  return sqrtf(dx*dx + ay*ay);
}

long mmToSteps(float mm) {
  return (long)(mm / MM_PER_STEP);
}

#define MAX_SEGMENT_LEN_MM 5.0f

void penMoveTo(float x_mm, float y_mm) {
  x_mm = constrain(x_mm, 10.0f, TABLEAU_W_MM - 10.0f);
  y_mm = constrain(y_mm, 10.0f, TABLEAU_H_MM - 10.0f);

  float dx = x_mm - pen_x_mm;
  float dy = y_mm - pen_y_mm;
  float dist = sqrtf(dx*dx + dy*dy);

  if (dist > MAX_SEGMENT_LEN_MM) {
    int n = (int)(dist / MAX_SEGMENT_LEN_MM);
    for (int i = 1; i <= n; i++) {
      float r = (float)i / (float)n;
      float sx = pen_x_mm + dx*r, sy = pen_y_mm + dy*r;
      long pos[2] = {
        -mmToSteps(INIT_LENGTH_W_MM - calcL_left(sx, sy)),
         mmToSteps(INIT_LENGTH_X_MM - calcL_right(sx, sy))
      };
      steppers.moveTo(pos);
      steppers.runSpeedToPosition();
    }
  }

  long pos_f[2] = {
    -mmToSteps(INIT_LENGTH_W_MM - calcL_left(x_mm, y_mm)),
     mmToSteps(INIT_LENGTH_X_MM - calcL_right(x_mm, y_mm))
  };
  steppers.moveTo(pos_f);
  steppers.runSpeedToPosition();

  pen_x_mm = x_mm;
  pen_y_mm = y_mm;
}

// ════════════════════════════════════════════════════════════
//  RECHERCHE D'UNE LETTRE DANS LA TABLE FONT
// ════════════════════════════════════════════════════════════

// Retourne l'index dans fontLetters[], ou -1 si non trouvée
int findLetterIndex(char c) {
  c = tolower(c);
  for (int i = 0; i < FONT_LETTER_COUNT; i++) {
    FontLetter fl;
    memcpy_P(&fl, &fontLetters[i], sizeof(FontLetter));
    if (fl.letter == c) return i;
  }
  return -1;
}

// ════════════════════════════════════════════════════════════
//  TRACÉ D'UNE LETTRE
//  originX, originY : coin supérieur-gauche de la lettre (mm)
//  scale            : px → mm
//  retourne le advance (largeur + spacing) en mm
// ════════════════════════════════════════════════════════════
float drawLetter(char c, float originX, float originY, float scale) {
  int li = findLetterIndex(c);
  if (li < 0) {
    // Lettre inconnue : espace
    Serial.printf("Lettre '%c' non trouvée, saut\n", c);
    FontLetter fl_space;
    // Retourner un avancement par défaut = FONT_REF_HEIGHT_PX * 0.5 * scale
    return (float)FONT_REF_HEIGHT_PX * 0.5f * scale + FONT_LETTER_SPACING_PX * scale;
  }

  FontLetter fl;
  memcpy_P(&fl, &fontLetters[li], sizeof(FontLetter));

  Serial.printf("Tracé lettre '%c' (%d formes) à X=%.1f Y=%.1f\n",
                c, fl.shapeCount, originX, originY);

  for (int s = 0; s < fl.shapeCount; s++) {
    int16_t shapeArr[2];
    memcpy_P(shapeArr, &fontShapes[fl.shapeStart + s], sizeof(shapeArr));
    int ptStart = shapeArr[0];
    int ptCount = shapeArr[1];

    if (ptCount < 2) continue;

    // Lever le stylo, aller au premier point
    leverStylo();
    {
      int16_t pt[2];
      memcpy_P(pt, &fontPoints[ptStart], sizeof(pt));
      float fx = originX + pt[0] * scale;
      float fy = originY + pt[1] * scale;
      penMoveTo(fx, fy);
    }
    baisserStylo();

    // Tracer les points de la forme
    for (int p = 0; p < ptCount; p++) {
      int16_t pt[2];
      memcpy_P(pt, &fontPoints[ptStart + p], sizeof(pt));
      float fx = originX + pt[0] * scale;
      float fy = originY + pt[1] * scale;
      penMoveTo(fx, fy);

      if (millis() - lastFeedback >= FEEDBACK_PERIOD_MS) {
        lastFeedback = millis();
        Serial.printf("  Forme %d/%d | Point %d/%d | X=%.1f Y=%.1f\n",
                      s+1, fl.shapeCount, p+1, ptCount, fx, fy);
      }
    }
  }

  leverStylo();
  return fl.advance * scale;
}

// ════════════════════════════════════════════════════════════
//  TRACÉ D'UN MOT COMPLET
// ════════════════════════════════════════════════════════════
void drawWord(const char *word) {
  // Echelle : TEXT_HEIGHT_MM correspond à FONT_REF_HEIGHT_PX pixels
  float scale = TEXT_HEIGHT_MM / (float)FONT_REF_HEIGHT_PX;

  Serial.printf("\n=== Dessin du mot : \"%s\" ===\n", word);
  Serial.printf("Echelle : %.4f mm/px | Hauteur : %.1f mm\n", scale, TEXT_HEIGHT_MM);

  float cursorX = TEXT_ORIGIN_X_MM;
  float cursorY = TEXT_ORIGIN_Y_MM;

  for (int i = 0; word[i] != '\0'; i++) {
    char c = word[i];
    if (c == ' ') {
      // Espace = demi-hauteur de police
      cursorX += TEXT_HEIGHT_MM * 0.45f;
      Serial.printf("Espace : curseur → X=%.1f\n", cursorX);
      continue;
    }
    float advance = drawLetter(c, cursorX, cursorY, scale);
    cursorX += advance;

    // Retour à la ligne si on déborde du tableau
    if (cursorX > TABLEAU_W_MM - 100.0f) {
      cursorX = TEXT_ORIGIN_X_MM;
      cursorY += TEXT_HEIGHT_MM * 1.3f;
      Serial.printf("Retour à la ligne : Y=%.1f\n", cursorY);
    }
  }

  Serial.println("=== Mot terminé ===\n");
}

// ════════════════════════════════════════════════════════════
//  INITIALISATIONS
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

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  analogWriteFreq(1000);
  Serial.begin(9600);
  while (!Serial) ;

  penServo.attach(SERVO_PIN, 500, 2500);
  leverStylo();

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

  pen_x_mm = TEXT_ORIGIN_X_MM;
  pen_y_mm = TEXT_ORIGIN_Y_MM;

  Serial.println("=== Traceur de mots prêt ===");
  Serial.println("Entrez un mot en minuscules puis appuyez sur Entrée.");
  Serial.println("(Appuyez d'abord sur Entrée pour calibrer la position initiale)");
  Serial.println();
}

// ════════════════════════════════════════════════════════════
//  LOOP — lecture de mot depuis la console série
// ════════════════════════════════════════════════════════════
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) {
      Serial.println("Mot vide ignoré. Entrez un mot :");
      return;
    }

    // Déplacer le stylo en position de départ (en l'air)
    leverStylo();
    penMoveTo(TEXT_ORIGIN_X_MM, TEXT_ORIGIN_Y_MM);

    // Tracer le mot
    drawWord(input.c_str());

    Serial.println("Prêt pour un nouveau mot :");
  }
}
