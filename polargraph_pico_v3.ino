

#include <TMCStepper.h>
#include <AccelStepper.h>

using namespace TMC2130_n;

// ════════════════════════════════════════════════════════════
//  ▶  PARAMÈTRES UTILISATEUR — à ajuster selon ton montage
// ════════════════════════════════════════════════════════════

// Diamètre de la poulie / bobine d'enroulement (mm)
// Mesure le diamètre extérieur de ta bobine avec ficelle enroulée
#define PULLEY_DIAM_MM     30.0f

// Dimensions physiques du tableau (mm)
#define TABLEAU_W_MM     1900.0f   // largeur  (190 cm)
#define TABLEAU_H_MM      800.0f   // hauteur  (80 cm)

// ── Géométrie du gondola ─────────────────────────────────────
// Distance en mm entre le point d'attache ficelle GAUCHE
// et le point d'attache ficelle DROITE sur le support stylo.
// Mesure sur ta pièce rouge la distance entre les deux nœuds.
#define GONDOLA_OFFSET_MM   40.0f

// Longueur du stylo qui dépasse SOUS le gondola (mm).
// = distance verticale entre les points d'attache ficelles
//   et la pointe du stylo.
#define PEN_TIP_OFFSET_MM   65.0f

// Paramètres des moteurs 11HS18-0674S
#define MOTOR_STEPS_PER_REV  200   // pas/tour (moteur pas à pas 1.8°)
#define MICROSTEPS            16   // 1/16 microstep

// mm parcourus par micro-pas
const float MM_PER_STEP = (PI * PULLEY_DIAM_MM) / (float)(MOTOR_STEPS_PER_REV * MICROSTEPS);

// Position de départ du stylo (pointe, mm depuis coin haut-gauche)
// Place le stylo manuellement ici avant de démarrer
#define START_X_MM   (TABLEAU_W_MM / 2.0f)
#define START_Y_MM   (TABLEAU_H_MM / 2.0f)

// Vitesse et accélération par défaut pour le tracé (steps/s, steps/s²)
#define DRAW_SPEED   3000.0f
#define DRAW_ACCEL  15000.0f

// Spirale paramétrique
// x(t) = cx + r(t)*cos(t),  y(t) = cy + r(t)*sin(t)
// r(t) = R_START + (R_END - R_START) * t / (2π * TOURS)
#define SPIRAL_CX_MM   (TABLEAU_W_MM / 2.0f)   // centre X (mm)
#define SPIRAL_CY_MM   (TABLEAU_H_MM / 2.0f)   // centre Y (mm)
#define SPIRAL_R_START  10.0f    // rayon initial (mm)
#define SPIRAL_R_END   300.0f    // rayon final   (mm)
#define SPIRAL_TOURS    12.0f    // nombre de tours
#define SPIRAL_PTS     600       // nombre de points (résolution)

// ════════════════════════════════════════════════════════════
//  cablage (ne pas modifier !!)

#define SW_MOSI 19
#define SW_MISO 16
#define SW_SCK  18

#define FEEDBACK_PERIOD_MS 100
#define R_SENSE            0.11f

// ── Axe W (moteur gauche) ───────────────────────────────────
#define W_CS_PIN      27
#define W_EN_PIN       3
#define W_DIR_PIN      4
#define W_STEP_PIN     5
#define W_DIAG0_PIN    2
#define W_MICROSTEPS  16
#define W_CURRENT_MA 500
#define W_STALL_VALUE  3

// ── Axe X (moteur droit) ────────────────────────────────────
#define X_CS_PIN      15
#define X_EN_PIN      11
#define X_DIR_PIN     12
#define X_STEP_PIN    13
#define X_DIAG0_PIN   10
#define X_MICROSTEPS  16
#define X_CURRENT_MA 500
#define X_STALL_VALUE  3

// ── Axe Z (conservé, non utilisé pour le traceur) ───────────
#define Z_CS_PIN      17
#define Z_EN_PIN      22
#define Z_DIR_PIN     21
#define Z_STEP_PIN    20
#define Z_DIAG0_PIN   26
#define Z_MICROSTEPS  16
#define Z_CURRENT_MA 500
#define Z_STALL_VALUE 13

#define UV_EN_PIN 28

#define W_MAX_POS_STEPS  17000
#define W_MAX_VEL_STEPS  10000
#define W_MAX_ACC_STEPS 200000

#define X_MAX_POS_STEPS  15000
#define X_MAX_VEL_STEPS  10000
#define X_MAX_ACC_STEPS 200000

#define Z_MIN_POS_STEPS      0
#define Z_MAX_POS_STEPS  72000
#define Z_MAX_VEL_STEPS   5000
#define Z_MAX_ACC_STEPS 100000

// ════════════════════════════════════════════════════════════
//  STRUCTS (conservées telles quelles)
// ════════════════════════════════════════════════════════════
typedef struct {
    char     name;
    uint8_t  cs;
    uint8_t  en;
    uint8_t  dir;
    uint8_t  step;
    int8_t   diag0;
    uint32_t microsteps;
    uint16_t current_mA;
    int16_t  stall_value;
    int32_t  min_pos;
    int32_t  max_pos;
    uint32_t max_vel;
    uint32_t max_acc;
} AxisConfig;

typedef struct {
    TMC2130Stepper *driver;
    AccelStepper   *stepper;
} AxisData;

// ════════════════════════════════════════════════════════════
//  INSTANCES GLOBALES
TMC2130Stepper driverX(X_CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
TMC2130Stepper driverY(W_CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // W = "Y slot" dans le tableau
TMC2130Stepper driverZ(Z_CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

AccelStepper stepperW(AccelStepper::DRIVER, W_STEP_PIN, W_DIR_PIN);  // ficelle gauche
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);  // ficelle droite
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

AxisConfig axisConfigs[] = {
    { 'W', W_CS_PIN, W_EN_PIN, W_DIR_PIN, W_STEP_PIN, W_DIAG0_PIN, W_MICROSTEPS, W_CURRENT_MA, W_STALL_VALUE, 0, W_MAX_POS_STEPS, W_MAX_VEL_STEPS, W_MAX_ACC_STEPS },
    { 'X', X_CS_PIN, X_EN_PIN, X_DIR_PIN, X_STEP_PIN, X_DIAG0_PIN, X_MICROSTEPS, X_CURRENT_MA, X_STALL_VALUE, 0, X_MAX_POS_STEPS, X_MAX_VEL_STEPS, X_MAX_ACC_STEPS },
    { 'Z', Z_CS_PIN, Z_EN_PIN, Z_DIR_PIN, Z_STEP_PIN, Z_DIAG0_PIN, Z_MICROSTEPS, Z_CURRENT_MA, Z_STALL_VALUE, Z_MIN_POS_STEPS, Z_MAX_POS_STEPS, Z_MAX_VEL_STEPS, Z_MAX_ACC_STEPS }
};
AxisData axes[3];

// Position courante stylo (mm)
float pen_x_mm = START_X_MM;
float pen_y_mm = START_Y_MM;

// Steps accumulés pour chaque ficelle (position absolue en steps)
long steps_W = 0;   // ficelle gauche
long steps_X = 0;   // ficelle droite

unsigned long lastFeedback = 0;
bool traceDone = false;

// ════════════════════════════════════════════════════════════
//  GÉOMÉTRIE
// ════════════════════════════════════════════════════════════

// px, py = coordonnées de la POINTE DU STYLO
// Les points d'attache ficelles sont en :
//   gauche : (px - GONDOLA_OFFSET_MM/2 ,  py - PEN_TIP_OFFSET_MM)
//   droite : (px + GONDOLA_OFFSET_MM/2 ,  py - PEN_TIP_OFFSET_MM)

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

long mmToSteps(float mm) { return (long)(mm / MM_PER_STEP); }

// Déplace le stylo vers (x_mm, y_mm) stylo en bas
void penMoveTo(float x_mm, float y_mm) {
    // Longueurs cibles
    float Lw_new = calcL_left (x_mm, y_mm);
    float Lx_new = calcL_right(x_mm, y_mm);

    // Longueurs actuelles
    float Lw_cur = calcL_left (pen_x_mm, pen_y_mm);
    float Lx_cur = calcL_right(pen_x_mm, pen_y_mm);

    // Delta en steps
    long dW = mmToSteps(Lw_new - Lw_cur);  // + = dérouler ficelle gauche
    long dX = mmToSteps(Lx_new - Lx_cur);  // + = dérouler ficelle droite

    steps_W += dW;
    steps_X += dX;

    // M_W : dérouler = avancer (direction normale)
    // M_X : dérouler = reculer (bobine en sens miroir) → inverser
    stepperW.moveTo( steps_W);
    stepperX.moveTo(-steps_X);

    // Synchronisation : attendre que les deux moteurs arrivent
    while (stepperW.distanceToGo() != 0 || stepperX.distanceToGo() != 0) {
        stepperW.run();
        stepperX.run();
    }

    pen_x_mm = x_mm;
    pen_y_mm = y_mm;
}

// ════════════════════════════════════════════════════════════
// (conservé tel quel)

void axis_init(int idx) {
    AxisConfig *cfg = &axisConfigs[idx];
    pinMode(cfg->cs,  OUTPUT); digitalWrite(cfg->cs,  HIGH);
    pinMode(cfg->en,  OUTPUT); digitalWrite(cfg->en,  LOW);
    if (cfg->diag0 >= 0) pinMode(cfg->diag0, INPUT);

    TMC2130Stepper *drv = (idx == 0 ? &driverY : idx == 1 ? &driverX : &driverZ);
    axes[idx].driver = drv;
    drv->begin();
    drv->rms_current(cfg->current_mA);
    drv->microsteps(cfg->microsteps);
    drv->en_pwm_mode(true);
    drv->pwm_autoscale(true);
    if (cfg->diag0 >= 0) {
        drv->sgt(cfg->stall_value);
        drv->diag0_stall(true);
        drv->diag0_int_pushpull(true);
    }

    AccelStepper *stp = (idx == 0 ? &stepperW : idx == 1 ? &stepperX : &stepperZ);
    axes[idx].stepper = stp;
    stp->setEnablePin(cfg->en);
    stp->setPinsInverted(true, false, true);
    stp->enableOutputs();
    stp->setMaxSpeed(cfg->max_vel);
    stp->setAcceleration(cfg->max_acc);
}


void axis_home(int idx, bool direction = 0, int home_position = 0) {
    AxisConfig *cfg = &axisConfigs[idx];
    if (cfg->diag0 < 0) return;
    TMC2130Stepper *drv = axes[idx].driver;
    AccelStepper   *stp = axes[idx].stepper;

    drv->en_pwm_mode(false);
    drv->pwm_autoscale(false);
    drv->TCOOLTHRS(0xFFFFF);
    drv->THIGH(0);
    delay(100);
    digitalWrite(cfg->dir, direction);
    tone(cfg->step, 2000);
    delay(100);
    while (!gpio_get(cfg->diag0));
    noTone(cfg->step);
    drv->TCOOLTHRS(0);
    drv->en_pwm_mode(true);
    drv->pwm_autoscale(true);
    stp->setCurrentPosition(home_position);
}

// ════════════════════════════════════════════════════════════
//  SET MOTION (conservé tel quel, pour commandes Serial)

void axis_setMotion(int idx, int32_t pos, uint32_t vel, uint32_t acc) {
    AxisConfig *cfg = &axisConfigs[idx];
    AccelStepper *stp = axes[idx].stepper;
    if (pos < cfg->min_pos) pos = cfg->min_pos;
    if (pos > cfg->max_pos) pos = cfg->max_pos;
    if (vel > cfg->max_vel) vel = cfg->max_vel;
    if (acc > cfg->max_acc) acc = cfg->max_acc;
    stp->setMaxSpeed(vel);
    stp->setAcceleration(acc);
    stp->moveTo(pos);
}

// ════════════════════════════════════════════════════════════
//  SPIRALE PARAMÉTRIQUE
//  Spirale d'Archimède : r(t) = R_START + (R_END-R_START)*t/(2π*TOURS)

void traceSpiral() {
    Serial.println("Debut trace spirale...");

    stepperW.setMaxSpeed(DRAW_SPEED);
    stepperW.setAcceleration(DRAW_ACCEL);
    stepperX.setMaxSpeed(DRAW_SPEED);
    stepperX.setAcceleration(DRAW_ACCEL);

    float t_max = 2.0f * PI * SPIRAL_TOURS;

    for (int i = 0; i <= SPIRAL_PTS; i++) {
        float t = t_max * (float)i / (float)SPIRAL_PTS;
        float r = SPIRAL_R_START + (SPIRAL_R_END - SPIRAL_R_START) * t / t_max;
        float x = SPIRAL_CX_MM + r * cosf(t);
        float y = SPIRAL_CY_MM + r * sinf(t);

        // Clamp pour rester dans le tableau
        x = constrain(x, 10.0f, TABLEAU_W_MM - 10.0f);
        y = constrain(y, 10.0f, TABLEAU_H_MM - 10.0f);

        penMoveTo(x, y);

        // Feedback Serial pendant le tracé
        if (millis() - lastFeedback >= FEEDBACK_PERIOD_MS) {
            lastFeedback = millis();
            Serial.printf(
                "X %ld %f Y %ld %f Z %ld %f\n",
                stepperX.currentPosition(), stepperX.speed(),
                stepperW.currentPosition(), stepperW.speed(),
                stepperZ.currentPosition(), stepperZ.speed()
            );
        }
    }

    Serial.println("Spirale terminee !");
}

// ════════════════════════════════════════════════════════════
//  SETUP

void setup() {
    analogWriteFreq(1000);
    Serial.begin(9600);
    while (!Serial);

    pinMode(UV_EN_PIN, OUTPUT);
    digitalWrite(UV_EN_PIN, 1);

    SPI.begin();

    axis_init(0);
    axis_home(0, 1);

    axis_init(1);
    axis_home(1, 1);

    axis_init(2);
    axis_home(2, 0, 72000);

    pen_x_mm = START_X_MM;
    pen_y_mm = START_Y_MM;
    steps_W  = 0;
    steps_X  = 0;
    stepperW.setCurrentPosition(0);
    stepperX.setCurrentPosition(0);

    Serial.printf("MM_PER_STEP = %.6f mm\n", MM_PER_STEP);
    Serial.printf("Position de depart : X=%.1f mm, Y=%.1f mm\n", pen_x_mm, pen_y_mm);
    Serial.println("Appuyez sur Entree pour lancer la spirale...");
    while (!Serial.available()) {}
    Serial.read();

    traceSpiral();
    traceDone = true;
}

// ════════════════════════════════════════════════════════════

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        char axis; uint32_t arg1, arg2, arg3;
        if (sscanf(cmd.c_str(), "%c %lu %lu %lu", &axis, &arg1, &arg2, &arg3) == 4) {
            for (int i = 0; i < 3; ++i) {
                if (axisConfigs[i].name == axis) {
                    axis_setMotion(i, arg1, arg2, arg3);
                    break;
                }
            }
        } else if (sscanf(cmd.c_str(), "%c %lu", &axis, &arg1) == 2) {
            if (axis == 'U') {
                analogWrite(UV_EN_PIN, arg1);
            }
        }
    }

    if (!traceDone) {
        axes[0].stepper->run();
        axes[1].stepper->run();
        axes[2].stepper->run();
    }

    // Feedback périodique
    if (millis() - lastFeedback >= FEEDBACK_PERIOD_MS) {
        lastFeedback = millis();
        Serial.printf(
            "X %ld %f Y %ld %f Z %ld %f\n",
            stepperX.currentPosition(), stepperX.speed(),
            stepperW.currentPosition(), stepperW.speed(),
            stepperZ.currentPosition(), stepperZ.speed()
        );
    }
}
