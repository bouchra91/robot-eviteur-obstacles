#include <Servo.h>

// ---------- Moteur Gauche ----------
#define AIN1 7
#define AIN2 8
#define PWMA 5

// ---------- Moteur Droit ----------
#define BIN1 4
#define BIN2 2
#define PWMB 6

// ---------- Capteur ultrason HC-SR04 ----------
#define TRIG 9
#define ECHO 10

// ---------- Servomoteur ----------
const int SERVO_PIN = 3;
Servo scanServo;

// ---------- Reglages ----------
int vitesse       = 120;   // vitesse normale
int vitesseLente  = 85;    // vitesse quand un obstacle approche
int vitesseVirage = 120;

const int ANGLE_DROITE = 20;
const int ANGLE_AVANT  = 90;
const int ANGLE_GAUCHE = 160;

const float DISTANCE_ARRET    = 20.0;  // en dessous : on s'arrete et on evite
const float DISTANCE_RALENTIR = 40.0;  // en dessous : on ralentit
const float DISTANCE_LIBRE    = 25.0;  // un cote est libre au-dela de cette distance

// A calibrer selon le robot
unsigned long TEMPS_RECUL     = 300;
unsigned long TEMPS_TOUR_90   = 450;
unsigned long TEMPS_DEMI_TOUR = 900;


void setup() {
  Serial.begin(9600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  scanServo.attach(SERVO_PIN, 500, 2400);
  scanServo.write(ANGLE_AVANT);

  stopRobot();
  delay(1000);
}

// ---------- Moteurs ----------
void avancer(int v) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  analogWrite(PWMA, v);
  analogWrite(PWMB, v);
}

void reculer() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, vitesse);
  analogWrite(PWMB, vitesse);
}

void tournerGauche() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  analogWrite(PWMA, vitesseVirage);
  analogWrite(PWMB, vitesseVirage);
}

void tournerDroite() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, vitesseVirage);
  analogWrite(PWMB, vitesseVirage);
}

void stopRobot() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// ---------- Mouvements temporises ----------
void reculerUnPeu() {
  reculer();
  delay(TEMPS_RECUL);
  stopRobot();
  delay(100);
}

void tournerDroite90() {
  tournerDroite();
  delay(TEMPS_TOUR_90);
  stopRobot();
  delay(150);
}

void tournerGauche90() {
  tournerGauche();
  delay(TEMPS_TOUR_90);
  stopRobot();
  delay(150);
}

void demiTour() {
  tournerDroite();
  delay(TEMPS_DEMI_TOUR);
  stopRobot();
  delay(200);
}

// ---------- Capteur ----------
float calcule_distance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  unsigned long duree = pulseIn(ECHO, HIGH, 30000); // timeout 30 ms

  // Aucun echo : rien de proche, la voie est libre
  if (duree == 0) return 999;

  return duree * 0.0343 / 2.0;
}

// Mediane de 3 mesures : elimine les valeurs aberrantes
float mesureFiltree() {
  float a = calcule_distance();
  delay(10);
  float b = calcule_distance();
  delay(10);
  float c = calcule_distance();

  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// Mesure dans une direction donnee
float scanner(int angle) {
  scanServo.write(angle);
  delay(300);              // laisser le servo arriver en position
  return mesureFiltree();
}

// ---------- Evitement ----------
void eviterObstacle() {
  stopRobot();
  delay(100);
  reculerUnPeu();

  float droite = scanner(ANGLE_DROITE);
  float gauche = scanner(ANGLE_GAUCHE);

  scanServo.write(ANGLE_AVANT);
  delay(300);

  Serial.print("Obstacle ! Droite = ");
  Serial.print(droite);
  Serial.print(" cm | Gauche = ");
  Serial.print(gauche);
  Serial.println(" cm");

  if (droite < DISTANCE_LIBRE && gauche < DISTANCE_LIBRE) {
    Serial.println("-> Demi-tour");
    demiTour();
  }
  else if (gauche > droite) {
    Serial.println("-> Gauche");
    tournerGauche90();
  }
  else {
    Serial.println("-> Droite");
    tournerDroite90();
  }
}

// ---------- Boucle principale ----------
void loop() {
  float devant = mesureFiltree();   // le servo regarde devant

  Serial.print("Devant = ");
  Serial.print(devant);
  Serial.println(" cm");

  if (devant > DISTANCE_RALENTIR) {
    avancer(vitesse);
  }
  else if (devant > DISTANCE_ARRET) {
    avancer(vitesseLente);
  }
  else {
    eviterObstacle();
  }

  delay(20);
}