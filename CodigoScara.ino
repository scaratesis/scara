#include <Servo.h>
#include <Fuzzy.h>
#include <avr/io.h>
#include <avr/interrupt.h>

Fuzzy *fuzzy = new Fuzzy();

//Numeros magicos
int PWMB = 35;
int PWMA = 200;

//Valores de altura de offset del ultrasonido
#define altMAX 0
#define altMIN 20
#define altOFFSET 4


int contaux = 0;
//Declaracion de servomotor
Servo svMotor;
// Definir estados
bool E0 = false;
bool E1 = false;
bool E2 = false;
bool E3 = false;
bool E4 = false;
bool E5 = false;
bool E6 = false;
bool E7 = false;

//Banderas para cambio de estado
bool flag0 = false;

bool flag1 = false;   //Flag del estado
bool flag11 = false;  //Flag del motor brazo
bool flag12 = false;  //Flag del motor antebrazo

bool flag2 = false;
bool flag3 = false;
bool flag4 = false;
bool flag5 = false;

bool flagError = false;

//Flags para encoders
bool flagEC1 = false;
bool flagEC2 = false;

//Flag para encerar
bool flagCero = false;

//Flag BobEsponja
bool flagListo = false;

//Pines de Interrupcion (encoders)
int ecdA = 2;   //
int ecdB = 18;  //


//Pines de actuadores
int dir = 4;     //
int paso = 5;    //
int pulsos = 0;  //Contador para el tren de pulsos del motor a pasos
int freno = 12;

//Eslabon 2 (Antebrazo: IBT2)
int pHApwm1 = 6;  //
int pHApwm2 = 7;  //
int pHA5V = 8;    //pin digital que sirve de 5V para alimentación del IBT del antebrazo

//Eslabon 1 (Brazo: IBT2)
int pHBpwm1 = 9;
int pHBpwm2 = 10;

//Indicadores (LEDS)
int LEDv = 14;
int LEDa = 15;
int LEDr = 16;

//Rele Electroiman
int eiman = 22;
//Sensores (Fines de carrera)
//Antebrazo
int FCAI = 23;
int FCAD = 25;

//Brazo
int FCBD = 37;
int FCBI = 39;

//Elevador
int FCEI = 51;
int FCB = 49;
int FCA = 27;

//Pines sensores
int echo = 29;
int trig = 31;

//Control servo
int srv = 33;

//Variables auxiliares
double contA = 0;
double contB = 0;
double angA = 0;  //Angulo en el que estoy
double angB = 0;  //Angulo en el que estoy
long t = 0;      //timepo que demora en llegar el eco
float d = 0;     //distancia en centimetros

//Variables String de lo que recibo
int angANR;     //Angulo al que quiero ir
int angBNR;     //Angulo al que quiero ir
int angSNR;     //Angulo al que quiero ir
int stepNR;    //Hago movimiento de subida y bajada?
int eimanNR;   //¿Se va a encender el iman o no?
int encerar;   //¿Se va a encerar o no?
//Flags
bool flagA = LOW;
bool flagB = LOW;

//Milis
long t1 = 0;
long t2 = 0;

void cero();
void separarMensaje(String msg);
void lecturas();


//Desborde de iteraciones de la interrupcion interna
int cuenta = 0;

//Diferencia de angulos
int diffA = 10;
int diffB = 10;

//Indicadores de actuadores
int motorB = 0;
int motorA = 0;
int motorServo = 0;
int motorPasos = 0;

void setup() {
  Serial.begin(115200);

  //Declaración de la entrada de control fuzzy
  FuzzyInput *diferencia = new FuzzyInput(1);
  //Declaracion de curvas de membresía
  FuzzySet *dPP = new FuzzySet(0, 0, 0, 35);
  diferencia->addFuzzySet(dPP);
  FuzzySet *dP = new FuzzySet(20, 40, 40, 60);
  diferencia->addFuzzySet(dP);
  FuzzySet *dM = new FuzzySet(45, 90, 90, 135);
  diferencia->addFuzzySet(dM);
  FuzzySet *dG = new FuzzySet(120, 140, 140, 160);
  diferencia->addFuzzySet(dG);
  FuzzySet *dGG = new FuzzySet(145, 180, 180, 180);
  diferencia->addFuzzySet(dGG);

  fuzzy->addFuzzyInput(diferencia);

  // Salida del control fuzzy (PWMBrazo)
  FuzzyOutput *pwmB = new FuzzyOutput(1);
  FuzzySet *pwmPP = new FuzzySet(32, 32, 33.5, 34);
  pwmB->addFuzzySet(pwmPP);
  FuzzySet *pwmP = new FuzzySet(32.5, 34.5, 34.5, 35);
  pwmB->addFuzzySet(pwmP);
  FuzzySet *pwmM = new FuzzySet(34.5, 37, 37, 38);
  pwmB->addFuzzySet(pwmM);
  FuzzySet *pwmG = new FuzzySet(36.5, 39.5, 39.5, 40.5);
  pwmB->addFuzzySet(pwmG);
  FuzzySet *pwmGG = new FuzzySet(39, 43, 43, 45);
  pwmB->addFuzzySet(pwmGG);
  fuzzy->addFuzzyOutput(pwmB);

  //Declaracion de reglas
  FuzzyRuleAntecedent *siDifPP = new FuzzyRuleAntecedent();
  siDifPP->joinSingle(dPP);
  FuzzyRuleConsequent *toncspwmPP = new FuzzyRuleConsequent();
  toncspwmPP->addOutput(pwmPP);
  FuzzyRule *regla1 = new FuzzyRule(1, siDifPP, toncspwmPP);
  fuzzy->addFuzzyRule(regla1);
  /******************************************************************/
  FuzzyRuleAntecedent *siDifP = new FuzzyRuleAntecedent();
  siDifP->joinSingle(dP);
  FuzzyRuleConsequent *toncspwmP = new FuzzyRuleConsequent();
  toncspwmP->addOutput(pwmP);
  FuzzyRule *regla2 = new FuzzyRule(2, siDifP, toncspwmP);
  fuzzy->addFuzzyRule(regla2);
  /******************************************************************/
  FuzzyRuleAntecedent *siDifM = new FuzzyRuleAntecedent();
  siDifM->joinSingle(dM);
  FuzzyRuleConsequent *toncspwmM = new FuzzyRuleConsequent();
  toncspwmM->addOutput(pwmM);
  FuzzyRule *regla3 = new FuzzyRule(3, siDifM, toncspwmM);
  fuzzy->addFuzzyRule(regla3);
  /******************************************************************/
  FuzzyRuleAntecedent *siDifG = new FuzzyRuleAntecedent();
  siDifG->joinSingle(dG);
  FuzzyRuleConsequent *toncspwmG = new FuzzyRuleConsequent();
  toncspwmG->addOutput(pwmG);
  FuzzyRule *regla4 = new FuzzyRule(4, siDifG, toncspwmG);
  fuzzy->addFuzzyRule(regla4);
  /******************************************************************/
  FuzzyRuleAntecedent *siDifGG = new FuzzyRuleAntecedent();
  siDifGG->joinSingle(dGG);
  FuzzyRuleConsequent *toncspwmGG = new FuzzyRuleConsequent();
  toncspwmGG->addOutput(pwmGG);
  FuzzyRule *regla5 = new FuzzyRule(5, siDifGG, toncspwmGG);
  fuzzy->addFuzzyRule(regla5);
  /******************************************************************/

  pinMode(paso, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(pHApwm1, OUTPUT);
  pinMode(pHApwm2, OUTPUT);
  pinMode(pHA5V, OUTPUT);
  pinMode(pHBpwm1, OUTPUT);
  pinMode(pHBpwm2, OUTPUT);
  pinMode(srv, OUTPUT);
  pinMode(eiman, OUTPUT);
  pinMode(LEDv, OUTPUT);
  pinMode(LEDa, OUTPUT);
  pinMode(LEDr, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(freno, OUTPUT);

  pinMode(echo, INPUT);
  pinMode(FCAD, INPUT);
  pinMode(FCAI, INPUT);
  pinMode(FCBD, INPUT);
  pinMode(FCBI, INPUT);
  pinMode(FCA, INPUT);
  pinMode(FCEI, INPUT);
  pinMode(FCB, INPUT);

  digitalWrite(freno, HIGH);
  digitalWrite(paso, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(pHApwm1, LOW);
  digitalWrite(pHApwm2, LOW);
  digitalWrite(pHA5V, HIGH);  //Pin que sirve de fuente para IBT2
  digitalWrite(pHBpwm1, LOW);
  digitalWrite(pHBpwm2, LOW);
  digitalWrite(srv, LOW);
  digitalWrite(eiman, LOW);
  digitalWrite(LEDv, LOW);
  digitalWrite(LEDa, LOW);
  digitalWrite(LEDr, LOW);
  digitalWrite(trig, LOW);

  //Interrupciones externas (encoders)
  attachInterrupt(digitalPinToInterrupt(ecdB), inte, RISING);
  attachInterrupt(digitalPinToInterrupt(ecdA), inte2, RISING);
  svMotor.attach(srv);
  cero();
  calculoDist();
  //Se inicia en el estado 0
  E0 = true;
}

void loop() {

  lecturas();

  //Existe error en la ejecución: Se regresa al estado 0 a esperar al siguiente mensaje
  if (flagError) {
    E0 = true;
    flag1 = false;
    flag11 = false;
    flag12 = false;
    flag2 = false;
    flag3 = false;
    flag4 = false;
    flag5 = false;
    flagCero = false;
  }

  //Cambios de estado
  if (E0 && flag0) {
    E0 = false;
    E1 = true;
    flag0 = false;
  }
  if (E1 && (flag1 || flagCero)) {
    E1 = false;
    flag11 = false;
    flag12 = false;
    flag1 = false;
    E2 = true;
  }
  if (E2 && (flag2 || flagCero)) {
    E2 = false;
    E3 = true;
    flag2 = false;
  }
  if (E3 && (flag3 || flagCero)) {
    E3 = false;
    E4 = true;
    flag3 = false;
  }
  if (E4 && (flag4 || flagCero)) {
    E4 = false;
    E5 = true;
    flag4 = false;
  }
  if (E5 && (flag5 || flagCero)) {
    E5 = false;
    E0 = true;
    flag5 = false;
    flagListo = true;
  }

  if (E0) {
    digitalWrite(LEDv, LOW);
    digitalWrite(LEDr, LOW);
    digitalWrite(LEDa, HIGH);
    flagListo = true;
    if (Serial.available()) {
      flagListo = false;
      digitalWrite(LEDa, LOW);
      digitalWrite(LEDr, LOW);
      digitalWrite(LEDv, HIGH);
      String mensaje = Serial.readStringUntil('\n');
      separarMensaje(mensaje);
      contA = 0;
      contB = 0;

      if (encerar == 1) {
        cero();
        flagCero = true;
      } else {
        flagCero = false;
      }
      flag0 = true;
    }
  }

  if (E1) {
    if (abs(contB - abs(angBNR)) >= 2 && !flag11) {
      //Control fuzzy de velocidad
      fuzzy->setInput(1, abs(contB - abs(angBNR)));
      fuzzy->fuzzify();
      float output = fuzzy->defuzzify(1);
      int PWMBaux = 0.87*output;
      if (angBNR > 0) {
        if (digitalRead(FCBI)) {
          analogWrite(pHBpwm2, PWMBaux);
          analogWrite(pHBpwm1, 0);
          motorB = 1;
          
        } else {
          analogWrite(pHBpwm1, 0);
          analogWrite(pHBpwm2, 0);
          motorB = 0;
          flag11 = true;
        }
      } else if (angBNR < 0) {
        if (digitalRead(FCBD)) {
          analogWrite(pHBpwm1, PWMBaux);
          analogWrite(pHBpwm2, 0);
          motorB = 1;
        } else {
          analogWrite(pHBpwm1, 0);
          analogWrite(pHBpwm2, 0);
          motorB = 0;
          flag11 = true;
        }
      } else {
        analogWrite(pHBpwm1, 0);
        analogWrite(pHBpwm2, 0);
        motorB = 0;
        flag11 = true;
      }
    } else {
      analogWrite(pHBpwm1, 0);
      analogWrite(pHBpwm2, 0);
      motorB = 0;
      flag11 = true;
    }

    if (abs(contA - abs(angANR)) >= 2.5 && !flag12) {
      int PWMAaux = PWMA;
      if ((abs(angANR) - contA) <= 5) {
        PWMAaux = PWMA - (15 - (abs(angANR) - contA)) ;
      }
      if (angANR > 0) {
        if (digitalRead(FCAI)) {
          analogWrite(pHApwm1, PWMA);
          analogWrite(pHApwm2, 0);
          motorA = 1;
        } else {
          analogWrite(pHApwm2, 0);
          analogWrite(pHApwm1, 0);
          flag12 = true;
          motorA = 0;
        }
      } else if (angANR < 0) {
        if (digitalRead(FCAD)) {
          analogWrite(pHApwm2, PWMA);
          analogWrite(pHApwm1, 0);
          motorA = 1;
        } else {
          analogWrite(pHApwm2, 0);
          analogWrite(pHApwm1, 0);
          motorA = 0;
          flag12 = true;
        }
      } else {
        analogWrite(pHApwm2, 0);
        analogWrite(pHApwm1, 0);
        motorA = 0;
        flag12 = true;
      }
    } else {
      analogWrite(pHApwm1, 0);
      analogWrite(pHApwm2, 0);
      motorA = 0;
      flag12 = true;
    }
    flag1 = flag11 * flag12;
  }

  if (E2) {
    if (angANR > 0) {
      angA = angA + contA;
    }
    if (angANR < 0) {
      angA = angA - contA;
    }
    if (angBNR > 0) {
      angB = angB + contB;
    }
    if (angBNR < 0) {
      angB = angB - contB;
    }
    contA = 0;
    contB = 0;
    if(angSNR!=0){
      motorServo = 1;
    }
    svMotor.write(angSNR);
    motorServo = 0;
    flag2 = true;
  }

  if (E3) {
    //Medir la distancia del Ultrasonido y guardar en "d" (restando el offset (~4cm))
    //Si la diferencia es positiva, se mueve hacia abajo

    if (stepNR == 1) {
      int i = 0;      // Contador de iteraciones del bucle while
      int tmp = 550;  //Tiempo entre pulsos
      digitalWrite(dir, LOW);
      digitalWrite(freno, LOW);
      motorPasos = 1;
      lecturas();
      while (digitalRead(FCB)) {
        
        i++;
        tmp = 500;
        if (i < 550) {
          tmp = 1000 - i;
        }
        digitalWrite(paso, HIGH);
        delayMicroseconds(tmp);
        digitalWrite(paso, LOW);
        delayMicroseconds(tmp);
      }
      motorPasos = 0;
      digitalWrite(paso, LOW);
      digitalWrite(freno, HIGH);
      flag3 = true;
    } else {
      motorPasos = 0;
      flag3 = true;
    }
    calculoDist();
  }

  if (E4) {
    lecturas();
    delay(200);
    if (eimanNR == 1) {
      digitalWrite(eiman, HIGH);
    }
    if (eimanNR == 0) {
      digitalWrite(eiman, LOW);
    }
    flag4 = true;
  }

  if (E5) {

    //Medir la distancia del Ultrasonido y guardar en "d" (restando el offset (~4cm))
    //Si la diferencia es positiva, se mueve hacia abajo
    if (stepNR == 1) {
      int i = 0;      // Contador de iteraciones del bucle while
      int tmp = 500;  //Tiempo entre pulsos
      digitalWrite(dir, HIGH);
      digitalWrite(freno, LOW);
      motorPasos = 1;
      lecturas();
      while (digitalRead(FCA)) {
        i++;
        tmp = 500;
        if (i < 500) {
          tmp = 1000 - i;
        }
        digitalWrite(paso, HIGH);
        delayMicroseconds(tmp);
        digitalWrite(paso, LOW);
        delayMicroseconds(tmp);
      }
      motorPasos = 0;
      flag5 = true;
      digitalWrite(paso, LOW);
      digitalWrite(freno, HIGH);
    } else {
      motorPasos = 0;
      flag5 = true;
    }
    calculoDist();
  }
}

void calculoDist() {
  digitalWrite(trig, HIGH);  //Enviamos un sonido ultrosonico por 10us
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  t = pulseIn(echo, HIGH);  //Guardamos el valor que se demoro en llegar al sensor
  d = t / 59.00;            //escalamos el tiempo a una distancia en cm
  d = abs(d); //Altura compensada
}

void separarMensaje(String str) {
  int t1 = (str.indexOf('X'));
  angBNR = (str.substring(0, t1)).toInt();

  int t2 = (str.indexOf('X', t1 + 1));
  angANR = (str.substring(t1 + 1, t2)).toInt();

  int t3 = (str.indexOf('X', t2 + 1));
  angSNR = (str.substring(t2 + 1, t3)).toInt();

  int t4 = (str.indexOf('X', t3 + 1));
  stepNR = (str.substring(t3 + 1, t4)).toInt();

  int t5 = (str.indexOf('X', t4 + 1));
  eimanNR = (str.substring(t4 + 1, t5)).toInt();

  int t6 = (str.indexOf('X', t5 + 1));
  encerar = (str.substring(t5 + 1, t6)).toInt();
}

void lecturas() {
  Serial.print(angB);  //Angulo antebrazo
  Serial.print('\t');
  Serial.print(angA);  //Angulo brazo
  Serial.print('\t');
  Serial.print(d);  //Distancia ultrasonido
  Serial.print('\t');
  Serial.print(svMotor.read());  //Angulo servo
  Serial.print('\t');
  Serial.print(digitalRead(eiman));  //Estado electroiman
  Serial.print('\t');
  Serial.print(digitalRead(FCAD));  //FC Antebrazo Derecha
  Serial.print('\t');
  Serial.print(digitalRead(FCAI));  //FC Antebrazo Izquierda
  Serial.print('\t');
  Serial.print(digitalRead(FCBD));  //FC Brazo Derecha
  Serial.print('\t');
  Serial.print(digitalRead(FCBI));  //FC Brazo Izquierda
  Serial.print('\t');
  Serial.print(digitalRead(FCA));  //FC Motor a pasos arriba
  Serial.print('\t');
  Serial.print(digitalRead(FCB));  //FC Motor a pasos abajo (Detecta piso)
  Serial.print('\t');
  Serial.print(digitalRead(FCEI));  //FC Electroiman (Detecta pieza)
  Serial.print('\t');
  Serial.print(motorB);
  Serial.print('\t');
  Serial.print(motorA);
  Serial.print('\t');
  Serial.print(motorServo);
  Serial.print('\t');
  Serial.print(motorPasos);
  Serial.print('\t');
  Serial.println(flagListo);
}
void cero() {
  //El electroimán sube
  digitalWrite(freno, LOW);
  digitalWrite(dir, HIGH);
  while (digitalRead(FCA)) {
    motorPasos = 1;
    pulsos++;
    int tmp = 550;
    if (pulsos < 550) {
      tmp = 1000 - pulsos;
    }
    digitalWrite(paso, HIGH);
    delayMicroseconds(tmp);
    digitalWrite(paso, LOW);
    delayMicroseconds(tmp);
  }
  motorPasos = 0;
  //El antebrazo recorre hacia la izquierda
  while (digitalRead(FCAD)) {
    //Mover IBT del antebrazo
    analogWrite(pHApwm2, PWMA);
    analogWrite(pHApwm1, 0);
    motorA = 1;
  }
  motorA = 0;
  analogWrite(pHApwm1, 0);
  analogWrite(pHApwm2, 0);
  //El brazo recorre hacia la derecha
  while (digitalRead(FCBD)) {
    //Mover IBT del brazo
    analogWrite(pHBpwm1, PWMB);
    analogWrite(pHBpwm2, 0);
    motorB = 1;
  }
  motorB = 0;
  analogWrite(pHBpwm2, 0);
  analogWrite(pHBpwm1, 0);
  //Se encera el ángulo del servomotor
  motorServo = 1;
  svMotor.write(0);
  delay(500);
  motorServo = 0;
  //Se encera todos los contadores
  delay(500);
  contA = 0;
  contB = 0;
  angA = 0;  //Angulo en el que estoy
  angB = 0;  //Angulo en el que estoy
  t = 0;      //timepo que demora en llegar el eco
  d = 0;     //distancia en centimetros
  digitalWrite(freno, HIGH);
}
void inte2() {
  if (!flag11 && E1) {
    contB = contB + 1.07;
    delayMicroseconds(1200);
    Serial.println(contB);
  }
}
void inte() {
  if (!flag12 && E1) {
    contA = contA + 1.825;
    delayMicroseconds(1100);
    Serial.println(contA);
  }
}
