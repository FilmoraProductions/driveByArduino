#include <PID_v1.h>

// Pinos dos potenciômetros de acelerador
const int acceleratorPotPin1 = A5;
const int acceleratorPotPin2 = A4;
// Valores de calibração do acelerador
// Pedal 1 //
const int pedal1Min = 104;
const int pedal1Max = 349;
// Pedal 2 //
const int pedal2Min = 208;
const int pedal2Max = 697;

// Pinos dos potenciômetros de borboleta
const int butterflyPotPin1 = A0;
const int butterflyPotPin2 = A1;
// Valores de calibração da borboleta
const int butterflyMin = 122;
const int butterflyMax = 925;

// Pinos da ponte H
const int pwmPin1 = 9;
const int pwmPin2 = 10;
const int dirPin = 11;



// Estratégia de Partida
const int minRpm = 450; // Rotação de partida do motor
const float crankButterflyPos = 3.0; // Posição alvo da borboleta na partida

// Desaceleração
const float slowdownSet = 6.0; // Posição alvo da borboleta em fase de desaceleração do motor
const int rpmSlowDown = 1500; // Rotação mínima para entrar no modo de desaceleração

// Marcha lenta
const float coldIdleSet = 8.6; // Ponto minimo de referência da borboleta com motor frio (porcentagem)
const float idleSet = 5.2; // Ponto minimo de referência da borboleta (porcentagem)
const int IdleFrio = 1300; // Valor alvo de marcha lenta com motor frio
const int IdleQuente = 920; // Valor alvo de marcha lenta com motor quente

// Antilag
const int tempoAntilag = 300; // Delay para manter a borboleta aberta após retirar o pé do acelerador (Antilag)
const float antilagValue = 5.0; // Porcentagem de abertura que será adicionada à borboleta no Antilag
const int antilagRpm = 2000; // Rotação mínima para ativar o antilag

// ==== //

const float precisionMultiplier = 8; 
const int maxAtuation = 160; // Força máxima do motor PWM (abrir)
const int maxReverseAtuation = -160; // Força máxima do motor PWM (fechar)
// const unsigned long butterflyProtection = 200;  // Tempo máximo permitido de erro da borboleta (milissegundos)
const unsigned long butterflyIdleTime = 10000;  // Tempo de desligamento da borboleta (milissegundos)


// Parâmetros da leitura de rotação
const int hallPin = 2; // Pino digital conectado ao sensor Hall
const unsigned int pulsesPerRotation = 2; // Pulsos por rotação
const unsigned long noRotationTimeout = 300; // Tempo limite para considerar que não há rotação em milissegundos

// Pino analógico onde o sensor de temperatura está conectado
const int TempSensorPin = A2;

// Pino analógico onde a saida para TPS está conectado
const int TpsOutPin = A3;

// === //



// =============================== Variáveis globais =============================== //

// Variáveis da borboleta
int error = 0; // Valor de diferença entre alvo e posição da borboleta
int output = 0; // Valor do PWM enviado ao motor elétrico
double acceleratorSet = 0; // Porcentagem da borboleta após as correções
double adjustedSetpoint = 0; // Correção da borboleta por marcha lenta
double rpmSet = 0; // Valor alvo de rotação de marcha lenta por temperatura (rpm)
int rotationCoefficient = 0; // Coeficiente de influencia na borboleta pela malha fechada de rotação
double rpmPid = 0; // Valor de saída do controle PID
double minPos = 0; // limiar mínimo de borboleta

// Variáveis de leitura de rotação
volatile double rpm = 0; // Rotações por minuto do motor
volatile unsigned long periodStartMicros = 0;
volatile unsigned long periodMicros = 0;
volatile unsigned int pulses = 0;
unsigned long lastRpmUpdateTime = 0; 

// Variáveis de proteção
unsigned long butterflyMillis = 0;  // Variável para armazenar o tempo do contador de proteção da borboleta
unsigned long butterflyIdleMillis = 0;  // Variável para armazenar o tempo do contador para desligamento da borboleta
boolean emergencyMode; // Variável do tipo Bool para armazenar o modo de emergencia

unsigned long millisAntilag = 0; // Armazena o ultimo tempo do contador do antilag
// === //



// Calibração do controle PID
const double Kp=0.350, // Ganho Proporcional
             Ki=0.100, // Ganho Integral
             Kd=0.020; // Ganho Derivativo

const double aggKp=0.340, // Ganho Proporcional marcha lenta
             aggKi=0.060, // Ganho Integral marcha lenta
             aggKd=0.008; // Ganho Derivativo marcha lenta

PID myPID(&rpm, &rpmPid, &rpmSet, Kp, Ki, Kd, DIRECT);

// Mapa de aceleração (entrada do acelerador -> abertura da borboleta)
const double acceleratorMap[10][2] = {
  {  0.0, 0.0},
  { 10.0, 5.0},
  { 20.0, 8.0},
  { 30.0, 10.0},
  { 40.0, 14.0},
  { 50.0, 20.0},
  { 60.0, 28.0},
  { 70.0, 40.0},
  { 80.0, 65.0},
  {100.0, 100.0}
}; // Valores ajustados para porcentagem

void setup() {

  // Inicializa a comunicação serial
  Serial.begin(9600);

  // Configuração dos pinos
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(TpsOutPin, OUTPUT);
  pinMode(hallPin, INPUT_PULLUP);


  // Habilita o PID
  myPID.SetMode(AUTOMATIC);

  // Inicializa a entrada de rotação
  attachInterrupt(digitalPinToInterrupt(2), hallSensorISR, FALLING); 

  // Habilita a ponte H
  digitalWrite(dirPin,HIGH); 

  // Inicia os Timers
  butterflyIdleMillis = millis();
  }


void loop() {
  
  

  // Leitura do valor dos potenciômetros de acelerador
  int acceleratorPotValue1 = analogRead(acceleratorPotPin1); // Faz a leitura do acelerador 1 em ADC
  int acceleratorPotValue2 = analogRead(acceleratorPotPin2); // Faz a leitura do acelerador 2 em ADC
  double pedal1percentage = mapFloat(acceleratorPotValue1,pedal1Min,pedal1Max,0.0,100.0); // Mapeia o valor do pedal 1 para porcentagem
  double pedal2percentage = mapFloat(acceleratorPotValue2,pedal2Min,pedal2Max,0.0,100.0); // Mapeia o valor do pedal 2 para porcentagem
  float pedalError = (pedal1percentage - pedal2percentage); // Faz o cálculo de diferença entre Pedal 1 e Pedal 2
  double acceleratorPercentage = (pedal1percentage + pedal2percentage) / 2 ; // Calcula a média dos pedais
  


  // Leitura do valor dos potenciômetros de borboleta 
  int butterflyPotValue1 = analogRead(butterflyPotPin1);
  int butterflyPotValue2 = analogRead(butterflyPotPin2);
  float butterflyError = ((butterflyPotValue1 + butterflyPotValue2)/2) - 511.5; // calcula a margem de erro dos potenciometros
  int input = ((butterflyPotValue1 - butterflyPotValue2) / 2) + 511.5; // junta a leitura dos dois potenciometros
  if(acceleratorPercentage > 0){
  digitalWrite(TpsOutPin,HIGH);
  }else{digitalWrite(TpsOutPin,LOW);}



// ========================== Cálculo de acelerador e fases da borboleta ========================== //

  // Cálculo do sensor de temperatura
  int TempSensorValue = analogRead(TempSensorPin); // Leitura do valor analógico

  if(TempSensorValue < 350){
     myPID.SetTunings(aggKp, aggKi, aggKd);
     rpmSet = IdleQuente;
     minPos = idleSet;
  }
  else {
     myPID.SetTunings(Kp, Ki, Kd);
     rpmSet = IdleFrio; 
     minPos = coldIdleSet;
  }
  
   // === //

  // Calcula PID de alvo de rotação
  myPID.Compute();
  rotationCoefficient = mapFloat(rpmPid,0,255,0.0,12.0);


 // Estratégia de partida fria do motor
  if(rpm < minRpm && acceleratorPercentage == 0) {
    acceleratorSet = mapFloat(rpm,0,minRpm,0.0,crankButterflyPos);
  }
  // Modo de desaceleração
  else if(rpm >= rpmSlowDown && acceleratorPercentage == 0){
    acceleratorSet = slowdownSet;
  }
  // Alvo de marcha lenta
  else { 
    acceleratorSet = (acceleratorPercentage+rotationCoefficient+minPos);
  }

  // ================== Estratégia de antilag ================== //
  if (acceleratorPercentage == 0 && rpm > antilagRpm) {
    if (millis() - millisAntilag <= tempoAntilag) {
    acceleratorSet = (acceleratorSet+antilagValue);
    } else {}
  } else {
    millisAntilag = millis();
  }
   
  acceleratorSet = constrain(acceleratorSet,0,100);

    // Encontra os dois pontos de referência no mapa de aceleração
  int lowerIndex = -1;
  int upperIndex = -1;
  for (int i = 0; i < 10; i++) {
    if (acceleratorSet >= acceleratorMap[i][0] && acceleratorSet <= acceleratorMap[i + 1][0]) {
      lowerIndex = i;
      upperIndex = i + 1;
      break;
  }}
  // Calcula a abertura da borboleta baseada na interpolação não linear
  double lowerValue = acceleratorMap[lowerIndex][1];
  double upperValue = acceleratorMap[upperIndex][1];
  double t = (acceleratorSet - acceleratorMap[lowerIndex][0]) / (acceleratorMap[upperIndex][0] - acceleratorMap[lowerIndex][0]);
  adjustedSetpoint = lowerValue + t * (upperValue - lowerValue);
  adjustedSetpoint = adjustedSetpoint*10;
  adjustedSetpoint = map(adjustedSetpoint,0,1000,butterflyMin,butterflyMax);


// ========================== // ============================================================================ // ========================== //



// ========================== Cálculo de Rotação ========================== //

  // Faz o calculo de rotação baseado no tamanho do pulso
  if (pulses >= pulsesPerRotation) {
    rpm = 30000000UL / periodMicros;
    lastRpmUpdateTime = millis(); // Atualiza o tempo da última atualização do RPM
    pulses = 0; // Reinicia a contagem de pulsos
  }
  // Verifica se houve uma atualização recente do RPM
  if (millis() - lastRpmUpdateTime >= noRotationTimeout) {
    // Não houve rotação nas últimas noRotationTimeout milissegundos
    rpm = 0; // Define RPM como 0 para indicar que não há rotação
  }

// ======================================================================== //


// ========================== Estratégias de Segurança ========================== //

  // Verifica se há alguma avaria na borboleta
  if(butterflyError < -10 || butterflyError > 10) {
   // if (millis() - butterflyMillis >= butterflyProtection) {
      digitalWrite(dirPin, LOW); // Desliga a ponte H
      emergencyMode = true;
   // }
  }//else {butterflyMillis = millis();}
  // Verifica se há alguma avaria no pedal
  if(pedalError < -3 || pedalError > 3) {
    digitalWrite(dirPin, LOW); // Desliga a ponte H
    emergencyMode = true;
  }
  if(emergencyMode==true){
    piscarLed(13,100);
  }

  // Verifica se o motor está desligado
  if(rpm == 0) {
    // Verifica se o tempo excedeu o limite
    if (millis() - butterflyIdleMillis >= butterflyIdleTime) {
      digitalWrite(dirPin, LOW); // Desliga a ponte H
    } 
  } 
  else {
    butterflyIdleMillis = millis();
    if(emergencyMode == false){
      digitalWrite(dirPin, HIGH);
    }
  }
  // Se o acelerador for tocado e está tudo ok > ativa a borboleta
  if(acceleratorPercentage>0 && emergencyMode == false) {
    digitalWrite(dirPin, HIGH);
    butterflyIdleMillis = millis();
  }

// ========================== ======================== ========================== //
  


  // Limita o valor alvo para proteger a borboleta
  adjustedSetpoint = constrain(adjustedSetpoint,butterflyMin,butterflyMax);

  // Cálculo do erro ponderado pelo coeficiente de precisão
  error = (adjustedSetpoint - input) * precisionMultiplier;
  
  // Mapeia o erro ajustado para a saída do motor
  if(input > 150){
  output = constrain(error,-50,maxAtuation);
  }else{
  output = constrain(error,maxReverseAtuation,maxAtuation);
  }
  
  // Define a direção da ponte H com base na saída do motor
  if (output > 0) {
    analogWrite(pwmPin1, output);
    analogWrite(pwmPin2, 0);
  } else {
    analogWrite(pwmPin1, 0);
    analogWrite(pwmPin2, -output);
  }



// ================================== Área de DEBUG ================================== //



  if(Serial.available() > 0) {
    
    /*
      Serial.print("  Sensor TPS1:");
      Serial.print(butterflyPotValue1);
      Serial.print("  Sensor TPS2:");
      Serial.print(butterflyPotValue2);
      Serial.print("  Valor de PWM:");
      Serial.print(output);
      Serial.print("  TPS1 + TPS2:");
      Serial.println(input);
    */
      Serial.print("  Sensor Pedal1:");
      Serial.print(acceleratorPotValue1);
      Serial.print("  Sensor Pedal2:");
      Serial.print(acceleratorPotValue2);
      Serial.print("  Pedal1 + Pedal2:");
      Serial.print(acceleratorPercentage);
      Serial.print("  Diferença dos Pedais:");
      Serial.println(pedalError);
  

  }






     /*
Serial.print(butterflyPotValue1);
Serial.print("  ");
Serial.print(butterflyPotValue2);
Serial.print("  ");
Serial.print(output);
Serial.print("  ");
Serial.println(input);
*/
//Serial.println(TempSensorValue);




// ================================== Área de DEBUG ================================== //



}

void hallSensorISR() {
  unsigned long currentTime = micros();
  periodMicros = currentTime - periodStartMicros;
  periodStartMicros = currentTime;
  pulses++;
}

void piscarLed(int led, int interv_ms)
{
          if((int)(millis()/interv_ms)%2==0)
                  digitalWrite(led, HIGH);
         else
                  digitalWrite(led, LOW);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  // Verifique se x está dentro do intervalo de entrada
  if (x < in_min) {
    x = in_min;
  }
  if (x > in_max) {
    x = in_max;
  }

  // Realize o mapeamento
  float result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

  return result;
}