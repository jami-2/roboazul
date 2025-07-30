#include <AFMotor.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <GY521.h>
#include <NewPing.h>
#include <Servo.h>
#include <QTRSensors.h> // Adicionando a biblioteca Pololu QTR
// Modificações:
// Trocar GY-521 para conexão direta
// Instalar QTR no chão, entre as rodas
// Mover os sensores de cor para trás

// TODO:
// Verificar se ele detecta tudo bonitinho
// Concertar todas as desgraças que vão com certeza aparecer,
// porque este código não foi testado ainda em hardware
// eu realmente espero que a gente termine a tempo.

Servo servoUltrassonico;

// Configurações de hardware
#define TRIGGER_PIN  48
#define ECHO_PIN     49
#define MAX_DISTANCE 50
#define TCAADDR 0x70
#define DIREITA 0
#define ESQUERDA 1
#define LEDA 46
#define LEDB 47

// Constantes PID
#define KP 0.1  // Ganho Proporcional (ajuste fino necessário)
#define KI 0.001 // Ganho Integral
#define KD 0.5  // Ganho Derivativo
#define VEL_BASE 100  // Velocidade base dos motores

// Outras constantes
#define TEMPO_PRE90 1000
#define TEMPO_ORBITA 2000
#define VEL_NORMAL 130
#define VEL_RESISTENCIA 120
#define VEL_CURVA 140
#define VEL_CURVA_EXTREMA 220
#define INTERVALO_LEITURA 50
#define DISTANCIA_OBSTACULO 10

#define SERVO_PIN 10
#define ANGULO_FRENTE 90
#define ANGULO_ESQUERDA 180
#define ANGULO_DIREITA 0
#define DISTANCIA_PARADA 15
#define DISTANCIA_MINIMA_VIRADA 10

#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN QTR_NO_EMITTER_PIN
#define LINHA_PRETA 0        // 0 para linha preta, 1 para linha branca
#define LIMITE_PERDA_LINHA 200 // Valor mínimo para considerar linha detectada
#define POSICAO_MAXIMA (NUM_SENSORS-1)*1000 // 7000 para 8 sensores

// Variáveis PID
float erroAnterior = 0;
float integral = 0;

// Pinos dos sensores QTR
const uint8_t pinosSensores[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};
QTRSensorsRC qtrrc(pinosSensores, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// Estados do robô
enum Estado {
  SEGUINDO_LINHA,
  RESOLVENDO_BIFURCACAO,
  DESVIANDO_OBSTACULO,
  INICIALIZANDO,
  SALA_DE_RESGATE,
  PARADO
};

// Estruturas de dados
struct CorSensor {
  Adafruit_TCS34725 tcs;
  bool inicializado = false;
};

// Variáveis globais
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
GY521 mpu(0x68);
Estado estadoAtual = INICIALIZANDO;
unsigned long ultimoTempoLeitura = 0;

AF_DCMotor motorFrenteEsquerdo(4);
AF_DCMotor motorFrenteDireito(1);
AF_DCMotor motorTrasEsquerdo(3);
AF_DCMotor motorTrasDireito(2);

CorSensor corSensores[2];
Adafruit_TCS34725 tcs0(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
Adafruit_TCS34725 tcs1(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);

void setup() {
  Serial.begin(9600);
  printlnA("Iniciando seguidor de linha...");
  Wire.begin();

  // Inicialização MPU6050
  mpu.begin();
  mpu.setAccelSensitivity(0);
  mpu.setGyroSensitivity(0);
  mpu.setThrottle(false);
  mpu.calibrate(1500);

  // Configuração LEDs
  pinMode(LEDA, OUTPUT);
  pinMode(LEDB, OUTPUT);
  desligarLEDs();

  // Inicialização motores
  pararMotores();

  // Inicialização sala de resgate
  servoUltrassonico.attach(SERVO_PIN);
  servoUltrassonico.write(ANGULO_FRENTE);

  // Calibração dos sensores QTR
  calibrarSensoresQTR();

  // Inicialização sensores de cor
  tcaSelect(0);
  corSensores[0].inicializado = tcs0.begin();
  tcaSelect(1);
  corSensores[1].inicializado = tcs1.begin();

  vencerResistenciaInicial();
  estadoAtual = SEGUINDO_LINHA;
}

void calibrarSensoresQTR() {
  printlnA("Calibrando sensores QTR...");
  delay(500);
  
  // Rotina de calibração manual (mover o robô sobre a linha durante a calibração)
  for (int i = 0; i < 200; i++) {
    qtrrc.calibrate();
    delay(20);
  }
  
  printlnA("Calibracao completa. Valores minimos:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  printlnA("Valores maximos:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
}

int lerSensoresQTR() {
  // Lê os sensores QTR e armazena os valores em sensorValues
  int position = qtrrc.readLine(sensorValues);
  
  // Debug: mostra os valores dos sensores
  for (unsigned char i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
  
  return position;
}

float calcularPosicaoLinha() {
  int position = lerSensoresQTR();
  
  bool todosAtivos = true;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] < 500) { // Ajuste este valor conforme calibração
      todosAtivos = false;
    }
  }
  
  if (todosAtivos) {
    estadoAtual = RESOLVENDO_BIFURCACAO; // Mudança direta de estado aqui
    return 0; // Valor arbitrário, pois o estado já foi alterado
  }
  
  
  return position 
}

void seguirLinhaPID() {
  // Lê os sensores e obtém a posição bruta (0-7000)
  uint16_t position = qtrrc.readLine(sensorValues);
  
  // Erro bruto (3500 = centro)
  int erro = position - 3500;
  
  // PID sem dó
  integral += erro;
  int derivativo = erro - erroAnterior;
  erroAnterior = erro;
  
  int correcao = KP * erro + KI * integral + KD * derivativo;
  
  // Aplica correção (velocidade base 150)
  int velEsq = 150 - correcao;
  int velDir = 150 + correcao;
  
  // Limita as velocidades
  velocidadeEsquerda = constrain(velEsq, -255, 255);
  velocidadeDireita = constrain(velDir, -255, 255);

  // Controla os motores
  controlarMotores(velocidadeEsquerda, velocidadeDireita);
  
  // Debug (opcional)
  Serial.print("Erro: "); Serial.print(erro);
  Serial.print(" Correcao: "); Serial.print(correcao);
  Serial.print(" VelEsq: "); Serial.print(velocidadeEsquerda);
  Serial.print(" VelDir: "); Serial.println(velocidadeDireita);
}

void loop() {
  debugHandle();

  switch(estadoAtual) {
    case INICIALIZANDO:
      estadoAtual = SEGUINDO_LINHA;
      break;
      
    case SEGUINDO_LINHA:
      // Verificação de obstáculo
      int distancia = sonar.ping_cm();
      if(distancia < DISTANCIA_OBSTACULO && distancia != 0) {
        estadoAtual = DESVIANDO_OBSTACULO;
        break;
      }

      int cor = detectarCor(0);
      if(strcmp(cor, "colorido") == 0) {
        estadoAtual = SALA_DE_RESGATE;
        break;
      }
      else if(strcmp(cor, "vermelho") == 0) {
        estadoAtual = PARADO;
        break;
      }
      
      // Substitui o controle antigo pelo PID
      seguirLinhaPID();
      break;
      
    case RESOLVENDO_BIFURCACAO:
      resolverBifurcacao();
      estadoAtual = SEGUINDO_LINHA;
      break;
      
    case DESVIANDO_OBSTACULO:
      desviarObstaculo();
      estadoAtual = SEGUINDO_LINHA;
      break;
    
    case SALA_DE_RESGATE:
      executarComportamentoSalaResgate();
      break;
      
    case PARADO:
      pararMotores();
      printV("PRONTO!");
      break;
  }
}

void resolverBifurcacao() {
  ligarLEDs();
  const char* corA = detectarCor(0);
  const char* corB = detectarCor(1);
  desligarLEDs();
  
  vencerResistenciaInicial();
  
  if(strcmp(corA, "preto") == 0 && strcmp(corB, "preto") == 0) {
    andarTras();
    delay(500);
  }
  else if(strcmp(corA, "preto") == 0) {
    virarForte(DIREITA);
    delay(50);
  }
  else if(strcmp(corB, "preto") == 0) {
    virarForte(ESQUERDA);
    delay(50);
  }
  else if(strcmp(corA, "colorido") == 0 && strcmp(corB, "colorido") != 0) {
    andarReto();
    delay(TEMPO_PRE90);
    virarComGiro(90, ESQUERDA);
  }
  else if(strcmp(corA, "colorido") != 0 && strcmp(corB, "colorido") == 0) {
    andarReto();
    delay(TEMPO_PRE90);
    virarComGiro(90, DIREITA);
  }
  else if(strcmp(corA, "colorido") == 0 && strcmp(corB, "colorido") == 0) {
    virarComGiro(90, DIREITA);
    virarComGiro(90, DIREITA);
  }
  else {
    andarReto();
    delay(1350);
  }
}

// Corrigir a função desviarObstaculo() para usar os sensores QTR
void desviarObstaculo() {
  pararMotores();
  delay(100);
  virarComGiro(90, ESQUERDA);
  pararMotores();
  delay(100);
  
  andarReto();
  delay(TEMPO_ORBITA / 2);
  
  unsigned long ultimaVirada = millis();
  
  while(true) {
    lerSensoresQTR();
    
    // Verifica se algum sensor detectou a linha
    bool linhaDetectada = false;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i] > 500) { // Ajuste este valor conforme a calibração
        linhaDetectada = true;
        break;
      }
    }
    
    if(linhaDetectada) {
      pararMotores();
      virarComGiro(90, ESQUERDA);
      return;
    }
    
    andarReto();
    
    if(millis() - ultimaVirada >= 2000) {
      pararMotores();
      delay(100);
      virarComGiro(90, DIREITA);
      pararMotores();
      delay(100);
      ultimaVirada = millis();
    }
    
    delay(10);
  }
}

void controlarMotores(int velocidadeEsq, int velocidadeDir) {
  motorFrenteEsquerdo.setSpeed(abs(velocidadeEsq));
  motorFrenteDireito.setSpeed(abs(velocidadeDir));
  motorTrasEsquerdo.setSpeed(abs(velocidadeEsq));
  motorTrasDireito.setSpeed(abs(velocidadeDir));
  
  motorFrenteEsquerdo.run(velocidadeEsq > 0 ? FORWARD : BACKWARD);
  motorTrasEsquerdo.run(velocidadeEsq > 0 ? FORWARD : BACKWARD);
  motorFrenteDireito.run(velocidadeDir > 0 ? FORWARD : BACKWARD);
  motorTrasDireito.run(velocidadeDir > 0 ? FORWARD : BACKWARD);
}

void pararMotores() {
  motorFrenteEsquerdo.run(RELEASE);
  motorFrenteDireito.run(RELEASE);
  motorTrasEsquerdo.run(RELEASE);
  motorTrasDireito.run(RELEASE);
}

void andarReto() {
  controlarMotores(VEL_NORMAL, VEL_NORMAL);
}

void andarRapido() {
  controlarMotores(200, 200);
}

void andarTras() {
  controlarMotores(-VEL_NORMAL, -VEL_NORMAL);
}

void virar(int direcao) {
  // Se direcao = 1 (direita), motor esquerdo vai pra frente (velocidade positiva) e direito pra trás (velocidade negativa)
  // Se direcao = 0 (esquerda), motor direito vai pra frente (velocidade positiva) e esquerdo pra trás (velocidade negativa)
  int velocidadeEsq = direcao ? VEL_CURVA : -VEL_CURVA;
  int velocidadeDir = direcao ? -VEL_CURVA : VEL_CURVA;
  controlarMotores(velocidadeEsq, velocidadeDir);
}

void virarForte(int direcao) {
  int velocidadeEsq = direcao ? VEL_CURVA_EXTREMA : -VEL_CURVA_EXTREMA;
  int velocidadeDir = direcao ? -VEL_CURVA_EXTREMA : VEL_CURVA_EXTREMA;
  controlarMotores(velocidadeEsq, velocidadeDir);
}

void virarComGiro(float anguloAlvo, int direcao) {
  float yawInicial = mpu.getYaw();
  float alvoYaw = fmod((yawInicial + (direcao == DIREITA ? anguloAlvo : -anguloAlvo) + 360), 360);
  
  while(true) {
    mpu.readGyro();
    float yawAtual = fmod(mpu.getYaw(), 360);
    float delta = fmod((yawAtual - alvoYaw + 360), 360);
    
    if(delta < 5 || delta > 355) break;
    
    virar(direcao);
    delay(10);
  }
}


void entrarSalaResgate() {
  printlnA("Entrando na sala de resgate...");
  estadoAtual = SALA_DE_RESGATE;
}

void executarComportamentoSalaResgate() {
  while(estadoAtual == SALA_DE_RESGATE) {
    // 1. Verificar se encontrou linha preta (saída)
    float posicao = calcularPosicaoLinha();
            
    // Verifica se pelo menos 1 sensor está ativo (linha preta)
    bool linhaPretaDetectada = false;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i] > 500) { // Se algum sensor detectar preto (ajuste o limiar)
        linhaPretaDetectada = true;
        break; // Sai do loop assim que detectar
      }
    }
        
    if(linhaPretaDetectada) { // Se pelo menos 1 sensor vê preto
      printlnA("Linha de saida detectada!");
      estadoAtual = SEGUINDO_LINHA;
      return;
    }

    // 2. Andar reto
    andarReto();
    
    // 3. Verificar obstáculo frontal
    int distanciaFrontal = sonar.ping_cm();
    
    if(distanciaFrontal < DISTANCIA_PARADA && distanciaFrontal != 0) {
      pararMotores();
      printlnA("Obstaculo frontal detectado!");
      
      // 4. Verificar lados
      int distanciaEsquerda = lerUltrassonicoLateral(ANGULO_ESQUERDA);
      delay(200);
      int distanciaDireita = lerUltrassonicoLateral(ANGULO_DIREITA);
      delay(200);
      
      // Retornar servo para frente
      servoUltrassonico.write(ANGULO_FRENTE);
      
      // 5. Decidir direção
      if(distanciaEsquerda > distanciaDireita) {
        printlnA("Virando para esquerda (mais espaco)");
        virarComGiro(90, ESQUERDA);
      } else {
        printlnA("Virando para direita (mais espaco)");
        virarComGiro(90, DIREITA);
      }
      
      // 6. Continuar andando
      andarReto();
      delay(500);
    }
    
    delay(50); // Pequena pausa entre leituras
  }
}

int lerUltrassonicoFrontal() {
  // Já temos a função sonar.ping_cm() para o frontal
  int distancia = sonar.ping_cm();
  printD("Distancia frontal: "); printlnD(distancia);
  return distancia;
}

int lerUltrassonicoLateral(int angulo) {
  servoUltrassonico.write(angulo);
  delay(300); // Tempo para o servo se mover
  
  // Criar um sensor temporário para a lateral
  NewPing sonarLateral(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
  int distancia = sonarLateral.ping_cm();
  
  printD("Distancia lateral ("); 
  printD(angulo == ANGULO_ESQUERDA ? "esq" : "dir");
  printD("): "); printlnD(distancia);
  
  return distancia;
}


void vencerResistenciaInicial() {
  controlarMotores(VEL_RESISTENCIA, VEL_RESISTENCIA);
  delay(100);
}

const char* detectarCor(uint8_t canal) {
  if(canal > 1 || !corSensores[canal].inicializado) return "erro";
  
  tcaSelect(canal);
  uint16_t r, g, b, c;
  corSensores[canal].tcs.getRawData(&r, &g, &b, &c);
  
  if(r < 3000 && g < 4400 && b < 3400) return "preto";
  if(r > 6000 && g > 7500 && b > 6500) return "branco";
  if(r > 9000 && g < 5000 && b < 5000) return "vermelho";
  return "colorido";
}

void desligarLEDs() {
  digitalWrite(LEDA, LOW);
  digitalWrite(LEDB, LOW);
  delay(200);
}

void ligarLEDs() {
  digitalWrite(LEDA, HIGH);
  digitalWrite(LEDB, HIGH);
  delay(200);
}

void tcaSelect(uint8_t channel) {
  if(channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void lerSensores() {
  return lerSensoresQTR();
}