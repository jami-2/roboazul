#include <AFMotor.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <GY521.h>
#include "SerialDebug.h"
#include <NewPing.h>
#include <QTRSensors.h>

// Configurações de hardware
#define TRIGGER_PIN  48
#define ECHO_PIN     49
#define MAX_DISTANCE 50
#define TCAADDR 0x70
#define DIREITA 0
#define ESQUERDA 1
#define LEDA 46
#define LEDB 47

// Constantes de configuração
#define TEMPO_PRE90 1000
#define TEMPO_ORBITA 2000
#define VEL_NORMAL 150  // Velocidade base aumentada para PID
#define VEL_RESISTENCIA 120
#define VEL_MAXIMA 255  // Velocidade máxima para os motores
#define INTERVALO_LEITURA 50
#define DISTANCIA_OBSTACULO 10

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

// Configuração QTR Sensors
#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 2

QTRSensorsRC qtrrc((unsigned char[]) {A0, A1, A2, A3, A4, A5, A6, A7}, 
              NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

CorSensor corSensores[2];
Adafruit_TCS34725 tcs0(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
Adafruit_TCS34725 tcs1(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);

// Variáveis do PID
float Kp = 0.1;  // Ganho Proporcional
float Ki = 0.001; // Ganho Integral
float Kd = 0.2;  // Ganho Derivativo

int erro = 0;
int erroAnterior = 0;
int integral = 0;
int derivativo = 0;
int saidaPID = 0;

// Protótipos de funções
void setup();
void loop();
void lerSensores();
int calcularPosicaoLinha();
void calcularPID();
void aplicarPID(int posicao);
const char* detectarCor(uint8_t canal);
void controlarMotores(int velocidadeEsq, int velocidadeDir);
void pararMotores();
void andarReto();
void andarRapido();
void andarTras();
void virar(int direcao);
void virarForte(int direcao);
void virarComGiro(float anguloAlvo, int direcao);
void vencerResistenciaInicial();
void desligarLEDs();
void ligarLEDs();
void tcaSelect(uint8_t channel);
void resolverBifurcacao();
void desviarObstaculo();
void calibrarSensores();

void setup() {
  Serial.begin(9600);
  printlnA("Iniciando seguidor de linha com PID...");
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

  // Calibração dos sensores QTR
  calibrarSensores();

  // Inicialização sensores de cor
  tcaSelect(0);
  corSensores[0].inicializado = tcs0.begin();
  tcaSelect(1);
  corSensores[1].inicializado = tcs1.begin();

  vencerResistenciaInicial();
  estadoAtual = SEGUINDO_LINHA;
}

void calibrarSensores() {
  printlnA("Calibrando sensores QTR...");
  for (int i = 0; i < 400; i++) {
    qtrrc.calibrate();
    delay(10);
  }
  printlnA("Calibração concluída!");
}

void loop() {
  debugHandle();

  // Máquina de estados principal
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

      cor = detectarCor(0);
      if(strcmp(cor, "colorido") == 0) {
        estadoAtual = SALA_DE_RESGATE;
        break;
      }
      else if(strcmp(cor, "vermelho") == 0) {
        estadoAtual = PARADO;
        break;
      }
      
      lerSensores();
      int posicao = calcularPosicaoLinha();
    
      // Verificação de bifurcação
      if(posicao == -999) {
        estadoAtual = RESOLVENDO_BIFURCACAO;
        break;
      }
        
      // Aplicar controle PID
      aplicarPID(posicao);
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
      // Implementar lógica da sala de resgate
      break;
      
    case PARADO:
      pararMotores();
      break;
  }
}

void lerSensores() {
  qtrrc.read(sensorValues);
  for (int i = 0; i < NUM_SENSORS; i++) {
    printV("Sensor"); printV(i); printV(":"); printlnV(sensorValues[i]);
  }
}

int calcularPosicaoLinha() {
  unsigned int position = qtrrc.readLine(sensorValues);
  
  // Verificar se todos os sensores estão detectando linha (bifurcação)
  bool todosAtivos = true;
  bool nenhumAtivo = true;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] < 500) {
      todosAtivos = false;
    } else {
      nenhumAtivo = false;
    }
  }
  
  if (todosAtivos || (sensorValues[0] > 500 && sensorValues[NUM_SENSORS-1] > 500)) {
    return -999; // Código especial para bifurcação
  }
  
  if (nenhumAtivo) {
    return 0;
  }
  
  // Retorna a posição da linha (0-7000)
  return position;
}

void calcularPID() {
  // Cálculo dos componentes do PID
  integral += erro;
  derivativo = erro - erroAnterior;
  
  // Fórmula do PID
  saidaPID = (Kp * erro) + (Ki * integral) + (Kd * derivativo);
  
  // Atualiza o erro anterior para o próximo cálculo
  erroAnterior = erro;
}

void aplicarPID(int posicao) {
  // Calcula o erro (posição ideal é 3500 para 8 sensores)
  erro = posicao - 3500;
  
  // Calcula o PID
  calcularPID();
  
  // Ajusta as velocidades dos motores baseado no PID
  int velocidadeEsq = VEL_NORMAL + saidaPID;
  int velocidadeDir = VEL_NORMAL - saidaPID;
  
  // Limita as velocidades para não ultrapassar o máximo
  velocidadeEsq = constrain(velocidadeEsq, -VEL_MAXIMA, VEL_MAXIMA);
  velocidadeDir = constrain(velocidadeDir, -VEL_MAXIMA, VEL_MAXIMA);
  
  // Aplica as velocidades
  controlarMotores(velocidadeEsq, velocidadeDir);
  
  // Debug
  printD("Erro:"); printD(erro); 
  printD(" PID:"); printD(saidaPID);
  printD(" Esq:"); printD(velocidadeEsq);
  printD(" Dir:"); printlnD(velocidadeDir);
}

// ... (as demais funções permanecem semelhantes, exceto controlarMotores)

void controlarMotores(int velocidadeEsq, int velocidadeDir) {
  // Motor esquerdo
  motorFrenteEsquerdo.setSpeed(abs(velocidadeEsq));
  motorTrasEsquerdo.setSpeed(abs(velocidadeEsq));
  
  if(velocidadeEsq > 0) {
    motorFrenteEsquerdo.run(FORWARD);
    motorTrasEsquerdo.run(FORWARD);
  } else {
    motorFrenteEsquerdo.run(BACKWARD);
    motorTrasEsquerdo.run(BACKWARD);
  }
  
  // Motor direito
  motorFrenteDireito.setSpeed(abs(velocidadeDir));
  motorTrasDireito.setSpeed(abs(velocidadeDir));
  
  if(velocidadeDir > 0) {
    motorFrenteDireito.run(FORWARD);
    motorTrasDireito.run(FORWARD);
  } else {
    motorFrenteDireito.run(BACKWARD);
    motorTrasDireito.run(BACKWARD);
  }
}