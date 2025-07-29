#include <AFMotor.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <GY521.h>
#include "SerialDebug.h"
#include <NewPing.h>
#include <QTRSensors.h>
#include <Servo.h>
// Modificações:
// Trocar GY-521 para conexão direta
// Instalar QTR no chão, entre as rodas
// Mover os sensores de cor para trás


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

#define SERVO_PIN 10
#define ANGULO_FRENTE 90
#define ANGULO_ESQUERDA 180
#define ANGULO_DIREITA 0
#define DISTANCIA_PARADA 15
#define DISTANCIA_MINIMA_VIRADA 10

Servo servoUltrassonico;

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
#define Kp 0.1f    // Ganho Proporcional  
#define Ki 0.001f  // Ganho Integral  
#define Kd 0.2f    // Ganho Derivativo  

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

  //Inicialização sala de resgate
  servoUltrassonico.attach(SERVO_PIN);
  servoUltrassonico.write(ANGULO_FRENTE);

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

      int cor = detectarCor(0);
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
      executarComportamentoSalaResgate();
      break;
      
    case PARADO:
      pararMotores();
      printV("PRONTO!");
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


void entrarSalaResgate() {
  printlnA("Entrando na sala de resgate...");
  estadoAtual = SALA_DE_RESGATE;
}

void executarComportamentoSalaResgate() {
  while(estadoAtual == SALA_DE_RESGATE) {
    // 1. Verificar se encontrou linha preta (saída)
    lerSensores();
    int posicao = calcularPosicaoLinha();
    
    if(posicao == -999) { // Todos sensores ativos (linha preta)
      printlnA("Linha de saida detectada!");
      pararMotores();
      delay(1000);
      estadoAtual = SEGUINDO_LINHA;
      return;
    }

    // 2. Andar reto
    andarReto();
    
    // 3. Verificar obstáculo frontal
    int distanciaFrontal = lerUltrassonicoFrontal();
    
    if(distanciaFrontal < DISTANCIA_PARADA) {
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

// Adicione estas implementações no seu código Arduino

void desligarLEDs() {
  digitalWrite(LEDA, LOW);
  digitalWrite(LEDB, LOW);
}

void ligarLEDs() {
  digitalWrite(LEDA, HIGH);
  digitalWrite(LEDB, HIGH);
}

void pararMotores() {
  motorFrenteEsquerdo.run(RELEASE);
  motorFrenteDireito.run(RELEASE);
  motorTrasEsquerdo.run(RELEASE);
  motorTrasDireito.run(RELEASE);
}

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void vencerResistenciaInicial() {
  controlarMotores(1, 1, VEL_RESISTENCIA);
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

void controlarMotores(int esqFrente, int dirFrente, int velocidade) {
  motorFrenteEsquerdo.setSpeed(velocidade);
  motorFrenteDireito.setSpeed(velocidade);
  motorTrasEsquerdo.setSpeed(velocidade);
  motorTrasDireito.setSpeed(velocidade);
  
  motorFrenteEsquerdo.run(esqFrente ? FORWARD : BACKWARD);
  motorTrasEsquerdo.run(esqFrente ? FORWARD : BACKWARD);
  motorFrenteDireito.run(dirFrente ? FORWARD : BACKWARD);
  motorTrasDireito.run(dirFrente ? FORWARD : BACKWARD);
}