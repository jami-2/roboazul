#include <AFMotor.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <GY521.h>
#include "SerialDebug.h"
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
#define VEL_NORMAL 88
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

// Configuração dos sensores QTR
#define NUM_SENSORS 4      // Número de sensores QTR
#define TIMEOUT 2500       // Tempo de espera para o sensor em microssegundos
#define EMITTER_PIN 2      // Pino para controle do LED IR (pode ser QTR_NO_EMITTER_PIN)

Servo servoUltrassonico;
QTRSensorsRC qtrrc((unsigned char[]) {A0, A1, A2, A3}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
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

void lerSensoresQTR() {
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
  
  // Verifica se todos os sensores estão detectando linha preta (bifurcação)
  bool todosAtivos = true;
  bool nenhumAtivo = true;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] < 500) { // Ajuste este valor conforme a calibração
      todosAtivos = false;
    } else {
      nenhumAtivo = false;
    }
  }
  
  if (todosAtivos) {
    pararMotores();
    return 69; // Código especial para bifurcação
  }
  
  if (nenhumAtivo) {
    return 0; // Nenhum sensor detectando linha
  }
  
  // Retorna a posição relativa da linha (0-3000, com 1500 sendo o centro)
  return (position - 1500) / 1000.0; // Normaliza para -1.5 a +1.5
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
      
      float media = calcularPosicaoLinha();
    
      // Verificação de bifurcação
      if(media == 69) {
        estadoAtual = RESOLVENDO_BIFURCACAO;
        break;
      }
        
      // Controle normal de seguimento
      if(media < -1.5) virarForte(ESQUERDA);
      else if(media > 1.5) virarForte(DIREITA);
      else if(media < -0.7) virar(ESQUERDA);
      else if(media > 0.7) virar(DIREITA);
      else andarReto();
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

const char* detectarCor(uint8_t canal) {
  if(canal > 1 || !corSensores[canal].inicializado) return "erro";
  
  tcaSelect(canal);
  uint16_t r, g, b, c;
  corSensores[canal].tcs.getRawData(&r, &g, &b, &c);
  
  if(r < 3000 && g < 4400 && b < 3400) return "preto";
  if(r > 6000 && g > 7500 && b > 6500) return "branco";
  return "colorido";
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
    lerSensores();
    if(sensores[0].valor || sensores[1].valor || sensores[2].valor || sensores[3].valor) {
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

void pararMotores() {
  motorFrenteEsquerdo.run(RELEASE);
  motorFrenteDireito.run(RELEASE);
  motorTrasEsquerdo.run(RELEASE);
  motorTrasDireito.run(RELEASE);
}

void andarReto() {
  controlarMotores(1, 1, VEL_NORMAL);
}

void andarRapido() {
  controlarMotores(1, 1, 200);
}

void andarTras() {
  controlarMotores(0, 0, VEL_NORMAL + 10);
}

void virar(int direcao) {
  controlarMotores(direcao, !direcao, VEL_CURVA);
}

void virarForte(int direcao) {
  controlarMotores(direcao, !direcao, VEL_CURVA_EXTREMA);
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
