
/*   Altímetro Supernova
 *    
 *   Implementação do altímetro em RTOS. As conexões do ESP32 seguem
 *   na póxima seção comentada.
 *   
 *   Conexões de dispositivos:
 *   BMP180:
 *   -SDA   GPIO21
 *   -SCL   GPIO22
 *   
 *   MPU6050:
 *   -
 *   -
 *   -
 *   
 *   SD:
 *   -MOSI  GPIO23 
 *   -MISO  GPIO19
 *   -SCK   GPIO18
 *   -CS    GPIO05
 *   


*/
//Configurações do altímetro

#define THRESHOLD_ALTURA 10

//Definições de portas

#define PINO_RELE 25
#define PINO_LEDR 27
#define PINO_LEDG 26
#define PINO_BUZZER 34
#define PINO_BOTAO 35

//Incluindo as bibliotecas referentes ao BMP180, SD e IMU respectivamente
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include <MPU6050.h>


#include "FS.h"
#include "SD.h"
#include "SPI.h"

//Definição de variáveis referentes ao RTOS
SemaphoreHandle_t semaforoBotao         = xSemaphoreCreateMutex();
SemaphoreHandle_t semaforoInicializa    = xSemaphoreCreateMutex();
SemaphoreHandle_t semaforoDadosProntos  = xSemaphoreCreateMutex();
SemaphoreHandle_t semaforoDadosInicia   = xSemaphoreCreateMutex();

TaskHandle_t handleInicializa, handleBotao;



struct structErro
{
  char bmp;
  char mpu;
  char sd;
} erroInicializacao;

struct structDados
{
  float temperatura;
  float altura;
  
  float aceleracaoX;
  float aceleracaoY;
  float aceleracaoZ;

  float rotacaoX;
  float rotacaoY;
  float rotacaoZ;
  
} dadosLidos;

//Definição de variáveis referente ao módulos
Adafruit_BMP085 bmp;
MPU6050 mpu;
uint8_t cardType;

char statusAtual = 'e';


void setup() {

  //Iniciando Comunicação e Protocolos
  Serial.begin(115200);
  Wire.begin();

  pinMode(PINO_BOTAO, INPUT);
  pinMode(PINO_LEDG,  OUTPUT);
  pinMode(PINO_LEDR,  OUTPUT);
  pinMode(PINO_RELE,  OUTPUT);

    Serial.println("Inciando Tasks");
  
  //Iniciando as funções do RTOS
  
  xTaskCreate(
                    taskInicializa,          /* Task function. */
                    "TaskInicializa",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    10,                /* Priority of the task. */
                    &handleInicializa);            /* Task handle. */
 
  xTaskCreate(
                    taskLeitura,          /* Task function. */
                    "taskLeitura",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */

 
  
  xTaskCreate(
                    taskEscrita,          /* Task function. */
                    "taskEscrita",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */

  xTaskCreate(
                    taskBotao,          /* Task function. */
                    "taskBotao",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    5,                /* Priority of the task. */
                    &handleBotao);            /* Task handle. */


  xTaskCreate(
                    taskRecuperacao,          /* Task function. */
                    "taskRecuperacao",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */


  xTaskCreate(
                    taskStatus,          /* Task function. */
                    "taskStatus",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */
 
 
}
 
void loop() {
  delay(1000);
}

void taskInicializa( void *parameters){

char erroSD  = 0;
char erroBMP = 0;
char erroMPU = 0;



xSemaphoreTake( semaforoInicializa, 0 );

Serial.println("Task de inicialização iniciada");


  while(1){

    if(!SD.begin()){
//        Serial.println("Falhou ao montar o cartão");
//        delay(500);
        erroSD = 1;
      }
    cardType = SD.cardType();
    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        //return;
    }

    //Serial.println("Inicializando o BMP");
    if(!bmp.begin()) {
//        Serial.println("Falha na inicialização do BMP180. Verifique as conexões");
//        delay(500);
        erroBMP = 1;
      }
    
    
    if(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
//    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
//    delay(500);
    erroMPU=1;
  }
  

      xSemaphoreTake(semaforoDadosInicia,portMAX_DELAY);
        
      erroInicializacao.bmp = erroBMP;
      erroInicializacao.mpu = erroMPU;
      erroInicializacao.sd  = erroSD;
      
      xSemaphoreGive(semaforoDadosInicia);

    
    if(!erroSD && !erroBMP && !erroMPU){
      
      xSemaphoreGive( semaforoInicializa ); 
      
      Serial.println("Sem erros, terminando a task de inicialização!");

      vTaskDelete( handleInicializa );

      statusAtual = 'e';
    }
  }
}

void taskLeitura( void *parameters){

  while(1){

    xSemaphoreTake(semaforoInicializa,portMAX_DELAY);

    xSemaphoreTake(semaforoDadosProntos,portMAX_DELAY);

    dadosLidos.altura       = bmp.readAltitude(101500);
    dadosLidos.temperatura  = bmp.readTemperature();

    Vector normAccel = mpu.readNormalizeAccel();
    Vector normGyro = mpu.readNormalizeGyro();

    dadosLidos.aceleracaoX  = normAccel.XAxis;
    dadosLidos.aceleracaoY  = normAccel.YAxis;
    dadosLidos.aceleracaoZ  = normAccel.ZAxis;

    dadosLidos.rotacaoX     = normGyro.XAxis;
    dadosLidos.rotacaoY     = normGyro.YAxis;
    dadosLidos.rotacaoZ     = normGyro.ZAxis;
      
    xSemaphoreGive(semaforoDadosProntos);
      
    xSemaphoreGive(semaforoInicializa);

    
    
  }
}

void taskEscrita( void *parameters){

  String message = "Testinho\n";
  File file = SD.open("/valor.txt", FILE_APPEND);

  while(1){

   
    
    xSemaphoreTake(semaforoInicializa,portMAX_DELAY);

    xSemaphoreTake(semaforoDadosProntos,portMAX_DELAY);
    
    xSemaphoreTake(semaforoBotao,portMAX_DELAY);
    File file = SD.open("/valor.txt", FILE_APPEND);
    
    statusAtual = 'g';
    
    char message[15];
    sprintf(message, "%d,",millis());
    file.print(message);
    sprintf(message, "%f,",dadosLidos.altura);
    file.print(message);
    sprintf(message, "%f,",dadosLidos.temperatura);
    file.print(message);
    sprintf(message, "%f,",dadosLidos.aceleracaoX);
    file.print(message);
    sprintf(message, "%f,",dadosLidos.aceleracaoY);
    file.print(message);
    sprintf(message, "%f,",dadosLidos.aceleracaoZ);
    file.print(message);
    sprintf(message, "%f,",dadosLidos.rotacaoX);
    file.print(message);
    sprintf(message, "%f,",dadosLidos.rotacaoY);
    file.print(message);
    sprintf(message, "%f,",dadosLidos.rotacaoZ);
    file.println(message);
    
    file.close();
    xSemaphoreGive(semaforoBotao);
      
    xSemaphoreGive(semaforoDadosProntos);
      
    xSemaphoreGive(semaforoInicializa);

    vTaskDelay( 50 /portTICK_PERIOD_MS);
    
  }
}

void taskBotao( void *parameters){

  xSemaphoreTake(semaforoBotao, portMAX_DELAY);

  while(1){

    if(digitalRead(PINO_BOTAO)){
    xSemaphoreGive(semaforoBotao);
    statusAtual = 'g';

    vTaskDelete( handleBotao);
    }
    
    
  }
}

void taskRecuperacao( void *parameters){

  int alturaMaxima = 0;
  int alturaAtual = 0;

  while(1){

    

    xSemaphoreTake(semaforoInicializa,portMAX_DELAY);

    xSemaphoreTake(semaforoDadosProntos,portMAX_DELAY);
    
    alturaAtual = dadosLidos.altura;
      
    xSemaphoreGive(semaforoDadosProntos);
      
    xSemaphoreGive(semaforoInicializa);

    if(statusAtual == 'g'){
    if(alturaAtual > alturaMaxima)
    alturaMaxima = alturaAtual;

    if(alturaAtual <= alturaMaxima - THRESHOLD_ALTURA)
    abreParaquedas();
    }

    
    
  }
}

void taskStatus( void *parameters){

  char erroSD  = 0;
  char erroBMP = 0;
  char erroMPU = 0;


  Serial.println("Task de status iniciada");
  
  while(1){
      
      //Serial.println("Tentando ler o erro na task de status");

      xSemaphoreTake(semaforoDadosInicia,portMAX_DELAY);
      //Serial.println("Consegui ler, semáforo adquirido");
        
      erroBMP = erroInicializacao.bmp;
      erroMPU = erroInicializacao.mpu;
      erroSD  = erroInicializacao.sd;
      
      xSemaphoreGive(semaforoDadosInicia);
     

    if(erroBMP){
      Serial.println("Erro no BMP!");
      
    }

    if(erroMPU){
      Serial.println("Erro no MPU!");
    }

    if(erroSD){
      Serial.println("Erro no SD!");
    }

    if(!erroSD && !erroBMP && !erroMPU){

    if(statusAtual == 'e'){

      digitalWrite(PINO_LEDG, !digitalRead(PINO_LEDG));
      Serial.println("Status Atual: Espera");
      digitalWrite(PINO_LEDR, LOW);
      
    }

    if(statusAtual == 'g'){

      digitalWrite(PINO_LEDR, !digitalRead(PINO_LEDR));
      Serial.println("Status Atual: Gravando");
      digitalWrite(PINO_LEDG, LOW);
      
    }
    }
    else{
      digitalWrite(PINO_LEDG, !digitalRead(PINO_LEDR));
      digitalWrite(PINO_LEDR, !digitalRead(PINO_LEDG));
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);

    
    
  }
}
 
void abreParaquedas(){

  digitalWrite(PINO_RELE, HIGH);
}

