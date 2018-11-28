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
//Definições referentes a configurações

//Incluindo as bibliotecas referentes ao BMP180, SD e IMU respectivamente
#include <Adafruit_BMP085.h>
#include <Wire.h>

#include "FS.h"
#include "SD.h"
#include "SPI.h"

//Definição de variáveis referentes ao RTOS
SemaphoreHandle_t semaforo = xSemaphoreCreateMutex();

//Definição de variáveis referente ao módulos
Adafruit_BMP085 bmp;

uint8_t cardType;



void setup() {
 
  Serial.begin(115200);
  Wire.begin();
  
  while(!SD.begin()){
        Serial.println("Card Mount Failed");
        delay(500);
  }
        
  while(!bmp.begin()) {
        Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
        delay(500);
  }

  cardType = SD.cardType();

  if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.println("Inciando Tasks");

  
 
  xTaskCreate(
                    taskOne,          /* Task function. */
                    "TaskOne",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */
    Serial.println("Inciou Task 1");
 
  xTaskCreate(
                    taskTwo,          /* Task function. */
                    "TaskTwo",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */
    Serial.println("Inciou Task 2");
                    
  xTaskCreate(
                    taskEscrita,          /* Task function. */
                    "taskEscrita",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */
    Serial.println("Inciou Task 3");
  
 
}
 
void loop() {
  delay(1000);
}
 
void taskOne( void * parameter )
{

      Serial.println("Entrou na Task 1");

  

  while(1){

    xSemaphoreTake(semaforo ,(TickType_t) 100 );
    for( int i = 0;i<10;i++ ){
        float pressure = bmp.readAltitude(101500);
        Serial.print ("Valor de Altura = ");
        Serial.println (pressure);

        if(i == 9){
        Serial.println ("Entregando o semáforo");
        xSemaphoreGive(semaforo);
        }
        delay(500);
    }
  }
 
}
 
void taskTwo( void * parameter)
{
  
    while(1){
      Serial.println("Entrou na Task 2");
      //Serial.println("Entrei na task2 e vou tentar pegar o semáforo");
    if(xSemaphoreTake(semaforo,(TickType_t) 100 ) == pdTRUE ){
        
        Serial.println("Hello from task 2");
        delay(500);
   
    }
    }
    
 
}

void taskEscrita( void * parameter){

  Serial.println("Entrou na Task 3");
    File file = SD.open("/valor.txt", FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
    }
    if(file.print("olar")){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();

}


