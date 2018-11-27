#include <Adafruit_BMP085.h>
#include <Wire.h>

SemaphoreHandle_t semaforo = xSemaphoreCreateMutex();
Adafruit_BMP085 bmp;

void setup() {
 
  Serial.begin(115200);
  Wire.begin();
  while(!bmp.begin()) {
        Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
  }
  delay(1000);
  
 
  xTaskCreate(
                    taskOne,          /* Task function. */
                    "TaskOne",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */
 
  xTaskCreate(
                    taskTwo,          /* Task function. */
                    "TaskTwo",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */
  
 
}
 
void loop() {
  delay(1000);
}
 
void taskOne( void * parameter )
{

  

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
      //Serial.println("Entrei na task2 e vou tentar pegar o semáforo");
    if(xSemaphoreTake(semaforo,(TickType_t) 100 ) == pdTRUE ){
        
        Serial.println("Hello from task 2");
        delay(500);
   
    }
    }
    
 
}
