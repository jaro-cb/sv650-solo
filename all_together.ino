#include <Arduino.h>

#include "sds.h"
#include "solo.h"
#include "state.h"

QueueHandle_t msgQueue;

void publisherTask(void *pvParameters) {
  EcuSnapshot ecu;

  while(1) {
    bool read = sdsRead(&ecu);
    if (!read) {
      continue;
    }
    BaseType_t status = xQueueOverwrite(msgQueue, &ecu);
    
    if (status == pdPASS) {
      Serial.print("[Publisher] Sent");
    } else {
      Serial.println("[Publisher] Queue full! Dropped message.");
    }
  }
}

void subscriberTask(void *pvParameters) {
  EcuSnapshot ecu;
  while(1) {
    if (xQueueReceive(msgQueue, &ecu, portMAX_DELAY) == pdPASS) {
      soloSend(&ecu);
      Serial.print("[Subscriber] Received");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  
  msgQueue = xQueueCreate(1, sizeof(EcuSnapshot));

  sdsInit();
  soloInit();

  xTaskCreatePinnedToCore(
    publisherTask,     // Function to implement the task
    "SDS_Publisher",        // Name of task
    2048,               // Stack size in words
    NULL,               // Task input parameter
    1,                  // Priority of the task
    NULL,               // Task handle
    0                   // Core ID
  );

  // Create the Subscriber Task on Core 1
  xTaskCreatePinnedToCore(
    subscriberTask,    // Function to implement the task
    "SOLO_Subscriber",       // Name of task
    2048,               // Stack size in words
    NULL,               // Task input parameter
    1,                  // Priority of the task
    NULL,               // Task handle
    1                   // Core ID
  );
}

// ------------------------------------------------------------------
void loop() {
  vTaskDelete(NULL); 
  // if (!g_connected) { 
  //   delay(1000); 
  //   return; 
  // }
  // sdsRead();
  // soloSend();

 
  //delay(1000);
}
