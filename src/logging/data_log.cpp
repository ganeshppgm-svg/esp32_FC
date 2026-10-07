#include "data_log.h"

File logFile;
QueueHandle_t telemetryQueue;

void SDcardSetup(){
// Initialize SPI and SD Card
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD Card Mount Failed");
        return;
    }

    // Create file and headers if missing
    if (!SD.exists("/datalog.csv")) {
        File initFile = SD.open("/datalog.csv", FILE_WRITE);
        if (initFile) {
            initFile.println("roll,pitch,yaw,throttle,esc1,esc2,esc3,esc4,esc5,esc6");
            initFile.close();
        }
    }

    // Open file in append mode for continuous logging
    logFile = SD.open("/datalog.csv", FILE_APPEND);
    if (!logFile) {
        Serial.println("Failed to open datalog.csv for appending");
    }


}

void logData(const DataLog_t data){
    if (logFile) {

        DataLog_t telemetry = data; 
        // Write formatted CSV line
        logFile.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                       telemetry.roll, telemetry.pitch, telemetry.yaw, telemetry.throttle,
                       telemetry.esc1, telemetry.esc2, telemetry.esc3, telemetry.esc4, 
                       telemetry.esc5, telemetry.esc6);
        
        // Periodically flush buffer to ensure data is not erased when power off
        static unsigned long lastFlush = 0;
        if (millis() - lastFlush > 1000) {
            logFile.flush();
            lastFlush = millis();
        }
    }
}

// Logging Task pinned to Core 0
void sdLoggingTask(void *pvParameters) {
    DataLog_t receivedData;
    
    while (1) {
        // Block indefinitely until data is available in the queue
        if (xQueueReceive(telemetryQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
            logData(receivedData);
        }
    }
}


void startLoggingTask(void* pvParameters){
         // Create a queue holding up to 50 items of DataLog_t
  telemetryQueue = xQueueCreate(50, sizeof(DataLog_t));

  xTaskCreatePinnedToCore(
        sdLoggingTask,   // Task function
        "SD_Log_Task",   // Name
        4096,            // Stack size (bytes)
        NULL,            // Parameters
        1,               // Priority (low priority so it doesn't starve control tasks)
        NULL,            // Task handle
        0                // Core ID (0)
    );

}

