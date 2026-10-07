#ifndef DATA_LOG_H
#define DATA_LOG_H

#include <stdint.h>
#include <SPI.h>
#include <SD.h>
#include "FC_config.h"

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
    float esc1;
    float esc2;
    float esc3;
    float esc4;
    float esc5;
    float esc6;
    uint32_t timestampMs;
} DataLog_t;

// FreeRTOS Queue Handle
extern QueueHandle_t telemetryQueue;

void SDcardSetup();
void logData(const DataLog_t data);
void startLoggingTask(void* pvParameters);
void startLoggingTask(void* pvParameters);


#endif