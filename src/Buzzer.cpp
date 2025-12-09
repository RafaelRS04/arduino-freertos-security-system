/***********************************************************************************

    MIT License

    Copyright (c) 2025 Rafael Rodrigues Sanches

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.

***********************************************************************************/

#include <Buzzer.hpp>
#include <debug.hpp>

#include <Arduino_FreeRTOS.h>
#include <timers.h>
#include <semphr.h>

#define MS_PER_MINUTE 60000

/* ---------------------------- Buzzer context ----------------------------- */

static SemaphoreHandle_t xMutex = NULL;
static TimerHandle_t xTimer = NULL;
static uint8_t ucBuzzerPin = 0;

static const Buzzer::Note_t *eCurrentNotes = NULL;
static uint16_t usCurrentSize = 0;
static uint16_t usIndex = 0;

/* ----------------------------- Buzzer Handler ---------------------------- */

static void prvBuzzerHandler(TimerHandle_t xTimer) {
    if(eCurrentNotes) {

        noTone(ucBuzzerPin);

        if(usIndex == usCurrentSize) {
            usIndex = 0;
        }

        Buzzer::Note_t eNote = eCurrentNotes[usIndex];

        if(eNote != Buzzer::NO_TONE) {
            tone(ucBuzzerPin, eNote);
        }

        usIndex++;
    }
}

/* ------------------------ Buzzer public interface ------------------------ */

void Buzzer::vBegin(uint8_t ucTonePin) {
    ucBuzzerPin = ucTonePin;
    pinMode(ucTonePin, OUTPUT);

    xMutex = xSemaphoreCreateMutex();

    if(xMutex == NULL) {
        LOG("SERIAL ERROR: unable to create Mutex on Buzzer::vBegin() function");
    }

    xTimer = xTimerCreate(
        "BuzzerTimer",
        portMAX_DELAY,
        pdTRUE,
        NULL,
        prvBuzzerHandler
    );

    if(xTimer == NULL) {
        LOG("SERIAL ERROR: unable to create Timer on Buzzer::vBegin() function");
    }
}

void Buzzer::vPlay(const Buzzer::Note_t *eNotes, uint16_t usSize, uint16_t usBpm) {
    if(eNotes == NULL) {
        LOG("SERIAL ERROR: NULL pointer passed to Buzzer::vPlay() function");
        return;
    }

    if(usSize == 0) {
        LOG("SERIAL ERROR: zero size passed to Buzzer::vPlay() function");
        return;
    }

    if(usBpm == 0) {
        LOG("SERIAL ERROR: zero bpm passed to Buzzer::vBegin() function");
        return;
    }

    uint16_t usNoteDuration = MS_PER_MINUTE / usBpm;

    if(usNoteDuration == 0) {
        usNoteDuration = 1;
    }

    xSemaphoreTake(xMutex, portMAX_DELAY);

    if(xTimerIsTimerActive(xTimer)) {
        xTimerStop(xTimer, portMAX_DELAY);
    }
    
    eCurrentNotes = eNotes;
    usCurrentSize = usSize;
    usIndex = 0;
    
    xTimerChangePeriod(
        xTimer,
        pdMS_TO_TICKS(usNoteDuration),
        portMAX_DELAY
    );

    xTimerReset(xTimer, portMAX_DELAY);

    xSemaphoreGive(xMutex);
}

void Buzzer::vPause() {
    xSemaphoreTake(xMutex, portMAX_DELAY);

    if(xTimerIsTimerActive(xTimer)) {
        xTimerStop(xTimer, portMAX_DELAY);
    }

    noTone(ucBuzzerPin);

    xSemaphoreGive(xMutex);
}

void Buzzer::vResume() {
    if(eCurrentNotes == NULL) {
        LOG("SERIAL ERROR: invalid call to Buzzer::vResume() function, current notes points to NULL");
        return; 
    }
    
    xSemaphoreTake(xMutex, portMAX_DELAY);

    xTimerReset(xTimer, portMAX_DELAY);
    
    xSemaphoreGive(xMutex);
}

void Buzzer::vStop() {
    xSemaphoreTake(xMutex, portMAX_DELAY);

    if(xTimerIsTimerActive(xTimer)) {
        xTimerStop(xTimer, portMAX_DELAY);
    }

    noTone(ucBuzzerPin);

    eCurrentNotes = NULL;
    usCurrentSize = 0;
    usIndex = 0;

    xSemaphoreGive(xMutex);
}