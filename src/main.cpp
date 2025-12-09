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

#include <Arduino_FreeRTOS.h>
#include <Arduino.h>
#include <timers.h>
#include <semphr.h>
#include <queue.h>

#include <LiquidCrystal.h>
#include <Ultrasonic.h>
#include <Buzzer.hpp>
#include <debug.hpp>
#include <Keypad.h>
#include <Servo.h>
#include <FSM.hpp>

/* ------------------------ Macros and Declarations ------------------------ */

#define NOT_USED(x) (void)(x)

#define TILT_SWITCH_PIN 8
#define LASER_PIN 13
#define BUZZER_PIN 5
#define SERVO_PIN 9
#define LDR_PIN A0
#define LED_PIN 6

#define ULTRASONIC_TIMEOUT 3000
#define ULTRASONIC_TRIGGER 12
#define ULTRASONIC_ECHO 11

#define KEYPAD_DEBOUNCE_MS 100
#define KEYPAD_COLS 4
#define KEYPAD_ROWS 4

#define LCD_D4_PIN 51
#define LCD_D5_PIN 50
#define LCD_D6_PIN 49
#define LCD_D7_PIN 48
#define LCD_RS_PIN 53
#define LCD_EN_PIN 52
#define LCD_COLS 16
#define LCD_ROWS 2

#define ULTRASONIC_THRESHOLD_CM 4
#define LDR_THRESHOLD 850

#define ULTRASONIC_READ_FREQ (TickType_t)(60)
#define KEYPAD_TASK_FREQ (TickType_t)(2)
#define FSM_TASK_FREQ (TickType_t)(5)

#define DISPLAY_BUFFER_LENGTH (LCD_COLS*LCD_ROWS)
#define DISPLAY_QUEUE_LENGTH 8
#define KEYPAD_QUEUE_LENGTH 64 

#define PASSWORD_LENGTH LCD_COLS
#define PASSWORD_BUFFER_LENGTH (PASSWORD_LENGTH+1)
#define PASSWORD_WAITING_TIME_MS 5000
#define PASSWORD_MAX_ATTEMPTS 3

typedef char displaybuff_t[DISPLAY_BUFFER_LENGTH];

/* ------------------------------- Components ------------------------------ */

LiquidCrystal display(
    LCD_RS_PIN,
    LCD_EN_PIN,
    LCD_D4_PIN,
    LCD_D5_PIN,
    LCD_D6_PIN,
    LCD_D7_PIN
);

char keys[KEYPAD_ROWS][KEYPAD_COLS] = {
	{'1','2','3','A'},
	{'4','5','6','B'},
	{'7','8','9','C'},
	{'*','0','#','D'}
};

byte rowPins[KEYPAD_ROWS] = {29, 28, 27, 26};
byte colPins[KEYPAD_COLS] = {25, 24, 23, 22};

Keypad keypad = Keypad(
    makeKeymap(keys),
    rowPins, colPins,
    KEYPAD_ROWS,
    KEYPAD_COLS
);

Ultrasonic ultrasound(ULTRASONIC_TRIGGER, ULTRASONIC_ECHO);

Servo servo;

/* ----------------------------- Global Context ---------------------------- */

const Buzzer::Note_t siren_melody[] = {
    Buzzer::NOTE_G4, Buzzer::NOTE_G5,
    Buzzer::NOTE_G4, Buzzer::NOTE_G5,
    Buzzer::NOTE_G4, Buzzer::NOTE_G5,
    Buzzer::NOTE_G4, Buzzer::NOTE_G5,
    Buzzer::NOTE_G4, Buzzer::NOTE_G5,
    Buzzer::NOTE_G4, Buzzer::NOTE_G5
};

const uint16_t melody_size = sizeof(siren_melody) / sizeof(Buzzer::Note_t);

QueueHandle_t xKeysQueue = NULL;
QueueHandle_t xDisplayQueue = NULL;

boolean isSecurityActivated = false;
boolean isAlarmActivated = false;

const char masterpass[PASSWORD_BUFFER_LENGTH] = "8086";

/* ------------------------------- Functions ------------------------------- */

void vWriteToDisplay(const displaybuff_t buffer) {
    for(uint8_t i = 0; i < DISPLAY_BUFFER_LENGTH; i++) {
        if(i % LCD_COLS == 0) {
            display.setCursor(0, i / LCD_COLS % LCD_ROWS);
        }
        
        display.print(buffer[i]);
    }
}

void vBufferWriteCenter(
    displaybuff_t buffer,
    const char *text,
    size_t row
) {
    if(text == NULL) {
        LOG("SERIAL ERROR: NULL pointer passed to vBufferWriteCenter() function");
        return;
    }

    size_t offset = row * LCD_COLS;
    
    if(offset >= sizeof(displaybuff_t)) {
        LOG("SERIAL ERROR: Offset exceeds the maximum buffer size");
        return;
    }
    
    size_t slice_size = sizeof(displaybuff_t) - offset;
    
    if(slice_size < LCD_COLS) {
        LOG("SERIAL ERROR: Buffer slice width is insufficient for LCD row");
        return;
    }
    
    size_t text_size = strlen(text);

    if(text_size > LCD_COLS) {
        LOG("SERIAL ERROR: Text exceeds the maximum row size for centering");
        return;
    }

    size_t left_padding = (LCD_COLS - text_size) / 2;

    memset(&buffer[offset], ' ', LCD_COLS);
    memcpy(&buffer[offset + left_padding], text, text_size);
}

void vActivateSecurity() {
    servo.write(190);
    digitalWrite(LASER_PIN, HIGH);
    digitalWrite(LED_PIN, LOW);

    isSecurityActivated = true;
}

void vDeactivateSecurity() {
    servo.write(0);
    digitalWrite(LASER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);

    isSecurityActivated = false;
}

void vActivateAlarm() {
    Buzzer::vPlay(siren_melody, melody_size, 600);
    digitalWrite(LED_PIN, HIGH);

    isAlarmActivated = true;
}

void vDeactivateAlarm() {
    digitalWrite(LED_PIN, LOW);
    Buzzer::vStop();

    isAlarmActivated = false;
}

boolean xVerifySensors() {
    static TickType_t xLastReadTime = xTaskGetTickCount();
    unsigned int distance;

    if(xTaskGetTickCount() - xLastReadTime >= ULTRASONIC_READ_FREQ) {
        vTaskSuspendAll();
        distance = ultrasound.read(CM);
        xTaskResumeAll();

        if(distance <= ULTRASONIC_THRESHOLD_CM) {
            LOG("SERIAL LOG: ultrasonic sensor detected an anomaly");
            return true;
        }

        xLastReadTime = xTaskGetTickCount();
    }

    if(analogRead(LDR_PIN) <= LDR_THRESHOLD) {
        LOG("SERIAL LOG: laser sensor detected an anomaly");
        return true;
    }

    if(digitalRead(TILT_SWITCH_PIN)) {
        LOG("SERIAL LOG: tilt sensor detected an anomaly");
        return true; 
    }

    return false;
}

/* --------------------------------- Tasks --------------------------------- */

void vTaskKeypad(void *pvParameters) {
    NOT_USED(pvParameters);

    static TickType_t xLastWakeTime = xTaskGetTickCount();
    static char key;

    while(true) {
        key = keypad.getKey();
    
        if(key != NO_KEY) {
            xQueueSend(
                xKeysQueue,
                (void *)&key,
                portMAX_DELAY
            );
        }

        xTaskDelayUntil(&xLastWakeTime, KEYPAD_TASK_FREQ);
    }
}

void vTaskFSM(void *pvParameters) {
    NOT_USED(pvParameters);

    /* Time related variables */
    static TickType_t xLastWakeTime = xTaskGetTickCount();
    static TickType_t xLastWaitTime;
    static TickType_t xWaitTime;

    /* Password related variables */
    static char censoredpass[PASSWORD_BUFFER_LENGTH];
    static char password[PASSWORD_BUFFER_LENGTH];
    static boolean isVisible;
    static uint8_t passindex;
    static uint8_t attempts;

    /* FSM related variables */
    static state_t current_state = SECURITY_DEACTIVATED;
    static boolean isStateChange = true;

    /* User Interface related variables */
    static displaybuff_t buffer;
    static char key = NO_KEY;
    
    while(true) {
        BaseType_t xIsReceived = xQueueReceive(xKeysQueue, (void *)&key, 0);
        
        if(!xIsReceived) {
            key = NO_KEY;
        }
        
        switch(current_state) {
        case SECURITY_DEACTIVATED:
            if(isStateChange) {
                vDeactivateSecurity();

                if(isAlarmActivated) {
                    vDeactivateAlarm();
                }

                vBufferWriteCenter(buffer, "SYS. DEACTIVATED", 0);
                vBufferWriteCenter(buffer, "USE # TO ENABLE", 1);

                xQueueSend(
                    xDisplayQueue,
                    (void *)&buffer,
                    portMAX_DELAY
                );

                isStateChange = false;
            }

            if(key == '#') {
                current_state = SECURITY_ACTIVATION;
                isStateChange = true;
            }

            break;
        case SECURITY_ACTIVATION:
            if(isStateChange) {
                censoredpass[0] = '\0';
                password[0] = '\0';
                isVisible = false;
                passindex = 0;
                attempts = 0;

                vBufferWriteCenter(buffer, "ACTIV. PASSWORD", 0);
                vBufferWriteCenter(buffer, "", 1);

                xQueueSend(
                    xDisplayQueue,
                    (void *)&buffer,
                    portMAX_DELAY
                );

                isStateChange = false;
            }

            if(key == '*') {
                current_state = SECURITY_DEACTIVATED;
                isStateChange = true;
            } else if(key == '#') {
                attempts++;

                if(strcmp(password, masterpass) == 0) {
                    current_state = SECURITY_ACTIVATED;
                    isStateChange = true;
                } else if(attempts >= PASSWORD_MAX_ATTEMPTS) {
                    current_state = ATTEMPTS_BLOCKED;
                    isStateChange = true;
                } else {
                    vBufferWriteCenter(buffer, "Wrong Password", 1);

                    xQueueSend(
                        xDisplayQueue,
                        (void *)&buffer,
                        portMAX_DELAY
                    );

                    isVisible = false;
                }
            } else if(key >= '0' && key <= '9') {
                if(passindex >= PASSWORD_LENGTH) {
                    break;
                }

                censoredpass[passindex] = '*';
                password[passindex] = key;
                passindex++;

                censoredpass[passindex] = '\0';
                password[passindex] = '\0';

                if(isVisible) {
                    vBufferWriteCenter(buffer, password, 1);
                } else {
                    vBufferWriteCenter(buffer, censoredpass, 1);
                }

                xQueueSend(
                    xDisplayQueue,
                    (void *)&buffer,
                    portMAX_DELAY
                );
            } else if(key == 'C') {
                if(passindex == 0) {
                    break;
                }

                passindex--;
                censoredpass[passindex] = '\0';
                password[passindex] = '\0';

                if(isVisible) {
                    vBufferWriteCenter(buffer, password, 1);
                } else {
                    vBufferWriteCenter(buffer, censoredpass, 1);
                }

                xQueueSend(
                    xDisplayQueue,
                    (void *)&buffer,
                    portMAX_DELAY
                );
            } else if(key == 'D') {
                isVisible = !isVisible;

                if(isVisible) {
                    vBufferWriteCenter(buffer, password, 1);
                } else {
                    vBufferWriteCenter(buffer, censoredpass, 1);
                }

                xQueueSend(
                    xDisplayQueue,
                    (void *)&buffer,
                    portMAX_DELAY
                );
            }
    
            break;
        case SECURITY_ACTIVATED:
            if(isStateChange) {
                vActivateSecurity();

                vBufferWriteCenter(buffer, "SYS. ACTIVATED", 0);
                vBufferWriteCenter(buffer, "USE # TO DISABLE", 1);

                xQueueSend(
                    xDisplayQueue,
                    (void *)&buffer,
                    portMAX_DELAY
                );

                isStateChange = false;
            }

            if(key == '#') {
                current_state = SECURITY_DEACTIVATION;
                isStateChange = true;
            } else if(xVerifySensors()) {
                current_state = SECURITY_DEACTIVATION;
                isStateChange = true;
                vActivateAlarm();
            }

            break;
        case SECURITY_DEACTIVATION:
            if(isStateChange) {
                censoredpass[0] = '\0';
                password[0] = '\0';
                isVisible = false;
                passindex = 0;
                attempts = 0;

                vBufferWriteCenter(buffer, "DEACT. PASSWORD", 0);
                vBufferWriteCenter(buffer, "", 1);

                xQueueSend(
                    xDisplayQueue,
                    (void *)&buffer,
                    portMAX_DELAY
                );

                isStateChange = false;
            }

            if(key == '#') {
                attempts++;

                if(strcmp(password, masterpass) == 0) {
                    current_state = SECURITY_DEACTIVATED;
                    isStateChange = true;
                } else if(attempts >= PASSWORD_MAX_ATTEMPTS) {
                    current_state = BLOCKED;
                    isStateChange = true;
                } else {
                    vBufferWriteCenter(buffer, "Wrong Password", 1);

                    xQueueSend(
                        xDisplayQueue,
                        (void *)&buffer,
                        portMAX_DELAY
                    );

                    isVisible = false;
                }
            } else if(key >= '0' && key <= '9') {
                if(passindex >= PASSWORD_LENGTH) {
                    break;
                }

                censoredpass[passindex] = '*';
                password[passindex] = key;
                passindex++;

                censoredpass[passindex] = '\0';
                password[passindex] = '\0';

                if(isVisible) {
                    vBufferWriteCenter(buffer, password, 1);
                } else {
                    vBufferWriteCenter(buffer, censoredpass, 1);
                }

                xQueueSend(
                    xDisplayQueue,
                    (void *)&buffer,
                    portMAX_DELAY
                );
            } else if(key == 'C') {
                if(passindex == 0) {
                    break;
                }

                passindex--;
                censoredpass[passindex] = '\0';
                password[passindex] = '\0';

                if(isVisible) {
                    vBufferWriteCenter(buffer, password, 1);
                } else {
                    vBufferWriteCenter(buffer, censoredpass, 1);
                }

                xQueueSend(
                    xDisplayQueue,
                    (void *)&buffer,
                    portMAX_DELAY
                );
            } else if(key == 'D') {
                isVisible = !isVisible;

                if(isVisible) {
                    vBufferWriteCenter(buffer, password, 1);
                } else {
                    vBufferWriteCenter(buffer, censoredpass, 1);
                }

                xQueueSend(
                    xDisplayQueue,
                    (void *)&buffer,
                    portMAX_DELAY
                );
            }

            break;
        case ATTEMPTS_BLOCKED:
            if(isStateChange) {
                vBufferWriteCenter(buffer, "ATTEMPTS BLOCKED", 0);
                vBufferWriteCenter(buffer, "PLEASE WAIT", 1);

                xQueueSend(
                    xDisplayQueue,
                    (void *)&buffer,
                    portMAX_DELAY
                );

                xWaitTime = pdMS_TO_TICKS(PASSWORD_WAITING_TIME_MS);
                xLastWaitTime = xTaskGetTickCount();

                isStateChange = false;
            }


            if(xTaskGetTickCount() - xLastWaitTime >= xWaitTime) {
                current_state = SECURITY_DEACTIVATED;
                isStateChange = true;
            }

            break;
        case BLOCKED:
            if(isStateChange) {
                if(!isAlarmActivated) {
                    vActivateAlarm();
                }

                vBufferWriteCenter(buffer, "SYSTEM BLOCKED", 0);
                vBufferWriteCenter(buffer, "<INTRUSION>", 1);

                xQueueSend(
                    xDisplayQueue,
                    (void *)&buffer,
                    portMAX_DELAY
                );

                isStateChange = true;
            }

            break;
        }

        xTaskDelayUntil(&xLastWakeTime, FSM_TASK_FREQ);
    }
}

void vTaskRefreshDisplay(void *pvParameters) {
    NOT_USED(pvParameters);

    static displaybuff_t buffer;

    while(true) {
        if(xQueueReceive(xDisplayQueue, (void *)&buffer, portMAX_DELAY)) {
            vWriteToDisplay(buffer);
        }
    }
}

/* --------------------------------- Setup --------------------------------- */

void setup() {
    Serial.begin(9600);

    display.begin(LCD_COLS, LCD_ROWS);
    display.setCursor(0, 0);
    display.noCursor();

    Buzzer::vBegin(BUZZER_PIN);
    servo.attach(SERVO_PIN);

    pinMode(TILT_SWITCH_PIN, INPUT);
    pinMode(LASER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(LDR_PIN, INPUT);

    keypad.setDebounceTime(KEYPAD_DEBOUNCE_MS);
    ultrasound.setTimeout(ULTRASONIC_TIMEOUT);

    xKeysQueue = xQueueCreate(
        KEYPAD_QUEUE_LENGTH,
        sizeof(char)
    );

    if(xKeysQueue == NULL) {
        LOG("SERIAL ERROR: unable to create 'xKeysQueue' on setup() function");
        while(true);  
    }

    xDisplayQueue = xQueueCreate(
        DISPLAY_QUEUE_LENGTH,
        sizeof(displaybuff_t)
    );

    if(xDisplayQueue == NULL) {
        LOG("SERIAL ERROR: unable to create 'xDisplayQueue' on setup() function");
        while(true);  
    }
    
    xTaskCreate(
        vTaskKeypad,
        "vTaskKeypad",
        196,
        NULL,
        3,
        NULL
    );

    xTaskCreate(
        vTaskFSM,
        "vTaskFSM",
        256,
        NULL,
        2,
        NULL
    );

    xTaskCreate(
        vTaskRefreshDisplay,
        "vTaskRefreshDisplay",
        196,
        NULL,
        1,
        NULL
    );
}

void loop() {
}