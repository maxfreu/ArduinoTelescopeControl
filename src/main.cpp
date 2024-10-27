#include <Arduino.h>
#include <TMCStepper.h>         // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <SoftwareSerial.h>     // Software serial for the UART to TMC2209 - https://www.arduino.cc/en/Reference/softwareSerial
#include <Streaming.h>          // For serial debugging output - https://www.arduino.cc/reference/en/libraries/streaming/
#include <math.h>
#include <FYSETC_TMC_2209_v4.h>

#define FASTER_BUTTON_PIN 8
#define SLOWER_BUTTON_PIN 9
#define MODE_SWITCH_PIN  10
#define POTI_PIN         A0

// #define SW_SCK           5      // Software Slave Clock (SCK) - BLUE
#define DRIVER_ADDRESS   0b00   // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f           // SilentStepStick series use 0.11 ...and so does my fysetc TMC2209 (?)


SoftwareSerial SoftSerial(SW_RX, SW_TX);                          // Be sure to connect RX to TX and TX to RX between both devices
TMC2209Stepper TMCdriver(&SoftSerial, R_SENSE, DRIVER_ADDRESS);   // Create TMC driver

// const long sidereal_speed = 5/(3600*(86164 / 86400))*48*25600/0.715;
// const long sidereal_speed = 2393;
// with correction after measuring the actual speed at ca 9C temperature
// correction factor was 0.9784650
const long sidereal_speed = 2341;

// const long sidereal_speed = 200000;
const long RA_slow = sidereal_speed * 0.5;
const long RA_fast = sidereal_speed * 1.5;

const long offsets[] = {long(0.1 * sidereal_speed),
                        long(0.3 * sidereal_speed),
                        long(0.5 * sidereal_speed),
                                   sidereal_speed,
                               2 * sidereal_speed,
                              10 * sidereal_speed,
                              50 * sidereal_speed,
                             100 * sidereal_speed};

bool initialized = false;
bool dir = true;
unsigned long last_sg_read_ms = 0;
unsigned long last_speed_button_pressed_ms = 0;

int index = 0;
long offset = sidereal_speed;

// void initialize();

void parse_serial(String command);
long poti_to_speed_offset(int poti_state);
void blink_n_times(int n);

void setup() {
    Serial.setTimeout(500);
    SoftSerial.setTimeout(500);
    Serial.begin(57600);               // initialize hardware serial for debugging
    SoftSerial.begin(115200);           // initialize software serial for UART motor control
    TMCdriver.beginSerial(115200);      // Initialize UART

    // Pin modes
    pinMode(POTI_PIN, INPUT);
    pinMode(SLOWER_BUTTON_PIN, INPUT_PULLUP);
    pinMode(FASTER_BUTTON_PIN, INPUT_PULLUP);
    pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);

    pinMode(EN_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, LOW);

    delay(100);
    TMCdriver.begin();                 // UART: Init SW UART (if selected) with default 115200 baudrate
    delay(100);
    TMCdriver.rms_current(500);        // Set motor RMS current
    TMCdriver.microsteps(256);         // Set microsteps

    TMCdriver.en_spreadCycle(false);
    TMCdriver.pwm_autoscale(true);     // Needed for stealthChop

    delay(200);

    digitalWrite(EN_PIN, LOW);         // Enable TMC2209 board
    TMCdriver.toff(5);                 // Enables driver in software
}


void loop() {
    // read control state
    bool mode = digitalRead(MODE_SWITCH_PIN);
    bool faster_button_pressed = !digitalRead(FASTER_BUTTON_PIN);  // negate because pin is in input_pullup
    bool slower_button_pressed = !digitalRead(SLOWER_BUTTON_PIN);


    if (mode == 0){
        // Computer controlled mode and speed adjustment

        // slew speed adjustment
        if (faster_button_pressed && ((millis() - last_speed_button_pressed_ms) > 500)){
            index = index >= 7 ? 7 : index + 1;
            offset = offsets[index];
            last_speed_button_pressed_ms = millis();
            blink_n_times(index + 1);
        }
        else if (slower_button_pressed && ((millis() - last_speed_button_pressed_ms) > 500)) {
            index = index <= 0 ? 0 : index - 1;
            offset = offsets[index];
            last_speed_button_pressed_ms = millis();
            blink_n_times(index + 1);
        }

        if (Serial.available() > 0) {
            String command = Serial.readStringUntil('#');
            parse_serial(command);
            Serial.readString(); // Clear the buffer
        }
    }
    else {
        // hand controlled mode

        // int poti_state = analogRead(POTI_PIN);
        // long offset = poti_to_speed_offset(poti_state);

        // if (Serial.availableForWrite()){
        //     Serial << "poti: " << poti_state << endl;
        //     Serial << "offset: " << offset << endl;
        // }
        if (faster_button_pressed && slower_button_pressed) {
            // do nothing
        }
        else if (faster_button_pressed)
        {
            long speed = sidereal_speed + offset;
            TMCdriver.shaft(true);
            TMCdriver.VACTUAL(speed);
        }
        else if (slower_button_pressed)
        {
            long speed = sidereal_speed - offset;
            if (speed >= 0) {
                TMCdriver.shaft(true);
            }
            else {
                TMCdriver.shaft(false);
                speed = -speed;
            }
            TMCdriver.VACTUAL(speed);
        }
        else {
            TMCdriver.shaft(true);
            TMCdriver.VACTUAL(sidereal_speed);
        }
    }

    delay(50);
}


void initialize() {
    if (!initialized) {
        digitalWrite(EN_PIN, LOW);
        TMCdriver.VACTUAL(sidereal_speed);
        TMCdriver.shaft(dir);
        initialized = true;
    }
}


void parse_serial(String command) {
    // Remove any leading or trailing whitespace
    // command.trim();

    // Log the received command (optional, for debugging purposes)
    Serial.print("Received command: ");
    Serial.println(command);

    // Check the command and call the respective functions
    if (command == "RA+") {
        TMCdriver.VACTUAL(RA_fast);
    } else if (command == "RA-") {
        TMCdriver.VACTUAL(RA_slow);
    } else if (command == "RA0") {
        TMCdriver.VACTUAL(sidereal_speed);
    } else if (command.startsWith("DEC")) {
        // Do nothing for DEC commands
    } else if (command == "STOP") {
        TMCdriver.VACTUAL(0);
        digitalWrite(EN_PIN, HIGH);
    } else if (command == "START") {
        digitalWrite(EN_PIN, LOW);
        TMCdriver.VACTUAL(sidereal_speed);
    } else {
        // Unknown command
        Serial.println("Unknown command");
    }
}

long poti_to_speed_offset(int poti_state){
    const long max_speed = sidereal_speed * 100;
    const int poti_min = 40;
    const int poti_max = 632;
    // guard bounds
    poti_state = poti_state < poti_min ? poti_min : poti_state;
    poti_state = poti_state > poti_max ? poti_max : poti_state;
    // map poti input to interval 0..1
    float state = float(poti_state - poti_min) / float(poti_max - poti_min);
    return long(state * state * state * state * state * max_speed);
}

void blink_n_times(int n){
    for(int i = 0; i<n; i++){
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }
}
