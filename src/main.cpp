#include <TMCStepper.h>         // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <SoftwareSerial.h>     // Software serial for the UART to TMC2209 - https://www.arduino.cc/en/Reference/softwareSerial
#include <Streaming.h>          // For serial debugging output - https://www.arduino.cc/reference/en/libraries/streaming/

#define EN_PIN           2      // Enable - PURPLE
#define DIR_PIN          9      // Direction - WHITE
#define STEP_PIN         8      // Step - ORANGE
// #define SW_SCK           5      // Software Slave Clock (SCK) - BLUE
#define SW_TX            6      // SoftwareSerial receive pin - BROWN
#define SW_RX            5      // SoftwareSerial transmit pin - YELLOW
#define DRIVER_ADDRESS   0b00   // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f           // SilentStepStick series use 0.11 ...and so does my fysetc TMC2209 (?)


SoftwareSerial SoftSerial(SW_RX, SW_TX);                          // Be sure to connect RX to TX and TX to RX between both devices
TMC2209Stepper TMCdriver(&SoftSerial, R_SENSE, DRIVER_ADDRESS);   // Create TMC driver

bool initialized = false;
bool dir = true;
// long sidereal_speed = 2387;
// const long sideral_speed = 5/(3600*(86164 / 86400))*48*25600/0.715;
const long sidereal_speed = 200000;
const long RA_slow = sidereal_speed * 0.5;
const long RA_fast = sidereal_speed * 1.5;
unsigned long last_sg_read_ms = 0;


// void initialize();

void parse_serial(String command);

void setup() {
    Serial.begin(57600);               // initialize hardware serial for debugging
    SoftSerial.begin(115200);           // initialize software serial for UART motor control
    TMCdriver.beginSerial(115200);      // Initialize UART

    pinMode(EN_PIN, OUTPUT);           // Set pinmodes
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);         // Enable TMC2209 board  

    delay(10);
    TMCdriver.begin();                 // UART: Init SW UART (if selected) with default 115200 baudrate
    delay(10);
    TMCdriver.toff(5);                 // Enables driver in software
    TMCdriver.rms_current(500);        // Set motor RMS current
    TMCdriver.microsteps(256);         // Set microsteps

    TMCdriver.en_spreadCycle(false);
    TMCdriver.pwm_autoscale(true);     // Needed for stealthChop
    
    // TMCdriver.VACTUAL(0);
    // TMCdriver.shaft(dir);              // SET DIRECTION
    // delay(200);
    // TMCdriver.VACTUAL(0);
    // TMCdriver.shaft(dir);              // SET DIRECTION
}


void loop() {
    // initialize();
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('#');
        parse_serial(command);
        Serial.readString(); // Clear the buffer
    }

    if (Serial.availableForWrite() >= 16 && millis() - last_sg_read_ms > 1000) {
        Serial << "OFS: " << TMCdriver.pwm_ofs_auto() << endl;
        uint16_t stallguard_result = TMCdriver.SG_RESULT();
        char str[16];
        sprintf(str, "SG %d", stallguard_result);
        Serial.println(str);
        last_sg_read_ms = millis();
    }

    delay(10);
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