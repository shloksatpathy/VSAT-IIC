// CanSat flight firmware draft for the rev 1.0 board (ESP32-S3-DevKitC-1-N16R8).
// Pins follow the board schematic (EasyEDA Sheet_1, 2026-10-05).
//
// Brakes: filtered roll (MPU-9250) plus GNSS steering toward the reference
// location, same as brake_test. Sensors are logged to SD at 10 Hz and sent
// over LoRa at 1 Hz.
//
// Reference location: set TARGET_LAT/TARGET_LON, or send "lat,lon" or "here"
// over USB Serial or LoRa.
//
// Init order: servos (brakes neutral), I2C sensors (IMU, barometer, INA260),
// UV, GNSS, LoRa, SD card last. Then IMU/barometer calibration.
//
// LoRa link: the ground station sends "CONNECT". The CanSat replies with an
// ACK packet carrying the init status, and only then starts LoRa telemetry.
// Sending "CONNECT" again repeats the ACK. After that LoRa keeps listening for:
//   START        send telemetry on LoRa (default)
//   STOP         standby: no telemetry; brakes and SD logging keep running
//   RECALIBRATE  brakes neutral, recalibrate IMU and barometer, back to the previous mode
// Each command is answered with "ACK,<command>".
//
// Keep the board still and level during startup calibration.

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <ESP32Servo.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_INA260.h>

// I2C: MPU-9250, BMP390L, MS5611, INA260
constexpr uint8_t PIN_SDA = 8;
constexpr uint8_t PIN_SCL = 9;

// GNSS (L89HA) on Serial1
constexpr uint8_t  PIN_GNSS_RX = 5;  // ESP RX <- GNSS TX
constexpr uint8_t  PIN_GNSS_TX = 4;  // ESP TX -> GNSS RX
constexpr uint32_t GNSS_BAUD   = 115200;

// LoRa (E220-900T30D) on Serial2, M0/M1 tied to GND (normal mode)
constexpr uint8_t  PIN_LORA_RX = 18;  // ESP RX <- E220 TXD
constexpr uint8_t  PIN_LORA_TX = 17;  // ESP TX -> E220 RXD
constexpr uint32_t LORA_BAUD   = 9600;

// Micro SD (SPI)
constexpr uint8_t PIN_SD_CS   = 6;
constexpr uint8_t PIN_SD_MOSI = 11;
constexpr uint8_t PIN_SD_MISO = 13;
constexpr uint8_t PIN_SD_SCK  = 12;

constexpr uint8_t PIN_UV     = 7;  // ML8511 OUT (analog)
constexpr uint8_t PIN_BUZZER = 2;

// Servo headers: S1 = GPIO 1, S2 = GPIO 38, S3 = GPIO 21, S4 = GPIO 48, S5 = GPIO 47
constexpr uint8_t  PIN_LEFT_BRAKE         = 1;   // S1
constexpr uint16_t LEFT_BRAKE_NEUTRAL_US  = 1100;
constexpr uint16_t LEFT_BRAKE_FULL_US     = 1900;
constexpr uint8_t  PIN_RIGHT_BRAKE        = 38;  // S2
constexpr uint16_t RIGHT_BRAKE_NEUTRAL_US = 1900;
constexpr uint16_t RIGHT_BRAKE_FULL_US    = 1100;

// I2C addresses
constexpr uint8_t IMU_I2C_ADDR  = 0x68;
constexpr uint8_t BARO_I2C_ADDR = 0x77;  // BMP390L
constexpr uint8_t INA_I2C_ADDR  = 0x40;

// MPU registers
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr float   GYRO_LSB_PER_DPS = 65.5f;  // +/-500 deg/s

// Reference location (0,0 = not set)
constexpr double TARGET_LAT = 0.0;
constexpr double TARGET_LON = 0.0;

// Tuning
constexpr float    FILTER_ALPHA        = 0.98f;  // complementary filter: gyro weight (0..1)
constexpr float    DEADBAND_DEG        = 2.0f;   // roll ignored inside +/- this
constexpr float    FULL_BRAKE_ROLL_DEG = 45.0f;  // roll (past the deadband) at which a brake is fully pulled
constexpr float    STEER_KP            = 0.02f;  // brake fraction per degree of heading error
constexpr float    ARRIVE_RADIUS_M     = 3.0f;   // no steering inside this distance
constexpr float    MIN_SPEED_MPS       = 0.5f;   // GNSS course is unreliable below this
constexpr uint8_t  GNSS_MIN_SATS       = 4;
constexpr uint32_t GNSS_MAX_AGE_MS     = 2000;
constexpr uint16_t CALIB_SAMPLES       = 500;

// Rates
constexpr uint32_t LOOP_PERIOD_US      = 10000;  // 100 Hz control
constexpr uint32_t LOG_PERIOD_MS       = 100;    // 10 Hz sensors + SD
constexpr uint32_t TELEMETRY_PERIOD_MS = 1000;   // 1 Hz LoRa

HardwareSerial &gnssSerial = Serial1;
HardwareSerial &loraSerial = Serial2;

TinyGPSPlus     gps;
Adafruit_BMP3XX bmp;
Adafruit_INA260 ina;
Servo           leftBrake;
Servo           rightBrake;
File            logFile;

bool imuOk, baroOk, inaOk, sdOk;
bool loraConnected = false;

enum class Mode { Start, Stop };
Mode mode = Mode::Start;

float gyroBiasDps = 0.0f;
float rollTrimDeg = 0.0f;
float rollDeg     = 0.0f;
float groundHpa   = 0.0f;

bool   hasRef = TARGET_LAT != 0.0 || TARGET_LON != 0.0;
double refLat = TARGET_LAT;
double refLon = TARGET_LON;

// Latest readings
float altitudeM, pressureHpa, temperatureC;
float busV, currentMa;
float uvMwCm2;
float steer, leftCmd, rightCmd;

uint32_t lastUs          = 0;
uint32_t lastLogMs       = 0;
uint32_t lastTelemetryMs = 0;

// ---------- Servos ----------

uint16_t fractionToUs(float f, uint16_t neutralUs, uint16_t fullUs) {
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    return static_cast<uint16_t>(neutralUs + f * (static_cast<float>(fullUs) - neutralUs));
}

void setBrakes(float left, float right) {
    leftBrake.writeMicroseconds(fractionToUs(left, LEFT_BRAKE_NEUTRAL_US, LEFT_BRAKE_FULL_US));
    rightBrake.writeMicroseconds(fractionToUs(right, RIGHT_BRAKE_NEUTRAL_US, RIGHT_BRAKE_FULL_US));
}

// ---------- IMU ----------

bool imuWrite(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

// Roll from the accelerometer (deg) and roll rate from the gyro (deg/s).
void readImu(float &accelRollDeg, float &gyroRateDps) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_I2C_ADDR, static_cast<uint8_t>(14));
    int16_t raw[7];
    for (uint8_t i = 0; i < 7; ++i) raw[i] = (Wire.read() << 8) | Wire.read();
    accelRollDeg = atan2f(raw[1], raw[2]) * RAD_TO_DEG;  // ay, az
    gyroRateDps  = raw[4] / GYRO_LSB_PER_DPS;            // gx
}

// ---------- GNSS ----------

bool fixOk() {
    return gps.location.isValid() && gps.location.age() < GNSS_MAX_AGE_MS &&
           gps.satellites.value() >= GNSS_MIN_SATS;
}

float wrap180(float deg) {
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

float distanceToRef() {
    return TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), refLat, refLon);
}

float bearingToRef() {
    return TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), refLat, refLon);
}

float gnssSteer() {
    if (!hasRef || !fixOk()) return 0.0f;
    if (distanceToRef() < ARRIVE_RADIUS_M) return 0.0f;
    if (!gps.course.isValid() || gps.speed.mps() < MIN_SPEED_MPS) return 0.0f;

    float error = wrap180(bearingToRef() - gps.course.deg());  // + = target to the right
    float s     = STEER_KP * error;
    if (s > 1.0f)  s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    return s;
}

// "lat,lon" or "here"; replies go back on the link it came from.
void applyRef(Stream &in, const String &line) {
    if (line == "here") {
        if (!fixOk()) {
            in.println("No GNSS fix yet");
            return;
        }
        refLat = gps.location.lat();
        refLon = gps.location.lng();
    } else {
        int comma = line.indexOf(',');
        if (comma < 0) {
            in.println("Enter reference as lat,lon or 'here'");
            return;
        }
        refLat = line.substring(0, comma).toDouble();
        refLon = line.substring(comma + 1).toDouble();
    }
    hasRef = true;
    in.print("Reference set: ");
    in.print(refLat, 6);
    in.print(",");
    in.println(refLon, 6);
}

void readRefInput(Stream &in) {
    if (!in.available()) return;
    String line = in.readStringUntil('\n');
    line.trim();
    applyRef(in, line);
}

// ACK,IMU=1,BARO=1,INA=1,SD=1,GNSS=1,FIX=0,REF=lat,lon
void sendAck() {
    loraSerial.print("ACK,IMU=");
    loraSerial.print(imuOk);
    loraSerial.print(",BARO=");
    loraSerial.print(baroOk);
    loraSerial.print(",INA=");
    loraSerial.print(inaOk);
    loraSerial.print(",SD=");
    loraSerial.print(sdOk);
    loraSerial.print(",GNSS=");
    loraSerial.print(gps.charsProcessed() > 0);  // receiving NMEA
    loraSerial.print(",FIX=");
    loraSerial.print(fixOk());
    loraSerial.print(",REF=");
    loraSerial.print(refLat, 6);
    loraSerial.print(",");
    loraSerial.println(refLon, 6);
}

void calibrate();

// Nothing goes out on LoRa until "CONNECT" arrives; the ACK is always the first packet.
void readLora() {
    if (!loraSerial.available()) return;
    String line = loraSerial.readStringUntil('\n');
    line.trim();
    if (line == "CONNECT") {
        sendAck();
        loraConnected = true;
        Serial.println("LoRa connected, ACK sent");
        return;
    }
    if (!loraConnected) return;

    if (line == "START") {
        mode = Mode::Start;
        loraSerial.println("ACK,START");
        Serial.println("Mode: START");
    } else if (line == "STOP") {
        mode = Mode::Stop;
        loraSerial.println("ACK,STOP");
        Serial.println("Mode: STOP (standby)");
    } else if (line == "RECALIBRATE") {
        calibrate();
        loraSerial.println("ACK,RECALIBRATE");
    } else {
        applyRef(loraSerial, line);
    }
}

// ---------- Sensors ----------

void readSensors() {
    bmp.performReading();
    pressureHpa  = bmp.pressure / 100.0f;
    temperatureC = bmp.temperature;
    altitudeM    = 44330.0f * (1.0f - powf(pressureHpa / groundHpa, 0.1903f));

    busV      = ina.readBusVoltage() / 1000.0f;
    currentMa = ina.readCurrent();

    // ML8511: 0.99 V = 0 mW/cm^2, 2.8 V = 15 mW/cm^2
    float uvV = analogReadMilliVolts(PIN_UV) / 1000.0f;
    uvMwCm2   = (uvV - 0.99f) * 15.0f / (2.8f - 0.99f);
    if (uvMwCm2 < 0.0f) uvMwCm2 = 0.0f;
}

// ---------- Logging and telemetry ----------

// time_ms,lat,lon,sats,gnss_alt_m,alt_m,pressure_hpa,temp_c,roll_deg,dist_m,bearing_deg,steer,left,right,bus_v,current_ma,uv_mw_cm2
void printRecord(Print &out) {
    out.print(millis());
    out.print(",");
    out.print(gps.location.lat(), 6);
    out.print(",");
    out.print(gps.location.lng(), 6);
    out.print(",");
    out.print(gps.satellites.value());
    out.print(",");
    out.print(gps.altitude.meters(), 1);
    out.print(",");
    out.print(altitudeM, 1);
    out.print(",");
    out.print(pressureHpa, 2);
    out.print(",");
    out.print(temperatureC, 1);
    out.print(",");
    out.print(rollDeg, 1);
    out.print(",");
    out.print(hasRef && fixOk() ? distanceToRef() : 0.0f, 1);
    out.print(",");
    out.print(hasRef && fixOk() ? bearingToRef() : 0.0f, 0);
    out.print(",");
    out.print(steer, 2);
    out.print(",");
    out.print(leftCmd, 2);
    out.print(",");
    out.print(rightCmd, 2);
    out.print(",");
    out.print(busV, 2);
    out.print(",");
    out.print(currentMa, 0);
    out.print(",");
    out.println(uvMwCm2, 2);
}

void beep(uint16_t ms) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(ms);
    digitalWrite(PIN_BUZZER, LOW);
}

// ---------- Setup and loop ----------

void printInit(const char *name, bool ok) {
    Serial.print(name);
    Serial.println(ok ? ": OK" : ": FAIL");
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_BUZZER, OUTPUT);

    // 1. Servos first so the brakes sit at neutral during the rest of startup
    leftBrake.attach(PIN_LEFT_BRAKE);
    rightBrake.attach(PIN_RIGHT_BRAKE);
    setBrakes(0.0f, 0.0f);
    Serial.println("Servos: neutral");

    // 2. I2C bus and sensors
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    imuOk = imuWrite(REG_PWR_MGMT_1, 0x01);   // wake, gyro clock
    delay(100);
    imuOk = imuOk && imuWrite(REG_CONFIG, 0x03)         // DLPF ~44 Hz
                  && imuWrite(REG_GYRO_CONFIG, 0x08);   // +/-500 deg/s
    printInit("IMU (MPU-9250)", imuOk);

    baroOk = bmp.begin_I2C(BARO_I2C_ADDR);
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    printInit("Barometer (BMP390L)", baroOk);

    inaOk = ina.begin(INA_I2C_ADDR);
    printInit("Current sensor (INA260)", inaOk);

    // 3. UV sensor (analog)
    analogReadResolution(12);
    Serial.println("UV sensor (ML8511): analog on GPIO 7");

    // 4. GNSS
    gnssSerial.begin(GNSS_BAUD, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);
    Serial.println("GNSS (L89HA): Serial1 started");

    // 5. LoRa
    loraSerial.begin(LORA_BAUD, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);
    Serial.println("LoRa (E220): Serial2 started, waiting for CONNECT");

    // 6. SD card last
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    sdOk = SD.begin(PIN_SD_CS);
    if (sdOk) {
        logFile = SD.open("/log.csv", FILE_APPEND);
        logFile.println("time_ms,lat,lon,sats,gnss_alt_m,alt_m,pressure_hpa,temp_c,roll_deg,dist_m,bearing_deg,steer,left,right,bus_v,current_ma,uv_mw_cm2");
        logFile.flush();
    }
    printInit("SD card", sdOk);

    calibrate();
    beep(200);
}

// Gyro bias, level roll trim and ground pressure. Brakes held at neutral.
void calibrate() {
    setBrakes(0.0f, 0.0f);
    Serial.println("Calibrating - keep the board still and level");
    float rollSum     = 0.0f;
    float rateSum     = 0.0f;
    float pressureSum = 0.0f;
    for (uint16_t i = 0; i < CALIB_SAMPLES; ++i) {
        float accelRoll, rate;
        readImu(accelRoll, rate);
        rollSum += accelRoll;
        rateSum += rate;
        if (i % 10 == 0) {
            bmp.performReading();
            pressureSum += bmp.pressure / 100.0f;
        }
        delay(2);
    }
    rollTrimDeg = rollSum / CALIB_SAMPLES;
    gyroBiasDps = rateSum / CALIB_SAMPLES;
    groundHpa   = pressureSum / (CALIB_SAMPLES / 10);
    Serial.print("Roll trim:");
    Serial.print(rollTrimDeg, 2);
    Serial.print(" Gyro bias:");
    Serial.print(gyroBiasDps, 3);
    Serial.print(" Ground hPa:");
    Serial.println(groundHpa, 2);

    rollDeg = 0.0f;
    lastUs  = micros();
}

void loop() {
    while (gnssSerial.available()) gps.encode(gnssSerial.read());
    readRefInput(Serial);
    readLora();

    uint32_t now = micros();
    if (now - lastUs < LOOP_PERIOD_US) return;
    float dt = (now - lastUs) * 1e-6f;
    lastUs = now;

    float accelRoll, rate;
    readImu(accelRoll, rate);
    accelRoll -= rollTrimDeg;
    rate      -= gyroBiasDps;

    // Complementary filter: gyro for fast changes, accelerometer to stop drift.
    rollDeg = FILTER_ALPHA * (rollDeg + rate * dt) + (1.0f - FILTER_ALPHA) * accelRoll;

    float cmd = 0.0f;
    if (rollDeg > DEADBAND_DEG)  cmd = (rollDeg - DEADBAND_DEG) / FULL_BRAKE_ROLL_DEG;
    if (rollDeg < -DEADBAND_DEG) cmd = (rollDeg + DEADBAND_DEG) / FULL_BRAKE_ROLL_DEG;
    steer = gnssSteer();
    cmd  += steer;
    leftCmd  = cmd < 0.0f ? -cmd : 0.0f;
    rightCmd = cmd > 0.0f ?  cmd : 0.0f;
    setBrakes(leftCmd, rightCmd);

    if (millis() - lastLogMs >= LOG_PERIOD_MS) {
        lastLogMs = millis();
        readSensors();
        if (sdOk) {
            printRecord(logFile);
            logFile.flush();
        }
        printRecord(Serial);
    }

    if (loraConnected && mode == Mode::Start && millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
        lastTelemetryMs = millis();
        printRecord(loraSerial);
    }
}
