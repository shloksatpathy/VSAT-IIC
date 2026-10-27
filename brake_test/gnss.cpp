#include "gnss.h"

#include <Arduino.h>
#include <TinyGPSPlus.h>

constexpr uint8_t  PIN_GNSS_RX     = 18;  // ESP32 RX <- GNSS TX
constexpr uint8_t  PIN_GNSS_TX     = 17;  // ESP32 TX -> GNSS RX
constexpr uint32_t GNSS_BAUD       = 9600;
constexpr uint8_t  GNSS_MIN_SATS   = 4;
constexpr uint32_t GNSS_MAX_AGE_MS = 2000;  // fix older than this is ignored

// Tuning
constexpr float STEER_KP         = 0.02f;  // brake fraction per degree of heading error
constexpr float ARRIVE_RADIUS_M  = 3.0f;   // no steering inside this distance
constexpr float MIN_SPEED_MPS    = 0.5f;   // GNSS course is unreliable below this

TinyGPSPlus gps;

bool   hasRef = false;
double refLat = 0.0;
double refLon = 0.0;

bool fixOk() {
    return gps.location.isValid() && gps.location.age() < GNSS_MAX_AGE_MS &&
           gps.satellites.value() >= GNSS_MIN_SATS;
}

float wrap180(float deg) {
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

void setRef(double lat, double lon) {
    refLat = lat;
    refLon = lon;
    hasRef = true;
    Serial.print("Reference set: ");
    Serial.print(refLat, 6);
    Serial.print(",");
    Serial.println(refLon, 6);
}

void readRefInput() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line == "here") {
        if (fixOk()) setRef(gps.location.lat(), gps.location.lng());
        else Serial.println("No GNSS fix yet");
        return;
    }
    int comma = line.indexOf(',');
    if (comma < 0) {
        Serial.println("Enter reference as lat,lon or 'here'");
        return;
    }
    setRef(line.substring(0, comma).toDouble(), line.substring(comma + 1).toDouble());
}

void gnssBegin() {
    Serial1.begin(GNSS_BAUD, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);
    Serial.println("Enter reference as lat,lon or 'here'");
}

void gnssUpdate() {
    while (Serial1.available()) gps.encode(Serial1.read());
    readRefInput();
}

float gnssSteer() {
    if (!hasRef || !fixOk()) return 0.0f;
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    if (TinyGPSPlus::distanceBetween(lat, lon, refLat, refLon) < ARRIVE_RADIUS_M) return 0.0f;
    if (!gps.course.isValid() || gps.speed.mps() < MIN_SPEED_MPS) return 0.0f;

    float bearing = TinyGPSPlus::courseTo(lat, lon, refLat, refLon);
    float error   = wrap180(bearing - gps.course.deg());  // + = target to the right
    float steer   = STEER_KP * error;
    if (steer > 1.0f)  steer = 1.0f;
    if (steer < -1.0f) steer = -1.0f;
    return steer;
}

void gnssPrint() {
    Serial.print(" Sats:");
    Serial.print(gps.satellites.value());
    if (!fixOk()) {
        Serial.print(" NoFix");
        return;
    }
    Serial.print(" Pos:");
    Serial.print(gps.location.lat(), 6);
    Serial.print(",");
    Serial.print(gps.location.lng(), 6);
    Serial.print(" Spd:");
    Serial.print(gps.speed.mps(), 1);
    Serial.print(" Crs:");
    Serial.print(gps.course.deg(), 0);
    if (!hasRef) {
        Serial.print(" NoRef");
        return;
    }
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    Serial.print(" Dist:");
    Serial.print(TinyGPSPlus::distanceBetween(lat, lon, refLat, refLon), 1);
    Serial.print(" Brg:");
    Serial.print(TinyGPSPlus::courseTo(lat, lon, refLat, refLon), 0);
    Serial.print(" Steer:");
    Serial.print(gnssSteer(), 2);
}
