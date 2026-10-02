#include "geo.h"

#include <math.h>

namespace {

constexpr double EARTH_RADIUS_M = 6371000.0;
constexpr double DEG_TO_RAD_D   = 0.017453292519943295;
constexpr float  RAD_TO_DEG_F   = 57.2957795131f;

}  // namespace

Vec2 toLocalXY(double lat, double lon, double originLat, double originLon) {
    const double dLat = (lat - originLat) * DEG_TO_RAD_D;
    const double dLon = (lon - originLon) * DEG_TO_RAD_D;
    Vec2 v;
    v.x = static_cast<float>(EARTH_RADIUS_M * dLon * cos(originLat * DEG_TO_RAD_D));
    v.y = static_cast<float>(EARTH_RADIUS_M * dLat);
    return v;
}

float bearingDeg(const Vec2& from, const Vec2& to) {
    return atan2f(to.x - from.x, to.y - from.y) * RAD_TO_DEG_F;
}

float magnitude(const Vec2& v) {
    return sqrtf(v.x * v.x + v.y * v.y);
}

float wrap180(float deg) {
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg <= 0.0f) deg += 360.0f;
    return deg - 180.0f;
}
