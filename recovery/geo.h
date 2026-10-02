// Local flat-earth geometry for the recovery area (a few km across).
// X points east, Y points north, both in metres from the origin.
#pragma once

struct Vec2 {
    float x, y;
};

// Equirectangular projection of (lat, lon) relative to an origin.
Vec2 toLocalXY(double lat, double lon, double originLat, double originLon);

// Bearing from one point to another, degrees clockwise from north, (-180, 180].
float bearingDeg(const Vec2& from, const Vec2& to);

float magnitude(const Vec2& v);

// Wraps an angle to (-180, 180].
float wrap180(float deg);
