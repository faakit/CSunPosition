#include <math.h>
#include <stdio.h>

#define PI 3.141592653589793
#define RAD (PI / 180)

const double dayMs = 60.0 * 60.0 * 24.0;
const double e = RAD * 23.4397;

typedef struct {
  double azimuth;
  double altitude;
} Position;

typedef struct {
  double dec;
  double ra;
} SunCoords;

double toDays(unsigned long date) { return date / dayMs - 10957.5; }

double azimuth(double H, double phi, double dec) {
  return atan2(sin(H), cos(H) * sin(phi) - tan(dec) * cos(phi));
}

double altitude(double H, double phi, double dec) {
  return asin(sin(phi) * sin(dec) + cos(phi) * cos(dec) * cos(H));
}

double solarMeanAnomaly(double d) { return RAD * (357.5291 + 0.98560028 * d); }

double eclipticLongitude(double M) {
  double C = RAD * (1.9148 * sin(M) + 0.02 * sin(2 * M) + 0.0003 * sin(3 * M));
  double P = RAD * 102.9372;
  return M + C + P + PI;
}

double declination(double l, double b) {
  return asin(sin(b) * cos(e) + cos(b) * sin(e) * sin(l));
}

double siderealTime(double d, double lw) {
  return RAD * (280.16 + 360.9856235 * d) - lw;
}


SunCoords sunCoords(double d) {
  double M = solarMeanAnomaly(d);
  double L = eclipticLongitude(M);
  SunCoords coords;
  coords.dec = declination(L, 0);
  coords.ra = atan2(sin(L) * cos(e) - tan(0) * sin(e), cos(L));
  return coords;
}

Position getPosition(unsigned long long date, double lat, double lng) {

  double lw = RAD * -lng;
  double phi = RAD * lat;
  double d = toDays(date);

  SunCoords c = sunCoords(d);
  double H = siderealTime(d, lw) - c.ra;

  Position pos;
  pos.azimuth = azimuth(H, phi, c.dec);
  pos.altitude = altitude(H, phi, c.dec);
  return pos;
}

void setup() {
  Serial.begin(115200);

  Position pos;
  unsigned long long epoch = 1726981200;
  pos = getPosition(epoch, 0, 0);
  
  Serial.print("azimuth: ");
  Serial.print(pos.azimuth/RAD + 180);
  Serial.print(" | altitude: ");
  Serial.print(pos.altitude/RAD);
  Serial.println("");

}

void loop() {

}
