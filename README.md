# Sun position in C

Sun position calculations in C based on Astronomy Answers articles about [position of the sun](http://aa.quae.nl/en/reken/zonpositie.html).

## Usage example

The getPosition function receives 3 parameters:
```
date: in miliseconds since epoch
latitude: (4 numbers after dot are sufficient)
longitude: (4 numbers after dot are sufficient)
```

and returns the Position object with:
```
azimuth: sun azimuth in radians (direction along the horizon, measured from south to west), e.g. 0 is south and Math.PI * 3/4 is northwest
altitude: sun altitude above the horizon in radians, e.g. 0 at the horizon and PI/2 at the zenith (straight over your head)
```

```C
#include <sys/time.h>

int main() {
  struct timeval tv;

  gettimeofday(&tv, NULL);

  unsigned long long millisecondsSinceEpoch =
      (unsigned long long)(tv.tv_sec) * 1000 +
      (unsigned long long)(tv.tv_usec) / 1000;

  Position sunPos = getPosition(millisecondsSinceEpoch, -15.8150, -48.1294);
  double sunAzimuth = (sunPos.azimuth * 180) / PI;
  double sunAlt = (sunPos.altitude * 180) / PI;

  printf("Sun Azimuth: %f\n", sunAzimuth);
  printf("Sun Altitude: %f\n", sunAlt);
  return 0;
}
```

## Hardware Used
- Arduino UNO
- MPU 6050
- 1x ServoMotor
