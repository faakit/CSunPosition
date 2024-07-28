# Sun position in C

Sun position calculations in C based on Astronomy Answers articles about [position of the sun](http://aa.quae.nl/en/reken/zonpositie.html).

With the article, [this mathematical model](https://www.geogebra.org/m/hspd5fyv) were implemented to guide on the code development.

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

# Sun timelapse

For the sun timelapse preview were used React + Vite for the main page and express NodeJs socket to communicate with the Aduino via Serial Port.

## React + Vite page

The sun follows a second grade funtion and adapts the path depending o the size of the screen.

It get the information of time from the socket subscription.

### Usage

After installing the depencdencies run this simple command on terminal. Note that for it to work, the socket has to be running.
``` bash
npm run dev
```

## NodeJs socket

Using serialport lib the connection with the Arduino serial port is established. Then, using express, a socket is set up to send the information to the React page.

### Usage

After installing the depencdencies run this simple command on terminal. Note that for it to work, the socket has to be running.
``` bash
node indes.js
```

# Used hardware
- Arduino UNO
- MPU 6050
- 1x ServoMotor
