#include <math.h>
// #include <stdio.h>
#include <Servo.h>
#include <Wire.h>

#define PI 3.141592653589793
#define RAD (PI / 180)
#define DEG (180 / PI)
#define SAMPLE_SIZE 5
#define FIVE_MINS 300000

Servo servo;
int i2c_add = 0x68;

// Váriaveis para modificar
unsigned long long EPOCH = 1726981200000; // mudar na hora de executar
double LAT = 0;
double LNG = 0;

struct SunCoords {
    double dec;
    double ra;
};

struct Position {
    double azimuth;
    double altitude;
    double rotation;
};

const double dayMs = 1000 * 60 * 60 * 24;
const double J1970 = 2440588;
const double J2000 = 2451545;
const double J0 = 0.0009;
const double e = RAD * 23.4397;

double toJulian(unsigned long long date) { return date / dayMs - 0.5 + J1970; }

double toDays(unsigned long long date) { return toJulian(date) - J2000; }

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
    pos.rotation = atan(tan(PI/2-pos.altitude)*sin(pos.azimuth)); // talvez corrigir o azimuth para ficar contar apartir do norte (adicionar PI ao azimuth)
    return pos;
}

int gyro_start = 0;
int gyro_end = 0;

// bagulho de i2c
void setup_gyro(int gyro_add) {
    Wire.begin();
    Wire.beginTransmission(gyro_add);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
}

// coloca os valores dos eixos x e y no array
// os eixos estao desenhados na placa (meio errado mas esta hardcoded aqui pra ser assim)

// OS VALORES RETORNADOS PRECISAM SER CALIBRADOS PRA SEREM USADOS COMO ANGULO (DEPENDE DA ESTRUTURA DO TRACKER)
void get_gyro_data(int gyro_add, int16_t* gyro_array) {
    Wire.beginTransmission(gyro_add);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(gyro_add, 12, true);

    gyro_array[1] = Wire.read() << 8 | Wire.read();
    gyro_array[0] = Wire.read() << 8 | Wire.read();
}

// ESSA FUNCAO PRECISA SER CONTINUAMENTE CHAMADA EM LOOP
void update_servo(double servo_speed, double starting_angle) {
    //'servo_speed' nao se trata de velocidade de verdade
    // valor entre -1 e 1 (0 = parado; 1 ou -1 = velocidade maxima (negativo = sentido horario))

    if (servo_speed < -1) {
        servo_speed = -1;
    } else if (servo_speed > 1) {
        servo_speed = 1;
    }

    static double servo_angle = starting_angle;

    if (servo_angle >= 150) {
        servo_angle = 150;
    } else if (servo_angle <= 0) {
        servo_angle = 0;
    }

    servo.write(servo_angle += servo_speed);
    delay(4);
}

double boot_angle = 0;
int agr = 0;
Position init_position;
double target = 0;

void setup() {
    // pino PWM do servo
    servo.attach(9);

    // Setup do acelerômetro
    setup_gyro(i2c_add);

    // Monitor serial
    Serial.begin(115200);

    // posicao inicial do servo
    //servo.write(boot_angle);
    delay(2000);  // <--- Enquanto o servo gira a execucao continua
    agr = millis();
    init_position = getPosition(EPOCH,LAT,LNG);
    target = init_position.azimuth-90.0;
}

double normalize(int16_t reading){
  return ((10.0-170.0)/28084)*(reading + 13872.44) + 170.0; 
}

// NAO PODE USAR DELAY NESSE LOOP POIS O
// SERVO NAO VAI FUNCIONAR DIREITO

int count = 1;

void loop() {
  /*                                          1 hora em ms*/
  init_position = getPosition(EPOCH + count * 3600 * 1000, LAT, LNG);
  count++;

  /*coloquei lat=0, lng=0 e epoch sendo 24 de setembro de 2024 (equinocio vernal)*/

  Serial.print("altitude ");
  Serial.print(init_position.altitude*DEG);

  Serial.print(" | azimuth ");
  Serial.println(init_position.azimuth*DEG);

  delay(1000);
}
