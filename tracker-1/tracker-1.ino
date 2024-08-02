#include <Servo.h>
#include <Wire.h>

#define PI 3.141592653589793
#define RAD (PI / 180)
#define DEG (180 / PI)
#define SAMPLE_SIZE 5

Servo servo;
int i2c_add = 0x68;

/********************************************************************/
/**************CODIGO ADAPTADO DO SITE SUNCALC.ORG*******************/
/********************************************************************/

unsigned long EPOCH = 1721849318UL; // mudar na hora de executar
double LAT = -20.27278;
double LNG = -40.30556;

struct SunCoords {
    double dec;
    double ra;
};

struct Position {
    double azimuth;
    double altitude;
    double rotation;
};

const double dayMs = 60.0 * 60.0 * 24.0;
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

/*****************************************************************/
/*****************************************************************/
/*****************************************************************/



// setup da conexao i2c
void setup_gyro(int gyro_add) {
    Wire.begin();
    Wire.beginTransmission(gyro_add);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
}

// coloca os valores dos eixos x e y no array
// os eixos estao desenhados na placa

// OS VALORES RETORNADOS PRECISAM SER CALIBRADOS PRA SEREM USADOS COMO ANGULO (DEPENDE DA ESTRUTURA DO TRACKER)
void get_gyro_data(int gyro_add, int16_t* gyro_array) {
    Wire.beginTransmission(gyro_add);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(gyro_add, 12, true);

    gyro_array[1] = Wire.read() << 8 | Wire.read();
    gyro_array[0] = Wire.read() << 8 | Wire.read();
}

//faz o servo dar um pequeno passo de tamanho servo_speed
//serve para simular o funcionamento de um motor onde se controla a velocidade
void update_servo(double servo_speed, double starting_angle) {
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

double boot_angle = 90;
Position init_position;

int agr = 0;
void setup() {
    servo.attach(9);
    setup_gyro(i2c_add);
    Serial.begin(115200);
    servo.write(boot_angle);

    agr = millis();
    delay(2000);
}

//calibracao do giroscopio (depende da estrutura montada)
double normalize(int16_t reading){
  return ((10.0-170.0)/28084)*(reading + 13872.44) + 170.0; 
}

double target = 5;
int dps = 0;

void loop() {
    int16_t gyro[2];
    get_gyro_data(i2c_add, gyro);
    double gyro_value = normalize(gyro[0]);

    dps = millis();
    init_position = getPosition(EPOCH + dps, LAT, LNG);

    double altitude = init_position.altitude;
    double rotation = init_position.rotation;

    if(dps-agr > 1000){
      agr = dps;

      //altitude > 0 => "esta de dia"
      if(altitude > 0){
        
        //permite que o painel tenha limites de operacao
        //(serve para simular uma situacao real)
        if(rotation <= -60.0){
          target = 30;
        }else if(rotation >= 60.0){
          target = 150; //180 + 30
        }else{
          target = rotation+90;
        }

      }else{
        // altitude < 0 => esta de noite
        target = 90;
      }

      //faz o servo dar um step cada vez menor em direcao ao target
      update_servo(gyro_value-target, 30);

      //como a funcao println nao aceita mais de 32 bits fazemos uma divisao do numero
      //mandando a parte inferior depois a parte superior

      uint64_t time = EPOCH + dps;
      uint32_t lower = time & 0xFFFFFFFF;
      uint32_t upper = (time >> 32) & 0xFFFFFFFF;
      

      //comunicacao com o app
      //manda o tempo atual para o site marcar o sol na possicao correta
      Serial.println(lower);
      Serial.println(upper);
    }
}
