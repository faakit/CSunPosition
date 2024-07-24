//#include <math.h>
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

unsigned long long EPOCH = 1721849318; // mudar na hora de executar
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

const double dayMs = 60 * 60 * 24;
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
    // Serial.print("H: ");
    // Serial.print(H);

    // Serial.print(" | d: ");
    // Serial.print(d);

    // Serial.print(" | phi: ");
    // Serial.print(phi);

    // Serial.print(" | c: ");
    // Serial.print(c.dec);
    // Serial.println("");

    pos.azimuth = azimuth(H, phi, c.dec);
    pos.altitude = altitude(H, phi, c.dec);

    pos.rotation = atan(tan(PI/2-pos.altitude)*sin(pos.azimuth)); // talvez corrigir o azimuth para ficar contar apartir do norte (adicionar PI ao azimuth)
    return pos;
}

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

double boot_angle = 0;
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

double normalize(int16_t reading){
  return ((10.0-170.0)/28084)*(reading + 13872.44) + 170.0; 
}

double altitude_array[144] = 
{-87.140433,-84.821122,-82.483262,-80.140357,-77.795675,-75.450466,-73.105364,-70.760762,-68.416948,-66.074156,-63.732595,-61.392458,-59.053933,-56.717207,-54.382469,-52.049912,-49.719735,-47.392143,-45.067353,-42.745591,-40.427092,-38.112108,-35.800903,-33.493757,-31.190970,-28.892858,-26.599763,-24.312045,-22.030096,-19.754332,-17.485205,-15.223198,-12.968836,-10.722685,-8.485359,-6.257524,-4.039905,-1.833290,0.361459,2.543403,4.711513,6.864661,9.001610,11.121000,13.221335,15.300966,17.358073,19.390650,21.396475,23.373095,25.317795,27.227572,29.099106,30.928726,32.712383,34.445618,36.123533,37.740768,39.291489,40.769382,42.167671,43.479150,44.696259,45.811176,46.815967,47.702764,48.463993,49.092630,49.582477,49.928434,50.126749,50.175211,50.073273,49.822079,49.424405,48.884504,48.207897,47.401104,46.471377,45.426419,44.274145,43.022467,41.679128,40.251572,38.746856,37.171596,35.531935,33.833538,32.081598,30.280854,28.435616,26.549795,24.626934,22.670235,20.682597,18.666638,16.624727,14.559005,12.471410,10.363700,8.237466,6.094155,3.935082,1.761443,-0.425670,-2.625261,-4.836420,-7.058313,-9.290173,-11.531298,-13.781041,-16.038805,-18.304040,-20.576236,-22.854920,-25.139654,-27.430031,-29.725670,-32.026216,-34.331338,-36.640723,-38.954080,-41.271131,-43.591617,-45.915289,-48.241913,-50.571264,-52.903125,-55.237289,-57.573551,-59.911712,-62.251570,-64.592923,-66.935556,-69.279236,-71.623692,-73.968588,-76.313461,-78.657582,-80.999613,-83.336526,-85.659071,-87.917152,-89.057041};
double rotation_array[144] = 
{2.797780,5.151460,7.503985,9.854825,12.203470,14.549438,16.892276,19.231572,21.566953,23.898096,26.224728,28.546631,30.863645,33.175671,35.482674,37.784682,40.081790,42.374159,44.662019,46.945666,49.225462,51.501837,53.775287,56.046372,58.315714,60.583999,62.851971,65.120434,67.390247,69.662325,71.937633,74.217188,76.502051,78.793331,81.092175,83.399771,85.717339,88.046130,-89.612577,-87.257484,-84.887276,-82.500631,-80.096220,-77.672724,-75.228835,-72.763269,-70.274777,-67.762158,-65.224267,-62.660035,-60.068481,-57.448730,-54.800031,-52.121773,-49.413504,-46.674953,-43.906045,-41.106920,-38.277951,-35.419758,-32.533217,-29.619474,-26.679946,-23.716325,-20.730567,-17.724885,-14.701735,-11.663788,-8.613906,-5.555108,-2.490532,0.576607,3.643057,6.705575,9.760973,12.806158,15.838173,18.854231,21.851748,24.828368,27.781983,30.710745,33.613078,36.487680,39.333517,42.149821,44.936078,47.692015,50.417585,53.112946,55.778449,58.414615,61.022118,63.601763,66.154473,68.681269,71.183255,73.661603,76.117538,78.552328,80.967268,83.363675,85.742878,88.106203,-89.545024,-87.209492,-84.885907,-82.572998,-80.269521,-77.974267,-75.686059,-73.403761,-71.126280,-68.852568,-66.581624,-64.312498,-62.044293,-59.776169,-57.507339,-55.237078,-52.964722,-50.689667,-48.411375,-46.129374,-43.843254,-41.552676,-39.257366,-36.957120,-34.651799,-32.341332,-30.025714,-27.705005,-25.379326,-23.048861,-20.713849,-18.374584,-16.031412,-13.684723,-11.334948,-8.982556,-6.628046,-4.271939,-1.914779,0.442881};

double target = 5;
double time_interval = 1000;


int count = 0;
int dps = 0;

void loop() {
    int16_t gyro[2];
    // get_gyro_data(i2c_add, gyro);
    // double gyro_value = normalize(gyro[0]);

    // if(gyro_value <= target){
    //   update_servo(0.5, 0);
    //   get_gyro_data(i2c_add, gyro);
    //   gyro_value = normalize(gyro[0]);
    // }

    dps = millis();
    if(dps-agr > 1000){
      agr = dps;

      if(altitude_array[count] > 0){
        
        if(rotation_array[count] <= -60.0){
          target = 30;
        }else if(rotation_array[count] >= 60.0){
          target = 150;
        }else{
          target = rotation_array[count]+90;
        }

      }else{
        target = 90;
      }

      servo.write(target);

      count++;
      if(count >= 144){
        count = 0;
      }

      Serial.println(count*(24.0/144.0));
    }
}
