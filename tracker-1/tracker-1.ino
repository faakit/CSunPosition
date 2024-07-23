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
unsigned long long EPOCH = 1721660902; // mudar na hora de executar
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

    // //"calibração do gyroscopio"
    // int16_t gyro[2];
    // int sum = 0;

    // servo.write(0);
    // for (int i = 0; i < 3; i++) {
    //     get_gyro_data(i2c_add, gyro);
    //     delay(100);
    //     sum += gyro[0];
    // }
    // gyro_start = sum / 3;

    // servo.write(180);
    // sum = 0;
    // for (int i = 0; i < 3; i++) {
    //     get_gyro_data(i2c_add, gyro);
    //     delay(100);
    //     sum += gyro[0];
    // }
    // gyro_end = sum / 3;
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

// Função responsável por converter o valor do giroscópio em um angulo entre 60-180 (em graus)
// int normalize_gyro(int gyro_angle) {
//     return ((gyro_angle - gyro_start) / (gyro_end - gyro_start)) * (120) + 60;
// }

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

// Função para pegar a média de um vetor
// float get_avg(int size , int* arr){
//     int sum = 0;
//     for(int i = 0; i < size; i++){
//         sum += arr[i];
//     }
//     return sum/float(size);
// }

// // Função que realiza o seguimento do sol
// void sun_track(unsigned long long date, double lat, double lng){
//     Position p = getPosition(date,lat,lng);
//     float target = p.altitude * DEG;
//     int avg_arr[SAMPLE_SIZE];
//     int cycle = 0;
//     float mov_avg = 0;
//     int gyro[2];

//     for(int i = 0; i < SAMPLE_SIZE; i++){
//         get_gyro_data(i2c_add,gyro);
//         delay(10);
//         avg_arr[i] = gyro[0];
//     }
//     mov_avg = normalize_gyro(get_avg(SAMPLE_SIZE, avg_arr));

//     // Vai fazendo a média móvel e termina quando o ângulo da média for maior ou igual ao alvo
//     while(mov_avg >= target){
//         get_gyro_data(i2c_add,gyro);
//         delay(10);

//         //Isso vai substituindo cada elemento do vetor de maneira circular
//         avg_arr[cycle++] = gyro[0];
//         cycle %= SAMPLE_SIZE;

//         //Vai movendo o servo linearmente
//         update_servo(1.0,normalize_gyro(gyro[0]));

//         mov_avg = normalize_gyro(get_avg(SAMPLE_SIZE, avg_arr));
//     }
// }

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
    init_position = getPosition(EPOCH+agr,LAT,LNG);
    target = init_position.azimuth-90.0;
}

double normalize(int16_t reading){
  return ((10.0-170.0)/28084)*(reading + 13872.44) + 170.0; 
}

// NAO PODE USAR DELAY NESSE LOOP POIS O
// SERVO NAO VAI FUNCIONAR DIREITO

int count = 0;
int dps = 0;

void loop() {

    init_position = getPosition(EPOCH,LAT,LNG);
    target = init_position.azimuth * DEG;
    Serial.println(target);

    // int16_t gyro[2];
    // get_gyro_data(i2c_add, gyro);
    // double gyro_value = normalize(gyro[0]);

    // if(gyro_value <= target){
    //   update_servo(0.2, 0);
    //   get_gyro_data(i2c_add, gyro);
    //   gyro_value = normalize(gyro[0]);
    // }

    // dps = millis();
    // if(dps-agr > 250){
    //   agr = dps;
    //   init_position = getPosition(EPOCH+millis(),LAT,LNG);
    //   // if(init_position.altitude * DEG < 0.0f) target = 90;
    //   target = init_position.azimuth * DEG;

    //   Serial.println(target);
    // }

    // Serial.println(init_position.azimuth);
}
