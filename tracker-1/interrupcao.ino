#define pinInt 2
#define SAFE_POS 90 //verificar qual o angulo de posição segura, ver se vai ser 90 ou 0 (ou etc.)
#define FIVE_MINS 300000

void track(double target){ //verificar a implementação do tracker propriamente dito
  return;
}

void safety_position(){
  track(SAFE_POS);
  delay(FIVE_MINS);
}

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(pinInt),safety_position,RISING);
}

void loop() {
  // put your main code here, to run repeatedly:

}
