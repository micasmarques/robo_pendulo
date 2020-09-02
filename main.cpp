#include<Wire.h>
#include <PID_v1.h>
  const int MPU=0x68;
  int16_t acelY;
  int16_t giroY;
 
  int pinoMotorEsquerdoA = 3;
  int pinoMotorEsquerdoB = 5;
  int pinoMotorDireitoA = 9;
  int pinoMotorDireitoB = 10;

 
//Pinos END
//Parâmetros START
  int quantidadeLeiturasMedia = 3;
  int intervaloEntreLeituras = 0;
  int tolerancia = 5;
//Parâmetros END
//Variáveis globais START
  int zeroConsiderado = 0;
  int sentidoVariacao = 1;
  double giroYKILL=50000;
  int pwmAplicarMotores = 0;
  int pwmAnterior = 0;
  int sentidoVariacaoAnterior = -1;
//Variáveis globais END
//PID
  double Setpoint = 0;
  double Input = 0;
  double Output;
  PID myPID(&Input, &Output, &Setpoint, 0.1314, 0.7152, 0.0063, DIRECT);
///

 void setup()
{

  Serial.begin(9600);         //inicia a comunicação serial
  Wire.begin();                 //inicia I2C
  Wire.beginTransmission(MPU);  //Inicia transmissão para o endereço do MPU
  Wire.write(0x6B);
  pinMode(A5,INPUT);
  pinMode(A4,INPUT);
  //Inicializa o MPU-6050
  Wire.write(0);
  Wire.endTransmission(true);
  pinMode(pinoMotorEsquerdoA,OUTPUT);
  pinMode(pinoMotorEsquerdoB,OUTPUT);
  pinMode(pinoMotorDireitoA,OUTPUT);
  pinMode(pinoMotorDireitoB,OUTPUT);

  //config PID
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
}
  int leituraMedia(int zeroConsiderado = 0) {
  int valorMedio = 0;
  int valorLeiturasAcumulado = 0;
  int con;
  for(con =0; con < quantidadeLeiturasMedia; con++) {
  valorLeiturasAcumulado = valorLeiturasAcumulado + giroY;
  delay(intervaloEntreLeituras);
}
  valorMedio = (valorLeiturasAcumulado / quantidadeLeiturasMedia);
  if (valorMedio >= zeroConsiderado) {
   sentidoVariacao = 1;
} else {
    sentidoVariacao = -1;
}
  valorMedio = abs(valorMedio);
  if (valorMedio < tolerancia) {
  valorMedio = 0;
}
  return valorMedio;
}
void acionaMotorEsquerdo(int sentido = 0, int valor = 0) {
digitalWrite(pinoMotorEsquerdoA, LOW);
digitalWrite(pinoMotorEsquerdoB, LOW);
if (sentido == -1){
digitalWrite(pinoMotorEsquerdoB, valor);
} else {
digitalWrite(pinoMotorEsquerdoA, valor);
}
}
void acionaMotorDireito(int sentido = 0, int valor = 255) {
digitalWrite(pinoMotorDireitoA, LOW);
digitalWrite(pinoMotorDireitoB, LOW);
if (sentido == 1){
analogWrite(pinoMotorDireitoB, valor);
} else {
analogWrite(pinoMotorDireitoA, valor);
}
}
void acionaMotores(int sentido = 0, int valor = 255) {
acionaMotorDireito(sentido, valor);
acionaMotorEsquerdo(sentido, valor);
}
void pararMotores(int sentidoVariacaoAnterior, int pwmAnterior){
int con = 0;
for ( con = pwmAnterior; con > 0; con--) {
acionaMotores(sentidoVariacaoAnterior, con);
}
}

 

void loop()
{
  Wire.beginTransmission(MPU);      //transmite
  Wire.write(0x3B);                 // Endereço 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);     //Finaliza transmissão
 
  Wire.requestFrom(MPU,14,true);   //requisita bytes
   
  //Armazena o valor dos sensores nas variaveis correspondentes
  acelY=Wire.read()8|Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  giroY=Wire.read()8|Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
 
   
  //Envia valores lidos do acelerômetro
  Serial.print("\tAcel:");
  Serial.print("\tY:");Serial.print(acelY/16.4);
 
  //Envia valores lidos do giroscópio
  Serial.print("\tGiro:");
  Serial.print("\tY:");Serial.println(giroY/16.4);
  int con = 0;
  //antes de serem atualizados pela nova leitura
  sentidoVariacaoAnterior = sentidoVariacao;
  pwmAnterior = pwmAplicarMotores;
  giroY = leituraMedia(zeroConsiderado);
  if (giroY >= giroYKILL) {
  pararMotores(sentidoVariacaoAnterior, pwmAnterior);
  } else {
  Input = giroY*-1;
  myPID.Compute();
  pwmAplicarMotores = Output;
  if (pwmAplicarMotores < 0) {
  pwmAplicarMotores = 0;
  }
  if (pwmAplicarMotores > 255) {
  pwmAplicarMotores = 255;
  }
  acionaMotores(sentidoVariacao, pwmAplicarMotores);
  }

}
