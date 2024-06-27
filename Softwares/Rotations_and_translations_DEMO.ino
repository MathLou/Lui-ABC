/*
MIT License

Copyright (c) 2024 Matheus Lourenço Aires de Pontes Siqueira

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
  
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40); // multiplexador servo
Adafruit_MPU6050 mpu; // giroscópio

//conexões servos
#define quadril_1  0   //quadril servo 5
#define pata_1  1  //pata servo 1
#define coxa_1 2 //coxa servo 11
#define quadril_2  3   //quadril servo 2
#define pata_2  4  //pata servo 3
#define coxa_2 5 //coxa servo 4
#define quadril_3  6   //quadril servo 6
#define pata_3  7  //pata servo 7
#define coxa_3 8 //coxa servo 8
#define quadril_4  9   //quadril servo 9
#define pata_4  10  //pata servo 10
#define coxa_4 11 //coxa servo 12

#define KNOB1 34 // entrada analógica 1
#define KNOB2 35 // entrada analógica 2
#define KNOB3 32 // entrada analógica 3

// Constantes cinemática inversa
const float l1 = 34;
const float l2 = 55;
const float l3 = 55;
float theta_1, theta_2, theta_3;

// offsets (em ângulos) de pernas : use para austar postura inicial do robô
const int of_q_1 = -3;
const int of_p_1 = 35;
const int of_c_1 = 0;
const int of_q_2 = 10; 
const int of_p_2 = 20; 
const int of_c_2 = 10; 
const int of_q_3 = 19;
const int of_p_3 = -10;
const int of_c_3 = 5;
const int of_q_4 = -10;
const int of_p_4 = 0;
const int of_c_4 = 2;
const int offsets_pernas[12]={of_q_1, of_p_1, of_c_1,of_q_2, of_p_2, of_c_2,of_q_3, of_p_3, of_c_3,of_q_4, of_p_4, of_c_4};

// Coordenadas
double h, c, betha, arg_theta1, arg_theta2, arg_theta3, arg_betha,x2,y2,z2,rotx;
//inicilizando valores do PWM, com seus respectivos coeficientes m e c
// servos: 5,1,11,2,3,4,6,7,8,9,10,12
float PWMs[12][3] = {{0,2.25,85.84},{0,2.23,73.08},{0,2.25,61.29},{0,2.24,130.21},{0,2.25,92.84},{0,2.21,60.57},{0,2.27,63.92},{0,2.22,59.97},{0,2.22,65.72},{0,2.16,65.35},{0,2.32,67.42},{0,2.22,84.11}};

// constantes giroscópio
double roll;
double pitch;
double yaw;

// constantes para controle em malha fechada
double x_init, y_init, z_init, prev_error_1, cumulative_error_1, prev_error_2, cumulative_error_2, P_1,I_1,D_1,P_2,I_2,D_2;

double integral_error[2] = {0,0};
double prev_error[2] = {0,0};

// posições globais de coordenadas de cada perna
float Z_legs[4];
float Y_legs[4];
float X_legs[4];

// coordenadas transformadas com rotação
float X_legs_T[4];
float Y_legs_T[4];
float Z_legs_T[4];

//pontos da curva de bézier
float po[2], p1[2], p2[2], p3[2];

// constantes moving avg e lowpass filter
double R_hat,P_hat,roll_prev,pitch_prev;
float K = 4.25;
double alpha = 0.387;
int count = 0;
unsigned long start_time;
int k = 0;

void setup(){
  // inicializando MPU6050
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); //4 ou 8 g
  mpu.setGyroRange(MPU6050_RANGE_250_DEG); //250 antes
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  // término de MPU6050
  // Knob input
  pinMode(KNOB1,INPUT);
  pinMode(KNOB2,INPUT);
  pinMode(KNOB3,INPUT);
  // Inicializando PCA9685
  pca9685.begin();
  // Frequência de PWM 50Hz
  pca9685.setPWMFreq(50);
  // posições iniciais para todos
  x_init = 0.0;
  y_init = l1;
  z_init = 90.0;
  pos_inicial(x_init, y_init, z_init);
  delay(2000);
  //iniciando variável de controle estático
  prev_error_1 =0;
  cumulative_error_1 = 0;
  roll = 0;
  prev_error_2 =0;
  cumulative_error_2 = 0;
  pitch = 0;
  Z_legs[0] = z_init;
  Z_legs[1] = z_init;
  Z_legs[2] = z_init;
  Z_legs[3] = z_init;
  X_legs[0] = x_init;
  X_legs[1] = x_init;
  X_legs[2] = x_init;
  X_legs[3] = x_init;
  Y_legs[0] = y_init;
  Y_legs[1] = y_init;
  Y_legs[2] = y_init;
  Y_legs[3] = y_init;
  R_hat = 0; // roll moving avg
  P_hat = 0; // pitch moving avg
  roll_prev = 0;
  pitch_prev = 0;
  start_time = millis();
  //Serial.println("Começo de coleta de dados"); // para identificação de sistemas
  // pontos para curva de Bézier (fase de swing)
  po[0] = 20; po[1] = z_init; // 20
  p1[0] = 20; p1[1] = z_init-25; //20
  p2[0] = -20; p2[1] = z_init-25; //20
  p3[0] = -20; p3[1] = z_init; //20
  k = 0;
}
void loop() {
 //Rotations testing
  int angle = 10;
  // centers of rotations
  int x_ctr = 0;
  int y_ctr = 0;
  int z_ctr = 0;
  
  for ( int i =-angle; i <=angle; i++){
    Rotations(i,0,0,x_ctr,y_ctr,z_ctr,true, true, true,true,false);
    delay(50);
  }
  for ( int i =angle; i >=-angle; i--){
    Rotations(i,0,0,x_ctr,y_ctr,z_ctr,true, true, true,true,false);
    delay(50);
  }
  for ( int i =-angle; i <=angle; i++){
    Rotations(0,i,0,x_ctr,y_ctr,z_ctr,true, true, true,true,false);
    delay(50);
  }
  for ( int i =angle; i >=-angle; i--){
    Rotations(0,i,0,x_ctr,y_ctr,z_ctr,true, true, true,true,false);
    delay(50);
  }
  for ( int i =-angle; i <=angle; i++){
    Rotations(0,0,i,0,0,0,true, true, true,true,false);
    delay(50);
  }
  for ( int i =angle; i >=-angle; i--){
    Rotations(0,0,i,0,0,0,true, true, true,true,false);
    delay(50);
  }
  delay(1000);
  //Translations testing
  translation_demo(1);//change steps from 1 to 10.
}
void roll_pitch_equilibrium(float roll, float yaw, float pitch, double P_1, double I_1, double D_1, double P_2, double I_2, double D_2, bool static_on, int k){
  double err_1 = -roll; // erro de controle 1
  double err_2 = pitch - 3.62; // erro de controle 2
  double U_roll  = P_1*err_1+I_1*integral_error[0] +D_1*(err_1 - prev_error[0]);
  double U_pitch = P_2*err_2+I_2*integral_error[1] +D_2*(err_2 - prev_error[1]);
  if (static_on){
    Rotations(U_roll,yaw, U_pitch,0,0,0,true, true, true, true,false); 
  }else{
    Rotations(U_roll,yaw, U_pitch,0,0,0,false, false, false, false,false);
  }
  if (k%3==0){
    Serial.print("Setpoint:");//labels
    Serial.print(0); // setpoint
    Serial.print(",");
    Serial.print("Roll:");//labels
    Serial.print(roll); //erro roll
    Serial.print(",");
    Serial.print("Pitch:");//labels
    Serial.println(err_2); //erro pitch
  }
  //atualizando valores de controle
  integral_error[0] = integral_error[0] + err_1; //valores de erro integral
  integral_error[1] = integral_error[1] + err_2;
  prev_error[0] = err_1; //valores passados para ganho derivativo
  prev_error[1] = err_2;
}
void trot_gait(float stp, int dly,float roll, float yaw, float pitch, float x_ctr, float y_ctr, float z_ctr, int forward, bool control_enable){
  float px,pz,u,stence;
  //k = 0;
  //stp = 0.05;// de 0.03 até 0.05 (com delay de 40ms) (sem delay de 0.009 até 0.015)
  for (u = 0;u<=1; u = u+stp){
    px = pow((1-u),3)*po[0]+3*u*pow((1-u),2)*p1[0]+3*pow(u,2)*(1-u)*p2[0]+pow(u,3)*p3[0];
    pz = pow((1-u),3)*po[1]+3*u*pow((1-u),2)*p1[1]+3*pow(u,2)*(1-u)*p2[1]+pow(u,3)*p3[1];
    //aplicando primeiro valor de rotação
    if(u==0){
      Rotations(-2,yaw,-1,0,-5,z_ctr,false,false,false,false,false);
    }
    //escalando valores do stence
    stence = map((int)100*u,0,100,po[0],p3[0]);
    //swing de pernas 3 e 2
    coordenadas3D(-px,Y_legs_T[2],pz,quadril_3,pata_3,coxa_3,false); // swing perna 3
    coordenadas3D(px,Y_legs_T[1],pz,quadril_2,pata_2,coxa_2,false); // swing perna 2
    //stence de pernas 1 e 4
    coordenadas3D(X_legs_T[0] - stence,Y_legs_T[0],Z_legs_T[0],quadril_1,pata_1,coxa_1,false);
    coordenadas3D(X_legs_T[3] + stence,Y_legs_T[3],Z_legs_T[3],quadril_4,pata_4,coxa_4,false);
    if (control_enable){
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      roll = 180/PI*atan(a.acceleration.y/(sqrt(pow(a.acceleration.x,2)+pow(a.acceleration.z,2))));
      pitch = 180/PI*atan(-a.acceleration.x/(sqrt(pow(a.acceleration.y,2)+pow(a.acceleration.z,2))));
      // filtro passa baixa
      R_hat = R_hat*0.70558967 +0.14720516*roll+0.14720516*roll_prev;
      P_hat = P_hat*0.70558967 +0.14720516*pitch + 0.14720516*pitch_prev;
      roll_prev = roll;
      pitch_prev = pitch;
      roll_pitch_equilibrium(-R_hat,0,P_hat,.35,0.01,0.05,0.35,0.01,0.05,false,k);
    }
    delay(dly);
    k++;
  }
  //reiniciando constantes de controle
  integral_error[0]=0;
  integral_error[1]=0;
  prev_error[0]=0;
  prev_error[0]=0;
  for (u = 0;u<=1; u = u+stp){
    px = pow((1-u),3)*po[0]+3*u*pow((1-u),2)*p1[0]+3*pow(u,2)*(1-u)*p2[0]+pow(u,3)*p3[0];
    pz = pow((1-u),3)*po[1]+3*u*pow((1-u),2)*p1[1]+3*pow(u,2)*(1-u)*p2[1]+pow(u,3)*p3[1];
    if(u==0){
      Rotations(2,yaw,-2,0,5,z_ctr,false,false,false,false,false);
    }
    //escalando valores do stence
    stence = map((int)100*u,0,100,po[0],p3[0]);
    //swing pernas 4 e 1
    coordenadas3D(-px,Y_legs_T[3],pz,quadril_4,pata_4,coxa_4,false); // swing perna 4
    coordenadas3D(px,Y_legs_T[0],pz,quadril_1,pata_1,coxa_1,false); // swing perna 1
    //stence perna 2 e 3
    coordenadas3D(X_legs_T[2] + stence,Y_legs_T[2],Z_legs_T[2],quadril_3,pata_3,coxa_3,false);
    coordenadas3D(X_legs_T[1] - stence,Y_legs_T[1],Z_legs_T[1],quadril_2,pata_2,coxa_2,false);
    if (control_enable){
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      roll = 180/PI*atan(a.acceleration.y/(sqrt(pow(a.acceleration.x,2)+pow(a.acceleration.z,2))));
      pitch = 180/PI*atan(-a.acceleration.x/(sqrt(pow(a.acceleration.y,2)+pow(a.acceleration.z,2))));
      // filtro passa baixa
      R_hat = R_hat*0.70558967 +0.14720516*roll+0.14720516*roll_prev;
      P_hat = P_hat*0.70558967 +0.14720516*pitch + 0.14720516*pitch_prev;
      roll_prev = roll;
      pitch_prev = pitch;
      roll_pitch_equilibrium(-R_hat,0,P_hat,.25,0.01,0.05,0.35,0.01,0.05,false,k);
    }
    delay(dly);
    k++;
    //Serial.println(millis()-start_time);
    //start_time = millis();
  }
  //reiniciando constantes de controle
  integral_error[0]=0;
  integral_error[1]=0;
  prev_error[0]=0;
  prev_error[0]=0;
}
void bound_gait(float stp, int dly,float roll, float yaw, float pitch, float x_ctr, float y_ctr, float z_ctr){
  float px,pz,u,stence;
  for (u = 0;u<=1; u = u+stp){
    px = pow((1-u),3)*po[0]+3*u*pow((1-u),2)*p1[0]+3*pow(u,2)*(1-u)*p2[0]+pow(u,3)*p3[0];
    pz = pow((1-u),3)*po[1]+3*u*pow((1-u),2)*p1[1]+3*pow(u,2)*(1-u)*p2[1]+pow(u,3)*p3[1];
    //aplicando primeiro valor de rotação
    if(u==0){
      Rotations(0,yaw,0,0,0,z_ctr,false,false,false,false,false);
    }
    //escalando valores do stence
    stence = map((int)100*u,0,100,po[0],p3[0]);
    //swing de pernas 3 e 4
    coordenadas3D(px,Y_legs_T[2],pz,quadril_3,pata_3,coxa_3,false); // swing perna 3
    coordenadas3D(px,Y_legs_T[3],pz,quadril_4,pata_4,coxa_4,false); // swing perna 4
     // swing perna 4
    //stence de pernas 1 e 2
    coordenadas3D(X_legs_T[0] + stence,Y_legs_T[0],Z_legs_T[0],quadril_1,pata_1,coxa_1,false);
    coordenadas3D(X_legs_T[1] + stence,Y_legs_T[1],Z_legs_T[1],quadril_2,pata_2,coxa_2,false);
    
    delay(dly);
  }
  for (u = 0;u<=1; u = u+stp){
    px = pow((1-u),3)*po[0]+3*u*pow((1-u),2)*p1[0]+3*pow(u,2)*(1-u)*p2[0]+pow(u,3)*p3[0];
    pz = pow((1-u),3)*po[1]+3*u*pow((1-u),2)*p1[1]+3*pow(u,2)*(1-u)*p2[1]+pow(u,3)*p3[1];
    if(u==0){
      Rotations(0,yaw,0,0,0,z_ctr,false,false,false,false,false);
    }
    //escalando valores do stence
    stence = map((int)100*u,0,100,po[0],p3[0]);
    //swing pernas 2 e 1
    coordenadas3D(-px,Y_legs_T[1],pz,quadril_2,pata_2,coxa_2,false); // swing perna 2
    coordenadas3D(-px,Y_legs_T[0],pz,quadril_1,pata_1,coxa_1,false); // swing perna 1
    //stence perna 3 e 4
    coordenadas3D(X_legs_T[2] - stence,Y_legs_T[2],Z_legs_T[2],quadril_3,pata_3,coxa_3,false);
    coordenadas3D(X_legs_T[3] - stence,Y_legs_T[3],Z_legs_T[3],quadril_4,pata_4,coxa_4,false);
    delay(dly);
  }
}
void Rotations(float roll, float yaw, float pitch, float center_x, float center_y, float center_z, bool leg_1_on, bool leg_2_on, bool leg_3_on, bool leg_4_on,bool print){
  //offsets
  float Xoff = 75.38;
  float Yoff = 40.84;
  float Zoff = 0;

  //convertendo para radianos
  roll = PI*roll/180.0;
  yaw = PI*yaw/180.0;
  pitch = PI*pitch/180.0;

  // general rotation matrix 
  float rot[3][3] = {{cos(roll)*cos(yaw),sin(pitch)*sin(roll)*cos(yaw)-cos(pitch)*sin(yaw),cos(pitch)*sin(roll)*cos(yaw)+sin(pitch)*sin(yaw)},
  {cos(roll)*sin(yaw),sin(pitch)*sin(roll)*sin(yaw)+cos(pitch)*cos(yaw),cos(pitch)*sin(roll)*sin(yaw)-sin(pitch)*cos(yaw)},
  {-sin(roll),sin(pitch)*cos(roll), cos(pitch)*cos(roll)}
  };

  float A14 = rot[0][0]*center_x + rot[0][1]*center_y + rot[0][2]*center_z;
  float A24 = rot[1][0]*center_x + rot[1][1]*center_y + rot[1][2]*center_z;
  float A34 = rot[2][0]*center_x + rot[2][1]*center_y + rot[2][2]*center_z;
  // leg 1
  float FR[3] = {(rot[0][0]*X_legs[0] + rot[0][1]*Y_legs[0] + rot[0][2]*Z_legs[0] + rot[0][0]* Xoff + rot[0][1]*Yoff+ A14 + rot[0][2]*Zoff),
  (rot[1][0]*X_legs[0] + rot[1][1]*Y_legs[0] + rot[1][2]*Z_legs[0] + rot[1][0]* Xoff + rot[1][1]*Yoff+ A24 + rot[1][2]*Zoff),
  (rot[2][1]*X_legs[0] + rot[2][1]*Y_legs[0] + rot[2][2]*Z_legs[0] + rot[2][0]* Xoff + rot[2][1]*Yoff+ A34 + rot[2][2]*Zoff)
  };
  //leg 2
  float FL[3] = {(rot[0][0]*X_legs[1] - rot[0][1]*Y_legs[1] + rot[0][2]*Z_legs[1] + rot[0][0]* Xoff - rot[0][1]*Yoff+ A14 + rot[0][2]*Zoff),
  (rot[1][0]*X_legs[1] - rot[1][1]*Y_legs[1] + rot[1][2]*Z_legs[1] + rot[1][0]* Xoff - rot[1][1]*Yoff+ A24 + rot[1][2]*Zoff),
  (rot[2][1]*X_legs[1] - rot[2][1]*Y_legs[1] + rot[2][2]*Z_legs[1] + rot[2][0]* Xoff - rot[2][1]*Yoff+ A34 + rot[2][2]*Zoff)
  };
  //leg 3
  float RR[3] = {(-rot[0][0]*X_legs[2] + rot[0][1]*Y_legs[2] + rot[0][2]*Z_legs[2] - rot[0][0]* Xoff + rot[0][1]*Yoff+ A14 + rot[0][2]*Zoff),
  (-rot[1][0]*X_legs[2] + rot[1][1]*Y_legs[2] + rot[1][2]*Z_legs[2] - rot[1][0]* Xoff + rot[1][1]*Yoff+ A24 + rot[1][2]*Zoff),
  (-rot[2][1]*X_legs[2] + rot[2][1]*Y_legs[2] + rot[2][2]*Z_legs[2] - rot[2][0]* Xoff + rot[2][1]*Yoff+ A34 + rot[2][2]*Zoff)
  };
  //leg 4
  float RL[3] = {(-rot[0][0]*X_legs[3] - rot[0][1]*Y_legs[3] + rot[0][2]*Z_legs[3] - rot[0][0]* Xoff - rot[0][1]*Yoff+ A14 + rot[0][2]*Zoff),
  (-rot[1][0]*X_legs[3] - rot[1][1]*Y_legs[3] + rot[1][2]*Z_legs[3] - rot[1][0]* Xoff - rot[1][1]*Yoff+ A24 + rot[1][2]*Zoff),
  (-rot[2][1]*X_legs[3] - rot[2][1]*Y_legs[3] + rot[2][2]*Z_legs[3] - rot[2][0]* Xoff - rot[2][1]*Yoff+ A34 + rot[2][2]*Zoff)
  };
  // escrevendo para array de coordenadas transformadas
  X_legs_T[0] = FR[0] - Xoff; X_legs_T[1] = FL[0] - Xoff; X_legs_T[2] = -RR[0] - Xoff; X_legs_T[3] = -RL[0] - Xoff;
  Y_legs_T[0] = FR[1] - Yoff; Y_legs_T[1] = -FL[1] - Yoff;  Y_legs_T[2] = RR[1] - Yoff; Y_legs_T[3]= -RL[1] - Yoff;
  Z_legs_T[0] = FR[2]; Z_legs_T[1] = FL[2];  Z_legs_T[2] = RR[2];  Z_legs_T[3]= RL[2];

  //escrevendo para cada perna
  if (leg_1_on){coordenadas3D(FR[0] - Xoff,FR[1] - Yoff,FR[2],quadril_1, pata_1, coxa_1, print);}
  if (leg_2_on){coordenadas3D(FL[0] - Xoff,-FL[1] - Yoff,FL[2],quadril_2, pata_2, coxa_2, print);}
  if (leg_3_on){coordenadas3D(-RR[0] - Xoff,RR[1] - Yoff,RR[2],quadril_3, pata_3, coxa_3, print);}
  if (leg_4_on){coordenadas3D(-RL[0] - Xoff,-RL[1] - Yoff,RL[2],quadril_4, pata_4, coxa_4, print);}

  //delay(40);
}
void dance(int stp){
  // translação em x
  int x,y;
  x = 15;
   // translação em y
   for (y = l1-30; y <=(l1+20);y=y+stp){
    //pernas 1-4
    coordenadas3D(x,y,90,quadril_1,pata_1,coxa_1,false);
    coordenadas3D(-x,l1-y,90,quadril_4,pata_4,coxa_4,false);
    //pernas 2-3
    coordenadas3D(x,l1-y,90,quadril_2,pata_2,coxa_2,false);
    coordenadas3D(-x,y,90,quadril_3,pata_3,coxa_3,false);
     delay(40);
  }
  // translação em x
  for (x = 12; x >=-42;x=x-stp){
    //pernas 1-4
    coordenadas3D(x,y,90,quadril_1,pata_1,coxa_1,false);
    coordenadas3D(-x,l1-y,90,quadril_4,pata_4,coxa_4,false);
    //pernas 2-3
    coordenadas3D(x,l1-y,90,quadril_2,pata_2,coxa_2,false);
    coordenadas3D(-x,y,90,quadril_3,pata_3,coxa_3,false);
    delay(40);
  }
  for (y = l1+20; y >=(l1-30);y=y-stp){
    //pernas 1-4
    coordenadas3D(x,y,90,quadril_1,pata_1,coxa_1,false);
    coordenadas3D(-x,l1-y,90,quadril_4,pata_4,coxa_4,false);
    //pernas 2-3
    coordenadas3D(x,l1-y,90,quadril_2,pata_2,coxa_2,false);
    coordenadas3D(-x,y,90,quadril_3,pata_3,coxa_3,false);
    delay(40);
  }for (x = -42; x <=15;x=x+stp){
    //pernas 1-4
    coordenadas3D(x,y,90,quadril_1,pata_1,coxa_1,false);
    coordenadas3D(-x,l1-y,90,quadril_4,pata_4,coxa_4,false);
    //pernas 2-3
    coordenadas3D(x,l1-y,90,quadril_2,pata_2,coxa_2,false);
    coordenadas3D(-x,y,90,quadril_3,pata_3,coxa_3,false);
    delay(40);
  }
}
void pos_inicial(int x_0, int y_0, int z_0){
  //pernas 1-4
  coordenadas3D(x_0,y_0,z_0,quadril_1,pata_1,coxa_1,true);
  coordenadas3D(x_0,y_0,z_0,quadril_4,pata_4,coxa_4,true);
  //pernas 2-3
  coordenadas3D(x_0,y_0,z_0,quadril_2,pata_2,coxa_2,true);
  coordenadas3D(x_0,y_0,z_0,quadril_3,pata_3,coxa_3,true);
}
void translation_demo(int stp){
  // translação em x
  for (int x = -42; x <15;x=x+stp){
    //pernas 1-4
    coordenadas3D(x,l1,90,quadril_1,pata_1,coxa_1,false);
    coordenadas3D(-x,l1,90,quadril_4,pata_4,coxa_4,false);
    //pernas 2-3
    coordenadas3D(x,l1,90,quadril_2,pata_2,coxa_2,false);
    coordenadas3D(-x,l1,90,quadril_3,pata_3,coxa_3,false);
    delay(40);
  }
  for (int x = 12; x >-42;x=x-stp){
    //pernas 1-4
    coordenadas3D(x,l1,90,quadril_1,pata_1,coxa_1,false);
    coordenadas3D(-x,l1,90,quadril_4,pata_4,coxa_4,false);
    //pernas 2-3
    coordenadas3D(x,l1,90,quadril_2,pata_2,coxa_2,false);
    coordenadas3D(-x,l1,90,quadril_3,pata_3,coxa_3,false);
    delay(40);
  }
  // translação em y
   for (float y = l1; y <(l1+25);y=y+stp){
    //pernas 1-4
    coordenadas3D(0,y,90,quadril_1,pata_1,coxa_1,false);
    coordenadas3D(0,l1-y+40,90,quadril_4,pata_4,coxa_4,false);
    //pernas 2-3
    coordenadas3D(0,l1-y+40,90,quadril_2,pata_2,coxa_2,false);
    coordenadas3D(0,y,90,quadril_3,pata_3,coxa_3,false);
     delay(40);
  }
  for (float y = l1+25; y >(l1);y=y-stp){
    //pernas 1-4
    coordenadas3D(0,y,90,quadril_1,pata_1,coxa_1,false);
    coordenadas3D(0,l1-y+40,90,quadril_4,pata_4,coxa_4,false);
    //pernas 2-3
    coordenadas3D(0,l1-y+40,90,quadril_2,pata_2,coxa_2,false);
    coordenadas3D(0,y,90,quadril_3,pata_3,coxa_3,false);
    delay(40);
  }
  // translação em z
  for (int z = 90; z > 65;z=z-stp){
    //pernas 1-4
    coordenadas3D(0,l1,z,quadril_1,pata_1,coxa_1,false);
    coordenadas3D(0,l1,z,quadril_4,pata_4,coxa_4,false);
    //pernas 2-3
    coordenadas3D(0,l1,z,quadril_2,pata_2,coxa_2,false);
    coordenadas3D(0,l1,z,quadril_3,pata_3,coxa_3,false);
    delay(40);
  }
   for (int z = 65; z < 90;z=z+stp){
    //pernas 1-4
    coordenadas3D(0,l1,z,quadril_1,pata_1,coxa_1,false);
    coordenadas3D(0,l1,z,quadril_4,pata_4,coxa_4,false);
    //pernas 2-3
    coordenadas3D(0,l1,z,quadril_2,pata_2,coxa_2,false);
    coordenadas3D(0,l1,z,quadril_3,pata_3,coxa_3,false);
     delay(40);
  }
}
void angle_debug(int quadril, int pata, int coxa){
  int angle1 = map(analogRead(KNOB1),0,4095,0,180);
  int angle2 = map(analogRead(KNOB2),0,4095,0,180);
  int angle3 = map(analogRead(KNOB3),0,4095,0,180);
  servoWrite(pata,angle1,true);
  servoWrite(coxa,angle2,true);
  servoWrite(quadril,angle3,true);
  delay(20);
}
void servoCalibrar(int servo){
  int leitura = map(analogRead(KNOB1),0,4095,60,700);
  PWMs[servo][0] = roundf(leitura);
  pca9685.setPWM(servo, 0, (int)PWMs[servo][0]);
  // Print to serial monitor
  Serial.print("Servo ");
  Serial.print(servo);
  Serial.print(", PWM: ");
  Serial.println(leitura);
  delay(20);
}
void servoWrite(int servo, int angle_deg, bool print){

  PWMs[servo][0] = PWMs[servo][1]*(angle_deg+offsets_pernas[servo]) + PWMs[servo][2];
  pca9685.setPWM(servo, 0, PWMs[servo][0]);
  // Print to serial monitor
  if (print){
    if(servo == coxa_1 || servo == coxa_2 || servo == coxa_3|| servo == coxa_4){
      Serial.print("Coxa ");
    }else if (servo == quadril_1 || servo == quadril_2 || servo == quadril_3 || servo == quadril_4){
      Serial.print("Quadril ");
    }else{
      Serial.print("Pata ");
    }
      Serial.print("angulo: ");
      Serial.println(angle_deg);
  }
  //delay(1);
}
void coordenadas3D(double x, double y, double z, int quadril, int pata, int coxa, bool print){
  c = sqrt(z*z+y*y);
  theta_1 = PI - atan2(z,y)-asin(l1/c);
  if(0 <= theta_1 && theta_1 <=2*PI){
    //aplicando transformação de coordenadas
    rotx = - PI/2 + theta_1;
    z2 = y*sin(rotx)+z*cos(rotx);
    h = sqrt(z2*z2+x*x);
    //checando por inconsistências
    arg_theta2 = (h*h+l2*l2-l3*l3)/(2*l2*h);
    // betha angle
    betha = (-h*h+l2*l2+l3*l3)/(2*l2*l3);
      if ((arg_theta2 >=-1 && arg_theta2 <=1) && (betha >= -1 && betha <= 1)){
        theta_2 = acos(arg_theta2)-atan2(x,z);
        theta_3 = -theta_2+ 3*PI/2 - acos(betha);
          if ((quadril == 0 && pata == 1 && coxa == 2)||(quadril == 9 && pata == 10 && coxa == 11)){
            servoWrite(pata,180/PI*theta_3,print);
            servoWrite(coxa,theta_2*180/PI,print);
            servoWrite(quadril,theta_1*180/PI,print);
          }else{
            servoWrite(pata,180 - 180/PI*theta_3,print);
            servoWrite(coxa,180- theta_2*180/PI,print);
            servoWrite(quadril,180 - theta_1*180/PI,print);
          }
          if (print){
            // Imprimindo valores na interface serial
            Serial.print("[");
            Serial.print(x);
            Serial.print(",");
            Serial.print(y);
            Serial.print(",");
            Serial.print(z);
            Serial.println("]");
          }
    }
  }
}
