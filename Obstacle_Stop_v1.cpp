/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
   B-O-S-S firmware rev 1.0 
   creator: Ferruccio Vicari
*/




#include<Servo.h>
#include <PID_v1.h>

Servo pitch;
Servo roll;


double pitch_pid_input_front,pitch_pid_input_back, pitch_pid_output_front,pitch_pid_output_back;
double roll_pid_input_left, roll_pid_input_right, roll_pid_output_left, roll_pid_output_right;
double setpoint= YOUR VALUE;
int out_pitch=5;
int out_roll=6;
float pitch_in;
float roll_in;
float mode_in;
float pitch_out;
float roll_out;
int distance_front;
int distance_back;
int distance_left;
int distance_right;
float pitch_output;
const int sonarFront = 14;
const int sonarBack = 15;
const int sonarLeft = 16;
const int sonarRight = 17;
long _sonarFront, _sonarBack, _sonarLeft, _sonarRight;
float pitch_constrained;
float roll_constrained;
int distance_threshold= YOUR VALUE;
const int numReadings=5;
int readingsFront[numReadings],readingsBack[numReadings], readingsLeft[numReadings],readingsRight[numReadings];
int readIndex_front=0,readIndex_back=0,readIndex_left=0,readIndex_right=0;
int totalFront=0, totalBack=0, totalLeft=0, totalRight=0;
int averageFront=0, averageBack=0, averageLeft=0, averageRight=0;






void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(23,INPUT);
pinMode(22,INPUT);
pinMode(21,INPUT);
pitch.attach(5);
roll.attach(6);

for (int i=0; i<numReadings; i++){
  readingsFront[i]=0;
  readingsBack[i]=0;
  readingsLeft[i]=0;
  readingsRight[i]=0;  
  }
}

void loop() {
  
  // put your main code here, to run repeatedly:
pitch_in=pulseIn(23,HIGH,21000);
roll_in=pulseIn(22,HIGH,21000);
mode_in=pulseIn(21,HIGH,21000);

get_distance();
pid_output();  
pwm_output();

 

}

void get_distance(){

  {  totalFront= totalFront - readingsFront[readIndex_front];
     readingsFront[readIndex_front]= analogRead(sonarFront);
     totalFront= totalFront + readingsFront[readIndex_front];
     readIndex_front=readIndex_front+1;

     if (readIndex_front >= numReadings){
     readIndex_front=0;
     }
     averageFront=totalFront/numReadings;
     distance_front = averageFront;  
     Serial.println(distance_front);
     delay (2);
  
  }

  {  totalBack= totalBack - readingsBack[readIndex_back];
     readingsBack[readIndex_back]= analogRead(sonarBack);
     totalBack= totalBack + readingsBack[readIndex_back];
     readIndex_back=readIndex_back+1;
    
  if (readIndex_back >= numReadings){
     readIndex_back=0;
     }
     averageBack=totalBack/numReadings;
     distance_back = averageBack;  
     Serial.println(distance_back);
     delay(2);
  
  }

  {
     totalLeft= totalLeft - readingsLeft[readIndex_left];
     readingsLeft[readIndex_left]= analogRead(sonarLeft);
     totalLeft= totalLeft + readingsLeft[readIndex_left];
     readIndex_left=readIndex_left+1;

     if (readIndex_left >= numReadings){
     readIndex_left=0;}
     averageLeft=totalLeft/numReadings;
     distance_left = averageLeft;  
     Serial.println(distance_left);
     delay(2);
  
  }

  {
     totalRight= totalRight - readingsRight[readIndex_right];
     readingsRight[readIndex_right]= analogRead(sonarRight);
     totalRight= totalRight + readingsRight[readIndex_right];
     readIndex_right=readIndex_right+1;

     if (readIndex_right >= numReadings){
     readIndex_right=0;}
     averageRight=totalRight/numReadings;
     distance_right = averageRight;  
     Serial.println(distance_right);
     delay(2);
  
  }
  
}

void pid_output(){
  { 
   PID frontPID(&pitch_pid_input_front, &pitch_pid_output_front, &setpoint,YOUR_P,YOUR_I,YOUR_D, DIRECT);
   frontPID.SetMode(AUTOMATIC);
   pitch_pid_input_front = distance_front ;
   frontPID.Compute(); 
  }

  { 
   PID backPID(&pitch_pid_input_back, &pitch_pid_output_back, &setpoint,YOUR_P,YOUR_I,YOUR_D, DIRECT);
   backPID.SetMode(AUTOMATIC);
   pitch_pid_input_back = distance_back ;
   backPID.Compute(); 
  }

  { 
   PID leftPID(&roll_pid_input_left, &roll_pid_output_left, &setpoint,YOUR_P,YOUR_I,YOUR_D, DIRECT);
   leftPID.SetMode(AUTOMATIC);
   roll_pid_input_left = distance_left ;
   leftPID.Compute(); 
  }

  { 
   PID rightPID(&roll_pid_input_right, &roll_pid_output_right, &setpoint,YOUR_P,YOUR_I,YOUR_D, DIRECT);
   rightPID.SetMode(AUTOMATIC);
   roll_pid_input_right = distance_right ;
   rightPID.Compute(); 
  }
     
  
}

void pwm_output(){
  if(mode_in>1500){
   if(distance_front <= distance_threshold || distance_back <= distance_threshold || distance_left <= distance_threshold || distance_right <= distance_threshold){
  Serial.println(mode_in);
  pitch_constrained=map(pitch_in,1000,2000,1250,1750);
  roll_constrained=map(roll_in,1000,2000,1250,1750);  
  pitch_out = pitch_pid_output_front + pitch_constrained - pitch_pid_output_back;
  roll_out = roll_pid_output_left - roll_pid_output_right + roll_constrained; 
  }else{
   pitch_out=pitch_in;
  roll_out=roll_in;      
  }
   }else{
  pitch_out=pitch_in;
  roll_out=roll_in;  
  }
    
  pitch.writeMicroseconds(pitch_out);
  roll.writeMicroseconds(roll_out); 
}
