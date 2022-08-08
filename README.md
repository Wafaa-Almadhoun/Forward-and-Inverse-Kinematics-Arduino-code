# Forward-and-Inverse-Kinematics-Arduino-code




## Table of contents
* [Introduction](#Introduction)
* [Technologies](#technologies)
* [Components required](#Components-required)
* [Connections](#Connections)
* [Block diagram & simulation ](#Block-diagram-&-simulation)



## Introduction
 Here is a project for callculate and determinde kinematics  for 3 DOF robot arm. The arm consists of 
 3 servo motors and an Arduino UNO microcontroller to sets the joint angles of the servo motors using PWM signals.


## Technologies
Project is created with:
* Arduino IDE 1.8.19 [To Downloud](https://www.arduino.cc/en/software)
* AUTODESK TINKERCAD [Open](https://www.tinkercad.com/)
	
## Components required

    1. Arduino UNO
    2. jumper wirs
    3. 3 servo motor 
    4. power supply 5 v
    5. breadboard
  
   
## Connections

    Connect the 5 v output of the power supply to the positive rail of the breadboard

    Connect the ground to the negative rail of the breadboard
    
    Connect the +ve and GND of servos in breadboard
    
    connect signal pin of servos to pin 9 , 10 and 11 on Arduino uno 
    
    Connect the arduino with you laptop 
     
## Block diagram & simulation

### Forward Kinematics . [see here](https://www.tinkercad.com/things/4MeRcJJYusd-forward-kinematics/editel)

![1](https://user-images.githubusercontent.com/64277741/183325896-6c049744-3ec9-4727-bc19-c203e5e7db1c.png)

Figure (1): Befor run the cod 

![2](https://user-images.githubusercontent.com/64277741/183325953-2ecc2741-3d5c-4fe3-a951-56201a72cd20.png)

Figure (2): initial position

![3](https://user-images.githubusercontent.com/64277741/183326050-6cf46d9a-2f39-49e1-8f84-1e49cd68912c.png)

Figure (3): Enter valus into Serial Monitor for links length 

![4](https://user-images.githubusercontent.com/64277741/183326256-272f96e3-a5b9-4c93-b51f-9b2d552a0021.png)

Figure (4) : Enter valus into Serial Monitor for angles

![5](https://user-images.githubusercontent.com/64277741/183326492-14a0f4b7-cebf-4710-97f5-6c4b0481f9ac.png)

Figure (5) : The result of X , Y and total degree


#### The Code 
 //Forward-Kinematics
 
#include <Servo.h>

Servo motor1;

Servo motor2;

Servo motor3;

 // length of each link 
 
float L1;
float L2;
float L3;


float pi = 3.14159265359;


void setup() 

{
    Serial.begin(9600);
    
  motor1.attach(9);
  
  motor2.attach(10);
  
  motor3.attach(11);
  
  //initial position 
  
  motor1.write(90);
  
  motor2.write(90);
  
  motor3.write(90);
  
}
void loop() {


  // Enter the input data joint-1-angle , joint-2-angle , joint-3-angle , link-1-length , link-2-length , link-3-length ) 
  
  // we have get (x,y) gripper position and total degree
  
 Serial.println("Enter the length of the first link L1");
 
 while(Serial.available()==0){}
 
 L1=Serial.parseFloat();
 
 Serial.print("L1 is "); 
 
 Serial.println(L1);
   
 Serial.println("Enter the length of the second link L2");
 
 while(Serial.available()==0){}
 
 L2=Serial.parseFloat();
 
 Serial.print("L2 is "); 
 
 Serial.println(L2);

 Serial.println("Enter the length of the third link L3");
 
 while(Serial.available()==0){}
 
 L3=Serial.parseFloat();
 
 Serial.print("L3 is "); 
 
 Serial.println(L3);

// angles between links 


float angle1 ;

float angle2;

float angle3;

float angleTotal;      
 
float radAngle1;

float radAngle2;

float radAngle3;

float radAngleTotal;

// to compute end effector

float x;      

float y;

float x1;

float x2;

float y1;

float y2;             

  Serial.println("Enter angle1 ");
  
  while(Serial.available()==0){}
  
  angle1=Serial.parseFloat();
  
  Serial.print("angle1 = "); 
  
  Serial.println(angle1);
  
  Serial.println("Enter angle2 ");
  
  while(Serial.available()==0){}
  
  angle2=Serial.parseFloat();
  
  Serial.print("angle2 = "); Serial.println(angle2);
  

  Serial.println("Enter angle3 ");
  
  while(Serial.available()==0){}
  
  angle3=Serial.parseFloat();
  
  Serial.print("angle3 = "); Serial.println(angle3);
  
  
// convert from radians to degrees 

  radAngle1 = (angle1*pi)/180;    
  
  radAngle2 = (angle2*pi)/180;
  
  radAngle3 = (angle3*pi)/180;
  
  radAngleTotal = (angleTotal*pi)/180;
  
  motor1.write(angle1); 
  
  motor2.write(angle2);
  
  motor3.write(angle3);
  
  x = L1 * cos(radAngle1) +L2 * cos(radAngle1 + radAngle2) + L3 * cos(radAngle1 + radAngle2 + radAngle3);
  
  y = L1 * sin(radAngle1) +L2 * sin (radAngle1 + radAngle2)+ L3 * sin (radAngle1 + radAngle2 + radAngle3);
  
  angleTotal = angle1 + angle2 + angle3;
  
   delay(1000);    
   


  
 Serial.print("x is "); Serial.println(x);
 
 Serial.print("y is = "); Serial.println(y); 
 
 Serial.print("Total angle is "); Serial.println(angleTotal);
 
 Serial.print("angle1 "); Serial.println(angle1);
 
 Serial.print("angle2 "); Serial.println(angle2);
 
 Serial.print("angle3 "); Serial.println(angle3);


 }


### 2.inverse-kinematics.[see here ](https://www.tinkercad.com/things/cQybxJfa4Cd-inverse-kinematics/editel)

![1](https://user-images.githubusercontent.com/64277741/183326947-43289a42-bef3-44eb-a923-3c2ccda88d6c.png)


Figure (1): Befor run the cod 

![2](https://user-images.githubusercontent.com/64277741/183326967-b5cf4137-2dfd-4aa1-8bae-feba16ff2f0f.png)

Figure (2): initial position

![3](https://user-images.githubusercontent.com/64277741/183326985-0fa904e8-aa7c-4b38-a2f5-ce357a653d84.png)

Figure (3): Enter valus into Serial Monitor for links length 

![4](https://user-images.githubusercontent.com/64277741/183327019-8bb4e2e6-e4c3-4cd5-9eb0-aecc757f251c.png)


Figure (4) : Enter valus into Serial Monitor of X , Y and total degree

![5](https://user-images.githubusercontent.com/64277741/183327157-8a1917e0-b2ec-4c86-9a94-b0ae33a7df79.png)

Figure (5) : The result of 3 angles

#### The code 

// inverse-kinematics

#include <Servo.h>

Servo motor1;

Servo motor2;

Servo motor3;

 // length of each link (arm)  
 
float L1;

float L2;

float L3;


float pi = 3.14159265359;


void setup() 
{
    Serial.begin(9600);
    
  motor1.attach(9);
  
  motor2.attach(10);
  
  motor3.attach(11);

  
  //initial position 
  
  motor1.write(90);
  
  motor2.write(90);
  
  motor3.write(90);

  
  
}

void loop() {


  // Enter the input data  , link-1-length , link-2-length , link-3-length ,(x,y) gripper position and total degree
   
  // we have get joint-1-angle , joint-2-angle , joint-3-angle 
  
 Serial.println("Enter the length of the first link L1");
 
 while(Serial.available()==0){}
 
 L1=Serial.parseFloat();
 
 Serial.print("L1 is "); 
 
 Serial.println(L1);
   
 Serial.println("Enter the length of the second link L2");
 
 while(Serial.available()==0){}
 
 L2=Serial.parseFloat();
 
 Serial.print("L2 is "); 
 
 Serial.println(L2);

 Serial.println("Enter the length of the third arm L3");
 
 while(Serial.available()==0){}
 
 L3=Serial.parseFloat();
 
 Serial.print("L3 is "); 
 
 
 Serial.println(L3);



     // angles between links of robot arm
     
float angle1 ;

float angle2;

float angle3;

float angleTotal;   

 
float radAngle1;

float radAngle2;

float radAngle3;

float radAngleTotal;


// to compute end effector

float x;      

float y;

float x1;

float x2;

float y1;


float y2; 

Serial.println("Enter the value x ");

      while(Serial.available()==0){}
      
      x=Serial.parseFloat();
      
      Serial.print("x is "); Serial.println(x);
      
      
      Serial.println("Enter the value y ");
      
      while(Serial.available()==0){}
      
      
      y=Serial.parseFloat();
      
      Serial.print("y is "); Serial.println(y);
      

      Serial.println("Enter the Total Angle");
      
      while(Serial.available()==0){}
      
      angleTotal=Serial.parseFloat();
      
      Serial.print("total angle is  ");Serial.println(angleTotal);
      
 // convert from radians to degrees 
 
  radAngleTotal = (angleTotal*pi)/180;
  
  x2=x-L3*cos(radAngleTotal);
  
  y2=y-L3*sin(radAngleTotal);   
  
  radAngle2 = acos((sq(x2)+ sq(y2) - sq(L1) - sq(L2)) / (2*L1*L2));
  
  radAngle1= acos(((L1 + L2 * cos(radAngle2))*x2+(L2 * y2 * sin(radAngle2))) / (sq(x2)+ sq(y2)));

      angle1= (radAngle1*180)/pi;
      
      angle2= (radAngle2*180)/pi;
      
      angle3= angleTotal-angle1-angle2;
      
       delay(1000);    

      motor1.write(angle1); 
      
      motor2.write(angle2);
      
      motor3.write(angle3);
      x1 = L1 * cos(radAngle1) ;
      
      y1 = L1 * sin(radAngle1) ;
  
 Serial.print("angle1 is "); Serial.println(angle1);
 
 Serial.print("angle2 is "); Serial.println(angle2);
 
 Serial.print("angle3 is "); Serial.println(angle3);
 
 


 }



 
