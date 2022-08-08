//Forward-Kinematics
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




 

 
