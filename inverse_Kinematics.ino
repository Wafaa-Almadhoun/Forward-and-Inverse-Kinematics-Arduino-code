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



 
