   // #include <Servo.h>
    #include <Ultrasonic.h>
    
    int rightMotorA = 9;

    int rightMotorB = 12;

    int leftMotorA = 10;

    int leftMotorB = 11;

    int RightFarSensor = A0;

    int RightNearSensor = A1;

    int middleSensorIN = A2;
    

    int LeftNearSensor = A3;

    int LeftFarSensor = A4;

    int right_speed = 0;

    int left_speed = 0;

    int sensor[] = {A0, A1, A2, A3, A4};

    int sensors_avg = 0;

    int sensors_sum = 0;

    int position = 0;

    int proportional = 0;

    long integral = 0;

    int derivative = 0;

    int last_proportional = 0;

    int error_value = 0;

    int max_speed = 220;
    
    int seuil = 25;

int s2 = 22;
int s3 = 24;// distance minimale pour laquelle on accepte un obstacle
int s4 = 23;//
int td=800;
int tg=700; 
int speed1=255;
int speed2=255;
int poshead = 0;                        //variable for head (eyeball) position
long timerVal = 50;                     // Variable to hold the timer for servo head
int countupdown = 4;
int trigPin = 6;  
int echoPin = 7;
int sensorstate3 = 0;
Ultrasonic ultrasonic(trigPin, echoPin);
//Servo servo1;
    float Kp = 0.081; // it was 0.09

    float Ki = 0;

    float Kd = 0.041;// it was 0.06

    void setup()

    {
//Serial.begin(9600); 
//servo1.attach(3);
pinMode(trigPin, OUTPUT);
 pinMode(echoPin, INPUT);

    pinMode(rightMotorA, OUTPUT);

    pinMode(rightMotorB, OUTPUT);

    pinMode(leftMotorA, OUTPUT);

    pinMode(leftMotorB, OUTPUT);

    digitalWrite(rightMotorA, HIGH);

    digitalWrite(rightMotorB, LOW);

    digitalWrite(leftMotorA, HIGH);

    digitalWrite(leftMotorB, LOW);

    }

    void loop()

    {

  long distance;
long microsec = ultrasonic.timing();
   distance = ultrasonic.convert(microsec, Ultrasonic::CM); 

  if (distance > seuil){   
     read_sensors();

    calc_avg();

    calc_sum();

    calc_position();

    calc_proportional();

    calc_integral();

    calc_derivative();

    calc_error();

    process_error();

    set_motors(right_speed, left_speed);
 
   
  }
  else {
     set_motors(0, 0);

 

    
      
}
    }


    
    void read_sensors()

    {

    for (int i = 0;i < 5;i++)

    {

    sensor[i] = analogRead(i);

    if (sensor[i] < 500)

    {

    sensor[i] = 1;

    }

    else

    {

    sensor[i]=0;

    } } }

    void calc_avg()

    {

    sensors_avg = 0;

    for (int i = 1; i < 5; i ++) //maybe 6 it was 5

    {

    sensors_avg += sensor[i] * (i) * 1000;  // 1000 2000 3000 4000 5000  

    }

    }

    void calc_sum()

    {

    sensors_sum = 0;

    for (int i = 0; i < 5; i++)

    {

    sensors_sum += int(sensor[i]);

    } }

    void calc_position()

    {

    position = sensors_avg/sensors_sum;
    // Serial.print("position =  "); 
    // Serial.println(position ); 

    }

    void calc_proportional()

    {

    proportional = position-2000; //maybe 3000 // first one was 2500

    }

    void calc_integral()

    {

    integral = integral + proportional;

    }

    void calc_derivative()

    {

    derivative = proportional - last_proportional;

    last_proportional = proportional;

    }

    void calc_error()

    {

    error_value = int(proportional * Kp + integral * Ki + derivative * Kd);
    //Serial.print("error =  "); 
     //Serial.println( error_value ); 

    }

    void process_error()

    {

    if (error_value > max_speed)

    {

    error_value = max_speed;

    }

    if (error_value < -max_speed)

    {

    error_value = -max_speed;

    }

    {

    if (error_value < 0)

    {

    left_speed = max_speed + error_value;

    right_speed = max_speed;

    }

    else

    {

    left_speed = max_speed;

    right_speed = max_speed - error_value;

    } } }

    void set_motors(int right_speed , int left_speed )

    {

    analogWrite(rightMotorA,right_speed);

    analogWrite(leftMotorA,left_speed);

    }

    







void avant (void){
  
                      analogWrite(leftMotorA,230); // set pin 10 on L293D High
                      digitalWrite(rightMotorB, LOW);

                     analogWrite(rightMotorA,230); // set pin 2 on L293D High marche avant 
                     digitalWrite(leftMotorB, LOW);
                     
                     
                    }
