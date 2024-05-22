#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
//setting up connections of motor and led pins(for headlights)
int pin1 = D1;
int pin2 = D2;
int pin3 = D3;
int pin4 = D4;
int Enable1 = D0;
int Enable2 = D5;
int LedF = D6;
int LedB = D7;
// setting up credentials for nodemcu wifi access point
const char *ssid = "lelebhai";
const char *password = "12345678";
// pre-defining callback function
void Callback_func(const geometry_msgs::Twist &msg);
// making static IP for connection
IPAddress server(192, 168, 4, 2); //ROS master's IP
// initializing the node
ros::NodeHandle nh;
// making the sub subscriber
ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", Callback_func); 

void setup()
{
  // setting up pin modes
  pinMode(LED_BUILTIN , OUTPUT);
  pinMode(LedF , OUTPUT);
  pinMode(LedB , OUTPUT);
  Serial.begin(115200);
  pinMode(pin1 , OUTPUT);
  pinMode(pin2 , OUTPUT);
  pinMode(Enable1 , OUTPUT);
  pinMode(pin3 , OUTPUT);
  pinMode(pin4 , OUTPUT);
  pinMode(Enable2 , OUTPUT);
  // Set up NodeMCU as a hotspot
  WiFi.softAP(ssid, password);
  // getting the ip of server
  IPAddress apIP = WiFi.softAPIP();
  Serial.println("Hotspot is set-up");
  Serial.print("IP Address: ");
  Serial.println(apIP);
  // Connect to ROS serial server for accessing the topic /turtle1/cmd_vel on it
  nh.getHardware()->setConnection(server);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
}

void Callback_func(const geometry_msgs::Twist &msg)
{ // confirming message for debugging
  Serial.print("Received message: ");
  Serial.println(msg.angular.z);
  Serial.println(msg.linear.x);
  // mapping speed, which varies from 0 to 7 , to 0 to 255
  int speed_analog = map(msg.angular.y, 0, 7, 0 ,255);
  // turning motors according to directions
  if(msg.angular.x == 0){
    digitalWrite(LedF, LOW);
  }
  if(msg.angular.x == 1){
    digitalWrite(LedF, HIGH);
  }
  if( msg.linear.x == 2 && msg.angular.z == 0){
    digitalWrite(LED_BUILTIN, LOW); 
    analogWrite(Enable1 , speed_analog);
    analogWrite(Enable2 , speed_analog);
    digitalWrite(LedB , LOW);
    digitalWrite(pin2 , HIGH);
    digitalWrite(pin1 , LOW);
    digitalWrite(pin4 , HIGH);
    digitalWrite(pin3 , LOW);
  }
  else if( msg.linear.x == -2 && msg.angular.z == 0){ 
    analogWrite(Enable1 , speed_analog);
    analogWrite(Enable2 , speed_analog);
    digitalWrite(pin1 , HIGH);
    digitalWrite(pin2 , LOW); 
    digitalWrite(pin3 , HIGH);
    digitalWrite(pin4 , LOW);
    digitalWrite(LedB , HIGH);

  }
  else if( msg.angular.z == 2 && msg.linear.x == 0){
    analogWrite(Enable1 , speed_analog);
    analogWrite(Enable2 , speed_analog);
    digitalWrite(pin2 , LOW);
    digitalWrite(pin1 , HIGH);
    digitalWrite(pin3 , LOW);
    digitalWrite(pin4 , HIGH);
    digitalWrite(LedB , LOW);
  }
  else if( msg.angular.z == -2 && msg.linear.x == 0){
    analogWrite(Enable1 , speed_analog);
    analogWrite(Enable2 , speed_analog);
    digitalWrite(pin2 , HIGH);
    digitalWrite(pin1 , LOW);
    digitalWrite(pin3 , HIGH);
    digitalWrite(pin4 , LOW);
    digitalWrite(LedB , LOW);
  }

  else if( msg.angular.z == 2 && msg.linear.x == 2){
    analogWrite(Enable1 , 180);
    analogWrite(Enable2 , 255);
    digitalWrite(pin2 , HIGH);
    digitalWrite(pin1 , LOW);
    digitalWrite(pin3 , LOW);
    digitalWrite(pin4 , HIGH);
    digitalWrite(LedB , LOW);
  }
  else if( msg.angular.z == -2 && msg.linear.x == 2){
    analogWrite(Enable1 , 255);
    analogWrite(Enable2 , 180);
    digitalWrite(pin2 , HIGH);
    digitalWrite(pin1 , LOW);
    digitalWrite(pin3 , LOW);
    digitalWrite(pin4 , HIGH);
    digitalWrite(LedB , LOW);
  }
  else if( msg.angular.z == 2 && msg.linear.x == -2){
    analogWrite(Enable1 , 180);
    analogWrite(Enable2 , 255);
    digitalWrite(pin1 , HIGH);
    digitalWrite(pin2 , LOW);
    digitalWrite(pin4 , LOW);
    digitalWrite(pin3 , HIGH);
    digitalWrite(LedB , HIGH);

  }
  else if( msg.angular.z == -2 && msg.linear.x == -2){
    analogWrite(Enable1 , 255);
    analogWrite(Enable2 , 180);
    digitalWrite(pin1 , HIGH);
    digitalWrite(pin2 , LOW);
    digitalWrite(pin4 , LOW);
    digitalWrite(pin3 , HIGH);
    digitalWrite(LedB , HIGH);
  }
}