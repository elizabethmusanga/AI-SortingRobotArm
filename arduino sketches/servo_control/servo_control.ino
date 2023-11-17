#include <Servo.h>

// Declare the Arduino pin where each servo is connected
#define BASE_WAIST_SERVO_PIN 11
#define WAIST_LINK1_SERVO_PIN 12
#define L1_L2_SERVO_PIN 13
#define L2_GRIPPER_BASE_SERVO_PIN 6
#define GRIPPER_SERVO_PIN 2

// Define the start configuration of the joints
#define BASE_WAIST_START 90
#define WAIST_L1_START 90
#define L1_L2_START 90
#define L2_GRIPPER_BASE_START 90
#define GRIPPER_START 0

// Register the servo motors of each joint
Servo base_waist_servo;  
Servo waist_link1_servo;  
Servo link1_link2_servo;  
Servo link2_gripperbase_servo;
Servo gripper; 

uint8_t idx = 0;
uint8_t val_idx = 0;
char value[4] = "000";


/*
 * This function moves a given servo smoothly from a given start position to a given end position.
 * The movement can be both clockwise or counterclockwise based on the values assigned to
 * the start position and end position
 */
void reach_goal(Servo& motor, int goal){
  if(goal>=motor.read()){
    // goes from the start point degrees to the end point degrees
    for (int pos = motor.read(); pos <= goal; pos += 1) { 
      motor.write(pos);     
      delay(5);                       
    }
  } else{
    // goes from the end point degrees to the start point degrees
    for (int pos = motor.read(); pos >= goal; pos -= 1) { 
      motor.write(pos);     
      delay(5);                       
    }
  }
}

void setup() {
  // Attach and Initialize each Servo to the Arduino pin where it is connected
  base_waist_servo.attach(BASE_WAIST_SERVO_PIN);
  waist_link1_servo.attach(WAIST_LINK1_SERVO_PIN);
  link1_link2_servo.attach(L1_L2_SERVO_PIN);
  link2_gripperbase_servo.attach(L2_GRIPPER_BASE_SERVO_PIN);
  gripper.attach(GRIPPER_SERVO_PIN); 

  // Set a common start point for each joint
  // This way, the start status of each joint is known
  base_waist_servo.write(BASE_WAIST_START);
  waist_link1_servo.write(WAIST_L1_START);
  link1_link2_servo.write(L1_L2_START);
  link2_gripperbase_servo.write(L2_GRIPPER_BASE_START);
  gripper.write(GRIPPER_START);

  // Start the Serial communication with ROS
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  if (Serial.available())
  {
    char chr = Serial.read();

    // base-waist motor
    if(chr == 'a')
    {
      idx = 0;
      val_idx = 0;
    }
    // waist-link1 motor
    else if(chr == 'b')
    {
      idx = 1;
      val_idx = 0;
    }
    // link1-link2 motor
    else if(chr == 'c')
    {
      idx = 2;
      val_idx = 0;
    }
    // link2-gripper base motor
    else if(chr == 'd'){
      idx = 3;
      val_idx = 0;
    }
    // gripper motor
    else if(chr == 'e')
    {
      idx = 4;
      val_idx = 0;
    }
    // Separator
    else if(chr == ',')
    {
      int val = atoi(value);
      if(idx == 0)
      {
        reach_goal(base_waist_servo, val);
      }
      else if(idx == 1)
      {
        reach_goal(waist_link1_servo, val);
      }
      else if(idx == 2)
      {
        reach_goal(link1_link2_servo, val);
      }
      else if(idx == 3){
        reach_goal(link2_gripperbase_servo, val);
      }
      else if(idx == 4)
      {
        reach_goal(gripper, val);
      }

      // reset the angle
      value[0] = '0';
      value[1] = '0';
      value[2] = '0';
      value[3] = '0';
      value[4] = '\0';
    }
    // Plain number
    else
    {
      value[val_idx] = chr;
      val_idx++;
    }
  }
}