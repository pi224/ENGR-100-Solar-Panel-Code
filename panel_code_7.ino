#include <Servo.h>
#include <SPI.h>
#include <EEPROM.h>
#include <boards.h>
#include <RBL_nRF8001.h>
#include "Boards.h"

//*NOTE: Representation invariants for servo angles:
//vertical angle should ALWAYS be between 60(max) and 125(min) degrees
//horizontal angle can be between 0 and 180 degrees

Servo horizontal, vertical;
const int horizontal_pin = 3, vertical_pin = 4;
const int horizontal_address = 2, vertical_address = 502;

const int horizontal_shutoff = 0, vertical_shutoff = 0, upper_bound = 5;
const float alpha = 1, momentum = 1;

int vertical_angle = 80;
int horizontal_angle = 90;

void setup(){
    Serial.begin(57600);
    
    horizontal.attach(horizontal_pin);
    vertical.attach(vertical_pin);
    
    pinMode(A0, INPUT); //Left pin
    pinMode(A1, INPUT); //Right pin ???
    pinMode(A2, INPUT); //Top pin
    pinMode(A3, INPUT); //Bottom pin ???
    
    horizontal.write(getHorizontal_angle());
    vertical.write(getVertical_angle());
    
    Serial.println("Tracking system activated");

    ble_set_name("Sunny");
    ble_begin();
    
    Serial.println("Bluetooth slave activated");
    
    delay(500);
}


//use these functions to get and change the servo angles
int getHorizontal_angle(){
  if (horizontal_angle < 0){
    horizontal_angle = 0;
  }
  if (horizontal_angle > 180){
    horizontal_angle = 180;
  }
}

int getVertical_angle(){
  return vertical_angle; //vertical angle must be between 125 and 60!!!!!!!!!!
}

void saveHorizontal_angle(int angle){
  EEPROM.write(angle, horizontal_address);
  horizontal_angle = angle;
  //fix_invariants();
}

void saveVertical_angle(int angle){
  EEPROM.write(angle, vertical_address);
  vertical_angle = angle;
  //fix_invariants();
}


const int restart_threshold = 40;
bool is_optimal = false;
void loop(){
    int counter = 1;
    while (true){
        
        
        int left = analogRead(A1);
        int right = analogRead(A3);
        int top = analogRead(A2);
        int bottom = analogRead(A0);

        int to_bluetooth = left + right + top + bottom;
        to_bluetooth /= 4;
        send_data(to_bluetooth);

        if (is_optimal){
          delay(1000);
          Serial.println("We have reached optimum position");
          if (abs(left - right) > restart_threshold || abs(bottom - top) > restart_threshold){
            is_optimal = false;
          }
            break;
        }
        
        Serial.print("Beginning Cycle: ");
        Serial.print(counter);
        Serial.print("\n");
        
        Serial.print("\n");
        Serial.print("diode readings:  \n");
        Serial.print("Bottom: ");
        Serial.println(bottom);
        Serial.print("Right: ");
        Serial.println(right);
        Serial.print("Top: ");
        Serial.println(top);
        Serial.print("Left: ");
        Serial.println(left);
        Serial.print("end diode readings: \n");
        
        if (vertical_gradient(bottom, top) == 0 && horizontal_gradient(left, right)){ //should be left, right
            delay(500);
            break;
        }
        if (vertical_gradient(bottom, top) != 0){
            Serial.println("MOVING VERTICAL SERVO");
        }
        if (horizontal_gradient(left, right) != 0){
            Serial.println("MOVING HORIZONTAL SERVO");
        }
       
        
        int new_vertical = vertical_gradient(bottom, top) + getVertical_angle();
        int new_horizontal = horizontal_gradient(left, right) + getHorizontal_angle(); //should be left, right

        if (new_vertical < 60 || new_vertical > 125){
          new_vertical = 80;
        }
        if (new_horizontal < 0 || new_horizontal > 180){
            new_horizontal = 90;
        }

        
        saveHorizontal_angle(new_horizontal);
        saveVertical_angle(new_vertical);
        Serial.print("Vertical angle: ");
        Serial.println(getVertical_angle());
        Serial.print("Horizontal angle: ");
        Serial.println(new_horizontal);
         Serial.println("\n\n");
        
        horizontal.write(new_horizontal);
        vertical.write(new_vertical);
        
        delay(500);
        counter = counter + 1;
        
        record_history(horizontal_gradient(left, right), vertical_gradient(bottom, top));
        is_optimal = determine_at_opt();
      }
    
}

int horizontal_gradient(int positive, int negative){
    if (abs(positive - negative) < horizontal_shutoff){
      return 0;
    }
    int grad = alpha * (positive - negative);
    
    if (abs(grad) > upper_bound){
        grad = grad * upper_bound / abs(grad);
    }

    prev_horizontal_gradient = grad;
    return grad;
}

int vertical_gradient(int positive, int negative){
    if (abs(positive - negative) < vertical_shutoff){
      return 0;
    }
    int grad = alpha * (positive - negative);
    
    if (abs(grad) > upper_bound){
        grad =  grad * upper_bound / abs(grad);
    }

    prev_vertical_gradient = grad;
    return grad;
}


//use these functions and global variables to determine if we are optimal
int horizontal_record[] = {1, 1, 1, 1, 1}; 
int vertical_record[] = {1, 1, 1, 1, 1};
void record_history(int horizontal_gradient, int vertical_gradient){
    for (int i = 1; i < 5; i++){
        horizontal_record[i - 1] = horizontal_record[i];
        vertical_record[i - 1] = vertical_record[i];
    }
    horizontal_record[4] = horizontal_gradient / abs(horizontal_gradient);
    vertical_record[4] = vertical_gradient  / abs(vertical_gradient);
}
bool determine_at_opt(){
    int horizontal_sum = 0, vertical_sum = 0;
    for (int i = 0; i < 5; i++){
          horizontal_sum += horizontal_record[i];
          vertical_sum += vertical_record[i];
    }
    
    if (abs(horizontal_sum) == 1 && abs(vertical_sum) == 1){
        return true;
    }
    
    return false;
}


void send_data(int data){
  char message[34];
  String primer = "Sunny's avg photodiode value: ", temp;
  temp=String(data);
  temp = primer + temp;
  temp.toCharArray(message,34);
  
  Serial.print("Sending data: ");
  Serial.println(message);
  ble_write_bytes((byte*)&message, 34);
  ble_do_events();
}

