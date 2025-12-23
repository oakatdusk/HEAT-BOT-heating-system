
// // #include <Arduino.h>
// // #include <OneWire.h> //to use the temperature sensors
// // #include <DallasTemperature.h> //for easier work with the temp sensors
// // // #include <Adafruit_SSD1306.h> //for the OLED displays

// // // // put function declarations here:
// // // int myFunction(int, int);

// // // void setup() {
// // //   // put your setup code here, to run once:
// // //   int result = myFunction(2, 3);
// // // }

// // // void loop() {
// // //   // put your main code here, to run repeatedly:
// // // }

// // // // put function definitions here:
// // // int myFunction(int x, int y) {
// // //   return x + y;
// // // }

// // // // Data wire is plugged into GPIO21
// // // #define ONE_WIRE_BUS 21

// // // // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
// // // OneWire oneWire(ONE_WIRE_BUS);

// // // // Pass our oneWire reference to Dallas Temperature.
// // // DallasTemperature sensors(&oneWire);

// // // /*
// // //  * The setup function. We only start the sensors here
// // //  */
// // // void setup(void)
// // // {
// // //   // start serial port
// // //   Serial.begin(9600);
// // //   Serial.println("Dallas Temperature IC Control Library Demo");

// // //   // Start up the library
// // //   sensors.begin();
// // // }

// // // /*
// // //  * Main function, get and show the temperature
// // //  */
// // // void loop(void)
// // // {
// // //   // call sensors.requestTemperatures() to issue a global temperature
// // //   // request to all devices on the bus
// // //   Serial.print("Requesting temperatures...");
// // //   sensors.requestTemperatures(); // Send the command to get temperatures
// // //   Serial.println("DONE");
// // //   delay(1500);
// // //   // After we got the temperatures, we can print them here.
// // //   // We use the function ByIndex, and as an example get the temperature from the first sensor only.
// // //   float tempC = sensors.getTempCByIndex(0);

// // //   // Check if reading was successful
// // //   if (tempC != DEVICE_DISCONNECTED)
// // //   {
// // //     Serial.print("Temperature for the device 1 (index 0) is: ");
// // //     Serial.println(tempC);
// // //   }
// // //   else
// // //   {
// // //     Serial.println("Error: Could not read temperature data");
// // //   }
// // // }


// // #include <Arduino.h>
// // #include <stdio.h>
// // #include <stdint.h>
// // #include <stdbool.h>
// // #include <OneWire.h>
// // #include <DallasTemperature.h>

// // #define ONE_WIRE_BUS 1
// // #define BAUD_RATE 9600
// // #define DELAY_AMOUNT 1500

// // OneWire oneWire(ONE_WIRE_BUS);
// // DallasTemperature ds18b20_sensors(&oneWire);

// // uint8_t numberOfDevices;

// // DeviceAddress tempDeviceAddress;

// // void printAddress(DeviceAddress deviceAddress) {
// //   for (uint8_t i = 0; i < 8; i++){
// //     if (deviceAddress[i] < 16) Serial.print("0");
// //       Serial.print(deviceAddress[i], HEX);
// //   }
// // }

// // void setup(){
// //   delay(5*DELAY_AMOUNT);

// //   Serial.begin(BAUD_RATE);

// //   ds18b20_sensors.begin();
  
// //   numberOfDevices = ds18b20_sensors.getDeviceCount();
  
// //   Serial.print("Locating devices...");
// //   Serial.print("Found ");
// //   Serial.print(numberOfDevices, DEC);
// //   Serial.println(" devices.");

// //   for(int i=0;i<numberOfDevices; i++){
// //     if(ds18b20_sensors.getAddress(tempDeviceAddress, i)){
// //       Serial.print("Found device ");
// //       Serial.print(i, DEC);
// //       Serial.print(" with address: ");
// //       printAddress(tempDeviceAddress);
// //       Serial.println();
// //     } else {
// //       Serial.print("Found ghost device at ");
// //       Serial.print(i, DEC);
// //       Serial.print(" but could not detect address. Check power and cabling");
// //     }
// //   }
// // }

// // void loop(){ 
// //   ds18b20_sensors.requestTemperatures();
  
// //   for(int i=0;i<numberOfDevices; i++){
// //     if(ds18b20_sensors.getAddress(tempDeviceAddress, i)){
// //       Serial.print("Temperature for device: ");
// //       Serial.println(i,DEC);

// //       float tempC = ds18b20_sensors.getTempC(tempDeviceAddress);

// //       Serial.print("Temp C: ");
// //       Serial.println(tempC);
// //     }
// //   }
// //   delay(DELAY_AMOUNT);
// // }



// // #include <Arduino.h>
// // #include <OneWire.h> //to use the temperature sensors
// // #include <DallasTemperature.h> //for easier work with the temp sensors
// // // #include <Adafruit_SSD1306.h> //for the OLED displays

// // const int LED_OUTPUT_PIN = 18;
// // const int PWM_FREQ = 1000; //
// // const int PWM_RES = 8;
// // const int MAX_DUTY_CYCLE = 255;
// // const int DELAY_MS = 4;

// // #define ONE_WIRE_BUS 4 //what pin the temp sensors are on
// // OneWire oneWire(ONE_WIRE_BUS); //initiates onewire
// // DallasTemperature sensors(&oneWire); //initiates stuff for dallastemp



// // void test_sensor()
// // {

// // }

// // void test_heating()
// // {}

// // void setup()
// // {
// //  Serial.begin(115200);
 
// //  sensors.begin();
// //  Serial.print("Locating devices...");
// //   Serial.print("Found ");
// //   Serial.print(sensors.getDeviceCount(), DEC);
// //   Serial.println(" devices.");


// // }

// // void loop() {
// //  // put main code here
// // }




// // //code that I used to put all the addresses
// // #include <Arduino.h>
// // #include <OneWire.h>           //to use the temperature sensors
// // #include <DallasTemperature.h> //for easier work with the temp sensors

// // #define ONE_WIRE_BUS 1 // what pin the temp sensors are on
// // #define MAX_SENSORS 15 // just in case

// // OneWire oneWire(ONE_WIRE_BUS);       // initiates onewire
// // DallasTemperature sensors(&oneWire); // initiates stuff for dallastemp

// // struct temp_sensors
// // {
// //   uint8_t address[8];
// //   const char* location;
// //   float last_temp;
// // } sensor_list[MAX_SENSORS] = {
// //     // hex addresses
// //     {{0x28, 0xF4, 0x1C, 0xF8, 0x0F, 0x00, 0x00, 0xDF}, "front_legs_a"},
// //     {{0x28, 0x72, 0xE6, 0xF8, 0x0F, 0x00, 0x00, 0x9C}, "front_legs_b"},
// //     {{0x28, 0x6A, 0x18, 0xF9, 0x0F, 0x00, 0x00, 0xED}, "side_legs_left"},
// //     {{0x28, 0x49, 0x71, 0x0B, 0x10, 0x00, 0x00, 0x83}, "side_legs_right"},
// //     {{0x28, 0x99, 0xD5, 0xF8, 0x0F, 0x00, 0x00, 0x87}, "front_torso_a"},
// //     {{0x28, 0xD9, 0x33, 0xF9, 0x0F, 0x00, 0x00, 0x3B}, "front_torso_b"},
// //     {{0x28, 0xDD, 0x6D, 0xF8, 0x0F, 0x00, 0x00, 0x5F}, "right_torso"},
// //     {{0x28, 0x43, 0x5F, 0xF9, 0x0F, 0x00, 0x00, 0x75}, "left_torso_a"},
// //     {{0x28, 0xB3, 0x1A, 0x0B, 0x10, 0x00, 0x00, 0x5F}, "left_torso_b"},
// //     {{0x28, 0xF3, 0x0B, 0xF8, 0x0F, 0x00, 0x00, 0x77}, "left_shoulder"},
// //     {{0x28, 0x1B, 0x09, 0xF8, 0x0F, 0x00, 0x00, 0x7C}, "right_shoulder"},
// //     {{0x28, 0x87, 0x2D, 0x0C, 0x10, 0x00, 0x00, 0x3D}, "left_head"},
// //     {{0x28, 0x8F, 0x0F, 0x0C, 0x10, 0x00, 0x00, 0xE7}, "right_head"},
// //     {{0x28, 0xCF, 0xFF, 0x0A, 0x10, 0x00, 0x00, 0x3D}, "top_head"},
// //     {{0x28, 0xFF, 0x64, 0x1F, 0x7F, 0x87, 0x74, 0x8C}, "front_head"}
// //   };


// // void setup(void)
// // {
// //   Serial.begin(115200);
// //   delay(1000);
// //   sensors.begin();

// // }

// // void loop()
// // {
// //   Serial.println(F("Sensor testing woo"));
// // while(1) {
// // sensors.requestTemperatures();
// // for (uint8_t i = 0; i < MAX_SENSORS; i++) {
// // float t = sensors.getTempC(sensor_list[i].address);
// // Serial.printf("%s  %.2f °C\n", sensor_list[i].location, t);
// // }
// // Serial.println();
// // delay(1000);


// // }

// // }

// // //stuff
// // #include <Arduino.h>
// // #include <OneWire.h>           //to use the temperature sensors
// // #include <DallasTemperature.h> //for easier work with the temp sensors

// // #define ONE_WIRE_BUS 1 // what pin the temp sensors are on
// // #define MAX_SENSORS 15 // just in case
// // #define PWM_PIN 2      // what pin the PWM is gonna be coming out of
// // #define PWM_CHANNEL 0  // what channel pwm we are using
// // #define PWM_FREQ 1000  // the frequency for the pwm, might lower this
// // #define PWM_RES 8      //


// // #define TEMP_DIFF 7 //how much higher the target temp needs to be compared to ambient temp

// // OneWire oneWire(ONE_WIRE_BUS);       // initiates onewire
// // DallasTemperature sensors(&oneWire); // initiates stuff for dallastemp

// // struct temp_sensors //struct to store all the temperature sensor information
// // {
// //   uint8_t address[8];
// //   const char *location;
// //   float last_temp;
// // } sensor_list[MAX_SENSORS] = {
// //     // hex addresses and locations for all the sensors
// //     {{0x28, 0xF4, 0x1C, 0xF8, 0x0F, 0x00, 0x00, 0xDF}, "front_legs_a"},
// //     {{0x28, 0x72, 0xE6, 0xF8, 0x0F, 0x00, 0x00, 0x9C}, "front_legs_b"},
// //     {{0x28, 0x6A, 0x18, 0xF9, 0x0F, 0x00, 0x00, 0xED}, "side_legs_left"},
// //     {{0x28, 0x49, 0x71, 0x0B, 0x10, 0x00, 0x00, 0x83}, "side_legs_right"},
// //     {{0x28, 0x99, 0xD5, 0xF8, 0x0F, 0x00, 0x00, 0x87}, "front_torso_a"},
// //     {{0x28, 0xD9, 0x33, 0xF9, 0x0F, 0x00, 0x00, 0x3B}, "front_torso_b"},
// //     {{0x28, 0xDD, 0x6D, 0xF8, 0x0F, 0x00, 0x00, 0x5F}, "right_torso"},
// //     {{0x28, 0x43, 0x5F, 0xF9, 0x0F, 0x00, 0x00, 0x75}, "left_torso_a"},
// //     {{0x28, 0xB3, 0x1A, 0x0B, 0x10, 0x00, 0x00, 0x5F}, "left_torso_b"},
// //     {{0x28, 0xF3, 0x0B, 0xF8, 0x0F, 0x00, 0x00, 0x77}, "left_shoulder"},
// //     {{0x28, 0x1B, 0x09, 0xF8, 0x0F, 0x00, 0x00, 0x7C}, "right_shoulder"},
// //     {{0x28, 0x87, 0x2D, 0x0C, 0x10, 0x00, 0x00, 0x3D}, "left_head"},
// //     {{0x28, 0x8F, 0x0F, 0x0C, 0x10, 0x00, 0x00, 0xE7}, "right_head"},
// //     {{0x28, 0xCF, 0xFF, 0x0A, 0x10, 0x00, 0x00, 0x3D}, "top_head"},
// //     {{0x28, 0xFF, 0x64, 0x1F, 0x7F, 0x87, 0x74, 0x8C}, "front_head"}};

// // //PID variables

// // float ambient_temp = 0;
// // float setpoint = ambient_temp + TEMP_DIFF;


// // float last_errror = 0;





// // void setup(void)
// // {
// //   Serial.begin(115200);
// //   ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES); //general setup for the pwm
// //   ledcAttachPin(PWM_PIN, PWM_CHANNEL); //attaches what channel the pin will be getting pwm from
// //   ledcWrite(PWM_CHANNEL, 0); // start off
// //   sensors.begin(); //starts the temperature sensors
// // }

// // void loop()
// // {
// //   Serial.println(F("pwm testing woo"));
// //   while (1)
// //   {
// //     sensors.requestTemperatures();
// //     for (uint8_t i = 0; i < MAX_SENSORS; i++)
// //     {
// //       float t = sensors.getTempC(sensor_list[i].address);
// //       Serial.printf("%s  %.2f °C\n", sensor_list[i].location, t);
// //     }
// //     Serial.println();
// //   }
// // }




// //CODE TO TEST IF PWM IS EVEN WORKING ON A PIN
// // void loop()
// // {
// //     sensors.requestTemperatures();
// //     delay(750);
// //   for (uint8_t i = 0; i < MAX_SENSORS; i++)
// //   {
// //     float t = sensors.getTempC(sensor_list[i].address);
// //     Serial.printf("%s  %.2f °C\n", sensor_list[i].location, t);
// //   }
 
// //   if (PWM_value < 255) {
// //   PWM_value++;
// //   ledcWrite(PWM_CHANNEL, PWM_value);
// //   float percentage = (PWM_value*100)/255;
// //   Serial.printf("PWM is at %d, duty cycle %.2f.\n", PWM_value, percentage);
// //   }
// //   if (PWM_value >=255) {
// //     PWM_value=0;
// //   }
// //    delay(1000);
// // }


// //CODE THAT I WAS USING BEFORE FOR PID THAT WAS NOT REALLY WORKING
// #include <Arduino.h>
// #include <OneWire.h>           //to use the temperature sensors
// #include <DallasTemperature.h> //for easier work with the temp sensors

// #define ONE_WIRE_BUS 4 // what pin the temp sensors are on
// #define MAX_SENSORS 2  // just in case
// #define PWM_PIN 2      // what pin the PWM is gonna be coming out of
// #define PWM_CHANNEL 0  // what channel pwm we are using
// #define PWM_FREQ 1000  // the frequency for the pwm, might lower this
// #define PWM_RES 8      // 8 bit aka 0-255

// #define TEMP_DIFF 7 // how much higher the target temp needs to be compared to ambient temp

// OneWire oneWire(ONE_WIRE_BUS);       // initiates onewire
// DallasTemperature sensors(&oneWire); // initiates stuff for dallastemp

// float percentage;

// struct temp_sensors // struct to store all the temperature sensor information
// {
//   uint8_t address[8];
//   const char *location;
//   float last_temp;
// } sensor_list[MAX_SENSORS] = {
//     // hex addresses and locations for all the sensors
//     {{0x28, 0xF4, 0x1C, 0xF8, 0x0F, 0x00, 0x00, 0xDF}, "outside"},
//     {{0x28, 0x72, 0xE6, 0xF8, 0x0F, 0x00, 0x00, 0x9C}, "heat_pad"},
// };

// float ambient_temp = 0;
// float setpoint = 7;
// float current_temp = 0;

// // PID variables
// float last_error = 0;
// float integral = 0;
// unsigned long last_time = 0;
// unsigned long ambient_time = 0;

// float Kp = 40;
// float Ki = 1.3;
// float Kd = 12;

// uint8_t PWM_value = 250;
// uint8_t time_passed = 0;

// void setup(void)
// {
//   Serial.begin(9600); // needed to use serial for testing
//   delay(2500);
//    Serial.printf("test1\n");

//   // PWM hardware setup
//   ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN, PWM_CHANNEL);       // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL, 0);                 // start off

//   sensors.begin(); // starts the temperature sensors
//   Serial.println(F("pwm testing woo"));

//   // locate devices on the bus
//   Serial.printf("Locating devices...");
//   Serial.printf("Found ");
//   Serial.print(sensors.getDeviceCount(), DEC);
//   Serial.println(" devices.");

//   sensors.requestTemperatures();
//   delay(750);
//   for (uint8_t i = 0; i < MAX_SENSORS; i++)
//   {
//     float t = sensors.getTempC(sensor_list[i].address);
//     Serial.printf("%s  %.2f °C\n", sensor_list[i].location, t);
//   }
//   ambient_temp = sensors.getTempC(sensor_list[0].address);
//   setpoint = ambient_temp + TEMP_DIFF;
//   sensors.setWaitForConversion(false);
// }


// void loop()
// {
//   unsigned long now = millis();
//   if (now - last_time >= 1000) // ← run PID only once per second
//   {
//     last_time = now;
//     time_passed++;
//     sensors.requestTemperatures();
//     current_temp = sensors.getTempC(sensor_list[1].address);
//     ambient_temp = sensors.getTempC(sensor_list[0].address);
//     setpoint = ambient_temp + TEMP_DIFF;
//     float error = setpoint - current_temp;
//   //   integral += error;

//   //   if (integral > 200)
//   //     integral = 200; // anti-windup: don’t let it grow forever
//   //   if (integral < 0)
//   //     integral = 0;
//   //   float derivative = error - last_error;                       // how fast is error changing?
//   //   float output = Kp * error + Ki * integral + Kd * derivative; // actual PID formula

//   //   if (output > 255)
//   //     output = 255;
//   //   if (output < 0)
//   //     output = 0;
//   //   PWM_value = (uint8_t)output;
//   //   ledcWrite(PWM_CHANNEL, PWM_value);
//   //   last_error = error;
//     float percentage = (PWM_value*100.0)/255.0;
//     ledcWrite(PWM_CHANNEL, PWM_value);

//     Serial.printf("heating pad: %.2f °C | Error: %+.2f | PWM: %3d/255 | DC: %.2f | ambient: %.2f | %d sec passed\n",
//                   current_temp, error, PWM_value, percentage, ambient_temp, time_passed);
//   }

//   // if (PWM_value < 255) {
//   // PWM_value++;
//   // ledcWrite(PWM_CHANNEL, PWM_value);
//   // float percentage = (PWM_value*100)/255;
//   // Serial.printf("PWM: %d | duty cycle: %.2f | room temp: %.2f | heat pad temp: %.2f.\n", PWM_value, percentage, sensor_list[0].last_temp, sensor_list[1].last_temp);
//   // sensors.requestTemperatures();
//   // sensor_list[1].last_temp = sensors.getTempC(sensor_list[1].address);
//   // sensor_list[0].last_temp = sensors.getTempC(sensor_list[0].address);
//   // }
//   // if (PWM_value >=255) {
//   //   PWM_value=100;
//   // }
//   //  delay(5000);
// }

// //WAITFORCONVERSION TEST CODE
// #include <OneWire.h>
// #include <DallasTemperature.h>

// // Data wire is plugged into port 2 on the Arduino
// #define ONE_WIRE_BUS 2

// // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
// OneWire oneWire(ONE_WIRE_BUS);

// // Pass our oneWire reference to Dallas Temperature.
// DallasTemperature sensors(&oneWire);

// void setup(void)
// {
//   // start serial port
//   Serial.begin(115200);
//   Serial.println("Dallas Temperature Control Library - Async Demo");
//   Serial.println("\nDemo shows the difference in length of the call\n\n");

//   // Start up the library
//   sensors.begin();
// }

// void loop(void)
// {
//   // Request temperature conversion (traditional)
//   Serial.println("Before blocking requestForConversion");
//   unsigned long start = millis();

//   sensors.requestTemperatures();

//   unsigned long stop = millis();
//   Serial.println("After blocking requestForConversion");
//   Serial.print("Time used: ");
//   Serial.println(stop - start);

//   // get temperature
//   Serial.print("Temperature: ");
//   Serial.println(sensors.getTempCByIndex(0));
//   Serial.println("\n");

//   // Request temperature conversion - non-blocking / async
//   Serial.println("Before NON-blocking/async requestForConversion");
//   start = millis();
//   sensors.setWaitForConversion(false);  // makes it async
//   sensors.requestTemperatures();
//   sensors.setWaitForConversion(true);
//   stop = millis();
//   Serial.println("After NON-blocking/async requestForConversion");
//   Serial.print("Time used: ");
//   Serial.println(stop - start);


//   // 9 bit resolution by default
//   // Note the programmer is responsible for the right delay
//   // we could do something usefull here instead of the delay
//   int resolution = 9;
//   delay(750 / (1 << (12 - resolution)));

//   // get temperature
//   Serial.print("Temperature: ");
//   Serial.println(sensors.getTempCByIndex(0));
//   Serial.println("\n\n\n\n");

//   delay(1500);
// }


// #include <Arduino.h>
// #include <OneWire.h>           //to use the temperature sensors
// #include <DallasTemperature.h> //for easier work with the temp sensors

// #define ONE_WIRE_BUS 1 // what pin the temp sensors are on
// #define MAX_SENSORS 15  // just in case

// #define PWM_PIN_0 11
// #define PWM_PIN_1 10     // what pin the PWM is gonna be coming out of
// #define PWM_PIN_2 13
// #define PWM_PIN_3 12
// #define PWM_PIN_4 21
// #define PWM_PIN_5 14
// #define PWM_PIN_6 48
// #define PWM_PIN_7 47

// #define PWM_CHANNEL_0 0  // what channel pwm we are using
// #define PWM_CHANNEL_1 1
// #define PWM_CHANNEL_2 2
// #define PWM_CHANNEL_3 3
// #define PWM_CHANNEL_4 4
// #define PWM_CHANNEL_5 5
// #define PWM_CHANNEL_6 6
// #define PWM_CHANNEL_7 7

// #define PWM_FREQ 1000  // the frequency for the pwm, might lower this
// #define PWM_RES 10      // 10 bit

// uint8_t PWM_value_0 = 273;
// uint8_t PWM_value_1 = 109;
// uint8_t PWM_value_2 = 273;
// uint8_t PWM_value_3 = 409;
// uint8_t PWM_value_4 = 166;
// uint8_t PWM_value_5 = 205;
// uint8_t PWM_value_6 = 166;
// uint8_t PWM_value_7 = 46;

// #define TEMP_DIFF_A 7 // how much higher the target temp needs to be compared to ambient temp
// #define TEMP_DIFF_B 14

// OneWire oneWire(ONE_WIRE_BUS);       // initiates onewire
// DallasTemperature sensors(&oneWire); // initiates stuff for dallastemp

// float percentage;

// struct temp_sensors // struct to store all the temperature sensor information
// {
//   uint8_t address[8];
//   const char *location;
//   float last_temp;
// } sensor_list[MAX_SENSORS] = {
//     // hex addresses and locations for all the sensors
//     {{0x28, 0xF4, 0x1C, 0xF8, 0x0F, 0x00, 0x00, 0xDF}, "top_head"},
//     {{0x28, 0x72, 0xE6, 0xF8, 0x0F, 0x00, 0x00, 0x9C}, "front_head"},
//     {{0x28, 0x6A, 0x18, 0xF9, 0x0F, 0x00, 0x00, 0xED}, "right_torso"},
//     {{0x28, 0x49, 0x71, 0x0B, 0x10, 0x00, 0x00, 0x83}, "right_shoulder"},
//     {{0x28, 0x99, 0xD5, 0xF8, 0x0F, 0x00, 0x00, 0x87}, "left_shoulder"},
//     {{0x28, 0xD9, 0x33, 0xF9, 0x0F, 0x00, 0x00, 0x3B}, "right_head"},
//     {{0x28, 0xDD, 0x6D, 0xF8, 0x0F, 0x00, 0x00, 0x5F}, "left_head"},
//     {{0x28, 0x43, 0x5F, 0xF9, 0x0F, 0x00, 0x00, 0x75}, "front_legs_a"},
//     {{0x28, 0xB3, 0x1A, 0x0B, 0x10, 0x00, 0x00, 0x5F}, "front_legs_b"},
//     {{0x28, 0xF3, 0x0B, 0xF8, 0x0F, 0x00, 0x00, 0x77}, "left_torso_a"},
//     {{0x28, 0x1B, 0x09, 0xF8, 0x0F, 0x00, 0x00, 0x7C}, "left_torso_b"},
//     {{0x28, 0x87, 0x2D, 0x0C, 0x10, 0x00, 0x00, 0x3D}, "front_torso_a"},
//     {{0x28, 0x8F, 0x0F, 0x0C, 0x10, 0x00, 0x00, 0xE7}, "front_torso_b"},
//     {{0x28, 0xCF, 0xFF, 0x0A, 0x10, 0x00, 0x00, 0x3D}, "side_legs_right"},
//     {{0x28, 0xFF, 0x64, 0x1F, 0x7F, 0x87, 0x74, 0x8C}, "side_legs_left"}
//   };

// float ambient_temp = 0;
// float setpoint_a = 7;
// float setpoint_b = 14;


// // // PID variables
// // float last_error = 0;
// // float integral = 0;
// unsigned long last_time = 0;
// // unsigned long ambient_time = 0;

// // float Kp = 40;
// // float Ki = 1.3;
// // float Kd = 12;

// uint8_t time_passed = 0;

// void setup(void)
// {
//   Serial.begin(9600); // needed to use serial for testing
//   delay(2500);
//    Serial.printf("test1\n");

//   // PWM hardware setup
//   ledcSetup(PWM_CHANNEL_0, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN_0, PWM_CHANNEL_0);       // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL_0, 0);                 // start off

//   ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN_1, PWM_CHANNEL_1);       // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL_1, 0);                 // start off

//   ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN_2, PWM_CHANNEL_2);       // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL_2, 0);                 // start off

//   ledcSetup(PWM_CHANNEL_3, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN_3, PWM_CHANNEL_3);       // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL_3, 0);                 // start off

//   ledcSetup(PWM_CHANNEL_4, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN_4, PWM_CHANNEL_4);       // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL_4, 0);                 // start off

//   ledcSetup(PWM_CHANNEL_5, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN_5, PWM_CHANNEL_5);       // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL_5, 0);                 // start off

//   ledcSetup(PWM_CHANNEL_6, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN_6, PWM_CHANNEL_6);       // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL_6, 0);                 // start off

//   ledcSetup(PWM_CHANNEL_7, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN_7, PWM_CHANNEL_7);       // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL_7, 0);                 // start off


//   sensors.begin(); // starts the temperature sensors
//   Serial.println(F("pwm testing woo"));

//   // locate devices on the bus
//   Serial.printf("Locating devices...");
//   Serial.printf("Found ");
//   Serial.print(sensors.getDeviceCount(), DEC);
//   Serial.println(" devices.");

//   sensors.requestTemperatures();
//   delay(750);
//   for (uint8_t i = 0; i < MAX_SENSORS; i++)
//   {
//     float t = sensors.getTempC(sensor_list[i].address);
//     Serial.printf("%s  %.2f °C\n", sensor_list[i].location, t);
//   }
//   // ambient_temp = sensors.getTempC(sensor_list[0].address);
//   // setpoint_a = ambient_temp + TEMP_DIFF_A;
//   // setpoint_b = ambient_temp + TEMP_DIFF_B;
//   sensors.setWaitForConversion(false);
// }


// void loop()
// {

//   ledcWrite(PWM_CHANNEL_0, PWM_value_0); 
//   ledcWrite(PWM_CHANNEL_1, PWM_value_1); 
//   ledcWrite(PWM_CHANNEL_2, PWM_value_2); 
//   ledcWrite(PWM_CHANNEL_3, PWM_value_3); 
//   ledcWrite(PWM_CHANNEL_4, PWM_value_4); 
//   ledcWrite(PWM_CHANNEL_5, PWM_value_5); 
//   ledcWrite(PWM_CHANNEL_6, PWM_value_6); 
//   ledcWrite(PWM_CHANNEL_7, PWM_value_7); 

//   unsigned long now = millis();
//   if (now - last_time >= 1000) // ← run PID only once per second
//   {
//     last_time = now;
//     time_passed++;
//     printf("time passed: %d", time_passed);
//   }
//     // sensors.requestTemperatures();
//     // current_temp = sensors.getTempC(sensor_list[1].address);
//     // ambient_temp = sensors.getTempC(sensor_list[0].address);
//     // setpoint = ambient_temp + TEMP_DIFF;
//     // float error = setpoint - current_temp;
//   //   integral += error;
//    sensors.requestTemperatures();
//   for (int i = 0; i<15; i++) {
//   sensor_list[i].last_temp = sensors.getTempC(sensor_list[i].address);
//   Serial.printf("%s: %.2f\n", sensor_list[i].location, sensor_list[i].last_temp);
//   }





//   //   if (integral > 200)
//   //     integral = 200; // anti-windup: don’t let it grow forever
//   //   if (integral < 0)
//   //     integral = 0;
//   //   float derivative = error - last_error;                       // how fast is error changing?
//   //   float output = Kp * error + Ki * integral + Kd * derivative; // actual PID formula

//   //   if (output > 255)
//   //     output = 255;
//   //   if (output < 0)
//   //     output = 0;
//   //   PWM_value = (uint8_t)output;
//   //   ledcWrite(PWM_CHANNEL, PWM_value);
//   //   last_error = error;
//   //   float percentage = (PWM_value*100.0)/255.0;
//   //   ledcWrite(PWM_CHANNEL, PWM_value);

//   //   Serial.printf("heating pad: %.2f °C | Error: %+.2f | PWM: %3d/255 | DC: %.2f | ambient: %.2f | %d sec passed\n",
//   //                 current_temp, error, PWM_value, percentage, ambient_temp, time_passed);
//   // }

//   // if (PWM_value < 255) {
//   // PWM_value++;
//   // ledcWrite(PWM_CHANNEL, PWM_value);
//   // float percentage = (PWM_value*100)/255;
//   // Serial.printf("PWM: %d | duty cycle: %.2f | room temp: %.2f | heat pad temp: %.2f.\n", PWM_value, percentage, sensor_list[0].last_temp, sensor_list[1].last_temp);
//   // sensors.requestTemperatures();
//   // sensor_list[1].last_temp = sensors.getTempC(sensor_list[1].address);
//   // sensor_list[0].last_temp = sensors.getTempC(sensor_list[0].address);
//   // }
//   // if (PWM_value >=255) {
//   //   PWM_value=100;
//  // }
//   //  delay(5000);
// }





// #include <Arduino.h>
// #include <stdio.h>
// #include <stdint.h>
// #include <stdbool.h>
// #include <OneWire.h>
// #include <DallasTemperature.h>

// #define ONE_WIRE_BUS 1
// #define BAUD_RATE 9600
// #define DELAY_AMOUNT 1500

// OneWire oneWire(ONE_WIRE_BUS);
// DallasTemperature ds18b20_sensors(&oneWire);

// uint8_t numberOfDevices;

// DeviceAddress tempDeviceAddress;

// void printAddress(DeviceAddress deviceAddress) {
//   for (uint8_t i = 0; i < 8; i++){
//     if (deviceAddress[i] < 16) Serial.print("0");
//       Serial.print(deviceAddress[i], HEX);
//   }
// }

// void setup(){
//   delay(5*DELAY_AMOUNT);

//   Serial.begin(BAUD_RATE);

//   ds18b20_sensors.begin();
  
//   numberOfDevices = ds18b20_sensors.getDeviceCount();
  
//   Serial.print("Locating devices...");
//   Serial.print("Found ");
//   Serial.print(numberOfDevices, DEC);
//   Serial.println(" devices.");

//   for(int i=0;i<numberOfDevices; i++){
//     if(ds18b20_sensors.getAddress(tempDeviceAddress, i)){
//       Serial.print("Found device ");
//       Serial.print(i, DEC);
//       Serial.print(" with address: ");
//       printAddress(tempDeviceAddress);
//       Serial.println();
//     } else {
//       Serial.print("Found ghost device at ");
//       Serial.print(i, DEC);
//       Serial.print(" but could not detect address. Check power and cabling");
//     }
//   }
// }

// void loop(){ 
//   ds18b20_sensors.requestTemperatures();
  
//   for(int i=0;i<numberOfDevices; i++){
//     if(ds18b20_sensors.getAddress(tempDeviceAddress, i)){
//       Serial.print("Temperature for device: ");
//       Serial.println(i,DEC);

//       float tempC = ds18b20_sensors.getTempC(tempDeviceAddress);

//       Serial.print("Temp C: ");
//       Serial.println(tempC);
//     }
//   }
//   delay(DELAY_AMOUNT);
// }




// //CODE I WAS USING TO TEST SEPARATE CHANNELS WITH A FLAT PWM VALUE
// #include <Arduino.h>
// #include <OneWire.h>           //to use the temperature sensors
// #include <DallasTemperature.h> //for easier work with the temp sensors

// #define ONE_WIRE_BUS 1 // what pin the temp sensors are on
// #define MAX_SENSORS 14 // just in case 

// #define PWM_PIN_0 11
// #define PWM_PIN_1 10 // what pin the PWM is gonna be coming out of pin 1 should be 11
// #define PWM_PIN_2 13
// #define PWM_PIN_3 12
// #define PWM_PIN_4 21
// #define PWM_PIN_5 14
// #define PWM_PIN_6 48
// #define PWM_PIN_7 47

// #define PWM_CHANNEL_0 0 // what channel pwm we are using
// #define PWM_CHANNEL_1 1
// #define PWM_CHANNEL_2 2
// #define PWM_CHANNEL_3 3
// #define PWM_CHANNEL_4 4
// #define PWM_CHANNEL_5 5
// #define PWM_CHANNEL_6 6
// #define PWM_CHANNEL_7 7

// #define PWM_FREQ 1000 // the frequency for the pwm, might lower this
// #define PWM_RES 10    // 10 bit

// uint16_t PWM_value_0 = 273;
// uint16_t PWM_value_1 = 109;
// uint16_t PWM_value_2 = 273;
// uint16_t PWM_value_3 = 409;
// uint16_t PWM_value_4 = 166;
// uint16_t PWM_value_5 = 205;
// uint16_t PWM_value_6 = 166;
// uint16_t PWM_value_7 = 46;

// #define TEMP_DIFF_A 7 // how much higher the target temp needs to be compared to ambient temp
// #define TEMP_DIFF_B 14

// OneWire oneWire(ONE_WIRE_BUS);       // initiates onewire
// DallasTemperature sensors(&oneWire); // initiates stuff for dallastemp

// float percentage;
// uint8_t PWM_value = 0;

// struct temp_sensors // struct to store all the temperature sensor information
// {
//   uint8_t address[8];
//   const char *location;
//   float last_temp;
// } sensor_list[MAX_SENSORS] = {
//     // hex addresses and locations for all the sensors
//     {{0x28, 0x72, 0xE6, 0xF8, 0x0F, 0x00, 0x00, 0x9C}, "front_head"},
//     {{0x28, 0x6A, 0x18, 0xF9, 0x0F, 0x00, 0x00, 0xED}, "right_torso"},
//     {{0x28, 0x49, 0x71, 0x0B, 0x10, 0x00, 0x00, 0x83}, "right_shoulder"},
//     {{0x28, 0x99, 0xD5, 0xF8, 0x0F, 0x00, 0x00, 0x87}, "left_shoulder"},
//     {{0x28, 0xD9, 0x33, 0xF9, 0x0F, 0x00, 0x00, 0x3B}, "right_head"},
//     {{0x28, 0x35, 0x45, 0xF8, 0x0F, 0x00, 0x00, 0x11}, "left_head"},
//     {{0x28, 0xDD, 0x6D, 0xF8, 0x0F, 0x00, 0x00, 0x5F}, "front_legs_a"},
//     {{0x28, 0x43, 0x5F, 0xF9, 0x0F, 0x00, 0x00, 0x75}, "front_legs_b"},
//     {{0x28, 0xB3, 0x1A, 0x0B, 0x10, 0x00, 0x00, 0x5F}, "left_torso_a"},
//     {{0x28, 0xF3, 0x0B, 0xF8, 0x0F, 0x00, 0x00, 0x77}, "left_torso_b"},
//     {{0x28, 0x1B, 0x09, 0xF8, 0x0F, 0x00, 0x00, 0x7C}, "front_torso_a"},
//     {{0x28, 0x87, 0x2D, 0x0C, 0x10, 0x00, 0x00, 0x3D}, "front_torso_b"},
//     {{0x28, 0x8F, 0x0F, 0x0C, 0x10, 0x00, 0x00, 0xE7}, "side_legs_right"},
//     {{0x28, 0xCF, 0xFF, 0x0A, 0x10, 0x00, 0x00, 0x3D}, "side_legs_left"}
//   };

// // float ambient_temp = 0;
// // float setpoint_a = 7;
// // float setpoint_b = 14;
// unsigned long last_time = 0;
// uint16_t time_passed = 0;

// void setup(void)
// {
//   Serial.begin(9600); // needed to use serial for testing
//   delay(2500);

//   // PWM hardware setup
//   ledcSetup(PWM_CHANNEL_0, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN_0, PWM_CHANNEL_0);     // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL_0, 0);                 // start off

//   ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN_1, PWM_CHANNEL_1);     // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL_1, 0);                 // start off

//   ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN_2, PWM_CHANNEL_2);     // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL_2, 0);                 // start off

//   ledcSetup(PWM_CHANNEL_3, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN_3, PWM_CHANNEL_3);     // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL_3, 0);                 // start off

//   ledcSetup(PWM_CHANNEL_4, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN_4, PWM_CHANNEL_4);     // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL_4, 0);                 // start off

//   ledcSetup(PWM_CHANNEL_5, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN_5, PWM_CHANNEL_5);     // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL_5, 0);                 // start off

//   ledcSetup(PWM_CHANNEL_6, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN_6, PWM_CHANNEL_6);     // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL_6, 0);                 // start off

//   ledcSetup(PWM_CHANNEL_7, PWM_FREQ, PWM_RES); // general setup for the pwm
//   ledcAttachPin(PWM_PIN_7, PWM_CHANNEL_7);     // attaches what channel the pin will be getting pwm from
//   ledcWrite(PWM_CHANNEL_7, 0);                 // start off

//   sensors.begin(); // starts the temperature sensors
//   Serial.println(F("pwm testing woo\n"));

//   // locate devices on the bus
//   Serial.printf("Locating devices...");
//   Serial.printf("Found ");
//   Serial.print(sensors.getDeviceCount(), DEC);
//   Serial.println(" devices.\n");

//   Serial.printf("starting pwm!!\n");

//   ledcWrite(PWM_CHANNEL_0, PWM_value_0);
//   ledcWrite(PWM_CHANNEL_1, PWM_value_1);
//   ledcWrite(PWM_CHANNEL_2, PWM_value_2);
//   ledcWrite(PWM_CHANNEL_3, PWM_value_3);
//   ledcWrite(PWM_CHANNEL_4, PWM_value_4);
//   ledcWrite(PWM_CHANNEL_5, PWM_value_5);
//   ledcWrite(PWM_CHANNEL_6, PWM_value_6);
//   ledcWrite(PWM_CHANNEL_7, PWM_value_7);

//   Serial.printf("Channel 0 has %d which is %.2f dtc\n", PWM_value_0, (PWM_value_0*100.0)/1023.0);
// Serial.printf("Channel 1 has %d which is %.2f dtc\n", PWM_value_1, (PWM_value_1*100.0)/1023.0);
// Serial.printf("Channel 2 has %d which is %.2f dtc\n", PWM_value_2, (PWM_value_2*100.0)/1023.0);
// Serial.printf("Channel 3 has %d which is %.2f dtc\n", PWM_value_3, (PWM_value_3*100.0)/1023.0);
// Serial.printf("Channel 4 has %d which is %.2f dtc\n", PWM_value_4, (PWM_value_4*100.0)/1023.0);
// Serial.printf("Channel 5 has %d which is %.2f dtc\n", PWM_value_5, (PWM_value_5*100.0)/1023.0);
// Serial.printf("Channel 6 has %d which is %.2f dtc\n", PWM_value_6, (PWM_value_6*100.0)/1023.0);
// Serial.printf("Channel 7 has %d which is %.2f dtc\n", PWM_value_7, (PWM_value_7*100.0)/1023,0);

//   sensors.setWaitForConversion(false);
// }

// void loop()
// {
//   unsigned long now = millis();
//   if (now - last_time >= 5000) // ← run PID only once per second
//   {
//     last_time = now;
//     time_passed = time_passed +5;
//     Serial.printf("time passed: %d\n", time_passed);
//      sensors.requestTemperatures();
//   for (int i = 0; i < MAX_SENSORS; i++)
//   {
//     sensor_list[i].last_temp = sensors.getTempC(sensor_list[i].address);
//     Serial.printf("%s: %.2f\n", sensor_list[i].location, sensor_list[i].last_temp);
//   }
//  Serial.printf("______________________\n");
//   }
// }



// //BENCE'S CODE FOR TESTING JUST THE TEMP SENSORS ALONE
// // #include <Arduino.h>
// // #include <stdio.h>
// // #include <stdint.h>
// // #include <stdbool.h>
// // #include <OneWire.h>
// // #include <DallasTemperature.h>

// // #define ONE_WIRE_BUS 1
// // #define BAUD_RATE 9600
// // #define DELAY_AMOUNT 1500

// // OneWire oneWire(ONE_WIRE_BUS);
// // DallasTemperature ds18b20_sensors(&oneWire);

// // uint8_t numberOfDevices;

// // DeviceAddress tempDeviceAddress;

// // void printAddress(DeviceAddress deviceAddress) {
// //   for (uint8_t i = 0; i < 8; i++){
// //     if (deviceAddress[i] < 16) Serial.print("0");
// //       Serial.print(deviceAddress[i], HEX);
// //   }
// // }

// // void setup(){
// //   delay(5*DELAY_AMOUNT);

// //   Serial.begin(BAUD_RATE);

// //   ds18b20_sensors.begin();
  
// //   numberOfDevices = ds18b20_sensors.getDeviceCount();
  
// //   Serial.print("Locating devices...");
// //   Serial.print("Found ");
// //   Serial.print(numberOfDevices, DEC);
// //   Serial.println(" devices.");

// //   for(int i=0;i<numberOfDevices; i++){
// //     if(ds18b20_sensors.getAddress(tempDeviceAddress, i)){
// //       Serial.print("Found device ");
// //       Serial.print(i, DEC);
// //       Serial.print(" with address: ");
// //       printAddress(tempDeviceAddress);
// //       Serial.println();
// //     } else {
// //       Serial.print("Found ghost device at ");
// //       Serial.print(i, DEC);
// //       Serial.print(" but could not detect address. Check power and cabling");
// //     }
// //   }
// // }

// // void loop(){ 
// //   ds18b20_sensors.requestTemperatures();
  
// //   for(int i=0;i<numberOfDevices; i++){
// //     if(ds18b20_sensors.getAddress(tempDeviceAddress, i)){
// //       Serial.print("Temperature for device: ");
// //       Serial.println(i,DEC);

// //       float tempC = ds18b20_sensors.getTempC(tempDeviceAddress);

// //       Serial.print("Temp C: ");
// //       Serial.println(tempC);
// //     }
// //   }
// //   delay(DELAY_AMOUNT);
// // }


// // CODE FOR TESTING A RISING PWM VALUE 

// // void loop()
// // {
// //     // sensors.requestTemperatures();
// //     // delay(750);
// //   // for (uint8_t i = 0; i < MAX_SENSORS; i++)
// //   // {
// //   //   float t = sensors.getTempC(sensor_list[i].address);
// //   //   Serial.printf("%s  %.2f °C\n", sensor_list[i].location, t);
// //   // }

// //   if (PWM_value < 255) {
// //   PWM_value++;
// //   ledcWrite(PWM_CHANNEL_0, PWM_value);
// //   float percentage = (PWM_value*100)/255;
// //   Serial.printf("PWM is at %d, duty cycle %.2f.\n", PWM_value, percentage);
// //   }
// //   if (PWM_value >=255) {
// //     PWM_value=0;
// //   }
// //    delay(1000);
// // }


// // CODE FOR ONE SENSOR
// #include <Arduino.h>
// #include <OneWire.h>           //to use the temperature sensors
// #include <DallasTemperature.h> //for easier work with the temp sensors

// #define ONE_WIRE_BUS 1

// OneWire oneWire(ONE_WIRE_BUS);       // initiates onewire
// DallasTemperature sensors(&oneWire); // initiates stuff for dallastemp

// uint8_t sensor_address[8];
// float ambient_temp = 0;

// unsigned long last_request_time = 0;
// const unsigned long conversion_time = 750;  // DS18B20 max conversion time (ms)

// void setup(void)
// {
//   Serial.begin(9600); // needed to use serial for testing
//   delay(2500);        // if this isn't here it's just not gonna print anything that is inside of setup
//   sensors.begin();    // starts the temperature sensors
//   Serial.println(F("Sensor testing yay!\n"));

//   // locate devices on the bus
//   Serial.printf("Locating devices...");
//   Serial.printf("Found ");
//   Serial.print(sensors.getDeviceCount(), DEC);
//   Serial.println(" devices.\n"); // should be just 1 but i put this in here for debugging purposes, so this will be removed

//     if (!sensors.getAddress(sensor_address, 0)) {
//     Serial.println("ERROR: No DS18B20 sensor found!");
//     } else {
//     Serial.printf("Device address: ");
//     printAddress(sensor_address);
//     }

//   sensors.setWaitForConversion(false); // the conversion when fetching a temp value usually takes about 750 ms,
//   // if this is set to false it won't wait for that time to pass but make sure at least 750ms pass
//   // until you actually need the value updated

//   }


// void loop(void)
// {
//   unsigned long now = millis();

//   if (now - last_request_time >= conversion_time) {

//     // Read the temperature
//     ambient_temp = sensors.getTempC(sensor_address);
//     Serial.printf("Temp C: %.2f\n", ambient_temp);
//     sensors.requestTemperatures(); //fetches the temperatures
//     last_request_time = now; //updates the interval
//   }


// }

// void printAddress(uint8_t deviceAddress[8]) {
//   for (uint8_t i = 0; i < 8; i++) {
//     if (deviceAddress[i] < 16) Serial.print("0"); // leading zero
//     Serial.print(deviceAddress[i], HEX);
//   }
//   Serial.println();
// }



// //GENERIC TESTING FOR PWM STAGE 1
// #include <Arduino.h>
// #include <OneWire.h>           //to use the temperature sensors
// #include <DallasTemperature.h> //for easier work with the temp sensors

// #define ONE_WIRE_BUS 5 // what pin the temp sensors are on (GPIO5)
// #define MAX_SENSORS 14 // how many sensors we should have
// #define MAX_CHANNELS 8

// #define PWM_FREQ 100 // the frequency for the pwm, might lower this
// #define PWM_RES 10    // 10 bit

// #define TEMP_DIFF_A 7 // how much higher the target temp needs to be compared to ambient temp
// #define TEMP_DIFF_B 14

// OneWire oneWire(ONE_WIRE_BUS);       // initiates onewire
// DallasTemperature sensors(&oneWire); // initiates stuff for dallastemp

// struct temp_sensors // struct to store all the temperature sensor information
// {
//   uint8_t address[8];
//   const char *location;
//   float last_temp;
// } sensor_list[MAX_SENSORS] = {
//     // hex addresses and locations for all the sensors
//     {{0x28, 0x72, 0xE6, 0xF8, 0x0F, 0x00, 0x00, 0x9C}, "front_head", 0},
//     {{0x28, 0x6A, 0x18, 0xF9, 0x0F, 0x00, 0x00, 0xED}, "right_torso", 0},
//     {{0x28, 0x49, 0x71, 0x0B, 0x10, 0x00, 0x00, 0x83}, "right_shoulder", 0},
//     {{0x28, 0x99, 0xD5, 0xF8, 0x0F, 0x00, 0x00, 0x87}, "left_shoulder", 0},
//     {{0x28, 0xD9, 0x33, 0xF9, 0x0F, 0x00, 0x00, 0x3B}, "right_head", 0},
//     {{0x28, 0x35, 0x45, 0xF8, 0x0F, 0x00, 0x00, 0x11}, "left_head", 0},
//     {{0x28, 0xDD, 0x6D, 0xF8, 0x0F, 0x00, 0x00, 0x5F}, "front_legs_a", 0},
//     {{0x28, 0x43, 0x5F, 0xF9, 0x0F, 0x00, 0x00, 0x75}, "front_legs_b", 0},
//     {{0x28, 0xB3, 0x1A, 0x0B, 0x10, 0x00, 0x00, 0x5F}, "left_torso_a", 0},
//     {{0x28, 0xF3, 0x0B, 0xF8, 0x0F, 0x00, 0x00, 0x77}, "left_torso_b", 0},
//     {{0x28, 0x1B, 0x09, 0xF8, 0x0F, 0x00, 0x00, 0x7C}, "front_torso_a", 0},
//     {{0x28, 0x87, 0x2D, 0x0C, 0x10, 0x00, 0x00, 0x3D}, "front_torso_b", 0},
//     {{0x28, 0x8F, 0x0F, 0x0C, 0x10, 0x00, 0x00, 0xE7}, "side_legs_right", 0},
//     {{0x28, 0xCF, 0xFF, 0x0A, 0x10, 0x00, 0x00, 0x3D}, "side_legs_left", 0}};

// struct channels
// {
//   uint8_t pwm_channel;      // hardware PWM channel
//   uint8_t pwm_pin;          // pin attached
//   uint16_t pwm_value;       // starting PWM duty
//   temp_sensors *sensors[2]; // pointers to 1 or 2 sensors controlling this channel
//   const char *area;         // what area the channel corresponds to
//   float target_temp;        // setpoint aka target value
//   float current_temp;
// } channel_list[8] = {
//     {0, 11, 273, {&sensor_list[0], &sensor_list[0]}, "Head - Front and Top", 14},
//     {1, 10, 109, {&sensor_list[1], &sensor_list[1]}, "Torso - Right side", 7},
//     {2, 13, 273, {&sensor_list[2], &sensor_list[3]}, "Shoulders - Left and Right side", 7},
//     {3, 12, 409, {&sensor_list[4], &sensor_list[5]}, "Head - Left and Right side", 14},
//     {4, 21, 166, {&sensor_list[6], &sensor_list[7]}, "Legs - Front side", 7},
//     {5, 14, 205, {&sensor_list[8], &sensor_list[9]}, "Torso - Fixed and Detachable left side", 7},
//     {6, 48, 166, {&sensor_list[10], &sensor_list[11]}, "Torso - Front side", 7},
//     {7, 47, 46, {&sensor_list[12], &sensor_list[13]}, "Legs - Right and Left side", 7},
// };

// float ambient_temp = 22; // would get this value from the other esp
// float setpoint_a = 7;
// float setpoint_b = 14;
// unsigned long last_time = 0;
// uint16_t time_passed = 0;

// float getChannelTemp(channels &ch)
// {
//   float t1 = ch.sensors[0]->last_temp;
//   float t2 = ch.sensors[1]->last_temp;

//   // Handle DS18B20 error values
//   if (t1 == -127 || t1 == 85)
//     t1 = NAN; // returns nan aka not a number if the sensor is not responding properly
//   if (t2 == -127 || t2 == 85)
//     t2 = NAN; // 85 tends to show right after booting up and -127 shows if it can't get a proper reading

//   // if
//   if (!isnan(t1) && !isnan(t2))
//   {
//     return (t1 + t2) * 0.5f;
//   }

//   // Only sensor 1 valid
//   if (!isnan(t1))
//     return t1;

//   // Only sensor 2 valid
//   if (!isnan(t2))
//     return t2;

//   // Neither sensor valid
//   return NAN;
// }

// void setup(void)
// {
//   Serial.begin(9600); // needed to use serial for testing
//   sensors.begin();    // starts the temperature sensors
//   delay(1000);
//   sensors.requestTemperatures();
//   delay(1500);

//   // PWM hardware setup
//   for (int i = 0; i < MAX_CHANNELS; i++)
//   {
//     ledcSetup(channel_list[i].pwm_channel, PWM_FREQ, PWM_RES);           // general pwm setup
//     ledcAttachPin(channel_list[i].pwm_pin, channel_list[i].pwm_channel); // attaches channel to pin
//     ledcWrite(channel_list[i].pwm_channel, 0);                           // starts outputting a pwm value, put 0 for now
//     channel_list[i].target_temp = channel_list[i].target_temp + ambient_temp;
//   }
//   Serial.println(F("pwm testing woo\n"));
//   // locate devices on the bus
//   // Serial.printf("Locating devices...");
//   // Serial.printf("Found ");
//   // Serial.print(sensors.getDeviceCount(), DEC);
//   // Serial.println(" devices.\n");
//   for (int i = 0; i < MAX_SENSORS; i++)
//   {
//     sensor_list[i].last_temp = sensors.getTempC(sensor_list[i].address);
//   }
//   Serial.printf("starting pwm!!\n");

//   for (int i = 0; i < MAX_CHANNELS; i++)
//   {
//     channel_list[i].current_temp = getChannelTemp(channel_list[i]);
//   }

//   for (int i = 0; i < MAX_CHANNELS; i++)
//   {
//     ledcWrite(channel_list[i].pwm_channel, channel_list[i].pwm_value);
//     Serial.printf("Channel %d: PWM: %d, DTC: %.2f, Pin: %d\n",
//                   channel_list[i].pwm_channel,
//                   channel_list[i].pwm_value,
//                   (channel_list[i].pwm_value * 100.0f) / 1023.0f,
//                   (int)channel_list[i].pwm_pin);
//   }

//   sensors.setWaitForConversion(false);
//   Serial.print("time");
// for (int i = 0; i < MAX_CHANNELS; i++) {
//     Serial.print(",ch");
//     Serial.print(i);
// }
// Serial.println();

// }

// void loop()
// {
//   unsigned long now = millis();
//   if (now - last_time >= 5000) // ← run only once every five seconds
//   {
//     last_time = now;
//     time_passed = time_passed + 5;
//     Serial.printf("%d, ", time_passed);
//     for (int i = 0; i < MAX_SENSORS; i++)
//     {
//       sensor_list[i].last_temp = sensors.getTempC(sensor_list[i].address);
//     }
//     for (int i = 0; i < MAX_CHANNELS; i++)
//     {
//       channel_list[i].current_temp = getChannelTemp(channel_list[i]);
//       Serial.printf("%.2f,", channel_list[i].current_temp);
//     }
//     Serial.println();
//     sensors.requestTemperatures();
//   }
// }


//BASIC PID CODE FOR EVERYTHING SEPARATELY
// #include <Arduino.h>
// #include <OneWire.h>           //to use the temperature sensors
// #include <DallasTemperature.h> //for easier work with the temp sensors

// #define ONE_WIRE_BUS 5 // what pin the temp sensors are on (GPIO5)
// #define MAX_SENSORS 14 // how many sensors we should have
// #define MAX_CHANNELS 8

// #define PWM_FREQ 500 // the frequency for the pwm, might lower this
// #define PWM_RES 10   // 10 bit

// #define TEMP_DIFF_A 7 // how much higher the target temp needs to be compared to ambient temp
// #define TEMP_DIFF_B 14

// #define TEMP_LIMIT 45

// OneWire oneWire(ONE_WIRE_BUS);       // initiates onewire
// DallasTemperature sensors(&oneWire); // initiates stuff for dallastemp

// // float percentage;
// // uint8_t PWM_value = 0;

// struct temp_sensors // struct to store all the temperature sensor information
// {
//   uint8_t address[8];
//   const char *location;
//   float last_temp;
// } sensor_list[MAX_SENSORS] = {
//     // hex addresses and locations for all the sensors
//     {{0x28, 0x72, 0xE6, 0xF8, 0x0F, 0x00, 0x00, 0x9C}, "front_head", 0},
//     {{0x28, 0x6A, 0x18, 0xF9, 0x0F, 0x00, 0x00, 0xED}, "right_torso", 0},
//     {{0x28, 0x49, 0x71, 0x0B, 0x10, 0x00, 0x00, 0x83}, "right_shoulder", 0},
//     {{0x28, 0x99, 0xD5, 0xF8, 0x0F, 0x00, 0x00, 0x87}, "left_shoulder", 0},
//     {{0x28, 0xD9, 0x33, 0xF9, 0x0F, 0x00, 0x00, 0x3B}, "right_head", 0},
//     {{0x28, 0x35, 0x45, 0xF8, 0x0F, 0x00, 0x00, 0x11}, "left_head", 0},
//     {{0x28, 0xDD, 0x6D, 0xF8, 0x0F, 0x00, 0x00, 0x5F}, "front_legs_a", 0},
//     {{0x28, 0x43, 0x5F, 0xF9, 0x0F, 0x00, 0x00, 0x75}, "front_legs_b", 0},
//     {{0x28, 0xB3, 0x1A, 0x0B, 0x10, 0x00, 0x00, 0x5F}, "left_torso_a", 0},
//     {{0x28, 0xF3, 0x0B, 0xF8, 0x0F, 0x00, 0x00, 0x77}, "left_torso_b", 0},
//     {{0x28, 0x1B, 0x09, 0xF8, 0x0F, 0x00, 0x00, 0x7C}, "front_torso_a", 0},
//     {{0x28, 0x87, 0x2D, 0x0C, 0x10, 0x00, 0x00, 0x3D}, "front_torso_b", 0},
//     {{0x28, 0x8F, 0x0F, 0x0C, 0x10, 0x00, 0x00, 0xE7}, "side_legs_right", 0},
//     {{0x28, 0xCF, 0xFF, 0x0A, 0x10, 0x00, 0x00, 0x3D}, "side_legs_left", 0}};

// struct PIDState
// {
//   float Kp, Ki, Kd;

//   float integral;
//   float last_error;
//   unsigned long last_pid;

//   bool stage2_enabled;
// };

// struct channels
// {
//   uint8_t pwm_channel; // hardware PWM channel
//   uint8_t pwm_pin;     // pin attached
//   uint16_t pwm_value;  // starting PWM duty
//   uint16_t pwm_max;
//   temp_sensors *sensors[2]; // pointers to 1 or 2 sensors controlling this channel
//   const char *area;         // what area the channel corresponds to
//   float target_temp;        // setpoint aka target value
//   float current_temp;
//   PIDState pid;

// } channel_list[8] = {
//     {0, 11, 327, 327, {&sensor_list[0], &sensor_list[0]}, "Head - Front and Top", 14},
//     {1, 10, 164, 164, {&sensor_list[1], &sensor_list[1]}, "Torso - Right side", 7},
//     {2, 13, 327, 327, {&sensor_list[2], &sensor_list[3]}, "Shoulders - Left and Right side", 7},
//     {3, 12, 460, 460, {&sensor_list[4], &sensor_list[5]}, "Head - Left and Right side", 14},
//     {4, 21, 225, 225, {&sensor_list[6], &sensor_list[7]}, "Legs - Front side", 7},
//     {5, 14, 256, 256, {&sensor_list[8], &sensor_list[9]}, "Torso - Fixed and Detachable left side", 7},
//     {6, 48, 225, 225, {&sensor_list[10], &sensor_list[11]}, "Torso - Front side", 7},
//     {7, 47, 102, 102, {&sensor_list[12], &sensor_list[13]}, "Legs - Right and Left side", 7},
// };

// float ambient_temp = 22; // would get this value from the other esp
// unsigned long last_time = 0;
// uint16_t time_passed = 0;
// unsigned long last_pid = 0;

// float getChannelTemp(channels &ch)
// {
//   float t1 = ch.sensors[0]->last_temp;
//   float t2 = ch.sensors[1]->last_temp;

//   // Handle DS18B20 error values
//   if (t1 == -127 || t1 == 85)
//     t1 = NAN; // returns nan aka not a number if the sensor is not responding properly
//   if (t2 == -127 || t2 == 85)
//     t2 = NAN; // 85 tends to show right after booting up and -127 shows if it can't get a proper reading

//   // if
//   if (!isnan(t1) && !isnan(t2))
//   {
//     return (t1 + t2) * 0.5f;
//   }

//   // Only sensor 1 valid
//   if (!isnan(t1))
//     return t1;

//   // Only sensor 2 valid
//   if (!isnan(t2))
//     return t2;

//   // Neither sensor valid
//   return NAN;
// }

// void initPID()
// {
//   for (int i = 0; i < MAX_CHANNELS; i++)
//   {
//     channel_list[i].pid.Kp = 2.0;
//     channel_list[i].pid.Ki = 0.05;
//     channel_list[i].pid.Kd = 0.0;

//     channel_list[i].pid.integral = 0;
//     channel_list[i].pid.last_error = 0;
//     channel_list[i].pid.stage2_enabled = false;
//     channel_list[i].pid.last_pid = 0;
//   }
// }
// void initPWM()
// {
//   // PWM hardware setup
//   for (int i = 0; i < MAX_CHANNELS; i++)
//   {
//     ledcSetup(channel_list[i].pwm_channel, PWM_FREQ, PWM_RES);           // general pwm setup
//     ledcAttachPin(channel_list[i].pwm_pin, channel_list[i].pwm_channel); // attaches channel to pin
//     ledcWrite(channel_list[i].pwm_channel, 0);                           // starts outputting a pwm value, put 0 for now
//     channel_list[i].target_temp = channel_list[i].target_temp + ambient_temp;
//   }
// }

// uint16_t updatePID(channels &ch, float dt)
// {
//   if (dt < 0.1f)
//     dt = 0.1f; // 100 ms minimum

//   float error = ch.target_temp - ch.current_temp;

//   ch.pid.integral += error * dt;
//   ch.pid.integral = constrain(ch.pid.integral, -200, 200);
//   float derivative = (error - ch.pid.last_error) / dt;

//   float output =
//       ch.pid.Kp * error +
//       ch.pid.Ki * ch.pid.integral +
//       ch.pid.Kd * derivative;

//   ch.pid.last_error = error;

//   // limiting outputs
//   output = constrain(output, 0, 1023);
//   return (uint16_t)output;
// }

// void setup(void)
// {
//   Serial.begin(9600); // needed to use serial for testing
//   sensors.begin();    // starts the temperature sensors
//   delay(1000);
//   sensors.requestTemperatures();
//   delay(1500);
//   initPWM();
//   initPID();

//   for (int i = 0; i < MAX_SENSORS; i++)
//   {
//     sensor_list[i].last_temp = sensors.getTempC(sensor_list[i].address);
//   }

//   for (int i = 0; i < MAX_CHANNELS; i++)
//   {
//     channel_list[i].current_temp = getChannelTemp(channel_list[i]);
//   }

//   for (int i = 0; i < MAX_CHANNELS; i++)
//   {
//     ledcWrite(channel_list[i].pwm_channel, channel_list[i].pwm_value);
//     Serial.printf("Channel %d: PWM: %d, DTC: %.2f, Pin: %d\n",
//                   channel_list[i].pwm_channel,
//                   channel_list[i].pwm_value,
//                   (channel_list[i].pwm_value * 100) / 1023,
//                   channel_list[i].pwm_pin);
//   }

//   sensors.setWaitForConversion(false);
//   Serial.printf("t,c0,c1,c2,c3,c4,c5,c6,c7\n");
// }

// void loop()
// {
//   unsigned long now = millis();
//   if (now - last_time >= 5000) // ← run only once every five seconds
//   {
//     last_time = now;
//     time_passed = time_passed + 5;
//     Serial.printf("%d,", time_passed);
//     for (int i = 0; i < MAX_SENSORS; i++)
//     {
//       sensor_list[i].last_temp = sensors.getTempC(sensor_list[i].address);
//     }
//     for (int i = 0; i < MAX_CHANNELS; i++)
//     {
//       channel_list[i].current_temp = getChannelTemp(channel_list[i]);
//       Serial.printf("%.2f", channel_list[i].current_temp);
//       if (i < MAX_CHANNELS - 1)
//       {
//         Serial.printf(",");
//       }
//       if (isnan(channel_list[i].current_temp)) // if the sensors stop working the channel gets turned off
//       {
//         ledcWrite(channel_list[i].pwm_channel, 0);
//         continue;
//       }

//       if (channel_list[i].current_temp >= channel_list[i].target_temp &&
//           !channel_list[i].pid.stage2_enabled)
//       {
//         channel_list[i].pid.stage2_enabled = true;
//         Serial.printf("#Channel %d switched to PID.\n", i);
//         channel_list[i].pid.integral = 0;
//         channel_list[i].pid.last_error = 0;
//         channel_list[i].pid.last_pid = millis();
//       }
//       if (channel_list[i].pid.stage2_enabled)
//       {
//         if (channel_list[i].current_temp > TEMP_LIMIT)
//         {
//           ledcWrite(channel_list[i].pwm_channel, 0);
//           channel_list[i].pid.stage2_enabled = false;
//           Serial.printf("!!! HARD SHUTOFF ch %d\n", i);
//           channel_list[i].pid.integral = 0;
//           channel_list[i].pid.last_error = 0;

//           continue;
//         }
//         channel_list[i].pwm_value = updatePID(channel_list[i], (now - channel_list[i].pid.last_pid) / 1000.0f);
//         if (channel_list[i].pwm_value > channel_list[i].pwm_max)
//         {
//           channel_list[i].pwm_value = channel_list[i].pwm_max;
//         }
//         channel_list[i].pid.last_pid = now;
//         ledcWrite(channel_list[i].pwm_channel, channel_list[i].pwm_value);
//       }
//     }
//     Serial.println();
//     sensors.requestTemperatures();
//   }
// }


// //CODE FOR TUNING PID ON A SINGLE CHANNEL
// #include <Arduino.h>
// #include <OneWire.h>           //to use the temperature sensors
// #include <DallasTemperature.h> //for easier work with the temp sensors

// #define ONE_WIRE_BUS 5 // what pin the temp sensors are on (GPIO5)
// #define MAX_SENSORS 14 // how many sensors we should have
// #define MAX_CHANNELS 8

// #define PWM_FREQ 500 // the frequency for the pwm, might lower this
// #define PWM_RES 10   // 10 bit

// #define TEMP_DIFF_A 7 // how much higher the target temp needs to be compared to ambient temp
// #define TEMP_DIFF_B 14

// #define TEMP_LIMIT 45

// OneWire oneWire(ONE_WIRE_BUS);       // initiates onewire
// DallasTemperature sensors(&oneWire); // initiates stuff for dallastemp

// // float percentage;
// // uint8_t PWM_value = 0;

// struct temp_sensors // struct to store all the temperature sensor information
// {
//   uint8_t address[8];
//   const char *location;
//   float last_temp;
// } sensor_list[MAX_SENSORS] = {
//     // hex addresses and locations for all the sensors
//     {{0x28, 0x72, 0xE6, 0xF8, 0x0F, 0x00, 0x00, 0x9C}, "front_head", 0},
//     {{0x28, 0x6A, 0x18, 0xF9, 0x0F, 0x00, 0x00, 0xED}, "right_torso", 0},
//     {{0x28, 0x49, 0x71, 0x0B, 0x10, 0x00, 0x00, 0x83}, "right_shoulder", 0},
//     {{0x28, 0x99, 0xD5, 0xF8, 0x0F, 0x00, 0x00, 0x87}, "left_shoulder", 0},
//     {{0x28, 0xD9, 0x33, 0xF9, 0x0F, 0x00, 0x00, 0x3B}, "right_head", 0},
//     {{0x28, 0x35, 0x45, 0xF8, 0x0F, 0x00, 0x00, 0x11}, "left_head", 0},
//     {{0x28, 0xDD, 0x6D, 0xF8, 0x0F, 0x00, 0x00, 0x5F}, "front_legs_a", 0},
//     {{0x28, 0x43, 0x5F, 0xF9, 0x0F, 0x00, 0x00, 0x75}, "front_legs_b", 0},
//     {{0x28, 0xB3, 0x1A, 0x0B, 0x10, 0x00, 0x00, 0x5F}, "left_torso_a", 0},
//     {{0x28, 0xF3, 0x0B, 0xF8, 0x0F, 0x00, 0x00, 0x77}, "left_torso_b", 0},
//     {{0x28, 0x1B, 0x09, 0xF8, 0x0F, 0x00, 0x00, 0x7C}, "front_torso_a", 0},
//     {{0x28, 0x87, 0x2D, 0x0C, 0x10, 0x00, 0x00, 0x3D}, "front_torso_b", 0},
//     {{0x28, 0x8F, 0x0F, 0x0C, 0x10, 0x00, 0x00, 0xE7}, "side_legs_right", 0},
//     {{0x28, 0xCF, 0xFF, 0x0A, 0x10, 0x00, 0x00, 0x3D}, "side_legs_left", 0}};

// struct PIDState
// {
//   float Kp, Ki, Kd;

//   float integral;
//   float last_error;
//   unsigned long last_pid;

//   bool stage2_enabled;
// };

// struct channels
// {
//   uint8_t pwm_channel; // hardware PWM channel
//   uint8_t pwm_pin;     // pin attached
//   uint16_t pwm_value;  // starting PWM duty
//   uint16_t pwm_max;
//   temp_sensors *sensors[2]; // pointers to 1 or 2 sensors controlling this channel
//   const char *area;         // what area the channel corresponds to
//   float target_temp;        // setpoint aka target value
//   float current_temp;
//   PIDState pid;

// } channel_list[8] = {
//     {0, 11, 327, 327, {&sensor_list[0], &sensor_list[0]}, "Head - Front and Top", 14},
//     {1, 10, 164, 164, {&sensor_list[1], &sensor_list[1]}, "Torso - Right side", 7},
//     {2, 13, 273, 273, {&sensor_list[2], &sensor_list[3]}, "Shoulders - Left and Right side", 7},
//     {3, 12, 460, 460, {&sensor_list[4], &sensor_list[5]}, "Head - Left and Right side", 14},
//     {4, 21, 225, 225, {&sensor_list[6], &sensor_list[7]}, "Legs - Front side", 7},
//     {5, 14, 256, 256, {&sensor_list[8], &sensor_list[9]}, "Torso - Fixed and Detachable left side", 7},
//     {6, 48, 225, 225, {&sensor_list[10], &sensor_list[11]}, "Torso - Front side", 7},
//     {7, 47, 102, 102, {&sensor_list[12], &sensor_list[13]}, "Legs - Right and Left side", 7},
// };

// float ambient_temp = 22; // would get this value from the other esp
// unsigned long last_time = 0;
// uint16_t time_passed = 0;
// unsigned long last_pid = 0;

// float getChannelTemp(channels &ch)
// {
//   float t1 = ch.sensors[0]->last_temp;
//   float t2 = ch.sensors[1]->last_temp;

//   // Handle DS18B20 error values
//   if (t1 == -127 || t1 == 85)
//     t1 = NAN; // returns nan aka not a number if the sensor is not responding properly
//   if (t2 == -127 || t2 == 85)
//     t2 = NAN; // 85 tends to show right after booting up and -127 shows if it can't get a proper reading

//   // if
//   if (!isnan(t1) && !isnan(t2))
//   {
//     return (t1 + t2) * 0.5f;
//   }

//   // Only sensor 1 valid
//   if (!isnan(t1))
//     return t1;

//   // Only sensor 2 valid
//   if (!isnan(t2))
//     return t2;

//   // Neither sensor valid
//   return NAN;
// }

// void initPID()
// {
//   for (int i = 0; i < MAX_CHANNELS; i++)
//   {
//     channel_list[i].pid.Kp = 100.0;
//     channel_list[i].pid.Ki = 2.0;
//     channel_list[i].pid.Kd = 10.0;

//     channel_list[i].pid.integral = 0;
//     channel_list[i].pid.last_error = 0;
//     channel_list[i].pid.stage2_enabled = false;
//     channel_list[i].pid.last_pid = 0;
//   }
// }
// void initPWM()
// {
//   // PWM hardware setup
//   for (int i = 0; i < MAX_CHANNELS; i++)
//   {
//     ledcSetup(channel_list[i].pwm_channel, PWM_FREQ, PWM_RES);           // general pwm setup
//     ledcAttachPin(channel_list[i].pwm_pin, channel_list[i].pwm_channel); // attaches channel to pin
//     ledcWrite(channel_list[i].pwm_channel, 0);                           // starts outputting a pwm value, put 0 for now
//     channel_list[i].target_temp = channel_list[i].target_temp + ambient_temp;
//   }
// }

// uint16_t updatePID(channels &ch, float dt)
// {
//   if (dt < 0.1f)
//     dt = 0.1f; // 100 ms minimum

//   float error = ch.target_temp - ch.current_temp;

//   ch.pid.integral += error * dt;
//   ch.pid.integral = constrain(ch.pid.integral, -200, 200);
//   float derivative = (error - ch.pid.last_error) / dt;

//   float output =
//       ch.pid.Kp * error +
//       ch.pid.Ki * ch.pid.integral +
//       ch.pid.Kd * derivative;

//   ch.pid.last_error = error;

//   // limiting outputs
//   output = constrain(output, 0, 1023);
//   return (uint16_t)output;
// }

// void setup(void)
// {
//   Serial.begin(9600); // needed to use serial for testing
//   sensors.begin();    // starts the temperature sensors
//   delay(1000);
//   sensors.requestTemperatures();
//   delay(1500);
//   initPWM();
//   initPID();
//   for (int i = 0; i < MAX_CHANNELS; i++)
//   {
//     channel_list[i].pwm_max = 0;
//   }
//   channel_list[2].pwm_max = 273;

//   for (int i = 0; i < MAX_SENSORS; i++)
//   {
//     sensor_list[i].last_temp = sensors.getTempC(sensor_list[i].address);
//   }

//   for (int i = 0; i < MAX_CHANNELS; i++)
//   {
//     channel_list[i].current_temp = getChannelTemp(channel_list[i]);
//   }

//   // for (int i = 0; i < MAX_CHANNELS; i++)
//   // {
//   //   ledcWrite(channel_list[i].pwm_channel, channel_list[i].pwm_value);
//   //   Serial.printf("Channel %d: PWM: %d, DTC: %.2f, Pin: %d\n",
//   //                 channel_list[i].pwm_channel,
//   //                 channel_list[i].pwm_value,
//   //                 (channel_list[i].pwm_value * 100.0f) / 1023.0f,
//   //                 (int)channel_list[i].pwm_pin);
//   // }

//   for (int i = 0; i < MAX_CHANNELS; i++)
//   {
//     if (i == 2)
//     {
//       ledcWrite(channel_list[i].pwm_channel, channel_list[i].pwm_value);
//     }
//     else
//     {
//       ledcWrite(channel_list[i].pwm_channel, 0);
//     }
//   }

//   sensors.setWaitForConversion(false);
//   Serial.printf("t,temp,kp,ki,kd,int,error\n");
// }

// void loop()
// {
//   unsigned long now = millis();
//   if (now - last_time >= 5000) // ← run only once every five seconds
//   {
//     last_time = now;
//     time_passed = time_passed + 5;
//     // Serial.printf("%d,", time_passed);
//     for (int i = 0; i < MAX_SENSORS; i++)
//     {
//       sensor_list[i].last_temp = sensors.getTempC(sensor_list[i].address);
//     }
//     int i = 2;
//     channel_list[i].current_temp = getChannelTemp(channel_list[i]);
//     Serial.printf(
//         "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
//         time_passed,
//         channel_list[i].current_temp,
//         channel_list[i].pid.Kp,
//         channel_list[i].pid.Ki,
//         channel_list[i].pid.Kd,
//         channel_list[i].pid.integral,
//         channel_list[i].pid.last_error);

//     // if (i < MAX_CHANNELS - 1)
//     // {
//     //   Serial.printf(",");
//     // }
//     if (isnan(channel_list[i].current_temp)) // if the sensors stop working the channel gets turned off
//     {
//       ledcWrite(channel_list[i].pwm_channel, 0);
//     }

//     if (channel_list[i].current_temp >= channel_list[i].target_temp - 0.4 &&
//         !channel_list[i].pid.stage2_enabled)
//     {
//       channel_list[i].pid.stage2_enabled = true;
//       // Serial.printf("# switch to PID.", i);
//       channel_list[i].pid.integral = 0;
//       channel_list[i].pid.last_error = 0;
//       channel_list[i].pid.last_pid = millis();
//     }
//     if (channel_list[i].pid.stage2_enabled)
//     {
//       if (channel_list[i].current_temp > TEMP_LIMIT)
//       {
//         ledcWrite(channel_list[i].pwm_channel, 0);
//         channel_list[i].pid.stage2_enabled = false;
//         Serial.printf("!!! HARD SHUTOFF ch %d\n", i);
//         channel_list[i].pid.integral = 0;
//         channel_list[i].pid.last_error = 0;
//       }
//       channel_list[i].pwm_value = updatePID(channel_list[i], (now - channel_list[i].pid.last_pid) / 1000.0f);
//       if (channel_list[i].pwm_value > channel_list[i].pwm_max)
//       {
//         channel_list[i].pwm_value = channel_list[i].pwm_max;
//       }
//       channel_list[i].pid.last_pid = now;
//       ledcWrite(channel_list[i].pwm_channel, channel_list[i].pwm_value);
//     }
//   }
//   sensors.requestTemperatures();
// }


//META CODE FOR ESP BROWN
// #include <Arduino.h>
// #include <WiFi.h>
// #include <esp_now.h>
// #include <OneWire.h>           //to use the temperature sensors
// #include <DallasTemperature.h> //for easier work with the temp sensors

// #define ONE_WIRE_BUS 16    //gpio where the sensor data line is connected

// OneWire oneWire(ONE_WIRE_BUS);       // initiates onewire
// DallasTemperature sensors(&oneWire); // initiates stuff for dallastemp


// // for 1 temp sensor
// uint8_t sensor_address[8]; // stores rom code of the sensor
// bool sensors_found = false; // i added this in case it doesnt find the sensor so that it moves on from running the sensor code (when true it means it found it)
// float ambient_temp = 0.0f; 
// bool  temp_valid = false; // turns true after first reading, this is so that it doesnt send nonsence before the sensor is ready

// unsigned long last_request_time = 0;
// const unsigned long conversion_time = 750;  // DS18B20 max conversion time (ms)

// //mac address of esp that it talks to
// static const uint8_t red_peer_mac[6] = {0xCC, 0x8D, 0xA2, 0x2B, 0x9E, 0x50};  // red esp mac address, needed lated


// //for sending stuff with esp
// struct message {
//   char     source;  //B = brown,R = red
//   char     type;  //T = temp, K = typing in keyboard
//   float    temperature; //valid when type = T
//   char     command; //valid when type = K
//   uint32_t counter; //grouws every time we send a message, this is here to count when things are dropped or out of order
// };


// bool initialized = false;
// bool esp_now_ready = false; //turns true when the peer is added, if its false then its not sending anything

// uint32_t send_counter = 0; //counts how many messages have been sent 
// unsigned long last_send_time = 0;  // send temp at an interval
// const unsigned long send_interval = 4000; //send every 4 sec



// // setup has nothing else because for some reason nothing that is in setup works on my laptop no matter how i adjust it
// void setup() {
//     Serial.begin(115200);
// }


// //callback for esp:
// void sending(const uint8_t *mac_address, esp_now_send_status_t status) { //called automatically by esp now every time it attempts to send 
  
//     Serial.print("Message sent status: ");
//     Serial.println(status == ESP_NOW_SEND_SUCCESS ? "succeeded" : "failed");

// }

// //callback for esp:
// void receiving(const uint8_t *mac_address, const uint8_t *data, int len) {
//     //len is the number of bytes received in the incoming esp now packet
//     //this is needed for esp to understand what data its receiving, along with mac address and data

//     if (len != sizeof(message)) {   // check if len matches message strust size
//         Serial.print("unexpected message length: ");
//         Serial.println(len);
//         return;
//     }

//     message msg;
//     memcpy(&msg, data, sizeof(msg));

//     Serial.print("received from ");
//     Serial.print(msg.source);
//     Serial.print("\n type=");
//     Serial.print(msg.type);


//     if (msg.type == 'T') {
//         Serial.print(" \n temp = ");
//         Serial.print(msg.temperature, 2);
//         Serial.print(" °C");
//     } else if (msg.type == 'K') {
//         Serial.print(" \n command = '");
//         Serial.print(msg.command);
//         Serial.print("'");
//     }

//     Serial.print(" \n counter = ");
//     Serial.println(msg.counter);
// }

// bool init_esp_now() {
//     if (esp_now_init() != ESP_OK) {
//         Serial.println("esp now init failed");
//         return false;
//     }

//     esp_now_register_send_cb(sending);
//     esp_now_register_recv_cb(receiving);

//     esp_now_peer_info_t peerInfo{};
//     memcpy(peerInfo.peer_addr, red_peer_mac, 6);
//     peerInfo.channel = 0;
//     peerInfo.encrypt = false;


//     if (!esp_now_is_peer_exist(red_peer_mac)) {
//         if (esp_now_add_peer(&peerInfo) != ESP_OK) {
//             Serial.println("failed to add esp now as a peer");
//             return false;
//         }
//     }

//   Serial.println("esp now initialized");
//   return true;
// }


// void init_sensors() {
//     sensors.begin();

//     if (!sensors.getAddress(sensor_address, 0)) {
//         Serial.println("no DS18B20 sensor found");
//         sensors_found = false;
//         return;
//     }

//     sensors_found = true;
//     temp_valid = false;

//     sensors.setWaitForConversion(false); 
//     sensors.requestTemperatures();  
//     last_request_time = millis();

//     Serial.print("DS18B20 address: ");

//     for (uint8_t i = 0; i < 8; i++) {
//         if (sensor_address[i] < 16) Serial.print("0");
//         Serial.print(sensor_address[i], HEX);
//     }

//     Serial.println();
// }

// void update_temperature() {
//     if (!sensors_found) return;

//     unsigned long now = millis();
//     if (now - last_request_time >= conversion_time) {
//         float t = sensors.getTempC(sensor_address);

//         //checking, ignore weird values
//         if (t > -100.0f && t < 125.0f) {
//         ambient_temp = t;
//         temp_valid   = true;
//         }

//         sensors.requestTemperatures(); //start next conversion
//         last_request_time = now;
//     }
// }


// void send_temperature_if_needed() {
//     if (!esp_now_ready || !sensors_found || !temp_valid) return;

//     unsigned long now = millis();
//     if (now - last_send_time < send_interval) {
//         return;  //not time yet
//     }

//     message msg{};
//     msg.source = 'B';
//     msg.type = 'T';
//     msg.temperature = ambient_temp;
//     msg.command = 0;
//     msg.counter = ++send_counter;

//     esp_err_t result = esp_now_send(red_peer_mac, (uint8_t *)&msg, sizeof(msg));

//     Serial.print("TX to RED (temp) \n temp=");
//     Serial.print(msg.temperature, 2);
//     Serial.print(" °C \n counter=");
//     Serial.print(msg.counter);
//     Serial.print(" \n status=");
//     Serial.println(result == ESP_OK ? "OK" : "ERROR");

//     last_send_time = now;
// }

// void for_keyboard_commands() {
//     if (!esp_now_ready) return;

//     while (Serial.available()) {
//         char c = Serial.read();
//         if (c == '\n' || c == '\r') {
//         continue;  //ignore line endings
//         }

//         message msg{};
//         msg.source      = 'B';
//         msg.type        = 'K';
//         msg.temperature = ambient_temp;  // optional extra info
//         msg.command     = c;
//         msg.counter     = ++send_counter;

//         esp_err_t result = esp_now_send(red_peer_mac, (uint8_t *)&msg, sizeof(msg));

//         Serial.print("TX to RED (command) | '");
//         Serial.print(c);
//         Serial.print("' \n counter=");
//         Serial.print(msg.counter);
//         Serial.print(" \n status=");
//         Serial.println(result == ESP_OK ? "OK" : "ERROR");
//     }
// }


// void loop() {
//     if (!initialized) {
        
//         delay(1000); //delay for serial

        
//         Serial.print("my mac: ");
//         Serial.println(WiFi.macAddress());

//         WiFi.mode(WIFI_STA); //neded before esp now
//         esp_now_ready = init_esp_now(); //init esp now + peer
//         init_sensors(); //init temp sensor

//         initialized = true;
//     }

    
//     update_temperature(); 
//     send_temperature_if_needed(); //send temp
//     for_keyboard_commands(); //send keyboard commands

// }

//METAS CODE FOR RED ESP


//BASIC PID PLUS BASIC COMMS
//BASIC PID CODE PLUS BASIC SENDING CODE

//LIBRARIES___________________________________________________________________________________________________________________________
#include <Arduino.h>
#include <OneWire.h>           //to use the temperature sensors
#include <DallasTemperature.h> //for easier work with the temp sensors
#include <WiFi.h> 
#include <esp_now.h>


//VARIABLES FOR COMMUNICATION____________________________________________________________________________________________________________________________

//mac address of esp that this red esp talks to (brown esp)
static const uint8_t brown_peer_mac[6] = {0xCC, 0x8D, 0xA2, 0x2B, 0xA8, 0x7C};

//for sending stuff with esp now
struct message {
  char     source; //B = brown, R = red
  char     type; //T = temp, K = keyboard typing
  float    temperature; //valid when type = T
  char     command; //valid when type = K
  uint32_t counter; //grows every time we send a message
};

bool initialized = false;
bool esp_now_ready = false; //turns true when peer is added successfully

uint32_t send_counter = 0; //counts how many messages have been sent
unsigned long last_send_time = 0;
const unsigned long send_interval = 4000; //send temp every 4 sec, would it not be easier to have this as a #DEFINE?



//VARIABLES FOR HEATING________________________________________________________________________________________________________________________________________

#define ONE_WIRE_BUS 5 // what pin the temp sensors are on (GPIO5)
#define MAX_SENSORS 14 // how many sensors we should have
#define MAX_CHANNELS 8

#define PWM_FREQ 500 // the frequency for the pwm, might lower this
#define PWM_RES 10   // 10 bit

#define TEMP_DIFF_A 7 // how much higher the target temp needs to be compared to ambient temp
#define TEMP_DIFF_B 14 //this is for head areas
#define TEMP_LIMIT 45 //just in case

OneWire oneWire(ONE_WIRE_BUS);       // initiates onewire
DallasTemperature sensors(&oneWire); // initiates stuff for dallastemp

struct temp_sensors // struct to store all the temperature sensor information
{
  uint8_t address[8];
  const char *location;
  float last_temp;
} sensor_list[MAX_SENSORS] = {
    // hex addresses and locations for all the sensors
    {{0x28, 0x72, 0xE6, 0xF8, 0x0F, 0x00, 0x00, 0x9C}, "front_head", 0},
    {{0x28, 0x6A, 0x18, 0xF9, 0x0F, 0x00, 0x00, 0xED}, "right_torso", 0},
    {{0x28, 0x49, 0x71, 0x0B, 0x10, 0x00, 0x00, 0x83}, "right_shoulder", 0},
    {{0x28, 0x99, 0xD5, 0xF8, 0x0F, 0x00, 0x00, 0x87}, "left_shoulder", 0},
    {{0x28, 0xD9, 0x33, 0xF9, 0x0F, 0x00, 0x00, 0x3B}, "right_head", 0},
    {{0x28, 0x35, 0x45, 0xF8, 0x0F, 0x00, 0x00, 0x11}, "left_head", 0},
    {{0x28, 0xDD, 0x6D, 0xF8, 0x0F, 0x00, 0x00, 0x5F}, "front_legs_a", 0},
    {{0x28, 0x43, 0x5F, 0xF9, 0x0F, 0x00, 0x00, 0x75}, "front_legs_b", 0},
    {{0x28, 0xB3, 0x1A, 0x0B, 0x10, 0x00, 0x00, 0x5F}, "left_torso_a", 0},
    {{0x28, 0xF3, 0x0B, 0xF8, 0x0F, 0x00, 0x00, 0x77}, "left_torso_b", 0},
    {{0x28, 0x1B, 0x09, 0xF8, 0x0F, 0x00, 0x00, 0x7C}, "front_torso_a", 0},
    {{0x28, 0x87, 0x2D, 0x0C, 0x10, 0x00, 0x00, 0x3D}, "front_torso_b", 0},
    {{0x28, 0x8F, 0x0F, 0x0C, 0x10, 0x00, 0x00, 0xE7}, "side_legs_right", 0},
    {{0x28, 0xCF, 0xFF, 0x0A, 0x10, 0x00, 0x00, 0x3D}, "side_legs_left", 0}};

struct PIDState
{
  float Kp, Ki, Kd;

  float integral;
  float last_error;
  unsigned long last_pid;

  bool stage2_enabled;
};

struct channels
{
  uint8_t pwm_channel; // hardware PWM channel
  uint8_t pwm_pin;     // pin attached
  uint16_t pwm_value;  // starting PWM duty
  uint16_t pwm_max;
  temp_sensors *sensors[2]; // pointers to 1 or 2 sensors controlling this channel
  const char *area;         // what area the channel corresponds to
  float target_temp;        // setpoint aka target value
  float current_temp;
  PIDState pid;

} channel_list[8] = {
    {0, 11, 327, 327, {&sensor_list[0], &sensor_list[0]}, "Head - Front and Top", 14},
    {1, 10, 164, 164, {&sensor_list[1], &sensor_list[1]}, "Torso - Right side", 7},
    {2, 13, 327, 327, {&sensor_list[2], &sensor_list[3]}, "Shoulders - Left and Right side", 7},
    {3, 12, 460, 460, {&sensor_list[4], &sensor_list[5]}, "Head - Left and Right side", 14},
    {4, 21, 225, 225, {&sensor_list[6], &sensor_list[7]}, "Legs - Front side", 7},
    {5, 14, 256, 256, {&sensor_list[8], &sensor_list[9]}, "Torso - Fixed and Detachable left side", 7},
    {6, 48, 225, 225, {&sensor_list[10], &sensor_list[11]}, "Torso - Front side", 7},
    {7, 47, 102, 102, {&sensor_list[12], &sensor_list[13]}, "Legs - Right and Left side", 7},
};

float ambient_temp = 21; // would get this value from the other esp
unsigned long last_time = 0;
uint16_t time_passed = 0;
unsigned long last_pid = 0;

//FUNCTIONS FOR COMMS_____________________________________________________________________________________________________________________________

//callback for esp now every time esp now tries to send something
void sending(const uint8_t *mac_address, esp_now_send_status_t status) {
    Serial.print("Send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

//callback for esp now when it receives data
void receiving(const uint8_t *mac_address, const uint8_t *data, int len) {
    //len is the number of bytes in the incoming esp message

    if (len != sizeof(message)) { //check if len matches message struct size
        Serial.print("unexpected message length: ");
        Serial.println(len);
        return;
    }

    message msg;
    memcpy(&msg, data, sizeof(msg));

    Serial.print("RX from ");
    Serial.print(msg.source);
    Serial.print(" \n type=");
    Serial.print(msg.type);

    if (msg.type == 'T') {
        Serial.print(" \n temp = ");
        Serial.print(msg.temperature, 2);
        Serial.print(" °C");
    } else if (msg.type == 'K') {
        Serial.print(" \n command = '");
        Serial.print(msg.command);
        Serial.print("'");
    }

    Serial.print(" \n counter = ");
    Serial.println(msg.counter);
}


//for initalizing
bool init_esp_now() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("esp now init failed");
        return false;
    }

    esp_now_register_send_cb(sending);
    esp_now_register_recv_cb(receiving);

    esp_now_peer_info_t peerInfo{};
    memcpy(peerInfo.peer_addr, brown_peer_mac, 6);
    peerInfo.channel = 0; //same wifi channel
    peerInfo.encrypt = false;  //no encryption

    if (!esp_now_is_peer_exist(brown_peer_mac)) {
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.println("failed to add esp now as a peer");
            return false;
        }
    }

    Serial.println("esp now initialized");
    return true;
}

void for_keyboard_commands() {
    if (!esp_now_ready) return;

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') continue; //ignore enter

        message msg{};
        msg.source = 'R';
        msg.type = 'K';
        msg.temperature = ambient_temp;
        msg.command = c;
        msg.counter = ++send_counter;

        esp_err_t result = esp_now_send(brown_peer_mac, (uint8_t *)&msg, sizeof(msg));

        Serial.print("TX to BROWN (command) \n '");
        Serial.print(c);
        Serial.print("' counter=");
        Serial.print(msg.counter);
        Serial.print(" \n status=");
        Serial.println(result == ESP_OK ? "OK" : "ERROR");
    }
}

void send_temperature_if_needed(channels &ch) {
    if (!esp_now_ready) return;

    unsigned long now = millis();
    if (now - last_send_time < send_interval) {
        return; //not time yet
    } //maybe modify this to just wait until it has been enough instead of just giving up??

    float temp = ch.current_temp;

    message msg{};
    msg.source = 'R';  //this is the red esp
    msg.type = 'T';
    msg.temperature = temp;
    msg.command = 0;
    msg.counter = ++send_counter;

    esp_err_t result = esp_now_send(brown_peer_mac, (uint8_t *)&msg, sizeof(msg));

    Serial.print("TX to BROWN (temp) | temp=");
    Serial.print(msg.temperature, 2);
    Serial.print(" °C \n counter=");
    Serial.print(msg.counter);
    Serial.print(" \n status=");
    Serial.println(result == ESP_OK ? "OK" : "ERROR");

    last_send_time = now;
}

//FUNCTIONS FOR HEATING___________________________________________________________________________________________________________________________________

float getChannelTemp(channels &ch)
{
  float t1 = ch.sensors[0]->last_temp;
  float t2 = ch.sensors[1]->last_temp;

  // Handle DS18B20 error values
  if (t1 == -127 || t1 == 85)
    t1 = NAN; // returns nan aka not a number if the sensor is not responding properly
  if (t2 == -127 || t2 == 85)
    t2 = NAN; // 85 tends to show right after booting up and -127 shows if it can't get a proper reading

  // if
  if (!isnan(t1) && !isnan(t2))
  {
    return (t1 + t2) * 0.5f;
  }

  // Only sensor 1 valid
  if (!isnan(t1))
    return t1;

  // Only sensor 2 valid
  if (!isnan(t2))
    return t2;

  // Neither sensor valid
  return NAN;
}

void initPID()
{
  for (int i = 0; i < MAX_CHANNELS; i++)
  {
    channel_list[i].pid.Kp = 2.0;
    channel_list[i].pid.Ki = 0.05;
    channel_list[i].pid.Kd = 0.0;

    channel_list[i].pid.integral = 0;
    channel_list[i].pid.last_error = 0;
    channel_list[i].pid.stage2_enabled = false;
    channel_list[i].pid.last_pid = 0;
  }
}

void initPWM()
{
  // PWM hardware setup
  for (int i = 0; i < MAX_CHANNELS; i++)
  {
    ledcSetup(channel_list[i].pwm_channel, PWM_FREQ, PWM_RES);           // general pwm setup
    ledcAttachPin(channel_list[i].pwm_pin, channel_list[i].pwm_channel); // attaches channel to pin
    ledcWrite(channel_list[i].pwm_channel, 0);                           // starts outputting a pwm value, put 0 for now
    channel_list[i].target_temp = channel_list[i].target_temp + ambient_temp;
  }
}

uint16_t updatePID(channels &ch, float dt)
{
  if (dt < 0.1f)
    dt = 0.1f; // 100 ms minimum

  float error = ch.target_temp - ch.current_temp;

  ch.pid.integral += error * dt;
  ch.pid.integral = constrain(ch.pid.integral, -200, 200);
  float derivative = (error - ch.pid.last_error) / dt;

  float output =
      ch.pid.Kp * error +
      ch.pid.Ki * ch.pid.integral +
      ch.pid.Kd * derivative;

  ch.pid.last_error = error;

  // limiting outputs
  output = constrain(output, 0, 1023);
  return (uint16_t)output;
}

//SETUP_________________________________________________________________________________________________________________________
void setup(void)
{
  Serial.begin(9600); // needed to use serial for testing
  sensors.begin();    // starts the temperature sensors
  delay(1000);
  sensors.requestTemperatures();
  delay(1000);
  initPWM();
  initPID();

  for (int i = 0; i < MAX_SENSORS; i++) //retrieves the temperature for all 14 sensors
  {
    sensor_list[i].last_temp = sensors.getTempC(sensor_list[i].address);
  }

  for (int i = 0; i < MAX_CHANNELS; i++) //uses the temps from the sensors to average the temp for all 8 channels
  {
    channel_list[i].current_temp = getChannelTemp(channel_list[i]);
  }

  for (int i = 0; i < MAX_CHANNELS; i++) //this is just here to have for reference
  {
    ledcWrite(channel_list[i].pwm_channel, channel_list[i].pwm_value);
    Serial.printf("Channel %d: PWM: %d, DTC: %.2f, Pin: %d\n",
                  channel_list[i].pwm_channel,
                  channel_list[i].pwm_value,
                  (channel_list[i].pwm_value * 100.0f) / 1023.0f,
                  (int)channel_list[i].pwm_pin);
  }

  sensors.setWaitForConversion(false); //so the sensors don't clog up the whole thing
  if (!initialized) {

        // Serial.print("my mac: ");
        // Serial.println(WiFi.macAddress());

        WiFi.mode(WIFI_STA);
        esp_now_ready = init_esp_now();

        initialized = true;
    }
  Serial.printf("t,c0,c1,c2,c3,c4,c5,c6,c7\n"); //this is for reference for data layout
}

//LOOP_________________________________________________________________________________________________________________________
void loop()
{

  //receive ASK IF READY TO HEAT UP
  //check if ready to heat up
  //send that READY TO HEAT UP
  //actually for that i would have to move initPWM into the main loop tbh

  unsigned long now = millis(); //current time
  if (now - last_time >= 4000) // ← run only once every four seconds
  {
    last_time = now;
    time_passed = time_passed + 4;
    Serial.printf("%d,", time_passed);
    for (int i = 0; i < MAX_SENSORS; i++) //updates all the temp sensor temperatures
    {
      sensor_list[i].last_temp = sensors.getTempC(sensor_list[i].address);
    }
    for (int i = 0; i < MAX_CHANNELS; i++) //updates and prints the temps for all channels
    {
      channel_list[i].current_temp = getChannelTemp(channel_list[i]); 
      Serial.printf("%.2f", channel_list[i].current_temp);
      if (i < MAX_CHANNELS - 1)
      {
        Serial.printf(",");
      }
      if (isnan(channel_list[i].current_temp)) // if the sensors stop working the channel gets turned off
      {
        ledcWrite(channel_list[i].pwm_channel, 0);
        continue;
      }

      if (channel_list[i].current_temp >= channel_list[i].target_temp &&
          !channel_list[i].pid.stage2_enabled) //if heated up and still in stage 1 it switches to stage 2
      {
        channel_list[i].pid.stage2_enabled = true;
        Serial.printf("# Channel %d switched to PID.\n", i); //to know at which point it switched
        channel_list[i].pid.integral = 0;
        channel_list[i].pid.last_error = 0;
        channel_list[i].pid.last_pid = millis();
      }
      if (channel_list[i].pid.stage2_enabled) //if it is in stage 2
      {
        if (channel_list[i].current_temp > TEMP_LIMIT)
        {
          ledcWrite(channel_list[i].pwm_channel, 0);
          channel_list[i].pid.stage2_enabled = false;
          Serial.printf("!!! HARD SHUTOFF ch %d\n", i);
          channel_list[i].pid.integral = 0;
          channel_list[i].pid.last_error = 0;

          continue;
        }
        channel_list[i].pwm_value = updatePID(channel_list[i], (now - channel_list[i].pid.last_pid) / 1000.0f);
        if (channel_list[i].pwm_value > channel_list[i].pwm_max)
        {
          channel_list[i].pwm_value = channel_list[i].pwm_max;
        }
        channel_list[i].pid.last_pid = now;
        ledcWrite(channel_list[i].pwm_channel, channel_list[i].pwm_value);
      }
    }
    Serial.println();
    sensors.requestTemperatures();

  }
  //send the channel temps at some point
}


//OTHER STUFF
//- adapt the sending temp function to send all 8 temps at the same time and put that into the main loop


//CODE THAT HOPEFULLY WILL WORK WITH SEPARATE PHASING TESTING VERSION
// BASIC PID CODE FOR EVERYTHING SEPARATELY
#include <Arduino.h>
#include <OneWire.h>           //to use the temperature sensors
#include <DallasTemperature.h> //for easier work with the temp sensors
#include <driver/ledc.h>

#define ONE_WIRE_BUS 5 // what pin the temp sensors are on (GPIO5)
#define MAX_SENSORS 14 // how many sensors we should have
#define MAX_CHANNELS 8

#define PWM_FREQ 500 // the frequency for the pwm, might lower this
#define PWM_RES 10   // 10 bit

#define TEMP_DIFF_A 7 // how much higher the target temp needs to be compared to ambient temp
#define TEMP_DIFF_B 14

#define TEMP_LIMIT 45

OneWire oneWire(ONE_WIRE_BUS);       // initiates onewire
DallasTemperature sensors(&oneWire); // initiates stuff for dallastemp

bool stage1_enabled = true;

struct temp_sensors // struct to store all the temperature sensor information
{
  uint8_t address[8];
  const char *location;
  float last_temp;
} sensor_list[MAX_SENSORS] = {
    // hex addresses and locations for all the sensors
    {{0x28, 0x72, 0xE6, 0xF8, 0x0F, 0x00, 0x00, 0x9C}, "front_head", 0},
    {{0x28, 0x6A, 0x18, 0xF9, 0x0F, 0x00, 0x00, 0xED}, "right_torso", 0},
    {{0x28, 0x49, 0x71, 0x0B, 0x10, 0x00, 0x00, 0x83}, "right_shoulder", 0},
    {{0x28, 0x99, 0xD5, 0xF8, 0x0F, 0x00, 0x00, 0x87}, "left_shoulder", 0},
    {{0x28, 0xD9, 0x33, 0xF9, 0x0F, 0x00, 0x00, 0x3B}, "right_head", 0},
    {{0x28, 0x35, 0x45, 0xF8, 0x0F, 0x00, 0x00, 0x11}, "left_head", 0},
    {{0x28, 0xDD, 0x6D, 0xF8, 0x0F, 0x00, 0x00, 0x5F}, "front_legs_a", 0},
    {{0x28, 0x43, 0x5F, 0xF9, 0x0F, 0x00, 0x00, 0x75}, "front_legs_b", 0},
    {{0x28, 0xB3, 0x1A, 0x0B, 0x10, 0x00, 0x00, 0x5F}, "left_torso_a", 0},
    {{0x28, 0xF3, 0x0B, 0xF8, 0x0F, 0x00, 0x00, 0x77}, "left_torso_b", 0},
    {{0x28, 0x1B, 0x09, 0xF8, 0x0F, 0x00, 0x00, 0x7C}, "front_torso_a", 0},
    {{0x28, 0x87, 0x2D, 0x0C, 0x10, 0x00, 0x00, 0x3D}, "front_torso_b", 0},
    {{0x28, 0x8F, 0x0F, 0x0C, 0x10, 0x00, 0x00, 0xE7}, "side_legs_right", 0},
    {{0x28, 0xCF, 0xFF, 0x0A, 0x10, 0x00, 0x00, 0x3D}, "side_legs_left", 0}};

struct PIDState
{
  float Kp, Ki, Kd;

  float integral;
  float last_error;
  unsigned long last_pid;

  bool stage2_enabled;
};

struct channels
{
  uint8_t pwm_channel; // hardware PWM channel
  uint8_t pwm_pin;     // pin attached
  uint16_t pwm_value;  // starting PWM duty
  uint16_t pwm_max;
  temp_sensors *sensors[2]; // pointers to 1 or 2 sensors controlling this channel
  const char *area;         // what area the channel corresponds to
  float target_temp;        // setpoint aka target value
  float current_temp;
  PIDState pid;

} channel_list[8] = {
    {0, 11, 327, 327, {&sensor_list[0], &sensor_list[0]}, "Head - Front and Top", 14},
    {1, 10, 164, 164, {&sensor_list[1], &sensor_list[1]}, "Torso - Right side", 7},
    {2, 13, 327, 327, {&sensor_list[2], &sensor_list[3]}, "Shoulders - Left and Right side", 7},
    {3, 12, 460, 460, {&sensor_list[4], &sensor_list[5]}, "Head - Left and Right side", 14},
    {4, 21, 225, 225, {&sensor_list[6], &sensor_list[7]}, "Legs - Front side", 7},
    {5, 14, 256, 256, {&sensor_list[8], &sensor_list[9]}, "Torso - Fixed and Detachable left side", 7},
    {6, 48, 225, 225, {&sensor_list[10], &sensor_list[11]}, "Torso - Front side", 7},
    {7, 47, 102, 102, {&sensor_list[12], &sensor_list[13]}, "Legs - Right and Left side", 7},
};

float ambient_temp = 22; // would get this value from the other esp
unsigned long last_time = 0;
uint16_t time_passed = 0;
unsigned long last_pid = 0;

float getChannelTemp(channels &ch)
{
  float t1 = ch.sensors[0]->last_temp;
  float t2 = ch.sensors[1]->last_temp;

  // Handle DS18B20 error values
  if (t1 == -127 || t1 == 85)
    t1 = NAN; // returns nan aka not a number if the sensor is not responding properly
  if (t2 == -127 || t2 == 85)
    t2 = NAN; // 85 tends to show right after booting up and -127 shows if it can't get a proper reading

  if (!isnan(t1) && !isnan(t2))
  {
    return (t1 + t2) * 0.5f;
  }

  // Only sensor 1 valid
  if (!isnan(t1))
    return t1;

  // Only sensor 2 valid
  if (!isnan(t2))
    return t2;

  // Neither sensor valid
  return NAN;
}

void initPID()
{
  for (int i = 0; i < MAX_CHANNELS; i++)
  {
    channel_list[i].pid.Kp = 50.0;
    channel_list[i].pid.Ki = 0.05;
    channel_list[i].pid.Kd = 0.0;

    channel_list[i].pid.integral = 0;
    channel_list[i].pid.last_error = 0;
    channel_list[i].pid.stage2_enabled = false;
    channel_list[i].pid.last_pid = 0;
  }
}

void initPWM()
{

  // timer configuration
  ledc_timer_config_t timer_cfg = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = (ledc_timer_bit_t)PWM_RES,
      .timer_num = LEDC_TIMER_0, // all channels on the same timer
      .freq_hz = PWM_FREQ,
      .clk_cfg = LEDC_AUTO_CLK // auto selects the best clock for the set parameters
  };
  esp_err_t err = ledc_timer_config(&timer_cfg);
  if (err != ESP_OK)
  {
    Serial.printf("Timer configuration failed: %d\n", err);
  }

  // PWM hardware setup with phase staggering
  for (int i = 0; i < MAX_CHANNELS; i++)
  {
    uint32_t period = (1 << PWM_RES);                              // total count for one period aka 1023
    uint32_t hpoint_offset = (i * period / MAX_CHANNELS) % period; // calculates the highpoint offset, each is 1/8 of the period

    ledc_channel_config_t channel_cfg = {                                     // structure variable to define settings for the current channel
                                         .gpio_num = channel_list[i].pwm_pin, // what gpio pin it is attached to
                                         .speed_mode = LEDC_LOW_SPEED_MODE,
                                         .channel = (ledc_channel_t)channel_list[i].pwm_channel, // what channel number
                                         .intr_type = LEDC_INTR_DISABLE,                         // disables interrupts for this
                                         .timer_sel = LEDC_TIMER_0,                              // all of them get timer0 with the right freq and res
                                         .duty = 0,                                              // initially off
                                         .hpoint = hpoint_offset,                                // applies the calculated highpoint offsets for phase staggering
                                         .flags = {.output_invert = 0}};
    err = ledc_channel_config(&channel_cfg); // actually applies the channel configuration we just did
    if (err != ESP_OK)
    {
      Serial.printf("CH %d config failed: %d\n", i, err); // error check
    }
    channel_list[i].target_temp = channel_list[i].target_temp + ambient_temp;
  }
}

void updateTemps()
{
  for (int i = 0; i < MAX_SENSORS; i++)
  {
    sensor_list[i].last_temp = sensors.getTempC(sensor_list[i].address);
  }

  for (int i = 0; i < MAX_CHANNELS; i++)
  {

    float t1 = channel_list[i].sensors[0]->last_temp;
    float t2 = channel_list[i].sensors[1]->last_temp;

    // Handle DS18B20 error values
    if (t1 == -127 || t1 == 85)
      t1 = NAN; // returns nan aka not a number if the sensor is not responding properly
    if (t2 == -127 || t2 == 85)
      t2 = NAN; // 85 tends to show right after booting up and -127 shows if it can't get a proper reading

    // Mutually Exclusive Logic:
    if (!isnan(t1) && !isnan(t2))
    {
      // Both valid: use average
      channel_list[i].current_temp = (t1 + t2) * 0.5f;
    }
    else if (!isnan(t1))
    {
      // Only sensor 1 valid
      channel_list[i].current_temp = t1;
    }
    else if (!isnan(t2))
    {
      // Only sensor 2 valid
      channel_list[i].current_temp = t2;
    }
    else
    {
      // Neither sensor valid: indicate failure
      channel_list[i].current_temp = NAN;
      ledcWrite(channel_list[i].pwm_channel, 0);
    }
  }
}

uint16_t updatePID(channels &ch, float dt)
{
  if (isnan(ch.current_temp))
  {
    return 0;
  }
  if (dt < 0.1f)
    dt = 0.1f; // 100 ms minimum

  float error = ch.target_temp - ch.current_temp;

  float P_term = ch.pid.Kp * error;
  float D_term = ch.pid.Kd * ((error - ch.pid.last_error) / dt);

  float output_prelim = P_term + ch.pid.Ki * ch.pid.integral + D_term;

  // Conditional Integration: Only accumulate integral if the system is NOT saturated.
  if (output_prelim < ch.pwm_max && output_prelim > 0.0f)
  {
    ch.pid.integral += error * dt;
    ch.pid.integral = constrain(ch.pid.integral, -500, 500); // Soft limit still fine
  }

  float output = P_term + ch.pid.Ki * ch.pid.integral + D_term;
  output = constrain(output, 0, ch.pwm_max);

  ch.pid.last_error = error;

  return (uint16_t)output;
}

void setup(void)
{
  Serial.begin(9600); // needed to use serial for testing
  sensors.begin();    // starts the temperature sensors
  delay(1000);
  sensors.requestTemperatures();
  delay(1500);
  initPWM();
  initPID();
  updateTemps();

  // for (int i = 0; i < MAX_CHANNELS; i++)
  // {
  //   ledcWrite(channel_list[i].pwm_channel, channel_list[i].pwm_value);
  //   Serial.printf("Channel %d: PWM: %d, DTC: %.2f, Pin: %d\n",
  //                 channel_list[i].pwm_channel,
  //                 channel_list[i].pwm_value,
  //                 (channel_list[i].pwm_value * 100) / 1023,
  //                 channel_list[i].pwm_pin);
  // }

  sensors.setWaitForConversion(false);
  Serial.printf("t,c0,c1,c2,c3,c4,c5,c6,c7\n");
}

void loop()
{
  
  unsigned long now = millis();
  if (now - last_time >= 4000) {
 sensors.requestTemperatures();
  }
  if (now - last_time >= 5000) // ← run only once every five seconds
  {
    updateTemps();
    last_time = now;
    time_passed = time_passed + 5;
    Serial.printf("%d,", time_passed);
    for (int i = 0; i < MAX_CHANNELS; i++)
    {
      Serial.printf("%.2f", channel_list[i].current_temp);
      if (i < MAX_CHANNELS - 1)
      {
        Serial.printf(",");
      }

      if (channel_list[i].current_temp >= channel_list[i].target_temp &&
          !channel_list[i].pid.stage2_enabled)
      {
        channel_list[i].pid.stage2_enabled = true;
        Serial.printf("#Channel %d switched to PID.\n", i);
        channel_list[i].pid.integral = 0;
        channel_list[i].pid.last_error = 0;
        channel_list[i].pid.last_pid = millis();
      }
      if (channel_list[i].pid.stage2_enabled)
      {
        if (channel_list[i].current_temp > TEMP_LIMIT)
        {
          ledcWrite(channel_list[i].pwm_channel, 0);
          channel_list[i].pid.stage2_enabled = false;
          Serial.printf("!!! HARD SHUTOFF ch %d\n", i);
          channel_list[i].pid.integral = 0;
          channel_list[i].pid.last_error = 0;

          continue;
        }
        channel_list[i].pwm_value = updatePID(channel_list[i], (now - channel_list[i].pid.last_pid) / 1000.0f);
        if (channel_list[i].pwm_value > channel_list[i].pwm_max)
        {
          channel_list[i].pwm_value = channel_list[i].pwm_max;
        }
        channel_list[i].pid.last_pid = now;
        ledcWrite(channel_list[i].pwm_channel, channel_list[i].pwm_value);
      }
    }
    Serial.println();
    
  }
}


//