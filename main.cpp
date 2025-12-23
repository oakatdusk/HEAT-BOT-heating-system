// BASIC PID CODE FOR TESTING EVERYTHING SEPARATELY
#include <Arduino.h>
#include <OneWire.h>           //to use the temperature sensors
#include <DallasTemperature.h> //for easier work with the temp sensors
#include <driver/ledc.h>

#define ONE_WIRE_BUS 5 // what pin the temp sensors are on (GPIO5)
#define MAX_SENSORS 14 // how many sensors we should have
#define MAX_CHANNELS 8

#define PWM_FREQ 250 // the frequency for the pwm, might lower this
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
  float Kp, Ki, Kd; //not really using Kd but might as well have it ig

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
    {0, 11, 273, 273, {&sensor_list[0], &sensor_list[0]}, "Head - Front and Top", 14},
    {1, 10, 109, 109, {&sensor_list[1], &sensor_list[1]}, "Torso - Right side", 7},
    {2, 13, 273, 273, {&sensor_list[2], &sensor_list[3]}, "Shoulders - Left and Right side", 7},
    {3, 12, 409, 409, {&sensor_list[4], &sensor_list[5]}, "Head - Left and Right side", 14},
    {4, 21, 225, 225, {&sensor_list[6], &sensor_list[7]}, "Legs - Front side", 7},
    {5, 14, 205, 205, {&sensor_list[8], &sensor_list[9]}, "Torso - Fixed and Detachable left side", 7},
    {6, 48, 166, 166, {&sensor_list[10], &sensor_list[11]}, "Torso - Front side", 7},
    {7, 47, 46, 46, {&sensor_list[12], &sensor_list[13]}, "Legs - Right and Left side", 7},
};

float ambient_temp = 20.3; // would get this value from the other esp, maybe should just measure it on our own??
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

  // only sensor 1 valid
  if (!isnan(t1))
    return t1;

  // only sensor 2 valid
  if (!isnan(t2))
    return t2;

  // neither valid
  ledcWrite(ch.pwm_channel, ch.pwm_value);
  return NAN;
}

void initPID() //just setting up all the pid values that we start with
{
  for (int i = 0; i < MAX_CHANNELS; i++)
  {
    channel_list[i].pid.Kp = 80.0;
    channel_list[i].pid.Ki = 0.50;
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

    if (i != 0) channel_list[i].pwm_value=0;

    ledc_channel_config_t channel_cfg = {                                     // structure variable to define settings for the current channel
                                         .gpio_num = channel_list[i].pwm_pin, // what gpio pin it is attached to
                                         .speed_mode = LEDC_LOW_SPEED_MODE,
                                         .channel = (ledc_channel_t)channel_list[i].pwm_channel, // what channel number
                                         .intr_type = LEDC_INTR_DISABLE,                         // disables interrupts for this
                                         .timer_sel = LEDC_TIMER_0,                              // all of them get timer0 with the right freq and res
                                         .duty = channel_list[i].pwm_value,                                              // initially at stage 1 value
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

void updateTemps() //updates all sensor temps and uses that to update all channel temps
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
      channel_list[i].pid.stage2_enabled = false;
    }
  }
}

uint16_t updatePID(channels &ch, float dt)
{
  if (isnan(ch.current_temp))
  {
    return 0;
  }

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

  for (int i = 0; i < MAX_CHANNELS; i++)
  {
    ledcWrite(channel_list[i].pwm_channel, channel_list[i].pwm_value);
    Serial.printf("Channel %d: PWM: %d, DTC: %.2f, Pin: %d\n",
                  channel_list[i].pwm_channel,
                  channel_list[i].pwm_value,
                  (channel_list[i].pwm_value * 100.0f) / 1023.0f,
                  (int)channel_list[i].pwm_pin);
                  ledcWrite(channel_list[i].pwm_channel, channel_list[i].pwm_value);
  }

  sensors.setWaitForConversion(false);
  Serial.printf("time,temp,pwm,Kp,Ki,Kd,integral,error, amb temp\n");
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
    int i = 0;

      if (channel_list[i].current_temp >= channel_list[i].target_temp - 0.3 &&
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
        if (channel_list[i].current_temp > TEMP_LIMIT) //safety cutoff
        {
          ledcWrite(channel_list[i].pwm_channel, 0);
          channel_list[i].pid.stage2_enabled = false;
          Serial.printf("!!! HARD SHUTOFF ch %d\n", i);
          channel_list[i].pid.integral = 0;
          channel_list[i].pid.last_error = 0;
        }

        channel_list[i].pwm_value = updatePID(channel_list[i], (now - channel_list[i].pid.last_pid) / 1000.0f);
        if (channel_list[i].pwm_value > channel_list[i].pwm_max)
        {
          channel_list[i].pwm_value = channel_list[i].pwm_max;
        }
        channel_list[i].pid.last_pid = now;
        ledcWrite(channel_list[i].pwm_channel, channel_list[i].pwm_value);

      }
      Serial.printf(
        "%d,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
        time_passed,
        channel_list[i].current_temp,
        channel_list[i].pwm_value,
        channel_list[i].pid.Kp,
        channel_list[i].pid.Ki,
        channel_list[i].pid.Kd,
        channel_list[i].pid.integral,
        channel_list[i].pid.last_error,
        channel_list[7].current_temp);
    
  }
}

// //_______________________________________________________________
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
//     {0, 11, 273, 273, {&sensor_list[0], &sensor_list[0]}, "Head - Front and Top", 14},
//     {1, 10, 109, 109, {&sensor_list[1], &sensor_list[1]}, "Torso - Right side", 7},
//     {2, 13, 273, 273, {&sensor_list[2], &sensor_list[3]}, "Shoulders - Left and Right side", 7},
//     {3, 12, 409, 409, {&sensor_list[4], &sensor_list[5]}, "Head - Left and Right side", 14},
//     {4, 21, 166, 166, {&sensor_list[6], &sensor_list[7]}, "Legs - Front side", 7},
//     {5, 14, 205, 205, {&sensor_list[8], &sensor_list[9]}, "Torso - Fixed and Detachable left side", 7},
//     {6, 48, 166, 166, {&sensor_list[10], &sensor_list[11]}, "Torso - Front side", 7},
//     {7, 47, 46, 46, {&sensor_list[12], &sensor_list[13]}, "Legs - Right and Left side", 7},
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
//   channel_list[3].pwm_max = 273;

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
//     if (i == 3)
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
//     int i = 3;
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

// #include <Arduino.h>
// #include <OneWire.h>           //to use the temperature sensors
// #include <DallasTemperature.h> //for easier work with the temp sensors

// #define ONE_WIRE_BUS 5 // what pin the temp sensors are on (GPIO4)
// #define MAX_SENSORS 14 // how many sensors we should have
// #define MAX_CHANNELS 8

// #define PWM_FREQ 250 // the frequency for the pwm, might lower this
// #define PWM_RES 10    // 10 bit

// #define TEMP_DIFF_A 7 // how much higher the target temp needs to be compared to ambient temp
// #define TEMP_DIFF_B 14

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
//     {7, 47, 102, {&sensor_list[12], &sensor_list[13]}, "Legs - Right and Left side", 7},
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
//                   (channel_list[i].pwm_value * 100) / 1023,
//                   channel_list[i].pwm_pin);
//   }

//   sensors.setWaitForConversion(false);
// }

// void loop()
// {
//   unsigned long now = millis();
//   if (now - last_time >= 5000) // ← run only once every five seconds
//   {
//     last_time = now;
//     time_passed = time_passed + 5;
//     Serial.printf("time passed: %d\n", time_passed);
//     for (int i = 0; i < MAX_SENSORS; i++)
//     {
//       sensor_list[i].last_temp = sensors.getTempC(sensor_list[i].address);
//     }
//     for (int i = 0; i < MAX_CHANNELS; i++)
//     {
//       channel_list[i].current_temp = getChannelTemp(channel_list[i]);
//       Serial.printf("CH %d: %.2f C\n", i, channel_list[i].current_temp);
//     }
//     Serial.printf("______________________\n");
//     sensors.requestTemperatures();
//   }
// }
