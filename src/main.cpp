#include <Arduino.h>
#include <PID_v1.h>
#include <TSIC.h>
#include <U8g2lib.h>
#include <math.h>

// TODO: millis doesn't give frame-time, but time since start

constexpr uint8_t DISPLAY_RESET = U8X8_PIN_NONE;
constexpr uint8_t DISPLAY_CLOCK = U8X8_PIN_NONE;
constexpr uint8_t DISPLAY_DATA = U8X8_PIN_NONE;

constexpr uint8_t DEBUG_OUTPUT = 0U;

constexpr uint8_t RELAY_PIN = 7U;
// 14 = analog_pin 0
constexpr uint8_t TEMP_SENSOR_PIN = 14U;

constexpr unsigned long MEASUREMENT_INTERVAL = 400UL;
constexpr unsigned long SERIAL_MONITOR_INTERVAL = 1000UL;
constexpr unsigned int WINDOW_SIZE = 1000;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,
                                         DISPLAY_RESET,
                                         DISPLAY_CLOCK,
                                         DISPLAY_DATA);

TSIC temperature_sensor(TEMP_SENSOR_PIN);

// input = variable we're trying to control
// output = variable that will be adjusted by PID
double input, output;
// temperature that we want to maintain
double set_point_temperature = 95.0; // 30.0

// reset-time
double pid_tn = 399.0;
// derivative-time
double pid_tv = 0.0;

// tuning-parameters, determine how PID will change output
double pid_kp = 69.0;
double pid_ki = 0.0;
double pid_kd = pid_tv * pid_kp;

bool is_heating = false;

unsigned long windowStartTime;

PID pid(&input,
        &output,
        &set_point_temperature,
        pid_kp,
        pid_ki,
        pid_kd,
        P_ON_E,
        DIRECT);

void
prepareDisplay()
{
  u8g2.setFont(u8g2_font_profont11_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void
setup()
{
  Serial.begin(9600);

  windowStartTime = millis();

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  u8g2.begin();

  // init PID
  pid.SetSampleTime(WINDOW_SIZE);
  pid.SetOutputLimits(0, WINDOW_SIZE);
  pid.SetMode(AUTOMATIC);
}

void
printDataToScreen()
{
  static char txt[16];
  static char txt_two[16];
  static char txt_three[16];

  int whole_input = floor(input);
  int fraction_input = roundf((input - floor(input)) * 100.0f);

  int whole_output = floor(output);
  int fraction_output = roundf((output - floor(output)) * 100.0f);

  sprintf(txt, "i: %d.%d", whole_input, fraction_input);
  sprintf(txt_two, "o: %d.%d", whole_output, fraction_output);
  if (is_heating) {
    txt_three[0] = 'o';
    txt_three[1] = 'n';
    txt_three[2] = '\0';
  } else {
    txt_three[0] = 'o';
    txt_three[1] = 'f';
    txt_three[2] = 'f';
    txt_three[3] = '\0';
  }

  u8g2.drawStr(0, 20, txt);
  u8g2.drawStr(0, 40, txt_two);
  u8g2.drawStr(0, 60, txt_three);
}

void
printPIDInputOutput()
{
  static unsigned long time_of_last_output = 0UL;

  unsigned long time_passed = millis();

  if (time_passed >= time_of_last_output + SERIAL_MONITOR_INTERVAL) {
    time_of_last_output = time_passed;

    Serial.print("input: ");
    Serial.println(input);
    Serial.print("output: ");
    Serial.println(output);
    Serial.println("\n\n");
  }
}

void
measureTemperature(double* celsius)
{
  static unsigned long time_of_last_measurement = 0UL;

  unsigned long time_passed = millis();

  if (time_passed >= time_of_last_measurement + MEASUREMENT_INTERVAL) {
    time_of_last_measurement = time_passed;

    uint16_t measurement = 0U;
    temperature_sensor.getTemperature(&measurement);

    *celsius = temperature_sensor.calc_Celsius(&measurement);
  }
}

void
turnOn()
{
  digitalWrite(RELAY_PIN, HIGH);
  is_heating = true;
  if (DEBUG_OUTPUT) {
    Serial.println("Turned on");
  }
}

void
turnOff()
{
  is_heating = false;
  digitalWrite(RELAY_PIN, LOW);
  if (DEBUG_OUTPUT) {
    Serial.println("Turned off");
  }
}

/**
 * time- & pid-based adaption of relay
 * */
void
adaptHeating()
{
  if (millis() - windowStartTime > WINDOW_SIZE) {
    // time to shift the Relay Window
    windowStartTime += WINDOW_SIZE;
  }
  if (output < millis() - windowStartTime) {
    turnOff();
  } else {
    turnOn();
  }
}

void
loop()
{
  u8g2.firstPage();

  measureTemperature(&input);
  pid.Compute();
  adaptHeating();

  if (DEBUG_OUTPUT != 0U) {
    printPIDInputOutput();
  }

  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    printDataToScreen();
  } while (u8g2.nextPage());
}