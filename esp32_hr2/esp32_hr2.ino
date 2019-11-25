#include "BluetoothSerial.h"
#include <Wire.h>
#include "LiquidCrystal_I2C.h"

#define MAX_SAFE_HR 100
#define MIN_SAFE_HR 40

#define THRESHOLD 85
#define POLLING_RATE 100 // Hz
#define DEBOUNCE_INTERVAL 300 // ms
#define BLINK_INTERVAL 300 // ms
#define RED_LED 16
#define BLUE_LED 17
#define GREEN_LED 18
#define ONBOARD_LED 2

#define BETA 3435.0
#define NOMINAL_TEMP 298.15

const float inv_beta = 1/BETA;
const float inv_nominal_temp = 1/NOMINAL_TEMP;
LiquidCrystal_I2C lcd(0x3F,16,2);

BluetoothSerial ESP_BT;

int sample_period = 1000000/POLLING_RATE;

int tempsum, average;
bool pulse_tracking = false;
int last_beat = 0;
volatile int record[30] = {0};
volatile int record_index = 0;

volatile int led_number = GREEN_LED;

int values[100];
int values_index = 0;

unsigned long blink_time = 0;

// Core 0 Interrupt thread
hw_timer_t * timer = NULL;
void IRAM_ATTR onTimer(){
  int reading = analogRead(35);
  values[values_index] = reading;
  values_index++;
  // If the index has reached 99, this is only possible if a second has passed
  if(values_index == 99){
    record_index++;
    record_index%=30;
    record[record_index] = 0;
  }
  values_index %= 100;
  tempsum = 0;
  for(int i = 0; i < 100; i++) tempsum += values[i];
  average = tempsum / 100;
  if(reading > average + THRESHOLD && millis() > last_beat + DEBOUNCE_INTERVAL){
    pulse_tracking = true;
    digitalWrite(led_number, HIGH);
    blink_time = millis();
  }
  else if(pulse_tracking){
    pulse_tracking = false;
    last_beat = millis();
    record[record_index]++;
    digitalWrite(led_number, LOW);
  }
  // So far, motion distorts the signal, how should we account for this?
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  ESP_BT.begin("ESP32_HR");
  ESP_BT.println("BT ready");

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Press to begin");

  // Just follow pulse until BOOT button is pressed, this lets user get device on wrist
  //  before the ESP reads garbage data
  while(digitalRead(0)){}

  lcd.clear();
  lcd.print("Please wait 30 seconds");
  
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, sample_period, true);
  timerAlarmEnable(timer);

  pinMode(2, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  
  for(int i = 0; i < 100; i++){
    values[i] = analogRead(35);
    delay(1);
  }
}

bool thirty_passed = false;

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  if(record_index == 29) thirty_passed = true;
  int sum = 0;
  for(int i = 0; i < 30; i++) sum += record[i];
  int hr = sum*2;
  if(thirty_passed == true){
    // Various forms of output: LED, Bluetooth, LCD, and Serial
    if(hr > MAX_SAFE_HR || hr < MIN_SAFE_HR) led_number = RED_LED;
    else led_number = GREEN_LED;
    ESP_BT.print("Pulse rate is: ");
    ESP_BT.println(hr);
    lcd.clear();
    lcd.print("Heart Rate: ");
    lcd.print(hr);
    lcd.setCursor(0,1);
    float inv_reading = 4096./analogRead(34);
    float kelvin = 1/((inv_beta*log(inv_reading-1))+inv_nominal_temp);
    float fahrenheit = 1.8*(kelvin-273.15)+32;
    lcd.print("Skin Temp: ");
    lcd.print(fahrenheit);
    ESP_BT.print("Temperature is: ");
    ESP_BT.println(fahrenheit);
    Serial.println(hr);
  }
}
