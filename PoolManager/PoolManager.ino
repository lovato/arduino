#include <Time.h>
#include <DS1302RTC.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include "DHT.h"
#include <pt.h> 

static struct pt clockThread;
static struct pt tempThread;
static struct pt lightThread;
static struct pt motionThread;

int motion_1 = 6;
bool motion = false;

DHT dht;

int photoRPin = 0; 
int minLight;          //Used to calibrate the readings
int maxLight;          //Used to calibrate the readings
int lightLevel;
int adjustedLightLevel;

byte newChar[8] = {
  B01000,
  B10100,
  B01000,
  B00000,
  B00111,
  B01000,
  B01000,
  B00111
};

/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */

char buf_data[50];
char buf_hora[50];

// Init the DS1302
// Set pins:  CE, IO,CLK
DS1302RTC RTC(27, 29, 31);

// Optional connection for RTC module
#define DS1302_GND_PIN 33
#define DS1302_VCC_PIN 35

// pin 8 - Serial clock out (SCLK)
// pin 9 - Serial data out (DIN)
// pin 10 - Data/Command select (D/C)
// pin 11 - LCD chip select (CS/CE)
// pin 12 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(2, 3, 4, 10, 11);

String clock_data = "";
String clock_hora = "";
float temperatura = 0;
int humidade = 0;

String display_l1 = "";
String display_l2 = "";
String display_l3 = "";
String display_l4 = "";
String display_l5 = "";
String display_status = "";

String display_l1_curr = "z";
String display_l2_curr = "z";
String display_l3_curr = "z";
String display_l4_curr = "z";
String display_l5_curr = "z";
String display_status_curr = "z";

bool led_status = false;

void init_serial() {
  // initialize serial output for debugging
  Serial.begin(9600);
}

void init_display() {
  display.begin();
  display.setContrast(50); //Ajusta o contraste do display

  display.clearDisplay();   //Apaga o buffer e o display

  display.setTextSize(1);  //Seta o tamanho do texto
  display.setTextColor(BLACK); //Seta a cor do texto
}

void init_rtc() {
  // Activate RTC module
  digitalWrite(DS1302_GND_PIN, LOW);
  pinMode(DS1302_GND_PIN, OUTPUT);

  digitalWrite(DS1302_VCC_PIN, HIGH);
  pinMode(DS1302_VCC_PIN, OUTPUT);

  debug("RTC activated");
  
  delay(500);

//  setTime(23,14,00,05,12,2014);

  // Check clock oscillation  
  if (RTC.haltRTC())
    debug("Clock stopped!");
  else
    debug("Clock working.");
  
  if (RTC.writeEN())
    debug("Write allowed.");
  else
    debug("Write protected.");
    
  delay ( 2000 );

  // Setup Time library  
  setSyncProvider(RTC.get); // the function to get the time from the RTC
  if(timeStatus() == timeSet)
    debug("RTC Sync Ok!");
  else
    debug("RTC Sync FAIL!");

  delay ( 2000 );
  
}

void init_photosensor() {
 lightLevel=analogRead(photoRPin);
 minLight=lightLevel-20;
 maxLight=lightLevel;
}

// the setup function runs once when you press reset or power the board
void setup() {
  init_serial();
  debug("SYSTEM BOOT");
  pinMode(50, OUTPUT);
  pinMode(motion_1,INPUT);
  init_display();
  init_rtc();
  dht.setup(5);
  init_photosensor();
  PT_INIT(&clockThread);
  PT_INIT(&tempThread);
  PT_INIT(&lightThread);
  PT_INIT(&motionThread);
  display.clearDisplay();   //Apaga o buffer e o display
}

void update_photosensor() {
   lightLevel=analogRead(photoRPin);
   if(minLight>lightLevel){
     minLight=lightLevel;
   }
   if(maxLight<lightLevel){
     maxLight=lightLevel;
   }
   
   //Adjust the light level to produce a result between 0 and 100.
   adjustedLightLevel = map(lightLevel, minLight, maxLight, 0, 100);
   
   if (adjustedLightLevel > 90) {
     digitalWrite(50, HIGH);   // turn the LED on (HIGH is the voltage level)
   } else {
     digitalWrite(50, LOW);    // turn the LED off by making the voltage LOW
   } 
   
}

void update_clock() {
  //Format the time and date and insert into the temporary buffer
  snprintf(buf_data, sizeof(buf_data), "%s %02d/%02d/%04d",
           dayShortStr(weekday()),
           day(), month(), year());
  snprintf(buf_hora, sizeof(buf_hora), "%02d:%02d:%02d",
           hour(), minute(), second());
  
  clock_data = buf_data;
  clock_hora = buf_hora;
  
  if (second() == 0) {
    debug("Another minute has passed...");
  }
}

void update_motionsensor() {
  int sensor_1 = digitalRead(motion_1);
  if (sensor_1 == HIGH){
    motion = true;
  } else {
    motion = false;
  }
}

void update_temperature() {
  //delay(dht.getMinimumSamplingPeriod());
  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();
  
//  temperatura = int(temperature) - 2;
  temperatura = (temperatura + int(temperature) - 2) / 2;
  humidade = int(humidity);
}

static int f_clockThread(struct pt *pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis();
    update_clock();
  }
  PT_END(pt);
}

static int f_tempThread(struct pt *pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis();
    update_temperature();
  }
  PT_END(pt);
}

static int f_lightThread(struct pt *pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis();
    update_photosensor();
  }
  PT_END(pt);
}

static int f_motionThread(struct pt *pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis();
    update_motionsensor();
  }
  PT_END(pt);
}

// the loop function runs over and over again forever
void loop() {
  f_clockThread(&clockThread, 1000);
  f_tempThread(&tempThread, 2000);
  f_lightThread(&lightThread, 2000);
  f_motionThread(&motionThread, 500);
  
  display_l1 = clock_data;
  display_l2 = clock_hora + "  " + String(temperatura);
  
  display_l3 = "Humidity: " + String(humidade) + "%";
  display_l4 = "Darkness: " + String(adjustedLightLevel) + "%";

  display_l5 = "Motion: " + String(motion);
  
  // Warning!
  if(timeStatus() != timeSet) {
    debug(F("RTC ERROR: SYNC!"));
  }
  
  paint();
  
  if(display_status.length() > 0) {
    display_status = display_status.substring(1);
  }
  
  delay(100);
}

String draw_line(int line,String current,String new_one){
  if(current != new_one){
    display.setTextColor(WHITE); //Seta a cor do texto
    display.setCursor(0,(line-1)*8);  //Seta a posição do cursor
    display.print(current);  
    current = new_one;
    display.setTextColor(BLACK); //Seta a cor do texto
    display.setCursor(0,(line-1)*8);  //Seta a posição do cursor
    display.print(new_one);
  }
  return current;
}

void paint() {
  led_blink();

//  display.clearDisplay();   //Apaga o buffer e o display
  display_l1_curr = draw_line(1,display_l1_curr,display_l1);
  display_l2_curr = draw_line(2,display_l2_curr,display_l2);
  display_l3_curr = draw_line(3,display_l3_curr,display_l3);
  display_l4_curr = draw_line(4,display_l4_curr,display_l4);
  display_l5_curr = draw_line(5,display_l5_curr,display_l5);
  display_status_curr = draw_line(6,display_status_curr,display_status);

//  display.clearDisplay();   //Apaga o buffer e o display
//  display.setCursor(0,0);  //Seta a posição do cursor
//  display.print(display_l1);  
//  display.setCursor(0,8);  //Seta a posição do cursor
//  display.print(display_l2);  
//  display.setCursor(0,8*2);  //Seta a posição do cursor
//  display.print(display_l3);  
//  display.setCursor(0,8*3);  //Seta a posição do cursor
//  display.print(display_l4);  
//  display.setCursor(0,8*4);  //Seta a posição do cursor
//  display.print(display_l5);  
//  display.setCursor(0,8*5);  //Seta a posição do cursor
//  display.print(display_status);  
  display.display();
}

void debug(String str) {
  Serial.println(str);
  str = "              " + str;
  display_status = display_status + str;
}

void led_blink() {
  if (led_status) {
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    led_status = false;
  } else {
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    led_status = true;
  }
}
