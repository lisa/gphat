/* (c) Copyright 2010 Lisa Seelye. All Rights Reserved. */

#include <Math.h>
#include <TinyGPS.h>

#define GPHAT_DEBUG
// #define GPHAT_DEBUG_SERIAL

#define LIGHT_RING_DELAY 50 // 100ms delay between each light (1200ms total)
#define PI 3.14159265358979
#define STOP_SPEED_THRESHOLD 0.5

#define LED_0   13
#define LED_30  12
#define LED_60  11
#define LED_90  10
#define LED_120 9
#define LED_150 8
#define LED_180 7
#define LED_210 6
#define LED_240 5
#define LED_270 4
#define LED_300 3
#define LED_330 2


enum gphat_state { GPH_STARTUP,    GPH_READDATA,
                   GPH_AMISTOPPED, GPH_LIGHTRING,
                   GPH_SINGLELIGHT, GPH_BUTTONPRESS };

TinyGPS gps;           // TinyGPS Object
gphat_state gph_state; // State machine for loop()ing
bool button_press;
bool gps_lock;         // Do I have a GPS Lock true 
                       // if read_gps_data() && (fix_age > 0) && (fix_age<5000)
float home_lat, home_long;
float cur_lat,  cur_long;
unsigned long fix_age; // Age of last lock in ms

/* Prototypes */
inline void led_0  (const int);
inline void led_30 (const int);
inline void led_60 (const int);
inline void led_90 (const int);
inline void led_120(const int);
inline void led_150(const int);
inline void led_180(const int);
inline void led_210(const int);
inline void led_240(const int);
inline void led_270(const int);
inline void led_300(const int);
inline void led_330(const int);
inline void led_360(const int);
inline void light_ring(const unsigned int);

void set_home_point();
void cycle_led(const unsigned short, const unsigned int);

short heading_to_home();
bool am_i_stopped();
bool read_gps_data();
/* End prototypes */


void setup() {
  button_press = true; // Set my location at power-on.
  Serial.begin(9600);
  gph_state = GPH_STARTUP;
  gps_lock = false;
  fix_age = 500000;
  home_lat = 0.0;
  home_long = 0.0;
    
  // change digital pins for LED output
  pinMode(LED_0,OUTPUT);
  pinMode(LED_30,OUTPUT);
  pinMode(LED_60,OUTPUT);
  pinMode(LED_90,OUTPUT);
  pinMode(LED_120,OUTPUT);
  pinMode(LED_150,OUTPUT);
  pinMode(LED_180,OUTPUT);
  pinMode(LED_210,OUTPUT);
  pinMode(LED_240,OUTPUT);
  pinMode(LED_270,OUTPUT);
  pinMode(LED_300,OUTPUT);
  pinMode(LED_330,OUTPUT);
}

void loop() {
  short true_course = 0;
#ifdef GPHAT_DEBUG
  Serial.print("gph_state = ");
  Serial.println(gph_state,DEC);
  if (!gps_lock) {
    Serial.print("Pos  = "); Serial.print(cur_lat); Serial.print(", "); Serial.println(cur_long);
    Serial.print("Fix Age = ");
    Serial.println(fix_age,DEC);
    Serial.print("Course = ");
    Serial.println(gps.f_course());
  }
#endif
  switch (gph_state) {
    case GPH_STARTUP:
    gph_state = GPH_READDATA;
    break;
    
    case GPH_READDATA:
    gps_lock = read_gps_data();
    gps.f_get_position(&cur_lat,&cur_long,&fix_age);
    // when no lock fix_age == -1
    gps_lock |= (fix_age > 0) && (fix_age < 5000);
    gph_state = GPH_AMISTOPPED;
    break;
    
    case GPH_BUTTONPRESS:
    gps_lock = read_gps_data();
    gps.f_get_position(&cur_lat,&cur_long,&fix_age);
    button_press = false;
    gps_lock |= (fix_age > 0) && (fix_age < 5000);
    if (gps_lock) {
      // Set my home location, pulse lights once
      home_lat = cur_lat;
      home_long = cur_long;
#ifdef GPHAT_DEBUG
      Serial.print("Home = "); Serial.print(home_lat); Serial.print(", "); Serial.println(home_long);
#endif
      light_ring(0); light_ring(0); light_ring(0); light_ring(0);
      gph_state = GPH_READDATA;
    }
    else {
      /* Do not have a lock yet so look pretty! */
      light_ring(LIGHT_RING_DELAY);
      gph_state = GPH_BUTTONPRESS;
    }    
    break;
    
    case GPH_AMISTOPPED:
    if (! gps_lock) {
      // We don't have a lock so we have no idea! Around we go...
      gph_state = GPH_LIGHTRING;
    }
    else {
      // We have a lock so update get our position
      gps.f_get_position(&cur_lat,&cur_long,&fix_age);
      gph_state = am_i_stopped() ? GPH_SINGLELIGHT : GPH_LIGHTRING;
    }
    break;
    
    case GPH_LIGHTRING:
    light_ring(LIGHT_RING_DELAY >> 1);
    gph_state = GPH_READDATA;
    break;
    
    case GPH_SINGLELIGHT:
    /* 
     * 1. Figure out what my GPS coords are
     * 2. Recall my home point GPS coords
     * 3. Do math to figure out vector to home point
     * 4. Blink appropriate LED
    */
    true_course = heading_to_home();
    if (true_course == 360 || (true_course >= 0 && true_course < 30))
      cycle_led(0,LIGHT_RING_DELAY);
    else if (true_course >= 30 && true_course < 60)
      cycle_led(30,LIGHT_RING_DELAY);
    else if (true_course >= 60 && true_course < 90)
      cycle_led(60,LIGHT_RING_DELAY);
    else if (true_course >= 90 && true_course < 120)
      cycle_led(90,LIGHT_RING_DELAY);
    else if (true_course >= 120 && true_course < 150)
      cycle_led(120,LIGHT_RING_DELAY);
    else if (true_course >= 150 && true_course < 180)
      cycle_led(150,LIGHT_RING_DELAY);
    else if (true_course >= 180 && true_course < 210)
      cycle_led(180,LIGHT_RING_DELAY);
    else if (true_course >= 210 && true_course < 240)
      cycle_led(210,LIGHT_RING_DELAY);
    else if (true_course >= 240 && true_course < 270)
      cycle_led(240,LIGHT_RING_DELAY);
    else if (true_course >= 270 && true_course < 300)
      cycle_led(270,LIGHT_RING_DELAY);
    else if (true_course >= 300 && true_course < 330)
      cycle_led(300,LIGHT_RING_DELAY);
    else if (true_course >= 330 && true_course < 360)
      cycle_led(330,LIGHT_RING_DELAY);

    delay(LIGHT_RING_DELAY * 11);
    gph_state = GPH_READDATA;
    break;
  }
  if (button_press)
    gph_state = GPH_BUTTONPRESS;
#ifdef GPHAT_DEBUG
  Serial.println();
#endif
}

inline bool have_lock(const unsigned int age_threshold) {
  gps.f_get_position(&cur_lat,&cur_long,&fix_age);
  // when no lock fix_age == -1
  gps_lock |= (fix_age > 0) && (fix_age < 5000);
  return gps_lock;
}

inline void light_ring(const unsigned int delay_length) {
  cycle_led(0,delay_length);   cycle_led(30,delay_length);  
  cycle_led(60,delay_length);  cycle_led(90,delay_length);  
  cycle_led(120,delay_length); cycle_led(150,delay_length); 
  cycle_led(180,delay_length); cycle_led(210,delay_length); 
  cycle_led(240,delay_length); cycle_led(270,delay_length); 
  cycle_led(300,delay_length); cycle_led(330,delay_length);
}

void set_home_point() { }

void cycle_led(const unsigned short led_number, 
               const unsigned int delay_length) {
  switch (led_number) {
    case   0: led_0(HIGH);   delay(delay_length); led_0(LOW);   break;
    case  30: led_30(HIGH);  delay(delay_length); led_30(LOW);  break;
    case  60: led_60(HIGH);  delay(delay_length); led_60(LOW);  break;
    case  90: led_90(HIGH);  delay(delay_length); led_90(LOW);  break;
    case 120: led_120(HIGH); delay(delay_length); led_120(LOW); break;
    case 150: led_150(HIGH); delay(delay_length); led_150(LOW); break;
    case 180: led_180(HIGH); delay(delay_length); led_180(LOW); break;
    case 210: led_210(HIGH); delay(delay_length); led_210(LOW); break;
    case 240: led_240(HIGH); delay(delay_length); led_240(LOW); break;
    case 270: led_270(HIGH); delay(delay_length); led_270(LOW); break;
    case 300: led_300(HIGH); delay(delay_length); led_300(LOW); break;
    case 330: led_330(HIGH); delay(delay_length); led_330(LOW); break;
    case 360: led_360(HIGH); delay(delay_length); led_360(LOW); break;
  }
  return;
}

/* Algorithm from http://mathforum.org/library/drmath/view/55417.html */
short heading_to_home() {
  short tc;
  // Update position for heading home.
  gps.f_get_position(&cur_lat,&cur_long,&fix_age);
  if (!gps_lock)
    return 0;
  // Great circle initial heading: then convert to degrees and constrain to
  // normal 360 degree range, then force to a short
  tc = ((short) (atan2( 
                       sin(home_long - cur_long) * cos(home_lat),
                       cos(cur_lat) * sin(home_long) - sin(cur_lat) *
                         cos(home_long) * cos(home_long - cur_long)
                      ) * (180 / PI)
                )
       );
  if (tc < 0) {
    tc += 360;
  }
  tc %= 360;
  // now subtract the current course and mod 360 to get the angle we should
  // head to get to the home point.
  if (tc - gps.f_course() < 0)
    tc = (short) ((tc + 360) - gps.f_course());
  else
    tc = (short) (tc - gps.f_course()) % 360;
#ifdef GPHAT_DEBUG
  Serial.print("Course to home = "); Serial.println(tc,DEC);
#endif
  return tc;
}

bool am_i_stopped() {
  /* Simple calculation based on the average speed of a stationary GPS 
   * module. If we don't have a GPS lock we consider ourselves stopped, cos
   * how else do we know what the speed is?
  */
//   Serial.print("Speed = ");
//   Serial.println(gps.f_speed_mph());
  return (gps_lock && gps.f_speed_mph() < STOP_SPEED_THRESHOLD);
}

bool read_gps_data() {
  char c;
  while (Serial.available() > 0) {
    c = Serial.read();
#ifdef GPHAT_DEBUG
    Serial.print(c);
#endif
    if (gps.encode(c))
      return true; /* Valid GPS data */
  }
  return false; /* No GPS data yet */  
}

/* 
 * LED access functions. Write state to the digital pin assigned. This 
 * serves as a pin mapping too.
*/
//inline void led_0 (const int state)  { analogWrite(0, state == HIGH ? 255 : 0); }
inline void led_0  (const int state) { digitalWrite(LED_0,   state); return; }
inline void led_30 (const int state) { digitalWrite(LED_30,  state); return; }
inline void led_60 (const int state) { digitalWrite(LED_60,  state); return; }
inline void led_90 (const int state) { digitalWrite(LED_90,  state); return; }
inline void led_120(const int state) { digitalWrite(LED_120, state); return; }
inline void led_150(const int state) { digitalWrite(LED_150, state); return; }
inline void led_180(const int state) { digitalWrite(LED_180, state); return; }
inline void led_210(const int state) { digitalWrite(LED_210, state); return; }
inline void led_240(const int state) { digitalWrite(LED_240, state); return; }
inline void led_270(const int state) { digitalWrite(LED_270, state); return; }
inline void led_300(const int state) { digitalWrite(LED_300, state); return; }
inline void led_330(const int state) { digitalWrite(LED_330, state); return; }
/* The 360 degree pin is the same as led_0. This is convenience */
inline void led_360(const int state) { led_0(state); return; }