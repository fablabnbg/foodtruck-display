/* 
 *  blinky-OTA-jw -- exercise OTA a bit, and run some dots along the led array
 *  
 *  Frst upload using Tools -> Port: -> Serial ports -> /dev/ttyUSB0
 *  Subsequent uploads: wheon in the same network: networkPorts should show up underneath the serial port.
 */
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "credentials.h"
#ifndef WLAN_PASS
# error "WLAN_PASS is undefined"
#endif
#ifndef WLAN_SSID
# error "WLAN_SSID is undefined"
#endif
#ifndef OTA_PASSWORD_HASH
# error "OTA_PASSWORD_HASH is undefined"
#endif
#ifndef OTA_PORT
# define OTA_PORT 3232
#endif

#define HOSTNAME "jw-blinky"
#define WIFI_CONNECT_WAITS 4     // min. recommended: 6. then try a reconnect()
#define WIFI_CONNECT_ATTEMPTS 1  // min. recommended: 2. then continue without OTA

String hostname = String(HOSTNAME);

#define ACTIVE_ROWS 24		// 1, ... 24, 30 

#if ACTIVE_ROWS <= 24
// variables for blinking an LED with Millis
#define GPIO_LED 2                   // ESP32 Pin to which onboard LED is connected
int ledState = 0;                   // current bit position in ledPattern [0..15]
uint16_t ledPattern = 0x0005;       // a 16bit pattern 
#endif
const long tick_interval = 100;     // interval per bit at which to blink (milliseconds)
unsigned long previousMillis = 0;   // will store last time LED was updated

unsigned long wifi_check_ticks_delay = 200;  // 20 seconds, based on a 100msec tick_interval
unsigned long wifi_check_ticker = 0;    // counter

void initWiFi()
{
  const char *c_hostname = hostname.c_str();
  
  // give a long one second led pulse when starting initWiFi.
  digitalWrite(GPIO_LED, 1);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);   // https://github.com/espressif/arduino-esp32/issues/806#issuecomment-589621154
  WiFi.setHostname(c_hostname); // must be done before WiFi.begin()
  delay(150);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  delay(150);
  MDNS.begin(c_hostname);
  ArduinoOTA.setHostname(c_hostname); // does not work. the name is always "espressif"

  Serial.print("Connecting to ssid=");
  Serial.println(WLAN_SSID);

  delay(1000);

  // A flash every second, while waiting for the connection to connection, 
  // with one missing flash, when reconnecting
  digitalWrite(GPIO_LED, 0);
  for (int i=0; i < WIFI_CONNECT_ATTEMPTS; i++)
  {
    for (int j=0; j < WIFI_CONNECT_WAITS; j++)
    {
      if (WiFi.status() == WL_CONNECTED)
        break;
      Serial.print(".");
      digitalWrite(GPIO_LED, 1);
      delay(20);
      digitalWrite(GPIO_LED, 0);
      delay(1000);
    }
    if (WiFi.status() == WL_CONNECTED)
      break;
    Serial.print("|");
    WiFi.disconnect();
    WiFi.reconnect();   
    delay(1000);
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("Connection Failed! state=");
    Serial.println(WiFi.status());
    // for quick flashes say no...
    for (int i = 0; i < 4; i++)
    {
      digitalWrite(GPIO_LED, 1);
      delay(20);
      digitalWrite(GPIO_LED, 0);
      delay(50);
    }
    delay(150);
    return;
  }
  
  // Port defaults to 3232
  ArduinoOTA.setPort(OTA_PORT);
  delay(150);

  // No authentication by default
  // ArduinoOTA.setPassword(OTA_PASSWORD);

  // echo -n "$password" | md5sum
  ArduinoOTA.setPasswordHash(OTA_PASSWORD_HASH);
  delay(150);
  
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print( "Wifi.localIP: ");
  Serial.println(WiFi.localIP());

  Serial.print( "WiFi.macAddress: ");
  Serial.println(WiFi.macAddress());
  Serial.print( "WiFi.RSSI: ");
  Serial.println(WiFi.RSSI());
  
  Serial.print( "WiFi.softAPgetHostname: ");
  Serial.println(WiFi.softAPgetHostname());
  Serial.print( "WiFi.getHostname: ");
  Serial.println(WiFi.getHostname());    
}


#define FB_N_ROWS 30
#define FB_N_COLS 192
#define FB_N_GROUPS 4
#define FB_GROUPLEN (FB_N_COLS/FB_N_GROUPS*3)

// 30 rows, 16*9/3*4 = 192 columns
// split into 4 groups, with 3 bytes (rgb) per pixel
byte framebuffer[FB_N_ROWS][FB_N_GROUPS][FB_GROUPLEN];

// not using unsigned here, need to catch -1 when going backwards
int knightrider_pos[FB_N_ROWS] = {
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0
};


// 1 means forwards, -1 means backwads
int knightrider_dir[FB_N_ROWS] = {
  1,1,1,1,1,1,1,1,1,1,
  1,1,1,1,1,1,1,1,1,1,
  1,1,1,1,1,1,1,1,1,1
};

unsigned int knightrider_pos_ticker[FB_N_ROWS] = {
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0
};

const unsigned int knightrider_ticks_per_pos[FB_N_ROWS] = {
  5,2,3,4,6,8,12,16,24,32,
  48,64,96,128,192,256,384,512,768,1024,
  1536,2048,3072,4096,6144,8192,12288,16384,24576,32768
};

const unsigned int knightrider_ticks_per_color = 3000;
#define KNIGHTRIDER_NCOLORS 7
unsigned int knightrider_color_idx = 0;
unsigned int knightrider_color_ticker = 0;

const byte knightrider_colors[KNIGHTRIDER_NCOLORS][3] = {
// RED, GREEN, BLUE
  { 255, 0,   0   },
  { 0,   255, 0   },
  { 0,   0,   255 },
  { 255, 255, 0   },
  { 255, 0,   255 },
  { 0,   255, 255 },
  { 255, 255, 255 }
};

// see pin-mapping.txt
const int gpio_data_map[FB_N_ROWS] = {
  22, 22,
  21, 21,
  19, 19,
  18, 18,
  17, 17,	// TX2
  16, 16,	// RX2
  15, 15,
  14, 14,
  13, 13,
  12, 12,
   5,  5,
   4,  4,
   3,  3,	// RX0
   2,  2,
   1,  1,	// TX0
};

#define GPIO_D1		23
#define GPIO_DCLK1	25
#define GPIO_LAT1	26
#define GPIO_OE1	27
#define GPIO_B1		32
#define GPIO_A1		33

void send_one_row_bb(int row)
{
  // we keep the led's enabled (OE1 low), while we transfer data. 
  // only before we latch, we disable, then we select the correct group, and enable again.
  //
  // loop through all four groups
  // for each group send FB_GROUPLEN = 3*192/4 = 144 bits

  // Determine data and clock lines.
  // Numbering of the connectors starts with 1, but we start with 0 
  // DATA1 controls row 0 and 1.  (connected at J1 and J2)
  unsigned int gpio_data  = gpio_data_map[row];
  unsigned int gpio_clock = (row & 1) ? GPIO_D1 : GPIO_DCLK1;

  for (int g = 0; g < 4; g++)
  {
    digitalWrite(GPIO_LAT1, 0);
    digitalWrite(gpio_clock, 0);
    for (int i = 0; i < FB_GROUPLEN; i++)
    {
      digitalWrite(gpio_data, framebuffer[row][g][i] ? 1 : 0);
      digitalWrite(gpio_clock, 1);
      digitalWrite(gpio_clock, 1);
      digitalWrite(gpio_clock, 0);
    } 
  
    // output disable (aka everything dark)
    // Without that delay, there is a small glitch: a dim led on the right of the current dot.
    for (int i = 0; i < 5; i++)
      digitalWrite(GPIO_OE1, 1);		// 3 improves, 6 is perfect.
    // latch. So that the output flipflops store what we want to show next
    digitalWrite(GPIO_LAT1, 1);
    digitalWrite(GPIO_LAT1, 1);
    digitalWrite(GPIO_LAT1, 0);
    // select group 0..3, they seem to light up backwards. Are A1 and B1 also LOW active?
    digitalWrite(GPIO_A1, (g & 0x01) ? 0 : 1);
    digitalWrite(GPIO_B1, (g & 0x02) ? 0 : 1);

    // output enable LOW active. Make the LEDs shine ...
    // Without that delay, there is a small glitch: a dim led on the left of the current dot.
    for (int i = 0; i < 15; i++)
      digitalWrite(GPIO_OE1, 1);		// 10 improves, 15 is perfect
    digitalWrite(GPIO_OE1, 0);
  }
}


void send_rows_bb()
{
  // we keep the led's enabled (OE1 low), while we transfer data. 
  // only before we latch, we disable, then we select the correct group, and enable again.
  //
  // We divide into even and odd rows, as we can send all even rows and all odd rows together with the same clock.
  //
  // loop through all four groups
  // for each group send FB_GROUPLEN = 3*192/4 = 144 bits

  
  for (int odd = 0; odd < 2; odd++)
  {
    // Determine clock lines.
    unsigned int gpio_clock = odd ? GPIO_D1 : GPIO_DCLK1;

    for (int g = 0; g < 4; g++)
    {
      digitalWrite(GPIO_LAT1, 0);
      digitalWrite(gpio_clock, 0);
      for (int i = 0; i < FB_GROUPLEN; i++)
      {
        // Numbering of the row connectors starts with 1, but we start with 0 
        // DATA1 controls row 0 and 1.  (connected at J1 and J2)
        for (int row = odd; row < ACTIVE_ROWS; row += 2)
          digitalWrite(gpio_data_map[row], framebuffer[row][g][i] ? 1 : 0);
        digitalWrite(gpio_clock, 1);
        digitalWrite(gpio_clock, 1);
        digitalWrite(gpio_clock, 0);
      } 
    
      // output disable (aka everything dark)
      // Without that delay, there is a small glitch: a dim led on the right of the current dot.
      for (int i = 0; i < 5; i++)
        digitalWrite(GPIO_OE1, 1);		// 3 improves, 6 is perfect.

      // latch. So that the output flipflops store what we want to show next
      digitalWrite(GPIO_LAT1, 1);
      digitalWrite(GPIO_LAT1, 1);
      digitalWrite(GPIO_LAT1, 0);

      // select group 0..3, they seem to light up backwards. Are A1 and B1 also LOW active?
      digitalWrite(GPIO_A1, (g & 0x01) ? 0 : 1);
      digitalWrite(GPIO_B1, (g & 0x02) ? 0 : 1);

      // output enable LOW active. Make the LEDs shine ...
      // Without that delay, there is a small glitch: a dim led on the left of the current dot.
      for (int i = 0; i < 15; i++)
        digitalWrite(GPIO_OE1, 1);		// 10 improves, 15 is perfect
      digitalWrite(GPIO_OE1, 0);
    }
  }
}

void knightrider_tick()
{
  int i;	// pixel offset in a row
  int g;	// group for the pixel
  byte *p;	// pointer to the rgb value of that pixel.
  const byte *c;	// pointer to an rgb color definition;

  for (int row=0; row < ACTIVE_ROWS; row++)
  {
    // erase colored dot
    i = knightrider_pos[row];
    g = i & 0x03;	// two bits, for four groups
    i >>= 2;		// remove these two bits
    p = &framebuffer[row][g][3*i];  // pointer to rgb value
    *p++ = 0;
    *p++ = 0;
    *p = 0;

    // advance position
    int t = knightrider_pos_ticker[row] + 1;
    if (t >= knightrider_ticks_per_pos[row])
    {
      t = 0;
      if (knightrider_dir[row] > 0 && knightrider_pos[row] >= FB_N_COLS-1)
        knightrider_dir[row] = -1;	// switch to backwards movement
      else if (knightrider_dir[row] < 0 && knightrider_pos[row] < 1)
        knightrider_dir[row] = +1;	// switch to foreward movement
      knightrider_pos[row] += knightrider_dir[row];
    }
    knightrider_pos_ticker[row] = t;

    // paint new dot
    i = knightrider_pos[row];
    g = i & 0x03;	// two bits, for four groups
    i >>= 2;		// remove these two bits
    // p = &framebuffer[row][g][3*i];  // pointer to rgb value
    // cheat a bit: we want to see more than just one boring dot,
    p = &framebuffer[(row==8) ? 0 : row][g][3*i];  // pointer to rgb value
    c = &knightrider_colors[knightrider_color_idx][0];  // pointer to rgb color triplet
    *p++ = *c++;
    *p++ = *c++;
    *p = *c;
  }

  // advance color ticker
  knightrider_color_ticker += 1;
  if (knightrider_color_ticker >= knightrider_ticks_per_color)
  {
    knightrider_color_ticker = 0;
    knightrider_color_idx += 1;
    if (knightrider_color_idx >= KNIGHTRIDER_NCOLORS)
      knightrider_color_idx = 0;
  }
}

void setup()
{
#if ACTIVE_ROWS <= 24
  // in 30 ROWS mode, we use all GPIOs, sorry, we no longer control the LED. It will blink wildly
  pinMode(GPIO_LED, OUTPUT);
#endif

#if ACTIVE_ROWS <1 || ACTIVE_ROWS > FB_N_ROWS
# error "ACTIVE_ROWS cannot exceed FB_N_ROWS"
#endif
  for (int row=0; row < ACTIVE_ROWS; row++)
  {
    pinMode(gpio_data_map[row], OUTPUT);
  }

  pinMode(GPIO_D1,    OUTPUT);
  pinMode(GPIO_DCLK1, OUTPUT);
  pinMode(GPIO_LAT1,  OUTPUT);
  pinMode(GPIO_OE1,   OUTPUT);
  pinMode(GPIO_B1,    OUTPUT);
  pinMode(GPIO_A1,    OUTPUT);

  digitalWrite(GPIO_OE1, 1);	// an inverted output. Start with Output disabled.
  
  Serial.begin(115200);
  Serial.println("Booting");

  byte mac[6];
  WiFi.macAddress(mac);
  hostname += String("-") + String(mac[4], HEX) + String(mac[5], HEX);
  
  initWiFi();
}

void loop()
{
  ArduinoOTA.handle();

  // nonblocking blink
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= tick_interval)
  {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
#if 1	// it should not harm at all.... ACTIVE_ROWS <= 24
    ledState =(ledState +1 ) & 0x0f;
    digitalWrite(GPIO_LED, (ledPattern & (1<<ledState)) ? 1 : 0);
#endif

    wifi_check_ticker++;
  }
  if (wifi_check_ticker > wifi_check_ticks_delay)
  {
    // Reference:
    // https://microcontrollerslab.com/reconnect-esp32-to-wifi-after-lost-connection/
    // FIXME: better use events?
    wifi_check_ticker = 0;
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("WiFi lost, trying to reconnect ...");
      WiFi.disconnect();
      WiFi.reconnect();
    }
  }

  knightrider_tick();
  send_one_row_bb(0);
  send_one_row_bb(0);
}
