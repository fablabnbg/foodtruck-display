#include <stdio.h>
#define byte unsigned char

#define ACTIVE_ROWS 24		// 1, ... 24, 30 

const long tick_interval = 100;     // interval per bit at which to blink (milliseconds)
unsigned long previousMillis = 0;   // will store last time LED was updated

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
  1,2,3,4,6,8,12,16,24,32,
  48,64,96,128,192,256,384,512,768,1024,
  1536,2048,3072,4096,6144,8192,12288,16384,24576,32768
};

const unsigned int knightrider_ticks_per_color = 3000;
#define KNIGHTRIDER_NCOLORS 7
unsigned int knightrider_color_idx = 0;
unsigned int knightrider_color_ticker = 0;

const byte knightrider_colors[KNIGHTRIDER_NCOLORS][3] = {
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
        knightrider_pos[row] = +1;	// switch to foreward movement
      knightrider_pos[row] += knightrider_dir[row];
    }
    knightrider_pos_ticker[row] = t;

    // paint new dot
    i = knightrider_pos[row];
    g = i & 0x03;	// two bits, for four groups
    i >>= 2;		// remove these two bits
    p = &framebuffer[row][g][3*i];  // pointer to rgb value
    c = &knightrider_colors[knightrider_color_idx][0];  // pointer to rgb color triplet
    *p++ = *c++;
    *p++ = *c++;
    *p = *c;
  }

  // advance color ticker, do only once, not in the loop.
  knightrider_color_ticker += 1;
  if (knightrider_color_ticker >= knightrider_ticks_per_color)
  {
    knightrider_color_ticker = 0;
    knightrider_color_idx += 1;
    if (knightrider_color_idx >= KNIGHTRIDER_NCOLORS)
      knightrider_color_idx = 0;
  }
}

int main()
{
  for (;;)
  {
    for (int i=0; i < 190; i++)
      knightrider_tick();
    printf("190 ticks later\n");
  }
}
