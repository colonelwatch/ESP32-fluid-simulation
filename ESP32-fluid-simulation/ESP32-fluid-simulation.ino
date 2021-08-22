// Note: This NEEDS the attached platform.local.txt file. It forces O2 over Os, and it 
//  seems to improve performance significantly. To use it, copy it into:
//  C:\Users\%USERPROFILE%\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4

#include <Adafruit_Protomatter.h>

#include "iram_float.h"
#include "Vector.h"
#include "Field.h"
#include "operations.h"

#define N_ROWS 64
#define N_COLS 64
#define DT 0.1

// Follows the pin layout specified by mrfaptastic/ESP32-HUB75-MatrixPanel-I2S-DMA
//  but it is probably reconfigurable
unsigned char rgbPins[]  = {25, 27, 26, 14, 13, 12};
unsigned char addrPins[] = {23, 19, 5, 17, 32};
unsigned char clockPin   = 16;
unsigned char latchPin   = 4;
unsigned char oePin      = 15;
Adafruit_Protomatter matrix(64, 6, 1, rgbPins, 5, addrPins, clockPin, latchPin, oePin, true);

Field<Vector<float>, NEGATIVE> *velocity_field, *temp_vector_field;
Field<float, CLONE> *temp_scalar_field;
Field<iram_float_t, CLONE> *red_field, *green_field, *blue_field, *temp_color_field;

unsigned long t_start, t_end;
int refreshes = 0;
bool benchmarked = false;

TaskHandle_t Task1;
void Task1code( void * pvParameters ){
  ProtomatterStatus status = matrix.begin(); // Now core 0 will handle the display for us
  if(status != PROTOMATTER_OK)
    Serial.println("Protomatter error: " + String(status));
  
  while(1) delay(1000);
}

// A gamma correction LUT from Adafruit
// https://learn.adafruit.com/led-tricks-gamma-correction/the-quick-fix
const unsigned char gamma8[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

void draw_color_field(
  const Field<iram_float_t, CLONE> *red_field, 
  const Field<iram_float_t, CLONE> *green_field, 
  const Field<iram_float_t, CLONE> *blue_field)
{
  for(int i = 0; i < N_ROWS; i++){
    for(int j = 0; j < N_COLS; j++){
      // Conversion out of iram_float_t must be explicitly called
      int r = red_field->index(i, j).as_float()*255,
          g = green_field->index(i, j).as_float()*255,
          b = blue_field->index(i, j).as_float()*255;
      
      r = gamma8[r];
      g = gamma8[g];
      b = gamma8[b];
      matrix.drawPixel(j, i, matrix.color565(r, g, b));
    }
  }
}

void setup(void) {
  Serial.begin(115200);
  
  Serial.println("Allocating protomatter core...");
  xTaskCreatePinnedToCore(Task1code, "Task1", 1000, NULL, 0, &Task1, 0);
  delay(1000);
  
  Serial.println("Allocating fields...");
  velocity_field = new Field<Vector<float>, NEGATIVE>(N_ROWS, N_COLS);
  temp_vector_field = new Field<Vector<float>, NEGATIVE>(N_ROWS, N_COLS);
  temp_scalar_field = new Field<float, CLONE>(N_ROWS, N_COLS);
  red_field = new Field<iram_float_t, CLONE>(N_ROWS, N_COLS);
  green_field = new Field<iram_float_t, CLONE>(N_ROWS, N_COLS);
  blue_field = new Field<iram_float_t, CLONE>(N_ROWS, N_COLS);
  temp_color_field = new Field<iram_float_t, CLONE>(N_ROWS, N_COLS);
  
  Serial.println("Setting color and velocity fields...");
  const int center_i = N_ROWS/2, center_j = N_COLS/2;
  for(int i = 0; i < N_ROWS; i++){
    for(int j = 0; j < N_COLS; j++){
      red_field->index(i, j) = 0;
      green_field->index(i, j) = 0;
      blue_field->index(i, j) = 0;

      float angle = atan2(i-center_i, j-center_j);
      if((angle >= -PI && angle < -PI/3)) red_field->index(i, j) = 1.0;
      else if(angle >= -PI/3 && angle < PI/3) green_field->index(i, j) = 1.0;
      else blue_field->index(i, j) = 1.0;

      velocity_field->index(i, j) = {0, 0};
    }
  }

  Serial.println();
  Serial.println("Initaliziation complete!");
  pinMode(0, INPUT_PULLUP);
  t_start = millis();
}

void loop(void) {
  // Advect the velocity field
  advect(temp_vector_field, velocity_field, velocity_field, DT);
  *velocity_field = *temp_vector_field;

  // Apply a force in the center of the screen if the BOOT button is pressed
  // NOTE: This force pattern below causes divergences so large that jacobi_pressure() 
  //  does not coverge quickly. For that reason, the fluid sim is only accurate when the 
  //  button is NOT pressed, so don't hold the button for long when playing with this sim.
  const int center_i = N_ROWS/2, center_j = N_COLS/2;
  Vector<float> dv = Vector<float>({0, 10});
  if(digitalRead(0) == LOW){
    velocity_field->index(center_i, center_j) += dv;
    velocity_field->index(center_i+1, center_j) += dv;
    velocity_field->index(center_i, center_j+1) += dv;
    velocity_field->index(center_i+1, center_j+1) += dv;
  }

  // Zero out the divergence of the new velocity field
  delete temp_vector_field; // temporarily deallocate temp_vector_field because jacobi_pressure() needs extra memory
  jacobi_pressure(temp_scalar_field, velocity_field);
  temp_vector_field = new Field<Vector<float>, NEGATIVE>(N_ROWS, N_COLS);
  gradient(temp_vector_field, temp_scalar_field);
  *velocity_field -= *temp_vector_field;

  // Advect the color field
  advect(temp_color_field, red_field, velocity_field, DT);
  *red_field = *temp_color_field;
  advect(temp_color_field, green_field, velocity_field, DT);
  *green_field = *temp_color_field;
  advect(temp_color_field, blue_field, velocity_field, DT);
  *blue_field = *temp_color_field;

  // Render the color fields and display it
  matrix.fillScreen(0);
  draw_color_field(red_field, green_field, blue_field);
  matrix.show();

  // After 5 seconds, print out the calculated refresh rate
  refreshes++;
  float t_stop = millis();
  if(t_stop-t_start > 5000 && !benchmarked){
    float refresh_rate = 1000*(float)refreshes/(t_stop-t_start);
    Serial.println();
    Serial.print("Refresh rate: ");
    Serial.println(refresh_rate);
    benchmarked = true;
  }
}