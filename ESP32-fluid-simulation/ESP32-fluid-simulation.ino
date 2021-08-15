// Note: This NEEDS the attached platform.local.txt file. It forces O2 over Os, and it 
//  seems to improve performance significantly. To use it, copy it into:
//  C:\Users\%USERPROFILE%\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4

// Aggressive memory optimizations options. Will impact performance significantly.
#define AGGRESSIVE_DEALLOCATION // Temporarily deallocate vector field during memory-intensive jacobi iteration

#include <Adafruit_Protomatter.h>

#include "Vector.h"
#include "Field.h"
#include "operations.h"

#define N_ROWS 64
#define N_COLS 64
#define DT 0.1

typedef Vector<float> FloatVector;

uint8_t rgbPins[]  = {25, 27, 26, 14, 13, 12};
uint8_t addrPins[] = {23, 19, 5, 17, 32};
uint8_t clockPin   = 16; // Must be on same port as rgbPins
uint8_t latchPin   = 4;
uint8_t oePin      = 15;

Adafruit_Protomatter matrix(
  64,          // Width of matrix (or matrix chain) in pixels
  6,           // Bit depth, 1-6
  1, rgbPins,  // # of matrix chains, array of 6 RGB pins for each
  5, addrPins, // # of address pins (height is inferred), array of pins
  clockPin, latchPin, oePin, // Other matrix control pins
  false        // No double-buffering here (see "doublebuffer" example)
);

Field<float, CLONE> *red_field, *green_field, *blue_field, *temp_scalar_field;
Field<FloatVector, NEGATIVE> *velocity_field, *temp_vector_field;

unsigned long t_start, t_end;
int refreshes = 0;
bool benchmarked = false;

TaskHandle_t Task1;
void Task1code( void * pvParameters ){
  ProtomatterStatus status = matrix.begin();
  if(status != PROTOMATTER_OK)
    Serial.println("Protomatter error: " + String(status));
  
  while(1) delay(1000);
}

// https://learn.adafruit.com/led-tricks-gamma-correction/the-quick-fix
const uint8_t gamma8[] = {
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

void draw_color_field(const Field<float, CLONE> *red_field, const Field<float, CLONE> *green_field, const Field<float, CLONE> *blue_field){
  for(int i = 0; i < N_ROWS; i++){
    for(int j = 0; j < N_COLS; j++){
      int r = 255*red_field->index(i, j);
      r = gamma8[r];
      int g = 255*green_field->index(i, j);
      g = gamma8[g];
      int b = 255*blue_field->index(i, j);
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
  red_field = new Field<float, CLONE>(N_ROWS, N_COLS);
  green_field = new Field<float, CLONE>(N_ROWS, N_COLS);
  blue_field = new Field<float, CLONE>(N_ROWS, N_COLS);
  Serial.print("Color fields allocated! Remaining contiguous heap: ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  velocity_field = new Field<FloatVector, NEGATIVE>(N_ROWS, N_COLS);
  temp_vector_field = new Field<FloatVector, NEGATIVE>(N_ROWS, N_COLS);
  Serial.print("Vector fields allocated! Remaining contiguous heap: ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  temp_scalar_field = new Field<float, CLONE>(N_ROWS, N_COLS);
  Serial.print("Pressure fields allocated! Remaining contiguous heap: ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  
  Serial.println("Zeroing color and velocity fields...");
  const int center_i = N_ROWS/2, center_j = N_COLS/2;
  for(int i = 0; i < N_ROWS; i++){
    for(int j = 0; j < N_COLS; j++){
      red_field->index(i, j) = i <= center_i ? 0 : j > center_j ? 0 : 1;
      green_field->index(i, j) = j <= center_j ? 0 : i <= center_i ? 1 : 0;
      blue_field->index(i, j) = i <= center_i ? j <= center_j ? 1 : 0 : 0;

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
  const int center_i = N_ROWS/2, center_j = N_COLS/2;
  FloatVector dv = FloatVector({0, 10});
  if(digitalRead(0) == LOW){
    velocity_field->index(center_i, center_j) += dv;
    velocity_field->index(center_i+1, center_j) += dv;
    velocity_field->index(center_i, center_j+1) += dv;
    velocity_field->index(center_i+1, center_j+1) += dv;
  }

  // Zero out the divergence of the new velocity field
  #ifdef AGGRESSIVE_DEALLOCATION
  delete temp_vector_field;
  jacobi_pressure(temp_scalar_field, velocity_field);
  temp_vector_field = new Field<FloatVector, NEGATIVE>(N_ROWS, N_COLS);
  #else
  jacobi_pressure(temp_scalar_field, velocity_field);
  #endif
  gradient(temp_vector_field, temp_scalar_field);
  *velocity_field -= *temp_vector_field;

  // Advect the color field
  advect(temp_scalar_field, red_field, velocity_field, DT);
  *red_field = *temp_scalar_field;
  advect(temp_scalar_field, green_field, velocity_field, DT);
  *green_field = *temp_scalar_field;
  advect(temp_scalar_field, blue_field, velocity_field, DT);
  *blue_field = *temp_scalar_field;

  // Render the color field and display it
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