// Note: This NEEDS the attached platform.local.txt file. It forces O2 over Os, and it 
//  seems to improve performance significantly. To use it, copy it into:
//  C:\Users\%USERPROFILE%\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4

// Aggressive memory optimizations options.
//  Each of these options saves about 4096 bytes of memory, which is either one additional
//  color bit OR one additional field. Will impact performance significantly.
#define INLINE_DIVERGENCE_IN_JACOBI // Calculate divergence repeatedly instead of beforehand (affects operations.h)
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
  true        // No double-buffering here (see "doublebuffer" example)
);

Field<float, CLONE> *color_field, *temp_scalar_field;
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

void draw_color_field(const Field<float, CLONE> *field){
  for(int i = 0; i < N_ROWS; i++){
    for(int j = 0; j < N_COLS; j++){
      int color_ij = 255*field->index(i, j); // mapped within [0, 255]
      matrix.drawPixel(j, i, matrix.color565(0, 0, color_ij));
    }
  }
}

void setup(void) {
  Serial.begin(115200);
  
  Serial.println("Allocating protomatter core...");
  xTaskCreatePinnedToCore(Task1code, "Task1", 1000, NULL, 0, &Task1, 0);
  delay(1000);
  
  Serial.println("Allocating fields...");
  color_field = new Field<float, CLONE>(N_ROWS, N_COLS);
  temp_scalar_field = new Field<float, CLONE>(N_ROWS, N_COLS);
  velocity_field = new Field<FloatVector, NEGATIVE>(N_ROWS, N_COLS);
  temp_vector_field = new Field<FloatVector, NEGATIVE>(N_ROWS, N_COLS);
  
  Serial.println("Filling color field...");
  const int center_i = N_ROWS/2, center_j = N_COLS/2;
  for(int i = 0; i < N_ROWS; i++){
    for(int j = 0; j < N_COLS; j++){
      float distance = sqrt(pow(i-center_i, 2) + pow(j-center_j, 2));
      if(distance < 8) color_field->index(i, j) = 1;
      else color_field->index(i, j) = 0;

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
  advect(temp_scalar_field, color_field, velocity_field, DT);
  *color_field = *temp_scalar_field;

  // Render the color field and display it
  matrix.fillScreen(0);
  draw_color_field(color_field);
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