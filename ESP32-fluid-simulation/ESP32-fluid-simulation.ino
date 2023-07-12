// Note: This NEEDS the attached platform.local.txt file. It forces O2 over Os, and it 
//  seems to improve performance significantly. To use it, copy it into:
//  C:\Users\%USERPROFILE%\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4

#include <TFT_eSPI.h>

#include "iram_float.h"
#include "Vector.h"
#include "Field.h"
#include "operations.h"

#define N_ROWS 64
#define N_COLS 64
#define DT 0.1

TFT_eSPI tft = TFT_eSPI();

Field<Vector<float>> *velocity_field;
Field<iram_float_t> *red_field, *green_field, *blue_field;

unsigned long t_start, t_end;
int refreshes = 0;
bool benchmarked = false;


void draw_color_field(
  const Field<iram_float_t> *red_field, 
  const Field<iram_float_t> *green_field, 
  const Field<iram_float_t> *blue_field)
{
  tft.startWrite();
  tft.setCursor(0, 0);
  for(int i = 0; i < N_ROWS; i++){
    for(int j = 0; j < N_COLS; j++){
      int r = red_field->index(i, j)*255,
          g = green_field->index(i, j)*255,
          b = blue_field->index(i, j)*255;
      
      tft.drawPixel(j, i, tft.color565(r, g, b));
    }
  }
  tft.endWrite();
}


void setup(void) {
  // set up I/O things like Serial, TFT, and the input
  Serial.begin(115200);
  Serial.println("Setting up TFT screen and input...");
  tft.init();
  tft.fillScreen(TFT_BLACK);
  pinMode(0, INPUT_PULLUP);
  
  
  Serial.println("Allocating fields...");
  velocity_field = new Field<Vector<float>>(N_ROWS, N_COLS, NEGATIVE);
  red_field = new Field<iram_float_t>(N_ROWS, N_COLS, CLONE);
  green_field = new Field<iram_float_t>(N_ROWS, N_COLS, CLONE);
  blue_field = new Field<iram_float_t>(N_ROWS, N_COLS, CLONE);
  

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

  red_field->update_boundary();
  green_field->update_boundary();
  blue_field->update_boundary();
  velocity_field->update_boundary();


  Serial.println("Initaliziation complete!");
  t_start = millis();
}


void loop(void) {
  // Swap the velocity field with the advected one
  Field<Vector<float>> *to_delete_vector = velocity_field,
      *temp_vector_field = new Field<Vector<float>>(N_ROWS, N_COLS, NEGATIVE);
  advect(temp_vector_field, velocity_field, velocity_field, DT);
  velocity_field = temp_vector_field;
  delete to_delete_vector;


  // Apply a force in the center of the screen if the BOOT button is pressed
  // NOTE: This force pattern below causes divergences so large that gauss_seidel_pressure() 
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
  Field<float> *divergence_field = new Field<float>(N_ROWS, N_COLS, CLONE),
      *pressure_field = new Field<float>(N_ROWS, N_COLS, CLONE);
  divergence(divergence_field, velocity_field);
  gauss_seidel_pressure(pressure_field, divergence_field);
  gradient_and_subtract(velocity_field, pressure_field);
  delete divergence_field;
  delete pressure_field;


  // Replace the color field with the advected one, but do so by rotating the memory used
  Field<iram_float_t> *temp, *temp_color_field = new Field<iram_float_t>(N_ROWS, N_COLS, CLONE);
  
  advect(temp_color_field, red_field, velocity_field, DT);
  temp = red_field;
  red_field = temp_color_field;
  temp_color_field = temp;

  advect(temp_color_field, green_field, velocity_field, DT);
  temp = green_field;
  green_field = temp_color_field;
  temp_color_field = temp;

  advect(temp_color_field, blue_field, velocity_field, DT);
  temp = blue_field;
  blue_field = temp_color_field;
  temp_color_field = temp;

  delete temp_color_field; // drop the memory that go rotated out


  // Render the color fields and display it
  draw_color_field(red_field, green_field, blue_field);


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