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
#define TILE_HEIGHT 32
#define TILE_WIDTH 32
#define DT 0.1

// we'll use the two tiles for double-buffering
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite tiles[2] = {TFT_eSprite(&tft), TFT_eSprite(&tft)};
uint16_t *tile_buffers[2] = {
    (uint16_t*)tiles[0].createSprite(TILE_WIDTH, TILE_HEIGHT), 
    (uint16_t*)tiles[1].createSprite(TILE_WIDTH, TILE_HEIGHT)};
const int N_TILES = N_ROWS/TILE_HEIGHT + ((N_ROWS%TILE_HEIGHT != 0)? 1 : 0),
    M_TILES = N_COLS/TILE_WIDTH + ((N_COLS%TILE_WIDTH != 0)? 1 : 0);

Field<Vector<float>> *velocity_field;
Field<iram_float_t> *red_field, *green_field, *blue_field;

unsigned long last_reported;
unsigned long point_timestamps[5];
float max_abs_pct_density = 0;
int refresh_count = 0;


void draw_color_field(
  const Field<iram_float_t> *red_field, 
  const Field<iram_float_t> *green_field, 
  const Field<iram_float_t> *blue_field)
{
  int buffer_select = 0;
  tft.startWrite(); // start a single transfer for all the tiles

  for(int ii = 0; ii < N_TILES; ii++){
    for(int jj = 0; jj < M_TILES; jj++){
      int offset_i = ii*TILE_HEIGHT, offset_j = jj*TILE_WIDTH;

      for(int i = 0; i < TILE_HEIGHT; i++){
        for(int j = 0; j < TILE_WIDTH; j++){
          if(offset_i+i >= N_ROWS || offset_j+j >= N_COLS){
            tiles[buffer_select].drawPixel(j, i, TFT_BLACK);
            continue;
          }
          
          int r = red_field->index(offset_i+i, offset_j+j)*255,
              g = green_field->index(offset_i+i, offset_j+j)*255,
              b = blue_field->index(offset_i+i, offset_j+j)*255;
          tiles[buffer_select].drawPixel(j, i, tft.color565(r, g, b));
        }
      }

      tft.pushImageDMA(offset_j, offset_i, TILE_WIDTH, TILE_HEIGHT, tile_buffers[buffer_select]);
      buffer_select = buffer_select? 0 : 1;
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
  tft.initDMA();
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
  last_reported = millis();
}


void loop(void) {
  // Swap the velocity field with the advected one
  Field<Vector<float>> *to_delete_vector = velocity_field,
      *temp_vector_field = new Field<Vector<float>>(N_ROWS, N_COLS, NEGATIVE);
  advect(temp_vector_field, velocity_field, velocity_field, DT);
  velocity_field = temp_vector_field;
  delete to_delete_vector;

  if(refresh_count == 0) // if this is the first refresh since the last report, collect the timestamp
    point_timestamps[0] = millis();


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
  Field<float> *divergence_field = new Field<float>(N_ROWS, N_COLS, DONTCARE),
      *pressure_field = new Field<float>(N_ROWS, N_COLS, CLONE);
  divergence(divergence_field, velocity_field);
  gauss_seidel_pressure(pressure_field, divergence_field);
  gradient_and_subtract(velocity_field, pressure_field);
  delete divergence_field;
  delete pressure_field;

  if(refresh_count == 0) point_timestamps[1] = millis();


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

  if(refresh_count == 0) point_timestamps[2] = millis();


  // Render the color fields and display it
  draw_color_field(red_field, green_field, blue_field);
  if(refresh_count == 0) point_timestamps[3] = millis();

  // Assuming density is constant over the domain in the current time (which 
  //  is only a correct assumption if the divergence is equal to zero for all 
  //  time because the density is obviously constant over the domain at t=0), 
  //  I'd think that the Euler equations say that the change in density over 
  //  time is equal to the divergence of the velocity field times the density 
  //  because the advection term is therefore zero.
  // Furthermore, I'd argue that "expected density error in pct" is equal to 
  //  the divergence times the time step. This is a thing we can track.
  // TODO: research this and find a source?
  float current_abs_divergence = 0, current_abs_pct_density; // "current" -> worst over domain at current time
  Field<float> *new_divergence_field = new Field<float>(N_ROWS, N_COLS, DONTCARE); // "new" divergence after projection
  divergence(new_divergence_field, velocity_field);
  for(int i = 0; i < N_ROWS; i++)
    for(int j = 0; j < N_COLS; j++)
      if(abs(new_divergence_field->index(i, j)) > current_abs_divergence)
        current_abs_divergence = abs(new_divergence_field->index(i, j));
  current_abs_pct_density = 100*current_abs_divergence*DT;
  if(current_abs_pct_density > max_abs_pct_density) max_abs_pct_density = current_abs_pct_density;
  delete new_divergence_field;

  if(refresh_count == 0) point_timestamps[4] = millis();


  // Every 5 seconds, print out the calculated stats including the refresh rate
  refresh_count++;
  long now = millis();
  if(now-last_reported > 5000){
    float refresh_rate = 1000*(float)refresh_count/(now-last_reported);

    float time_taken[5], total_time, pct_taken[5];
    time_taken[0] = (point_timestamps[0]-last_reported)/1000.0;
    for(int i = 1; i < 5; i++)
      time_taken[i] = (point_timestamps[i]-point_timestamps[i-1])/1000.0;
    total_time = (point_timestamps[4]-last_reported)/1000.0;
    for(int i = 0; i < 5; i++)
      pct_taken[i] = 100*time_taken[i]/total_time;

    Serial.print("Refresh rate: ");
    Serial.print(refresh_rate);
    Serial.print(", ");
    Serial.print("Percent time taken: (");
    for(int i = 0; i < 5; i++){
      Serial.print(pct_taken[i]);
      Serial.print("%");
      if(i < 4) Serial.print(", ");
    }
    Serial.print(")");
    Serial.print(", ");
    Serial.print("Current error: +/- ");
    Serial.print(current_abs_pct_density);
    Serial.print("%");
    Serial.print(", ");
    Serial.print("Max error: +/- ");
    Serial.print(max_abs_pct_density);
    Serial.print("%");
    Serial.print(", ");
    Serial.println();

    refresh_count = 0;
    last_reported = now;
  }
}