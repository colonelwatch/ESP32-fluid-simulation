#include <SPI.h>

#include <TFT_eSPI.h> // WARNING: before uploading, acquire custom User_Setup.h (see monorepo)
#include <XPT2046_Touchscreen.h>

#include "iram_float.h"
#include "Vector.h"
#include "Field.h"
#include "operations.h"

// configurables
#define N_ROWS 60 // size of sim domain
#define N_COLS 80 // size of sim domain
#define SCALING 4 // integer scaling of domain -> screen size is inferred from this
#define TILE_HEIGHT 60 // multiple of SCALING and a factor of (N_ROWS*SCALING)
#define TILE_WIDTH 80  // multiple of SCALING and a factor of (N_COLS*SCALING)
#define DT 1/12.0 // s, size of time step in sim time (should roughly match real FPS)
#define POLLING_PERIOD 20 // ms, for the touch screen
// #define DIVERGENCE_TRACKING // if commented out, disables divergence tracking for some extra FPS


// touch resources
struct touch{
  Vector<uint16_t> coords;
  Vector<float> velocity;
};
QueueHandle_t touch_queue = xQueueCreate(10, sizeof(struct touch));
const int XPT2046_IRQ = 36;
const int XPT2046_MOSI = 32;
const int XPT2046_MISO = 39;
const int XPT2046_CLK = 25;
const int XPT2046_CS = 33;
SPIClass ts_spi = SPIClass(HSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);

// essential sim resources
// TODO: allocation here causes a crash, AND runtime allocation of the 
//  velocity field AFTER the color fields causes a crash?
Field<Vector<float>> *velocity_field;
Field<iram_float_t> *red_field, *green_field, *blue_field;

// draw resources
SemaphoreHandle_t color_consumed = xSemaphoreCreateBinary(), // read preceded by a write, and vice versa
    color_produced = xSemaphoreCreateBinary();
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite tiles[2] = {TFT_eSprite(&tft), TFT_eSprite(&tft)}; // we'll use the two tiles for double-buffering
uint16_t *tile_buffers[2] = {
    (uint16_t*)tiles[0].createSprite(TILE_WIDTH, TILE_HEIGHT),
    (uint16_t*)tiles[1].createSprite(TILE_WIDTH, TILE_HEIGHT)};
const int SCREEN_HEIGHT = N_ROWS*SCALING, SCREEN_WIDTH = N_COLS*SCALING;
const int N_TILES = SCREEN_HEIGHT/TILE_HEIGHT, M_TILES = SCREEN_WIDTH/TILE_WIDTH;

// stats resources
SemaphoreHandle_t stats_consumed = xSemaphoreCreateBinary(), 
    stats_produced = xSemaphoreCreateBinary();
struct stats{
  unsigned long point_timestamps[6];
  float current_abs_pct_density; // "current" -> worst over domain at current time
  float max_abs_pct_density; // "max" -> worst over domain and all time
  int refresh_count;
};
struct stats global_stats;


void touch_routine(void *args){
  ts.setRotation(1); // landscape rotation
  ts_spi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(ts_spi);

  Vector<uint16_t> last_coords, current_coords; // used for calculating the velocity
  bool last_touched = false, touched; // used for detecting when to send a touch struct
  
  while(1){
    // invoke the tirqTouched and touched routines to check if the screen is touched
    touched = ts.tirqTouched() && ts.touched();

    // get the touch coordinates if we're supposed to
    if(touched){
      last_coords = current_coords; // current_coords is now history
      
      // getPoint() follows the coordinate system defined by Adafruit, but my 
      //  coordinates and velocities use the "ij" indexing that is typical for 
      //  row-major C arrays
      // Fortunately, the Adafruit coordinate system is just a rename of the 
      //  "ij" indexing, where x is j and y is i
      // furthermore, we'll map from a 4096x4096 domain to a N_ROWSxN_COLS one
      TS_Point raw_coords = ts.getPoint();
      current_coords = (Vector<uint16_t>){
          .x = (uint16_t)(raw_coords.y * N_ROWS / 4096),  // TODO: don't use a uint16_t Vector because it's confusing.
          .y = (uint16_t)(raw_coords.x * N_COLS / 4096)}; //        this reads like a transpose when its not.
    }
    // else current_coords should never end up being used

    // we're supposed to send a touch struct only if we have a previous touch 
    //  to calculate velocity with, or else the velocity is undefined
    bool send_touch = touched && last_touched;
    last_touched = touched; // update memory

    // send the touch struct if we're supposed to
    if(send_touch){
      // calculate and send the velocity and location
      Vector<float> current_velocity = {
          .x = ((float)current_coords.x - (float)last_coords.x) * 1000 / POLLING_PERIOD,  // i-direction
          .y = ((float)current_coords.y - (float)last_coords.y) * 1000 / POLLING_PERIOD}; // j-direction
      struct touch current_touch = { .coords = current_coords, .velocity = current_velocity };
      xQueueSend(touch_queue, &current_touch, 0); // TODO: don't just use send and pray
    }

    vTaskDelay(POLLING_PERIOD / portTICK_PERIOD_MS);
  }
}


void sim_routine(void* args){
  // local stats and timing the reporting of those stats
  unsigned long now, last_reported = millis();
  struct stats local_stats = (struct stats){ .max_abs_pct_density = 0, .refresh_count = 0 };
  
  while(1){
    local_stats.point_timestamps[0] = millis(); // holds the millis() for when calculating the time step started
    

    // Swap the velocity field with the advected one
    Field<Vector<float>> *to_delete_vector = velocity_field,
        *temp_vector_field = new Field<Vector<float>>(N_ROWS, N_COLS, NEGATIVE);
    semilagrangian_advect(temp_vector_field, velocity_field, velocity_field, DT);
    velocity_field = temp_vector_field;
    delete to_delete_vector;

    local_stats.point_timestamps[1] = millis();


    // apply the captured drag (encoded as a sequence of touch structs) to the velocity field
    struct touch current_touch;
    while(xQueueReceive(touch_queue, &current_touch, 0) == pdTRUE) // empty the queue
      velocity_field->index(current_touch.coords.x, current_touch.coords.y) = current_touch.velocity;
    velocity_field->update_boundary(); // in case the dragging went near the boundary, we need to update it


    // Get a divergence-free projection of the velocity field
    const float sor_omega = 1.90; // 1.0 reverts SOR to Gauss-Seidel, but 2/(1+sin(pi/60)) = 1.90 is optimal?
    Field<float> *divergence_field = new Field<float>(N_ROWS, N_COLS, DONTCARE),
        *pressure_field = new Field<float>(N_ROWS, N_COLS, CLONE);
    divergence(divergence_field, velocity_field);
    sor_pressure(pressure_field, divergence_field, 10, sor_omega);
    gradient_and_subtract(velocity_field, pressure_field);
    delete divergence_field;
    delete pressure_field;

    local_stats.point_timestamps[2] = millis();


    // Wait for the color field to be read/consumed already, and time this wait
    xSemaphoreTake(color_consumed, portMAX_DELAY);
    local_stats.point_timestamps[3] = millis();


    // Replace the color field with the advected one, but do so by rotating the memory used
    Field<iram_float_t> *temp, *temp_color_field = new Field<iram_float_t>(N_ROWS, N_COLS, CLONE);
    
    semilagrangian_advect(temp_color_field, red_field, velocity_field, DT);
    temp = red_field;
    red_field = temp_color_field;
    temp_color_field = temp;

    semilagrangian_advect(temp_color_field, green_field, velocity_field, DT);
    temp = green_field;
    green_field = temp_color_field;
    temp_color_field = temp;

    semilagrangian_advect(temp_color_field, blue_field, velocity_field, DT);
    temp = blue_field;
    blue_field = temp_color_field;
    temp_color_field = temp;

    delete temp_color_field; // drop the memory that got rotated out

    // Signal that the color field has been written/produced as is ready to be read/consumed
    xSemaphoreGive(color_produced);
    
    local_stats.point_timestamps[4] = millis();


    #ifdef DIVERGENCE_TRACKING
    // Assuming density is constant over the domain in the current time (which 
    //  is only a correct assumption if the divergence is equal to zero for all 
    //  time because the density is obviously constant over the domain at t=0), 
    //  I'd think that the Euler equations say that the change in density over 
    //  time is equal to the divergence of the velocity field times the density 
    //  because the advection term is therefore zero.
    // Furthermore, I'd argue that "expected density error in pct" is equal to 
    //  the divergence times the time step. This is a thing we can track.
    // TODO: research this and find a source?
    float current_abs_divergence = 0; // "current" -> worst over domain at current time
    Field<float> *new_divergence_field = new Field<float>(N_ROWS, N_COLS, DONTCARE); // "new" divergence after projection
    divergence(new_divergence_field, velocity_field);
    for(int i = 0; i < N_ROWS; i++)
      for(int j = 0; j < N_COLS; j++)
        if(abs(new_divergence_field->index(i, j)) > current_abs_divergence)
          current_abs_divergence = abs(new_divergence_field->index(i, j));
    local_stats.current_abs_pct_density = 100*current_abs_divergence*DT;
    if(local_stats.current_abs_pct_density > local_stats.max_abs_pct_density)
      local_stats.max_abs_pct_density = local_stats.current_abs_pct_density;
    delete new_divergence_field;
    #endif

    local_stats.point_timestamps[5] = millis();


    // Update the global stats
    now = millis();
    local_stats.refresh_count++;
    if(now - last_reported > 5000){
      xSemaphoreTake(stats_consumed, portMAX_DELAY);
      global_stats = local_stats;
      xSemaphoreGive(stats_produced);

      // don't reset max_abs_pct_density because it's a running max
      last_reported = now;
      local_stats.refresh_count = 0;
    }

    vTaskDelay(1); // give a tick to lower-priority tasks (including the IDLE task?)
  }
}


void draw_routine(void* args){
  // TFT_eSPI uses the (again) Adafruit coordinate system, but our colors 
  //  fields are in "ij". x is j and y is i. width is M and height is N.
  tft.setRotation(1); // landscape rotation
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.initDMA();

  while(1){
    xSemaphoreTake(color_produced, portMAX_DELAY);
    int buffer_select = 0;

    tft.startWrite(); // start a single transfer for all the tiles

    for(int ii = 0; ii < N_TILES; ii++){
      for(int jj = 0; jj < M_TILES; jj++){
        int i_start = ii*TILE_HEIGHT, j_start = jj*TILE_WIDTH,
            i_end = (ii+1)*TILE_HEIGHT, j_end = (jj+1)*TILE_WIDTH;
        int i_cell_start = i_start/SCALING, j_cell_start = j_start/SCALING,
            i_cell_end = i_end/SCALING, j_cell_end = j_end/SCALING;

        for(int i_cell = i_cell_start; i_cell < i_cell_end; i_cell++){
          for(int j_cell = j_cell_start; j_cell < j_cell_end; j_cell++){
            int r = red_field->index(i_cell, j_cell)*255,
                g = green_field->index(i_cell, j_cell)*255,
                b = blue_field->index(i_cell, j_cell)*255;
            
            // don't go out of bounds
            if(r < 0) r = 0; else if(r > 255) r = 255;
            if(g < 0) g = 0; else if(g > 255) g = 255;
            if(b < 0) b = 0; else if(b > 255) b = 255;
            
            // keeping in mind that i is y and j is x, drawing the rectangle onto a tile
            int i_local = i_cell*SCALING-i_start, j_local = j_cell*SCALING-j_start;
            tiles[buffer_select].fillRect(j_local, i_local, SCALING, SCALING, tft.color565(r, g, b));
          }
        }

        // keeping in mind that i is y and j is x, drawing the tile onto the screen
        tft.pushImageDMA(j_start, i_start, TILE_WIDTH, TILE_HEIGHT, tile_buffers[buffer_select]);
        buffer_select = buffer_select? 0 : 1;
      }
    }

    tft.endWrite();

    xSemaphoreGive(color_consumed);
  }
}


void stats_routine(void* args){
  struct stats local_stats;
  unsigned long now, last_reported = millis(), elapsed;
  while(1){
    xSemaphoreTake(stats_produced, portMAX_DELAY);
    local_stats = global_stats;
    global_stats.refresh_count = 0;
    xSemaphoreGive(stats_consumed);

    now = millis();
    elapsed = now - last_reported;
    last_reported = now;

    float refresh_rate = 1000*(float)local_stats.refresh_count/elapsed;
    float time_taken[5], total_time, pct_taken[5];
    for(int i = 0; i < 5; i++)
      time_taken[i] = (local_stats.point_timestamps[i+1]-local_stats.point_timestamps[i])/1000.0;
    total_time = (local_stats.point_timestamps[5]-local_stats.point_timestamps[0])/1000.0;
    for(int i = 0; i < 5; i++)
      pct_taken[i] = 100*time_taken[i]/total_time;

    Serial.print("FPS: ");
    Serial.print(refresh_rate, 1);
    Serial.print(", ");
    Serial.print("Pct times: (");
    for(int i = 0; i < 5; i++){
      Serial.print(pct_taken[i], 1);
      Serial.print("%");
      if(i < 4) Serial.print(", ");
    }
    Serial.print(")");
    Serial.print(", ");

    #ifdef DIVERGENCE_TRACKING
    Serial.print("Err now: +/- ");
    Serial.print(local_stats.current_abs_pct_density, 1);
    Serial.print("%");
    Serial.print(", ");
    Serial.print("Err max: +/- ");
    Serial.print(local_stats.max_abs_pct_density, 1);
    Serial.print("%");
    Serial.print(", ");
    #endif

    Serial.print("Touch queue sz: ");
    Serial.print(uxQueueMessagesWaiting(touch_queue));
    Serial.println();
  }
}


void setup(void) {
  Serial.begin(115200);
  pinMode(0, INPUT_PULLUP);


  Serial.println("Initializing velocity field...");
  velocity_field = new Field<Vector<float>>(N_ROWS, N_COLS, NEGATIVE);
  for(int i = 0; i < N_ROWS; i++)
    for(int j = 0; j < N_COLS; j++)
      velocity_field->index(i, j) = {0, 0};
  velocity_field->update_boundary();
  
  
  // Init the raw fields using rules, then smooth them with the kernel for the final color fields
  Serial.println("Initializing color fields...");
  float kernel[3][3] = {{1/16.0, 1/8.0, 1/16.0}, {1/8.0, 1/4.0, 1/8.0}, {1/16.0, 1/8.0, 1/16.0}};
  red_field = new Field<iram_float_t>(N_ROWS, N_COLS, CLONE);
  green_field = new Field<iram_float_t>(N_ROWS, N_COLS, CLONE);
  blue_field = new Field<iram_float_t>(N_ROWS, N_COLS, CLONE);

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
    }
  }

  red_field->update_boundary();
  green_field->update_boundary();
  blue_field->update_boundary();

  for(int i = 0; i < N_ROWS; i++){
    for(int j = 0; j < N_COLS; j++){
      float smoothed_red = 0, smoothed_green = 0, smoothed_blue = 0;

      for(int di = 0; di < 3; di++){
        for(int dj = 0; dj < 3; dj++){
          int ii = i+di, jj = j+dj;

          // extend the edge of the field by repeating the last row/column
          if(ii > N_ROWS-1) ii = N_ROWS-1;
          if(jj > N_COLS-1) jj = N_COLS-1;
          
          smoothed_red += kernel[di][dj]*red_field->index(ii, jj);
          smoothed_green += kernel[di][dj]*green_field->index(ii, jj);
          smoothed_blue += kernel[di][dj]*blue_field->index(ii, jj);
        }
      }

      red_field->index(i, j) = smoothed_red;
      green_field->index(i, j) = smoothed_green;
      blue_field->index(i, j) = smoothed_blue;
    }
  }

  red_field->update_boundary();
  green_field->update_boundary();
  blue_field->update_boundary();


  Serial.println("Launching tasks...");
  xSemaphoreGive(color_consumed); // start with a write not a read
  xSemaphoreGive(stats_consumed);
  xTaskCreate(draw_routine, "draw", 2000, NULL, configMAX_PRIORITIES-1, NULL);
  xTaskCreate(touch_routine, "touch", 2000, NULL, configMAX_PRIORITIES-2, NULL);
  xTaskCreate(sim_routine, "sim", 2000, NULL, configMAX_PRIORITIES-3, NULL);
  xTaskCreate(stats_routine, "stats", 2000, NULL, configMAX_PRIORITIES-4, NULL);


  vTaskDelete(NULL); // delete the setup-and-loop task
}


void loop(void) {
  // Not actually used
}