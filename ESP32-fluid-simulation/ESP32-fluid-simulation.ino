#include <SPI.h>

#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

#include "iram_float.h"
#include "Vector.h"
#include "Field.h"
#include "operations.h"

// assuming that screen aspect ratio matches domain aspect ratio
#define N_ROWS 80
#define N_COLS 60
#define SCALING 4 // integer scaling -> screen size is inferred from this
#define TILE_HEIGHT 80 // multiple of SCALING and a factor of (N_ROWS*SCALING)
#define TILE_WIDTH 60 // multiple of SCALING and a factor of (N_COLS*SCALING)
#define DT 0.1
#define POLLING_PERIOD 20 // ms, for the touch screen

TFT_eSPI tft = TFT_eSPI();
const int SCREEN_HEIGHT = N_ROWS*SCALING, SCREEN_WIDTH = N_COLS*SCALING;

// we'll use the two tiles for double-buffering
TFT_eSprite tiles[2] = {TFT_eSprite(&tft), TFT_eSprite(&tft)};
uint16_t *tile_buffers[2] = {
    (uint16_t*)tiles[0].createSprite(TILE_WIDTH, TILE_HEIGHT), 
    (uint16_t*)tiles[1].createSprite(TILE_WIDTH, TILE_HEIGHT)};
const int N_TILES = SCREEN_HEIGHT/TILE_HEIGHT, M_TILES = SCREEN_WIDTH/TILE_WIDTH;

SemaphoreHandle_t color_mutex, // mutex protects simultaneous read/write...
    color_semaphore; // ... but semaphore predicates read on an unread write
Field<Vector<float>> *velocity_field;
Field<iram_float_t> *red_field, *green_field, *blue_field;

const int XPT2046_MOSI = 32;
const int XPT2046_MISO = 39;
const int XPT2046_CLK = 25;
const int XPT2046_CS = 33;
SPIClass ts_spi = SPIClass(HSPI);
XPT2046_Touchscreen ts(XPT2046_CS); // TODO: use the IRQ pin?

QueueHandle_t touch_queue;

SemaphoreHandle_t stats_mutex, stats_semaphore;
struct stats{
  unsigned long point_timestamps[6];
  float current_abs_pct_density; // "current" -> worst over domain at current time
  float max_abs_pct_density; // "max" -> worst over domain and all time
  int refresh_count;
};
struct stats global_stats = (struct stats){ .max_abs_pct_density = 0, .refresh_count = 0 };


void draw_routine(void* args){
  // Serial.println("Setting up TFT screen...");
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.initDMA();
  while(1){
    xSemaphoreTake(color_semaphore, portMAX_DELAY);
    xSemaphoreTake(color_mutex, portMAX_DELAY);
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
            
            int i_local = i_cell*SCALING-i_start, j_local = j_cell*SCALING-j_start;
            tiles[buffer_select].fillRect(j_local, i_local, SCALING, SCALING, tft.color565(r, g, b));
          }
        }

        tft.pushImageDMA(j_start, i_start, TILE_WIDTH, TILE_HEIGHT, tile_buffers[buffer_select]);
        buffer_select = buffer_select? 0 : 1;
      }
    }

    tft.endWrite();

    xSemaphoreGive(color_mutex);
  }
}

void touch_routine(void *args){
  ts_spi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(ts_spi);

  while(1){
    if(ts.touched()){
      // the output falls in a 4096x4096 domain and follows the i-j scheme...
      // ... but we'll map to a N_ROWSxN_COLS domain and the x-y scheme
      TS_Point raw_coords = ts.getPoint();
      Vector<uint16_t> mapped_coords = {raw_coords.x * N_ROWS / 4096, 
          (4096-raw_coords.y) * N_COLS / 4096}; 
      xQueueSend(touch_queue, &mapped_coords, 0); // TODO: don't just use send and pray
    }

    vTaskDelay(POLLING_PERIOD / portTICK_PERIOD_MS);
  }
}


void sim_routine(void* args){
  bool dragging = false; int n_fails = 0; // together form the state
  Vector<uint16_t> last_coords, current_coords;
  
  unsigned long now, last_reported = millis();
  struct stats local_stats = (struct stats){ .max_abs_pct_density = 0, .refresh_count = 0 };
  
  while(1){
    local_stats.point_timestamps[0] = millis();
    
    // Swap the velocity field with the advected one
    Field<Vector<float>> *to_delete_vector = velocity_field,
        *temp_vector_field = new Field<Vector<float>>(N_ROWS, N_COLS, NEGATIVE);
    semilagrangian_advect(temp_vector_field, velocity_field, velocity_field, DT);
    velocity_field = temp_vector_field;
    delete to_delete_vector;

    local_stats.point_timestamps[1] = millis();


    bool touched = xQueuePeek(touch_queue, &current_coords, 0); // we define touched as "the queue is not empty"
    if(!dragging){ // n_fails is a don't-care here
      if(touched){
        dragging = true;
        last_coords = current_coords;
        n_fails = 0;
      }
      // else do nothing
    }
    else if(dragging && n_fails < 3){
      if(touched){
        while(xQueueReceive(touch_queue, &current_coords, 0) == pdTRUE){ // empty the queue
          Vector<float> drag_velocity = { // TODO: I had to do another i-j to x-y conversion here, is that right?
            .x = ((float)current_coords.y - (float)last_coords.y) * 1000 / POLLING_PERIOD,
            .y = -((float)current_coords.x - (float)last_coords.x) * 1000 / POLLING_PERIOD
          };
          velocity_field->index(current_coords.x, current_coords.y) = drag_velocity; // boundary condition?
          last_coords = current_coords;
        }
        n_fails = 0;
      }
      else{
        n_fails++;
        if(n_fails == 3)
          dragging = false;
      }
    }
    // dragging && n_fails >= 5 should never happen


    // Zero out the divergence of the new velocity field
    const float sor_omega = 1.90; // 1.0 reverts SOR to Gauss-Seidel, but 2/(1+sin(pi/60)) = 1.90 is optimal?
    Field<float> *divergence_field = new Field<float>(N_ROWS, N_COLS, DONTCARE),
        *pressure_field = new Field<float>(N_ROWS, N_COLS, CLONE);
    divergence(divergence_field, velocity_field);
    sor_pressure(pressure_field, divergence_field, 10, sor_omega);
    gradient_and_subtract(velocity_field, pressure_field);
    delete divergence_field;
    delete pressure_field;

    local_stats.point_timestamps[2] = millis();


    // Wait for the color field to be released by the draw routine, and time this wait
    xSemaphoreTake(color_mutex, portMAX_DELAY);
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

    delete temp_color_field; // drop the memory that go rotated out

    // Release the color field and allow the draw routine to use it
    xSemaphoreGive(color_mutex);
    xSemaphoreGive(color_semaphore);
    
    local_stats.point_timestamps[4] = millis();


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

    local_stats.point_timestamps[5] = millis();


    // Update the global stats
    now = millis();
    local_stats.refresh_count++;
    if(now - last_reported > 5000){
      xSemaphoreTake(stats_mutex, portMAX_DELAY);
      global_stats = local_stats;
      xSemaphoreGive(stats_mutex);
      xSemaphoreGive(stats_semaphore);

      // don't reset max_abs_pct_density because it's a running max
      last_reported = now;
      local_stats.refresh_count = 0;
    }

    vTaskDelay(1); // give a tick to lower-priority tasks (including the IDLE task?)
  }
}


void stats_routine(void* args){
  struct stats local_stats;
  unsigned long now, last_reported = millis(), elapsed;
  while(1){
    xSemaphoreTake(stats_semaphore, portMAX_DELAY);
    xSemaphoreTake(stats_mutex, portMAX_DELAY);
    local_stats = global_stats;
    global_stats.refresh_count = 0;
    xSemaphoreGive(stats_mutex);

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

    Serial.print("Refresh rate: ");
    Serial.print(refresh_rate, 1);
    Serial.print(", ");
    Serial.print("Percent time taken: (");
    for(int i = 0; i < 5; i++){
      Serial.print(pct_taken[i], 1);
      Serial.print("%");
      if(i < 4) Serial.print(", ");
    }
    Serial.print(")");
    Serial.print(", ");
    Serial.print("Current error: +/- ");
    Serial.print(local_stats.current_abs_pct_density, 1);
    Serial.print("%");
    Serial.print(", ");
    Serial.print("Max error: +/- ");
    Serial.print(local_stats.max_abs_pct_density, 1);
    Serial.print("%");
    Serial.print(", ");
    Serial.println();
  }
}


void setup(void) {
  Serial.begin(115200);
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


  Serial.println("Launching tasks...");
  touch_queue = xQueueCreate(10, sizeof(Vector<uint16_t>));
  color_semaphore = xSemaphoreCreateBinary();
  color_mutex = xSemaphoreCreateMutex();
  stats_semaphore = xSemaphoreCreateBinary();
  stats_mutex = xSemaphoreCreateMutex();
  xTaskCreate(draw_routine, "draw", 2000, NULL, configMAX_PRIORITIES-1, NULL);
  xTaskCreate(touch_routine, "touch", 2000, NULL, configMAX_PRIORITIES-2, NULL);
  xTaskCreate(sim_routine, "sim", 2000, NULL, configMAX_PRIORITIES-3, NULL);
  xTaskCreate(stats_routine, "stats", 2000, NULL, configMAX_PRIORITIES-4, NULL);

  vTaskDelete(NULL);
}


void loop(void) {
  // Not actually used
}