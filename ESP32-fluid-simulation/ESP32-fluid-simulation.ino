#include <cmath>

#include <SPI.h>

#include <TFT_eSPI.h> // WARNING: before uploading, acquire custom User_Setup.h (see monorepo)
#include <XPT2046_Touchscreen.h>

#include "vector.h"
#include "advect.h"
#include "finitediff.h"
#include "poisson.h"
#include "uq16.h"

// configurables
#define SCALING 4
#define DT 1/30.0 // s, size of time step in sim time (should roughly match real FPS)
#define POLLING_PERIOD 10 // ms, for the touch screen
// #define DIVERGENCE_TRACKING // if commented out, disables divergence tracking for some extra FPS

#define SCREEN_ROTATION 1
#define SCREEN_HEIGHT TFT_WIDTH
#define SCREEN_WIDTH TFT_HEIGHT

#define N_ROWS (SCREEN_HEIGHT / SCALING) // size of sim domain
#define N_COLS (SCREEN_WIDTH / SCALING) // size of sim domain

#define SWAP(x, y) do { auto temp = x; x = y; y = temp; } while(0)


// touch resources
struct drag {
  Vector2<uint16_t> coords;
  Vector2<float> velocity;
};
QueueHandle_t drag_queue = xQueueCreate(10, sizeof(struct drag));
const int XPT2046_IRQ = 36;
const int XPT2046_MOSI = 32;
const int XPT2046_MISO = 39;
const int XPT2046_CLK = 25;
const int XPT2046_CS = 33;
SPIClass ts_spi = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);
// essential sim resources
// TODO: allocation here causes a crash, AND runtime allocation of the
//  velocity field AFTER the color fields causes a crash?
Vector2<float> *velocity_field;
Vector3<UQ16> *color_field;

// draw resources
SemaphoreHandle_t color_consumed = xSemaphoreCreateBinary(), // read preceded by a write, and vice versa
    color_produced = xSemaphoreCreateBinary();
TFT_eSPI tft = TFT_eSPI();

// stats resources
SemaphoreHandle_t stats_consumed = xSemaphoreCreateBinary(),
    stats_produced = xSemaphoreCreateBinary();
struct stats{
  unsigned long point_timestamps[6];
  float current_error; // "current" -> worst over domain at current time
  float max_error; // "max" -> worst over domain and all time
  int refresh_count;
};
struct stats global_stats;


void touch_routine (void *args)
{
  ts.setRotation(1); // landscape rotation
  ts_spi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(ts_spi);

  Vector2<int> last_coords;
  bool last_touched = false;
  while (true) {
    bool touched = ts.touched();

    if (touched) {
      // We need to map from a 4096x4096 domain to a N_ROWSxN_COLS one
      TS_Point raw_coords = ts.getPoint();
      Vector2<int> coords(raw_coords.x * N_COLS / 4096,
                          raw_coords.y * N_ROWS / 4096);

      // only send a drag struct if a velocity can be calculated
      if (last_touched) {
        Vector2<int> delta = coords - last_coords;
        Vector2<float> velocity = delta * 1000.f / POLLING_PERIOD;
        struct drag msg = { .coords = coords, .velocity = velocity };
        xQueueSend(drag_queue, &msg, 0);
      }

      last_coords = coords;
    }
    // else last_coords is now stale

    last_touched = touched;

    vTaskDelay(POLLING_PERIOD / portTICK_PERIOD_MS);
  }
}


void sim_routine(void* args){
  // local stats and timing the reporting of those stats
  unsigned long now, last_reported = millis();
  struct stats local_stats = (struct stats){ .max_error = 0, .refresh_count = 0 };

  while(1){
    local_stats.point_timestamps[0] = millis(); // holds the millis() for when calculating the time step started

    // Swap the velocity field with the advected one
    Vector2<float> *v_temp = new Vector2<float>[N_ROWS*N_COLS];
    advect(v_temp, velocity_field, velocity_field, N_ROWS, N_COLS,
           DT, true);
    SWAP(v_temp, velocity_field);
    delete[] v_temp;

    local_stats.point_timestamps[1] = millis();


    /* Apply the captured drag. They're defined in the graphics coordinate
    system, i.e matrix indexing, but the sim uses Cartesian indexing.
           Λ   y i.e. j  Cartesian     ─┼─> j i.e. "x"  Graphics
          ─┼─> x i.e. i                 V   i i.e. "y"
    The response: if the sim domain is rotated 90 deg relative to the actual
    domain, the transform is just to swap x and y. */
    struct drag msg;
    while(xQueueReceive(drag_queue, &msg, 0) == pdTRUE){
      int ij = index(msg.coords.y, msg.coords.x, N_ROWS);
      Vector2<float> swapped(msg.velocity.y, msg.velocity.x);
      velocity_field[ij] = swapped;
    }


    // Get divergence-free velocity (with 1.96 as a found omega for 60x80 grid)
    float *div_v = new float[N_ROWS * N_COLS];
    float *p = new float[N_ROWS * N_COLS];
    calculate_divergence(div_v, velocity_field, N_ROWS, N_COLS, 1);
    poisson_solve(p, div_v, N_ROWS, N_COLS, 1, 10, 1.96);
    subtract_gradient(velocity_field, p, N_ROWS, N_COLS, 1);
    delete[] div_v;
    delete[] p;

    local_stats.point_timestamps[2] = millis();


    Vector3<UQ16> *c_temp = new Vector3<UQ16>[N_ROWS*N_COLS];
    advect(c_temp, color_field, velocity_field, N_ROWS, N_COLS, DT, false);

    local_stats.point_timestamps[3] = millis();


    // Swap the color field with the advected one
    xSemaphoreTake(color_consumed, portMAX_DELAY);
    SWAP(c_temp, color_field);
    delete[] c_temp;
    xSemaphoreGive(color_produced);

    local_stats.point_timestamps[4] = millis();


    #ifdef DIVERGENCE_TRACKING
    // compute post-projection divergence
    float *new_div_v = new float[N_ROWS * N_COLS];
    calculate_divergence(new_div_v, velocity_field, N_ROWS, N_COLS, 1);

    /* The continuity equation is ∂ρ/∂t = ∇ · (ρv). If hypothetically ρ were
    constant everywhere, it becomes ∂ρ/∂t = ρ * (∇ · v). Therefore, (∇ · v)
    alone, times a finite Δt, gives a factor by which ρ increases. */
    int abs_argmax_ij = 0;
    float abs_max = 0, worst_div_v;
    for (int i = 0; i < N_ROWS; ++i) {
      for (int j = 0; j < N_COLS; ++j) {
        int ij = index(i, j, N_ROWS);
        float abs_ij = fabsf(new_div_v[ij]);
        if(abs_ij > abs_max) {
          abs_max = abs_ij;
          abs_argmax_ij = ij;
        }
      }
    }
    worst_div_v = new_div_v[abs_argmax_ij];

    // Update stats tracker
    local_stats.current_error = 100 * worst_div_v * DT;
    if(local_stats.current_error > local_stats.max_error) {
      local_stats.max_error = local_stats.current_error;
    }

    delete[] new_div_v;
    #endif

    local_stats.point_timestamps[5] = millis();


    // Update the global stats
    now = millis();
    local_stats.refresh_count++;
    if(now - last_reported > 5000){
      xSemaphoreTake(stats_consumed, portMAX_DELAY);
      global_stats = local_stats;
      xSemaphoreGive(stats_produced);

      // don't reset max_error because it's a running max
      last_reported = now;
      local_stats.refresh_count = 0;
    }
  }
}


void draw_routine(void* args){
  // As mentioned earlier, the simulation operates on a rotated view of the
  //  screen, so draw_routine needs to account for that
  tft.setRotation(SCREEN_ROTATION); // landscape rotation
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.initDMA();

  // pointers to tiles to be used for double-buffering
  uint16_t *write_tile = new uint16_t[SCALING * SCREEN_WIDTH];
  uint16_t *read_tile = new uint16_t[SCALING * SCREEN_WIDTH];

  const float inv_scaling = 1.0f / SCALING;
  Vector3<UQ16> c1, c2, c3, c4;
  Vector3<float> dc;
  Vector3<float> interp[SCALING][SCALING + 1];

  while(1){
    xSemaphoreTake(color_produced, portMAX_DELAY);

    tft.startWrite(); // start a single transfer for all the tiles

    for (int i = 0; i < N_ROWS; i++) {
      for (int j = 0; j < N_COLS; j++) {

        bool is_right_border = (j == N_COLS - 1);
        bool is_bottom_border = (i == N_ROWS - 1);
        c1 = color_field[index(i, j, N_ROWS)];
        c2 = is_right_border ? c1 : color_field[index(i, j + 1, N_ROWS)];
        c3 = is_bottom_border ? c1 : color_field[index(i + 1, j, N_ROWS)];
        if (!(is_right_border || is_bottom_border)) {
          c4 = color_field[index(i + 1, j + 1, N_ROWS)];
        }else if (is_right_border) {
          c4 = c3;
        } else {  // is_bottom_border
          c4 = c2;
        }

        if (j == 0) {
          dc = ((Vector3<float>)c3 - (Vector3<float>)c1) * inv_scaling;
          interp[0][0] = c1;
          for (int ii = 1; ii < SCALING; ii++) {
            interp[ii][0] = interp[ii - 1][0] + dc;
          }
        } else {
          for (int ii = 0; ii < SCALING; ii++) {
            interp[ii][0] = interp[ii][SCALING];
          }
        }

        dc = ((Vector3<float>)c4 - (Vector3<float>)c2) * inv_scaling;
        interp[0][SCALING] = c2;
        for (int ii = 1; ii < SCALING; ii++) {
          interp[ii][SCALING] = interp[ii - 1][SCALING] + dc;
        }

        for (int ii = 0; ii < SCALING; ii++) {
          dc = (interp[ii][SCALING] - interp[ii][0]) * inv_scaling;
          for (int jj = 1; jj < SCALING; jj++) {
            interp[ii][jj] = interp[ii][jj - 1] + dc;
          }
        }

        int offset = SCALING * j;
        for (int ii = 0; ii < SCALING; ii++) {
          for (int jj = 0; jj < SCALING; jj++) {
            Vector3<UQ16> color = interp[ii][jj];
            uint16_t color_565;
            color_565 = ((color.x.raw & 0xF800) |
                         ((color.y.raw & 0xFC00) >> 5) |
                         ((color.z.raw & 0xF800) >> 11));
            color_565 = __builtin_bswap16(color_565);
            write_tile[offset + SCREEN_WIDTH * ii + jj] = color_565;
          }
        }
      }

      while (tft.dmaBusy()) {
        vTaskDelay(0);
      }
      tft.pushImageDMA(0, i * SCALING, SCREEN_WIDTH, SCALING, write_tile);
      SWAP(write_tile, read_tile);
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
    Serial.print("Err now: ");
    Serial.print(local_stats.current_error, 1);
    Serial.print("%");
    Serial.print(", ");
    Serial.print("Err max: ");
    Serial.print(local_stats.max_error, 1);
    Serial.print("%");
    Serial.print(", ");
    #endif

    Serial.print("Drag queue sz: ");
    Serial.print(uxQueueMessagesWaiting(drag_queue));
    Serial.println();
  }
}


void setup(void) {
  Serial.begin(115200);
  pinMode(0, INPUT_PULLUP);


  Serial.println("Initializing velocity field...");
  velocity_field = new Vector2<float>[N_COLS*N_ROWS];
  for(int i = 0; i < N_ROWS; i++)
    for(int j = 0; j < N_COLS; j++)
      velocity_field[index(i, j, N_ROWS)] = Vector2<float>(0, 0);

  // Init the raw fields using rules, then smooth them with the kernel for the final color fields

  Serial.println("Initializing color fields...");
  float kernel[3][3] = {{1/16.0, 1/8.0, 1/16.0}, {1/8.0, 1/4.0, 1/8.0}, {1/16.0, 1/8.0, 1/16.0}};
  color_field = new Vector3<UQ16>[N_COLS*N_ROWS];

  const int center_i = N_ROWS/2, center_j = N_COLS/2;
  for(int i = 0; i < N_ROWS; i++){
    for(int j = 0; j < N_COLS; j++){
      // From matrix indexing of the actual domain...
      float x = i-center_i, y = j-center_j; // ...to Cartesian indexing of the rotated domain...
      float x_rotated = y, y_rotated = -x;  // ...to Cartesian indexing of the actual domain
      float angle = atan2(y_rotated, x_rotated);

      if (angle < -PI/3) {
        color_field[index(i, j, N_ROWS)] = Vector3<float>(65535, 0, 0);
      } else if (angle >= -PI/3 && angle < PI/3) {
        color_field[index(i, j, N_ROWS)] = Vector3<float>(0, 65535, 0);
      } else {
        color_field[index(i, j, N_ROWS)] = Vector3<float>(0, 0, 65535);
      }
    }
  }

  for(int i = 0; i < N_ROWS; i++){
    for(int j = 0; j < N_COLS; j++){
      Vector3<float> smoothed(0, 0, 0);

      for(int di = 0; di < 3; di++){
        for(int dj = 0; dj < 3; dj++){
          int ii = i+di, jj = j+dj;

          // extend the edge of the field by repeating the last row/column
          if(ii > N_ROWS-1) ii = N_ROWS-1;
          if(jj > N_COLS-1) jj = N_COLS-1;

          smoothed += kernel[di][dj]*color_field[index(ii, jj, N_ROWS)];
        }
      }

      color_field[index(i, j, N_ROWS)] = smoothed;
    }
  }


  Serial.println("Launching tasks...");
  xSemaphoreGive(color_consumed); // start with a write not a read
  xSemaphoreGive(stats_consumed);
  xTaskCreate(touch_routine, "touch", 2000, NULL, configMAX_PRIORITIES-1, NULL);
  xTaskCreate(stats_routine, "stats", 2000, NULL, configMAX_PRIORITIES-2, NULL);
  xTaskCreate(draw_routine, "draw", 2000, NULL, configMAX_PRIORITIES-3, NULL);
  xTaskCreate(sim_routine, "sim", 2000, NULL, configMAX_PRIORITIES-4, NULL);


  // vTaskDelete(NULL); // delete the setup-and-loop task
}


void loop(void) {
  // Not actually used
}
