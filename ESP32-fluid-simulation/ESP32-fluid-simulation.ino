#include <cmath>

#include <SPI.h>
#include <TFT_eSPI.h> // WARNING: acquire custom User_Setup.h from monorepo
#include <XPT2046_Touchscreen.h>

#include "vector.h"
#include "advect.h"
#include "finitediff.h"
#include "poisson.h"
#include "uq16.h"


// configurable defines
#define SCALING 4  // determines size of sim domain, factor of 240 and 320
#define DT (1 / 30.0f)  // s, size of sim time step (should match real FPS)
#define POLLING_PERIOD 10  // ms, for the touch screen
// #define DIVERGENCE_TRACKING  // enables divergence tracking (costs FPS)

// tft defines
#define SCREEN_ROTATION 1
#define SCREEN_HEIGHT TFT_WIDTH
#define SCREEN_WIDTH TFT_HEIGHT

// touch defines
#define XPT2046_IRQ  36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK  25
#define XPT2046_CS   33

// sim defines
#define N_ROWS (SCREEN_HEIGHT / SCALING) // size of sim domain
#define N_COLS (SCREEN_WIDTH / SCALING) // size of sim domain

// macros
#define SWAP(x, y) do { auto temp = x; x = y; y = temp; } while(0)


// touch resources
struct drag {
  Vector2<uint16_t> coords;
  Vector2<float> velocity;
};
QueueHandle_t drag_queue = xQueueCreate(10, sizeof(struct drag));
SPIClass ts_spi = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);

// sim resources
Vector2<float> *velocity_field = new Vector2<float>[N_COLS * N_ROWS];
Vector3<UQ16> *color_field = new Vector3<UQ16>[N_COLS * N_ROWS];

// draw resources
SemaphoreHandle_t color_consumed = xSemaphoreCreateBinary();
SemaphoreHandle_t color_produced = xSemaphoreCreateBinary();
TFT_eSPI tft = TFT_eSPI();


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

  while(1){
    // Swap the velocity field with the advected one
    Vector2<float> *v_temp = new Vector2<float>[N_ROWS*N_COLS];
    advect(v_temp, velocity_field, velocity_field, N_ROWS, N_COLS,
           DT, true);
    SWAP(v_temp, velocity_field);
    delete[] v_temp;

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

    Vector3<UQ16> *c_temp = new Vector3<UQ16>[N_ROWS*N_COLS];
    advect(c_temp, color_field, velocity_field, N_ROWS, N_COLS, DT, false);

    // Swap the color field with the advected one
    xSemaphoreTake(color_consumed, portMAX_DELAY);
    SWAP(c_temp, color_field);
    delete[] c_temp;
    xSemaphoreGive(color_produced);

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


void setup(void) {
  for (int i = 0; i < N_ROWS; i++) {
    for (int j = 0; j < N_COLS; j++) {
      velocity_field[index(i, j, N_ROWS)] = Vector2<float>(0, 0);
    }
  }

  const int center_i = N_ROWS / 2, center_j = N_COLS / 2;
  const Vector3<float> red(UINT16_MAX, 0, 0), green(0, UINT16_MAX, 0),
                       blue(0, 0, UINT16_MAX);
  for (int i = 0; i < N_ROWS; i++) {
    for (int j = 0; j < N_COLS; j++) {
      // color based on angle, computing Cartesian x and y from indices i and j
      float angle = atan2f(-(i - center_i), j - center_j);
      if (angle < -PI / 3) {
        color_field[index(i, j, N_ROWS)] = red;
      } else if (angle >= -PI / 3 && angle < PI / 3) {
        color_field[index(i, j, N_ROWS)] = green;
      } else {
        color_field[index(i, j, N_ROWS)] = blue;
      }
    }
  }
  for (int i = 0; i < N_ROWS; i++) {
    for (int j = 0; j < N_COLS; j++) {
      // smooth in the horitzontal dimension with a triangular kernel
      Vector3<UQ16> left, center, right;
      center = color_field[index(i, j, N_ROWS)];
      left = (j == 0) ? center : color_field[index(i, j - 1, N_ROWS)];
      right = (j == N_COLS - 1) ? center : color_field[index(i, j + 1, N_ROWS)];
      color_field[index(i, j, N_ROWS)] = (0.25f * left + 0.5f * center +
                                          0.25f * right);
    }
  }
  for (int i = 0; i < N_ROWS; i++) {
    for (int j = 0; j < N_COLS; j++) {
      // smooth in the horitzontal dimension with the same kernel
      Vector3<UQ16> bot, center, top;
      center = color_field[index(i, j, N_ROWS)];
      top = (i == 0) ? center : color_field[index(i - 1, j, N_ROWS)];
      bot = (i == N_ROWS - 1) ? center : color_field[index(i + 1, j, N_ROWS)];
      color_field[index(i, j, N_ROWS)] = (0.25f * top + 0.5f * center +
                                          0.25f * bot);
    }
  }

  xSemaphoreGive(color_consumed); // start with a write not a read
  xTaskCreate(touch_routine, "touch", 2000, NULL, configMAX_PRIORITIES-1, NULL);
  xTaskCreate(draw_routine, "draw", 2000, NULL, configMAX_PRIORITIES-2, NULL);
  xTaskCreate(sim_routine, "sim", 2000, NULL, configMAX_PRIORITIES-3, NULL);
}


void loop(void) {}
