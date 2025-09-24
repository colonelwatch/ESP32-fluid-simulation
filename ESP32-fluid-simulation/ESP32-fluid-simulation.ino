#include <cmath>

#include <SPI.h>
#include <TFT_eSPI.h> // WARNING: acquire custom User_Setup.h from monorepo
#include <XPT2046_Touchscreen.h>

#include "vector.h"
#include "advect.h"
#include "finitediff.h"
#include "poisson.h"
#include "uq32.h"


// configurable defines
#define SCALING 4           // sets size of sim domain, factor of 240 and 320
#define DT (1 / 30.0f)      // s, size of sim time step (should match real FPS)
#define POLLING_PERIOD 10   // ms, for the touch screen
#define TOUCH_MIN_X 200     // input min x, determined by calibration
#define TOUCH_MAX_X 3700    // input max x, determined by calibration
#define TOUCH_MIN_Y 240     // input min y, determined by calibration
#define TOUCH_MAX_Y 3800    // input max y, determined by calibration

// draw defines
#define SCREEN_ROTATION 1
#define SCREEN_HEIGHT TFT_WIDTH
#define SCREEN_WIDTH TFT_HEIGHT
#define INV_SCALING (1.0f / SCALING)

// touch defines
#define XPT2046_IRQ  36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK  25
#define XPT2046_CS   33

// sim defines (NOTE: one row and column are added as lerp endpoints)
#define N_ROWS (SCREEN_HEIGHT / SCALING + 1)
#define N_COLS (SCREEN_WIDTH / SCALING + 1)

// macros
#define SWAP(x, y) do { auto temp = (x); (x) = (y); (y) = temp; } while(0)


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
Vector3<UQ32> *color_field = new Vector3<UQ32>[N_COLS * N_ROWS];

// draw resources
SemaphoreHandle_t color_consumed = xSemaphoreCreateBinary();
SemaphoreHandle_t color_produced = xSemaphoreCreateBinary();
TFT_eSPI tft = TFT_eSPI();


void touch_routine(void *args)
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
      TS_Point raw = ts.getPoint();
      Vector2<int> coords(map(raw.x, TOUCH_MIN_X, TOUCH_MAX_X, 0, N_COLS),
                          map(raw.y, TOUCH_MIN_Y, TOUCH_MAX_Y, 0, N_ROWS));

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


void draw_routine(void* args)
{
  tft.setRotation(SCREEN_ROTATION); // landscape rotation
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.initDMA();

  // pointers to tiles to be used for double-buffering
  uint16_t *write_tile = new uint16_t[SCALING * SCREEN_WIDTH];
  uint16_t *read_tile = new uint16_t[SCALING * SCREEN_WIDTH];

  while(1){
    xSemaphoreTake(color_produced, portMAX_DELAY);

    tft.startWrite(); // start a single transfer for all the tiles

    // NOTE: the last row and column are just endpoints, with no screen-area
    for (int i = 0; i < N_ROWS - 1; i++) {
      Vector3<float> interp[SCALING][SCALING + 1];
      for (int j = 0; j < N_COLS - 1; j++) {
        Vector3<float> c, dc;

        /* Pull four points for bilinear interpolation, accounting for the sim
        domain being rotated and filling in points if on the boundary. */
        int ij1 = index(i, j, N_ROWS);
        int ij2 = index(i, j + 1, N_ROWS);
        int ij3 = index(i + 1, j, N_ROWS);
        int ij4 = index(i + 1, j + 1, N_ROWS);

        /* Bilinear interpolation is "separable" (i.e. can be implemented as
        a sequence of 1D operations, usually for lower big-O overall), and it
        can be done recursively, using the previous point to get the next.*/
        if (j == 0) {
          // Lerp between top-left and bottom-left with "strength reduction"
          c = color_field[ij1];
          dc = ((Vector3<float>)color_field[ij3] - c) * INV_SCALING;
          for (int ii = 0; ii < SCALING; ii++) {
            interp[ii][0] = c;
            c += dc;
          }
        } else {
          // Just copy the old "top-right" and "bottom-right"
          for (int ii = 0; ii < SCALING; ii++) {
            interp[ii][0] = interp[ii][SCALING];
          }
        }

        // lerp between top-right and bottom-right (in order to caluclate dc)
        c = color_field[ij2];
        dc = ((Vector3<float>)color_field[ij4] - c) * INV_SCALING;
        for (int ii = 0; ii < SCALING; ii++) {
          interp[ii][SCALING] = c;
          c += dc;
        }

        // lerp between the left-side lerp and the right-side lerp
        for (int ii = 0; ii < SCALING; ii++) {
          c = interp[ii][0];
          dc = (interp[ii][SCALING] - c) * INV_SCALING;
          for (int jj = 0; jj < SCALING; jj++) {
            interp[ii][jj] = c;
            c += dc;
          }
        }

        // render to tile
        int offset = SCALING * j;
        for (int ii = 0; ii < SCALING; ii++) {
          for (int jj = 0; jj < SCALING; jj++) {
            Vector3<UQ32> color = interp[ii][jj];
            uint16_t color_565;
            color_565 = (((color.x.raw & 0xF8000000) >> 16) |
                         ((color.y.raw & 0xFC000000) >> 21) |
                         ((color.z.raw & 0xF8000000) >> 27));
            color_565 = __builtin_bswap16(color_565);
            write_tile[offset + SCREEN_WIDTH * ii + jj] = color_565;
          }
        }
      }

      // yield to higher-priority tasks until ready, then start transfer
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


void setup(void)
{
  // initialize velocity field
  for (int i = 0; i < N_ROWS; i++) {
    for (int j = 0; j < N_COLS; j++) {
      velocity_field[index(i, j, N_ROWS)] = Vector2<float>(0, 0);
    }
  }

  // initialize color field
  const int center_i = N_ROWS / 2, center_j = N_COLS / 2;
  const Vector3<float> red(UINT32_MAX, 0UL, 0UL), green(0UL, UINT32_MAX, 0UL),
                       blue(0UL, 0UL, UINT32_MAX);
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
      Vector3<UQ32> left, center, right;
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
      Vector3<UQ32> bot, center, top;
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
}


void loop(void)
{
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

  // Advect color field on a temp field
  Vector3<UQ32> *c_temp = new Vector3<UQ32>[N_ROWS * N_COLS];
  advect(c_temp, color_field, velocity_field, N_ROWS, N_COLS, DT, false);

  // Swap the color field with the temp field, then delete
  xSemaphoreTake(color_consumed, portMAX_DELAY);
  SWAP(c_temp, color_field);
  delete[] c_temp;
  xSemaphoreGive(color_produced);
}
