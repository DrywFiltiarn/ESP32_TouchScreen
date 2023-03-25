#ifndef _TOUCHSCREEN_H_
#define _TOUCHSCREEN_H_
#include "TFT_eSPI.h"
#include "driver/rtc_io.h"
#include "stdint.h"

class TSPoint {
  public:
    TSPoint(void);
    TSPoint(int16_t x, int16_t y, int16_t z);

    bool operator==(TSPoint);
    bool operator!=(TSPoint);

    int16_t x, y, z;
};

class TouchScreen {
  public:
    TouchScreen(TFT_eSPI &tft, uint8_t xp, uint8_t yp, uint8_t xm, uint8_t ym);

    void setPin(uint32_t mask);
    void clearPin(uint32_t mask);
    void storePinState(void);
    void restorePinState(void);

    bool getTouch(int16_t *x, int16_t *y);
    bool getTouchRaw(int16_t *x, int16_t *y);

    int16_t readTouchX(void);
    int16_t readTouchY(void);
    int16_t readTouchZ(void);
    TSPoint getPoint(void);

    void calibrate(uint16_t *params);
    void setCalibration(uint16_t *params);
    void debug(void);

  private:
    TFT_eSPI &_tft;

    uint8_t _xm, _xp, _ym, _yp;
    uint8_t _xm_rtc, _yp_rtc;
    uint32_t _xm_mask, _xp_mask, _ym_mask, _yp_mask;
    uint32_t _oldMode;
    uint32_t _oldState;

    uint16_t _calib_x0 = 300, _calib_x1 = 3600, _calib_y0 = 300, _calib_y1 = 3600;
    uint8_t _calib_rotate = 1, _calib_x_inv = 2, _calib_y_inv = 0;
};

#endif