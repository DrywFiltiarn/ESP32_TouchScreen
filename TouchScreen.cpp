#include "TouchScreen.h"

#define ADC_SAMPLES         5     // Samplecount for smoothing purposes
#define ADC_SAMPLES_IN_AVG  3     // Samplecount used for averaging in the total sample count (ADC_SAMPLES_IN_AVG <= ADC_SAMPLES!)
#define ADC_WIDTH           12    // Bitwidth of the ADC
#define ADC_NOISE_TRESHOLD  20    // ADC noise treshold
#define ADC_Z_TRESHOLD      500   // Minimum pressure to trigger touch

static void arr_sort(int16_t arr[]) {
  uint8_t j;
  int16_t save;

  for (uint8_t i = 1; i < ADC_SAMPLES; i++) {
    save = arr[i];
    for (j = i; j >= 1 && save < arr[j - 1]; j--)
      arr[j] = arr[j - 1];
    arr[j] = save;
  }
}

static int16_t arr_filter(int16_t arr[]) {
  int16_t min_distance = INT16_MAX;
  int16_t avg = 0;

  for (uint8_t i = 0; i < ADC_SAMPLES - ADC_SAMPLES_IN_AVG; ++i) {
    int16_t tmp_avg = 0;
    for (uint8_t j = 0; j < i + ADC_SAMPLES_IN_AVG; ++j)
      tmp_avg += arr[j];
    tmp_avg /= ADC_SAMPLES_IN_AVG;

    int16_t distance = 0;
    for (uint8_t j = 0; j < i + ADC_SAMPLES_IN_AVG; ++j)
      distance += tmp_avg - arr[j];

    if (distance < min_distance) {
      min_distance = distance;
      avg = tmp_avg;
    }
  }

  return avg;
}

TSPoint::TSPoint(void) {
  x = y = z = 0;
}
TSPoint::TSPoint(int16_t _x, int16_t _y, int16_t _z) {
  x = _x, y = _y, z = _z;
}

bool TSPoint::operator==(TSPoint t) {
  return ((t.x == x) && (t.y == y) && (t.z == z));
}

bool TSPoint::operator!=(TSPoint t) {
  return ((t.x != x) || (t.y != y) || (t.z != z));

}

TouchScreen::TouchScreen(TFT_eSPI &tft, uint8_t xp, uint8_t yp, uint8_t xm, uint8_t ym) : _tft(tft) {
  _xm = xm, _xp = xp, _ym = ym, _yp = yp;

  // Resolve GPIO bitmasks
  _xm_mask = digitalPinToBitMask(_xm);
  _xp_mask = digitalPinToBitMask(_xp);
  _ym_mask = digitalPinToBitMask(_ym);
  _yp_mask = digitalPinToBitMask(_yp);
  
  // Resolve RTC GPIO if available for preferred pins
  if (rtc_gpio_is_valid_gpio((gpio_num_t)xm))
    _xm_rtc = rtc_io_number_get((gpio_num_t)xm);
  else
    _xm_rtc = -1;
  
  if (rtc_gpio_is_valid_gpio((gpio_num_t)yp))
    _yp_rtc = rtc_io_number_get((gpio_num_t)yp);
  else
    _yp_rtc;
}

void TouchScreen::setPin(uint32_t mask) {
  GPIO.out_w1ts = mask & ~3;
  GPIO.out1_w1tc.data = mask & 3;
}

void TouchScreen::clearPin(uint32_t mask) {
  GPIO.out_w1tc = mask & ~3;
  GPIO.out1_w1tc.data = mask & 3;
}

void TouchScreen::storePinState(void) {
  _oldMode = GPIO.enable & ~3;
  _oldMode |= GPIO.enable1.data & 3;
  _oldState = GPIO.out & ~3;
  _oldState |= GPIO.out1.data & 3;
}

void TouchScreen::restorePinState(void) {
  ((_oldMode & _xm) > 0) ? pinMode(_xm, OUTPUT) : pinMode(_xm, INPUT);
  ((_oldMode & _xp) > 0) ? pinMode(_xp, OUTPUT) : pinMode(_xp, INPUT);
  ((_oldMode & _ym) > 0) ? pinMode(_ym, OUTPUT) : pinMode(_ym, INPUT);
  ((_oldMode & _yp) > 0) ? pinMode(_yp, OUTPUT) : pinMode(_yp, INPUT);

  setPin(_oldState & (_xm_mask | _xp_mask | _ym_mask | _yp_mask));
  clearPin(~_oldState & (_xm_mask | _xp_mask | _ym_mask | _yp_mask));
}

bool TouchScreen::getTouch(int16_t *x, int16_t *y) {
  int16_t x_tmp, y_tmp, z_tmp, xx, yy;

  uint16_t z1 = 1;
  uint16_t z2 = 0;
  while (z1 > z2) {
    z2 = z1;
    z1 = readTouchZ();
    delay(1);
  }

  x_tmp = readTouchX();
  y_tmp = readTouchY();
  z_tmp = readTouchZ();

  if (z_tmp > ADC_Z_TRESHOLD) {
    if (!_calib_rotate) {
      xx = (x_tmp-_calib_x0)*_tft.width()/_calib_x1;
      yy = (y_tmp-_calib_y0)*_tft.height()/_calib_y1;
    } else {
      xx = (y_tmp-_calib_x0)*_tft.width()/_calib_x1;
      yy = (x_tmp-_calib_y0)*_tft.height()/_calib_y1;
    }
    if (_calib_x_inv) xx = _tft.width()-xx;
    if (_calib_y_inv) yy = _tft.height()-yy;

    *x = xx;
    *y = yy;
    return true;
  }
  return false;
}

bool TouchScreen::getTouchRaw(int16_t *x, int16_t *y) {
  int16_t x_tmp, y_tmp, z_tmp;

  x_tmp = readTouchX();
  y_tmp = readTouchY();
  z_tmp = readTouchZ();

  if (z_tmp > ADC_Z_TRESHOLD) {
    *x = x_tmp;
    *y = y_tmp;
    return true;
  }
  return false;
}

int16_t TouchScreen::readTouchX(void) {
  // Store current pin state
  storePinState();

  gpio_num_t ym = (gpio_num_t)_ym;
  gpio_num_t yp = (gpio_num_t)_yp;
  // Prepare RTC pins
  rtc_gpio_init(ym);
  rtc_gpio_init(yp);

  rtc_gpio_set_direction(ym, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_set_direction(yp, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_dis(ym);
  rtc_gpio_pullup_dis(yp);
  rtc_gpio_pulldown_en(ym);
  rtc_gpio_pulldown_en(yp);

  // Prepare digital pins
  pinMode(_xm, OUTPUT);
  pinMode(_xp, OUTPUT);
  digitalWrite(_xm, LOW);
  digitalWrite(_xp, HIGH);

  // Reading
  int16_t samples[ADC_SAMPLES];
  for (int8_t i = 0; i < ADC_SAMPLES; i++) samples[i] = analogRead(yp);

  // Sorting
  arr_sort(samples);

  // Average and discard
  int16_t value = arr_filter(samples) - ADC_NOISE_TRESHOLD;
  if (value < 0) value = 0;

  // Restore RTC pins
  rtc_gpio_deinit(ym);
  rtc_gpio_deinit(yp);

  // Restore current pin state
  restorePinState();

  return value;
}

int16_t TouchScreen::readTouchY(void) {
  // Store current pin state
  storePinState();

  gpio_num_t xm = (gpio_num_t)_xm;
  gpio_num_t xp = (gpio_num_t)_xp;

  // Prepare RTC pins
  rtc_gpio_init(xm);
  rtc_gpio_init(xp);

  rtc_gpio_set_direction(xm, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_set_direction(xp, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_dis(xm);
  rtc_gpio_pullup_dis(xp);
  rtc_gpio_pulldown_en(xm);
  rtc_gpio_pulldown_en(xp);

  // Prepare digital pins
  pinMode(_ym, OUTPUT);
  pinMode(_yp, OUTPUT);
  digitalWrite(_ym, LOW);
  digitalWrite(_yp, HIGH);

  // Reading
  int16_t samples[ADC_SAMPLES];
  for (int8_t i = 0; i < ADC_SAMPLES; i++) samples[i] = analogRead(xm);

  // Sorting
  arr_sort(samples);

  // Average and discard
  int16_t value = arr_filter(samples) - ADC_NOISE_TRESHOLD;
  if (value < 0) value = 0;

  // Restore RTC pins
  rtc_gpio_deinit(xm);
  rtc_gpio_deinit(xp);

  // Restore current pin state
  restorePinState();

  return value;
}

int16_t TouchScreen::readTouchZ(void) {
  // Store current pin state
  storePinState();

  gpio_num_t xm = (gpio_num_t)_xm;
  gpio_num_t yp = (gpio_num_t)_yp;

  // Prepare RTC pins
  rtc_gpio_init(xm);
  rtc_gpio_init(yp);

  rtc_gpio_set_direction(xm, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_set_direction(yp, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_dis(xm);
  rtc_gpio_pullup_dis(yp);
  rtc_gpio_pulldown_en(xm);
  rtc_gpio_pulldown_en(yp);

  // Prepare digital pins
  pinMode(_xp, OUTPUT);
  pinMode(_ym, OUTPUT);
  digitalWrite(_xp, LOW);
  digitalWrite(_ym, HIGH);

  // Reading
  int16_t samples[ADC_SAMPLES];
  for (int8_t i = 0; i < ADC_SAMPLES; i++) samples[i] = (1<<12) - 1 - (analogRead(yp) - analogRead(xm));

  // Sorting
  arr_sort(samples);

  // Average and discard
  int16_t value = arr_filter(samples);

  // Restore RTC pins
  rtc_gpio_deinit(xm);
  rtc_gpio_deinit(yp);
  
  // Restore current pin state
  restorePinState();

  return value;
}

void TouchScreen::calibrate(uint16_t *params) {
  uint16_t width = _tft.width();
  uint16_t height = _tft.height();
  uint8_t size = 20;
  int16_t values[] = {0,0,0,0,0,0,0,0};
  int16_t x_tmp, y_tmp;

  for(uint8_t i = 0; i<4; i++) {
    _tft.fillRect(0, 0, size+1, size+1, TFT_BLACK);
    _tft.fillRect(0, height-size-1, size+1, size+1, TFT_BLACK);
    _tft.fillRect(width-size-1, 0, size+1, size+1, TFT_BLACK);
    _tft.fillRect(width-size-1, height-size-1, size+1, size+1, TFT_BLACK);

    if (i == 5) break;
    
    switch (i) {
      case 0: // TL
        _tft.fillCircle(size/2, size/2, size/2, TFT_WHITE);
        _tft.fillCircle(size/2, size/2, size/4, TFT_RED);
        break;
      case 1: // BL
        _tft.fillCircle(size/2, height-size/2-1, size/2, TFT_WHITE);
        _tft.fillCircle(size/2, height-size/2-1, size/4, TFT_RED);
        break;
      case 2: // TR
        _tft.fillCircle(width-size/2-1, size/2, size/2, TFT_WHITE);
        _tft.fillCircle(width-size/2-1, size/2, size/4, TFT_RED);
        break;
      case 3: // BR
        _tft.fillCircle(width-size/2-1, height-size/2-1, size/2, TFT_WHITE);
        _tft.fillCircle(width-size/2-1, height-size/2-1, size/4, TFT_RED);
        break;
      }

    if(i>0) delay(1000);

    for(uint8_t j= 0; j<8; j++) {
      while (!getTouchRaw(&x_tmp, &y_tmp));
      values[i*2  ] += x_tmp;
      values[i*2+1] += y_tmp;
      }
    values[i*2  ] /= 8;
    values[i*2+1] /= 8;
  }

  _calib_rotate = false;
  if(abs(values[0]-values[2]) > abs(values[1]-values[3])) {
    _calib_rotate = true;
    _calib_x0 = (values[1] + values[3])/2; // calc min x
    _calib_x1 = (values[5] + values[7])/2; // calc max x
    _calib_y0 = (values[0] + values[4])/2; // calc min y
    _calib_y1 = (values[2] + values[6])/2; // calc max y
  } else {
    _calib_x0 = (values[0] + values[2])/2; // calc min x
    _calib_x1 = (values[4] + values[6])/2; // calc max x
    _calib_y0 = (values[1] + values[5])/2; // calc min y
    _calib_y1 = (values[3] + values[7])/2; // calc max y
  }

  _calib_x_inv = false;
  if(_calib_x0 > _calib_x1) {
    values[0]=_calib_x0;
    _calib_x0 = _calib_x1;
    _calib_x1 = values[0];
    _calib_x_inv = true;
  }
  _calib_y_inv = false;
  if(_calib_y0 > _calib_y1) {
    values[0]=_calib_y0;
    _calib_y0 = _calib_y1;
    _calib_y1 = values[0];
    _calib_y_inv = true;
  }

  _calib_x1 -= _calib_x0;
  _calib_y1 -= _calib_y0;

  if(_calib_x0 == 0) _calib_x0 = 1;
  if(_calib_x1 == 0) _calib_x1 = 1;
  if(_calib_y0 == 0) _calib_y0 = 1;
  if(_calib_y1 == 0) _calib_y1 = 1;

  if(params != NULL) {
    params[0] = _calib_x0;
    params[1] = _calib_x1;
    params[2] = _calib_y0;
    params[3] = _calib_y1;
    params[4] = _calib_rotate | (_calib_x_inv <<1) | (_calib_y_inv <<2);
  }  
}

void TouchScreen::setCalibration(uint16_t *params){
  _calib_x0 = params[0];
  _calib_x1 = params[1];
  _calib_y0 = params[2];
  _calib_y1 = params[3];

  if(_calib_x0 == 0) _calib_x0 = 1;
  if(_calib_x1 == 0) _calib_x1 = 1;
  if(_calib_y0 == 0) _calib_y0 = 1;
  if(_calib_y1 == 0) _calib_y1 = 1;

  _calib_rotate = params[4] & 0x01;
  _calib_x_inv = params[4] & 0x02;
  _calib_y_inv = params[4] & 0x04;
}

void TouchScreen::debug(void) {
  Serial.println("== TOUCHSCREEN DEBUG INFO ==");
  Serial.println("-- GPIO --");
  Serial.print("XM GPIO pin: "); Serial.println(_xm);
  Serial.print("XM BIT MASK: "); Serial.println(_xm_mask);
  Serial.print("XM RTC pin:  "); Serial.println(_xm_rtc);

  Serial.print("XP GPIO pin: "); Serial.println(_xp);
  Serial.print("XM BIT MASK: "); Serial.println(_xp_mask);

  Serial.print("YM GPIO pin: "); Serial.println(_ym);
  Serial.print("XM BIT MASK: "); Serial.println(_ym_mask);

  Serial.print("YP GPIO pin: "); Serial.println(_yp);
  Serial.print("XM BIT MASK: "); Serial.println(_yp_mask);
  Serial.print("YP RTC pin:  "); Serial.println(_yp_rtc);

  Serial.println();
  Serial.println("-- READINGS --");
  while(1) {
    Serial.print(" X: "); Serial.print(readTouchX());
    Serial.print(" Y: "); Serial.print(readTouchY());
    Serial.print(" Z: "); Serial.println(readTouchZ());
    delay(100);
  }
}