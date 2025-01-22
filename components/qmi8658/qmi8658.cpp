#include "qmi8658.h"
#include "qmi8658_reg.h"

#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"
#include <cmath>

namespace esphome {
namespace qmi8658 {

static const char *TAG = "qmi8658";

void QMI8658Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up QMI8658...");

  uint8_t QMI8658_chip_id = 0x00;
  uint8_t QMI8658_revision_id = 0x00;
  this->read_register(QMI8658Register_WhoAmI, &QMI8658_chip_id, 1);

  if (QMI8658_chip_id != 0x05) {
    ESP_LOGE(TAG, "Unrecognized chip id %x", QMI8658_chip_id);
    this->mark_failed();
    return;
  }
  this->read_register(QMI8658Register_Revision, &QMI8658_revision_id, 1);

  ESP_LOGCONFIG(TAG, "qmi8658 chip %x, rev %x", QMI8658_chip_id, QMI8658_revision_id);

  // Even if we're not using it right now, let's read the registers so we have them.
  read_register(QMI8658Register_Ctrl1, ctrl1.packed, 1);
  read_register(QMI8658Register_Ctrl2, ctrl2.packed, 1);
  read_register(QMI8658Register_Ctrl3, ctrl3.packed, 1);
  read_register(QMI8658Register_Ctrl5, ctrl5.packed, 1);
  read_register(QMI8658Register_Ctrl7, ctrl7.packed, 1);
  read_register(QMI8658Register_Ctrl8, ctrl8.packed, 1);

  ctrl1.endianness = 1;
  ctrl1.addr_ai = true;  // address auto increment (needed for burst reads)
  this->write_register(QMI8658Register_Ctrl1, ctrl1.packed, 1);
  ctrl8.handshake_type = 1;
  this->write_register(QMI8658Register_Ctrl8, ctrl8.packed, 1);
  ctrl7.sync_sample = false;        // not necessary for our use case
  ctrl7.data_ready_disable = true;  // no real benefit to a firehose of data
  this->write_register(QMI8658Register_Ctrl7, ctrl7.packed, 1);

  if (this->interrupt_pin_1_ != nullptr) {
    this->interrupt_pin_1_->setup();
    this->interrupt_pin_1_->attach_interrupt(&QMI8658Component::interrupt_, this, gpio::INTERRUPT_RISING_EDGE);
  }

  if (this->interrupt_pin_2_ != nullptr) {
    this->interrupt_pin_2_->setup();
    this->interrupt_pin_2_->attach_interrupt(&QMI8658Component::interrupt_, this, gpio::INTERRUPT_RISING_EDGE);
  }

  this->configure_accelerometer_(this->accel_range_, this->accel_odr_, this->accel_lpf_mode_);
  this->configure_gyro_(this->gyro_range_, this->gyro_odr_, this->gyro_lpf_mode_);

  bool accel_en =
      (this->accel_x_sensor_ != nullptr || this->accel_y_sensor_ != nullptr || this->accel_z_sensor_ != nullptr) &&
      this->accel_en;
  bool gyro_en =
      (this->gyro_x_sensor_ != nullptr || this->gyro_y_sensor_ != nullptr || this->gyro_z_sensor_ != nullptr) &&
      this->gyro_en;
  // TODO: toggle these with switches

  if (/* enable tap detection */ false) {
    // TODO: move enable_tap_detection_() to here?
    //  Or toggle it with a switch? Also add config options.
    // For now this happens on the 3rd update()... for debugging purposes.
    this->enable_tap_detection_(true);
  }
  this->enable_sensors_(accel_en, gyro_en);

#ifdef QMI8658_ENABLE_WOM
  unsigned char womCmd[3];
  enum QMI8658_Interrupt interrupt = QMI8658_Int1;
  enum QMI8658_InterruptState initialState = QMI8658State_low;
  enum QMI8658_WakeOnMotionThreshold threshold = QMI8658WomThreshold_low;
  unsigned char blankingTime = 0x00;
  const unsigned char blankingTimeMask = 0x3F;

  // Disable all
  this->enable_sensors_(false, false);
  // Now configure the accelerometer in low-power mode
  this->configure_accelerometer_(QMI8658AccRange_2g, QMI8658AccOdr_LowPower_21Hz, QMI8658Lpf_Disable);

  womCmd[0] = QMI8658Register_Cal1_L;  // WoM Threshold: absolute value in mg (with 1mg/LSB resolution)
  womCmd[1] = threshold;
  womCmd[2] = (unsigned char) interrupt | (unsigned char) initialState | (blankingTime & blankingTimeMask);
  this->write_register(QMI8658Register_Cal1_L, &womCmd[1], 1);
  this->write_register(QMI8658Register_Cal1_H, &womCmd[2], 1);

  this->enable_sensors_(true, false);
#endif
}

static int xxx_int = 0;
void IRAM_ATTR QMI8658Component::interrupt_(QMI8658Component *args) {
  if (xxx_int++ % 100 == 0) {
    ESP_LOGCONFIG(TAG, "Interrupt[%d]!", xxx_int);
  }
  return;
  // TODO: just set a flag or schedule this to happen on the loop
  statusint_reg_t status_int;
  status0_reg_t status0;
  status1_reg_t status1;

  args->read_register(QMI8658Register_StatusInt, status_int.packed, 1);
  args->read_register(QMI8658Register_Status0, status0.packed, 1);
  args->read_register(QMI8658Register_Status1, status1.packed, 1);

  if (xxx_int++ % 100 == 0) {
    ESP_LOGCONFIG(TAG, "Interrupt[%d]! Status: int=%x, 0=%x, 1=%x. Tap?=%d", xxx_int, status_int.packed[0],
                  status0.packed[0], status1.packed[0], status1.tap_detected);
  }
  // if tap_detected, read from TAP_STATUS (0x59)
  if (status1.tap_detected) {
    qmi8658_tap_status_t tap_status;
    args->read_register(QMI8658Register_TapStatus, tap_status.packed, 1);
    ESP_LOGCONFIG(TAG, "Tap detected: type=%d, axis=%d, polarity=%d", tap_status.type, tap_status.axis,
                  tap_status.polarity);
    // TODO: dispatch an event:
    //  https://esphome.io/components/binary_sensor/index.html#on-click
    // To support esphome on_double_click and on_multi_click, we might need to adjust the double_tap_window config?
  }

  if (args->interrupt_pin_1_ != nullptr) {
    auto int1 = args->interrupt_pin_1_->digital_read();
    ESP_LOGD(TAG, "Interrupt 1: %d", int1);
  }
  if (args->interrupt_pin_2_ != nullptr) {
    auto int2 = args->interrupt_pin_2_->digital_read();
    ESP_LOGD(TAG, "Interrupt 2: %d", int2);
  }
}

void QMI8658Component::dump_config() {
  ESP_LOGCONFIG(TAG, "QMI8658:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with QMI8658 failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);

  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
  ESP_LOGCONFIG(TAG, "  Accel range: %d", this->accel_range_);
  ESP_LOGCONFIG(TAG, "  Accel ODR: %d", this->accel_odr_);

  LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
  ESP_LOGCONFIG(TAG, "  Gyro range: %d", this->gyro_range_);
  ESP_LOGCONFIG(TAG, "  Gyro ODR: %d", this->gyro_odr_);

  ESP_LOGCONFIG(TAG, "  FIFO Int: %d", this->ctrl1.fifo_sel);  // aka DRDY
  ESP_LOGCONFIG(TAG, "  Motion Int: %d", this->ctrl8.int_sel);
  LOG_PIN("  Interrupt pin 1: ", this->interrupt_pin_1_);
  ESP_LOGCONFIG(TAG, "  INT1 en: %d", this->ctrl1.int1_en);
  LOG_PIN("  Interrupt pin 2: ", this->interrupt_pin_2_);
  ESP_LOGCONFIG(TAG, "  INT2 en: %d", this->ctrl1.int2_en);
}

static int xxx_loop = 0;
void QMI8658Component::loop() {
  PollingComponent::loop();

  // FIXME: need to handle this better
  if (xxx_loop++ % 100 == 0) {
    ESP_LOGCONFIG(TAG, "Loop[%d]!", xxx_loop);
    statusint_reg_t status_int;
    status0_reg_t status0;
    status1_reg_t status1;

    read_register(QMI8658Register_StatusInt, status_int.packed, 1);
    read_register(QMI8658Register_Status0, status0.packed, 1);
    read_register(QMI8658Register_Status1, status1.packed, 1);

    // if tap_detected, read from TAP_STATUS (0x59)
    if (status1.tap_detected) {
      qmi8658_tap_status_t tap_status;
      read_register(QMI8658Register_TapStatus, tap_status.packed, 1);
      ESP_LOGCONFIG(TAG, "Tap detected: type=%d, axis=%d, polarity=%d", tap_status.type, tap_status.axis,
                    tap_status.polarity);
      // TODO: dispatch an event:
      //  https://esphome.io/components/binary_sensor/index.html#on-click
      // To support esphome on_double_click and on_multi_click, we might need to adjust the double_tap_window config?
    }
  }
}

void QMI8658Component::update() {
  statusint_reg_t status_int;
  status0_reg_t status0;
  status1_reg_t status1;

  this->read_register(QMI8658Register_StatusInt, status_int.packed, 1);
  this->read_register(QMI8658Register_Status0, status0.packed, 1);
  this->read_register(QMI8658Register_Status1, status1.packed, 1);
  ESP_LOGCONFIG(TAG, "Status: int=%x(%d,%d), 0=%x, 1=%x", status_int.packed[0], status_int.int1_mirror,
                status_int.int2_mirror, status0.packed[0], status1.packed[0]);

  // Read temperature
  if (this->temperature_sensor_ != nullptr) {
    temperature_data_t data;
    auto res = this->read_register(QMI8658Register_Tempearture_L, data.packed, 2);
    if (res != i2c::ERROR_OK) {
      ESP_LOGE(TAG, "Failed to read temperature (%04x): %x", data.raw, res);
    } else {
      float temp_f = (float) data.raw / 256.0f;
      ESP_LOGD(TAG, "Temperature: %x %x (%x) => %d + %d/256 (%f) => %f", data.packed[0], data.packed[1], data.raw,
               data.deg_c, data.deg_c_fraction, data.deg_c_fraction / 256.0f, temp_f);
      temperature_sensor_->publish_state(temp_f);
    }
  }

  // TODO: accel/gyro can also be internalized into input sensors like tap detection,
  //  without being exposed as axis sensors
  if (this->has_accel_()) {
    this->read_accelerometer();
  }
  if (this->has_gyro_()) {
    this->read_gyro();
  }

  // FIXME: for logging only; should be in setup()
  this->enable_tap_detection_(true);
}

void QMI8658Component::read_accelerometer() {
  imu_axis_data_t data_reg;
  this->read_register(QMI8658Register_Ax_L, data_reg.packed, 6);
  accel_data.x = (data_reg.raw_x * ONE_G) / acc_lsb_div;
  accel_data.y = (data_reg.raw_y * ONE_G) / acc_lsb_div;
  accel_data.z = (data_reg.raw_z * ONE_G) / acc_lsb_div;

  if (this->accel_x_sensor_ != nullptr) {
    accel_x_sensor_->publish_state(accel_data.x);
  }
  if (this->accel_y_sensor_ != nullptr) {
    accel_y_sensor_->publish_state(accel_data.y);
  }
  if (this->accel_z_sensor_ != nullptr) {
    accel_z_sensor_->publish_state(accel_data.z);
  }
}

void QMI8658Component::read_gyro() {
  imu_axis_data_t data_reg;
  this->read_register(QMI8658Register_Gx_L, data_reg.packed, 6);
  gyro_data.x = (data_reg.raw_x * 1.0f) / gyro_lsb_div;
  gyro_data.y = (data_reg.raw_y * 1.0f) / gyro_lsb_div;
  gyro_data.z = (data_reg.raw_z * 1.0f) / gyro_lsb_div;

  if (this->gyro_x_sensor_ != nullptr) {
    this->gyro_x_sensor_->publish_state(gyro_data.x);
  }
  if (this->gyro_y_sensor_ != nullptr) {
    this->gyro_y_sensor_->publish_state(gyro_data.y);
  }
  if (this->gyro_z_sensor_ != nullptr) {
    this->gyro_z_sensor_->publish_state(gyro_data.z);
  }
}

void QMI8658Component::configure_accelerometer_(QMI8658_AccRange range, QMI8658_AccOdr odr, QMI8658_LpfMode lpf_mode,
                                                bool lpf_en) {
  ctrl2.odr = odr;
  ctrl2.scale = range;
  this->write_register(QMI8658Register_Ctrl2, ctrl2.packed, 1);

  ctrl5.accel_lpf_en = lpf_en;
  ctrl5.accel_lpf_mode = lpf_mode;
  this->write_register(QMI8658Register_Ctrl5, ctrl5.packed, 1);
}

void QMI8658Component::configure_gyro_(QMI8658_GyrRange range, QMI8658_GyrOdr odr, QMI8658_LpfMode lpf_mode,
                                       bool lpf_en) {
  ctrl3.odr = odr;
  ctrl3.scale = range;
  this->write_register(QMI8658Register_Ctrl3, ctrl3.packed, 1);

  ctrl5.gyro_lpf_en = lpf_en;
  ctrl5.gyro_lpf_mode = lpf_mode;
  this->write_register(QMI8658Register_Ctrl5, ctrl5.packed, 1);
}

float QMI8658Component::get_setup_priority() const { return setup_priority::PROCESSOR; }

void QMI8658Component::enable_sensors_(bool accel_en, bool gyro_en) {
  ctrl7.accel_en = accel_en;
  ctrl7.gyro_en = gyro_en;
  this->write_register(QMI8658Register_Ctrl7, ctrl7.packed, 1);
}

void QMI8658Component::enable_wake_on_motion(bool enable, qmi8658_wom_config_t config) {
  ctrl9_cmd_parameters_t params = {.wom_config_page = {
                                       .wom_threshold = enable ? config.threshold : 0,
                                       .target_interrupt = config.target_interrupt,
                                       .interrupt_start_level = config.interrupt_state,
                                       .blanking_time = config.blanking_time,
                                   }};
  this->ctrl9_write(QMI8658_Ctrl9_Cmd_WoM_Setting, params);
}

static int _xxx_once = 0;  // FIXME
void QMI8658Component::enable_tap_detection_(bool enable, qmi8658_tap_config_t config) {
  if (++_xxx_once != 3)  // do this only on the 3rd update()
    return;
  // ^ FIXME: this is a hack to get it to run once on startup, as part of update()

  ESP_LOGD(TAG, "Tap detection: enable=%d", enable);
  if (enable) {
    ESP_LOGV(TAG, "Tap detection config:");
    ESP_LOGV(TAG, "  priority=%d", config.priority);
    ESP_LOGV(TAG, "  peak_window=%d", config.peak_window);
    ESP_LOGV(TAG, "  tap_window=%d", config.tap_window);
    ESP_LOGV(TAG, "  double_tap_window=%d", config.double_tap_window);
    ESP_LOGV(TAG, "  target_interrupt=%d", config.target_interrupt);
    ESP_LOGV(TAG, "  alpha=%f", config.alpha);
    ESP_LOGV(TAG, "  gamma=%f", config.gamma);
    ESP_LOGV(TAG, "  peak_mag_threshold=%f", config.peak_mag_threshold);
    ESP_LOGV(TAG, "  undefined_motion_threshold=%f", config.undefined_motion_threshold);

    if (config.target_interrupt == QMI8658_Int1 && !ctrl1.int1_en) {
      ESP_LOGW(TAG, "Tap detection: ctrl1 INT1 is not enabled.");
      ctrl1.int1_en = true;
      // Do we need to write it?
    } else if (config.target_interrupt == QMI8658_Int2 && !ctrl1.int2_en) {
      ESP_LOGW(TAG, "Tap detection: ctrl1 INT2 is not enabled.");
      ctrl1.int2_en = true;
      // Do we need to write it?
    }

    if (config.target_interrupt != ctrl8.int_sel) {
      ESP_LOGW(TAG, "Tap detection: ctrl8 is not selecting the same interrupt.");
      ctrl8.int_sel = config.target_interrupt;
      // Do we need to write it?
    }

    // TODO: allow these to be configured
    ctrl9_cmd_tap_config_page1_t page1 = {
        .peak_window = config.peak_window,  // TODO: convert using millis & ODR
        .priority = config.priority,
        .tap_window = config.tap_window,                // TODO: convert using millis & ODR
        .double_tap_window = config.double_tap_window,  // TODO: convert using millis & ODR,
        .cmd_info = {.cmd_page = 1},
    };

    ctrl9_cmd_tap_config_page2_t page2 = {
        .alpha = QMI8658_1F7(config.alpha),
        .gamma = QMI8658_1F7(config.gamma),
        .peak_mag_thr = QMI8658_2F10(config.peak_mag_threshold),
        .udm_thr = QMI8658_2F10(config.undefined_motion_threshold),
        .cmd_info = {.cmd_page = 2},
    };

    // Tap detection has to be configured while a/g are turned off:
    this->enable_sensors_(false, false);

    ctrl9_cmd_parameters_t params = {.tap_config_page1 = page1};

    ESP_LOGCONFIG(TAG, "Tap detection; page1(%d): %02x %02x %02x %02x %02x %02x %02x %02x",
                  params.tap_config_page1.cmd_info.cmd_page, params.packed[0], params.packed[1], params.packed[2],
                  params.packed[3], params.packed[4], params.packed[5], params.packed[6], params.packed[7]);

    this->ctrl9_write(QMI8658_Ctrl9_Cmd_Configure_Tap, params);

    params = {.tap_config_page2 = page2};

    ESP_LOGCONFIG(TAG, "Tap detection; page2(%d): %02x %02x %02x %02x %02x %02x %02x %02x",
                  params.tap_config_page2.cmd_info.cmd_page, params.packed[0], params.packed[1], params.packed[2],
                  params.packed[3], params.packed[4], params.packed[5], params.packed[6], params.packed[7]);

    this->ctrl9_write(QMI8658_Ctrl9_Cmd_Configure_Tap, params);

    // Now reenable the sensors
    if (this->accel_odr_ < QMI8658AccOdr_250Hz) {
      ESP_LOGW(TAG, "For best results, accelerometer ODR should be at least 200Hz. Setting to 250Hz.");
      this->accel_odr_ = QMI8658AccOdr_250Hz;
    }

    this->configure_accelerometer_(this->accel_range_, this->accel_odr_, this->accel_lpf_mode_, false);
    this->enable_sensors_(true, has_gyro_());

    // Then enable it in ctrl8
    ctrl8.tap_en = true;
  } else {
    ctrl8.tap_en = false;
  }
  this->write_register(QMI8658Register_Ctrl8, ctrl8.packed, 1);
}

void QMI8658Component::ctrl9_write(QMI8658_Ctrl9Command cmd, ctrl9_cmd_parameters_t const params) {
  statusint_reg_t status_int;
  uint8_t cmd_byte = cmd;

  ESP_LOGV(TAG, "Writing CTRL9 command 0x%02x, params(page %d)=%02x %02x %02x %02x %02x %02x %02x %02x", cmd,
           params.tap_config_page1.cmd_info.cmd_page, params.packed[0], params.packed[1], params.packed[2],
           params.packed[3], params.packed[4], params.packed[5], params.packed[6], params.packed[7]);
  ESP_LOGV(TAG, "  peak_window=%d", params.tap_config_page1.peak_window);
  ESP_LOGV(TAG, "  priority=%d", params.tap_config_page1.priority);
  ESP_LOGV(TAG, "  tap_window=%d", params.tap_config_page1.tap_window);
  ESP_LOGV(TAG, "  double_tap_window=%d", params.tap_config_page1.double_tap_window);
  ESP_LOGV(TAG, "  cmd_info=%d", params.tap_config_page1.cmd_info.cmd_page);

  // First write the 8 bytes of parameters
  this->write_register(QMI8658Register_Cal1_L, params.packed, 8);
  // followed by the command itself
  this->write_register(QMI8658Register_Ctrl9, &cmd_byte, 1);

  // wait for DONE (or we'll have to switch to interrupt-driven)
  for (int i = 0; i < 5; i++) {
    delay(10);
    this->read_register(QMI8658Register_StatusInt, status_int.packed, 1);
    if (status_int.done) {
      ESP_LOGV(TAG, "CTRL9 command %x received by the device", cmd);
      break;
    } else if (i == 4) {
      ESP_LOGE(TAG, "CTRL9 command not done after 50ms, giving up.");
      return;
    } else {
      ESP_LOGW(TAG, "CTRL9 command not done, trying again...");
      continue;
    }
  }

  // Acknowledge the doneness
  cmd_byte = QMI8658_Ctrl9_Cmd_ACK;
  this->write_register(QMI8658Register_Ctrl9, &cmd_byte, 1);

  // And done bit gets cleared, but we don't have to read it.
  delay(5);
  this->read_register(QMI8658Register_StatusInt, status_int.packed, 1);
  if (status_int.done) {
    ESP_LOGI(TAG, "CTRL9 command done bit still set after ACK");
  }
}

}  // namespace qmi8658
}  // namespace esphome
