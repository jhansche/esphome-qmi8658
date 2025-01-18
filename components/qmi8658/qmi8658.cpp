#include "qmi8658.h"

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

  delay(10);

  this->configure_accelerometer_(this->accel_range_, this->accel_odr_, this->accel_lpf_mode_);
  this->configure_gyro_(this->gyro_range_, this->gyro_odr_, this->gyro_lpf_mode_);

  bool accel_en =
      (this->accel_x_sensor_ != nullptr || this->accel_y_sensor_ != nullptr || this->accel_z_sensor_ != nullptr) &&
      this->accel_en;
  bool gyro_en =
      (this->gyro_x_sensor_ != nullptr || this->gyro_y_sensor_ != nullptr || this->gyro_z_sensor_ != nullptr) &&
      this->gyro_en;
  // this->enable_sensors_(accel_en, gyro_en, false);

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
}

void QMI8658Component::dump_config() {
  ESP_LOGCONFIG(TAG, "QMI8658:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with QMI8658 failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_PIN("  Interrupt pin 1: ", this->interrupt_pin_1_);
  LOG_PIN("  Interrupt pin 2: ", this->interrupt_pin_2_);
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

void QMI8658Component::loop() {
  PollingComponent::loop();

  auto has_a = has_accel_();
  auto has_g = has_gyro_();

  if (has_a || has_g) {
    auto interrupt = this->check_interrupt_();

    if (interrupt) {
      if (has_a) {
        this->read_accelerometer();
      }
      if (has_g) {
        this->read_gyro();
      }
    }
  }
}

bool QMI8658Component::check_interrupt_() {
  bool interrupt = false;
  if (this->interrupt_pin_1_ != nullptr) {
    interrupt = this->interrupt_pin_1_->digital_read() || interrupt;
  }
  if (this->interrupt_pin_2_ != nullptr) {
    interrupt = this->interrupt_pin_2_->digital_read() || interrupt;
  }
  return interrupt;
}

void QMI8658Component::update() {
  uint8_t data = 0;
  this->read_register(QMI8658Register_Status1, &data, 1);
  ESP_LOGCONFIG(TAG, "Status1: %x", data);

  // Read temperature
  if (this->temperature_sensor_ != nullptr) {
    uint8_t buf[2];
    int16_t temp = 0;
    float temp_f = 0;

    this->read_register(QMI8658Register_Tempearture_L, &buf[0], 1);
    this->read_register(QMI8658Register_Tempearture_H, &buf[1], 1);
    temp = ((int16_t) buf[1] << 8) | buf[0];
    temp_f = (float) temp / 256.0f;
    ESP_LOGD(TAG, "Temperature: %d °C", temp_f);
    temperature_sensor_->publish_state(temp_f);
  }

  if (this->has_accel_()) {
    this->read_accelerometer();
  }
  if (this->has_gyro_()) {
    this->read_gyro();
  }
}

void QMI8658Component::read_accelerometer() {
  uint8_t buf_reg[2];
  int16_t raw_acc_xyz[3];
  // FIXME: This should use stop=false and read 6 bytes at once via a repeated start read.
  //  But that does not appear to work correctly in esp-idf.

  this->read_register(QMI8658Register_Ax_L, &buf_reg[0], 1);
  this->read_register(QMI8658Register_Ax_H, &buf_reg[1], 1);
  raw_acc_xyz[0] = (int16_t) ((uint16_t) (buf_reg[1] << 8) | (buf_reg[0]));
  accel_data.x = (raw_acc_xyz[0] * ONE_G) / acc_lsb_div;

  this->read_register(QMI8658Register_Ay_L, &buf_reg[0], 1);
  this->read_register(QMI8658Register_Ay_H, &buf_reg[1], 1);
  raw_acc_xyz[1] = (int16_t) ((uint16_t) (buf_reg[1] << 8) | (buf_reg[0]));
  accel_data.y = (raw_acc_xyz[1] * ONE_G) / acc_lsb_div;

  this->read_register(QMI8658Register_Az_L, &buf_reg[0], 1);
  this->read_register(QMI8658Register_Az_H, &buf_reg[1], 1);
  raw_acc_xyz[2] = (int16_t) ((uint16_t) (buf_reg[1] << 8) | (buf_reg[0]));
  accel_data.z = (raw_acc_xyz[2] * ONE_G) / acc_lsb_div;

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
  uint8_t buf_reg[6];
  int16_t raw_gyro_xyz[3];

  this->read_register(QMI8658Register_Gx_L, &buf_reg[0], 1);
  this->read_register(QMI8658Register_Gx_H, &buf_reg[1], 1);
  this->read_register(QMI8658Register_Gy_L, &buf_reg[2], 1);
  this->read_register(QMI8658Register_Gy_H, &buf_reg[3], 1);
  this->read_register(QMI8658Register_Gz_L, &buf_reg[4], 1);
  this->read_register(QMI8658Register_Gz_H, &buf_reg[5], 1);

  raw_gyro_xyz[0] = (int16_t) ((uint16_t) (buf_reg[1] << 8) | (buf_reg[0]));
  raw_gyro_xyz[1] = (int16_t) ((uint16_t) (buf_reg[3] << 8) | (buf_reg[2]));
  raw_gyro_xyz[2] = (int16_t) ((uint16_t) (buf_reg[5] << 8) | (buf_reg[4]));

  this->gyro_data.x = (raw_gyro_xyz[0] * 1.0f) / gyro_lsb_div;
  this->gyro_data.y = (raw_gyro_xyz[1] * 1.0f) / gyro_lsb_div;
  this->gyro_data.z = (raw_gyro_xyz[2] * 1.0f) / gyro_lsb_div;

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

void QMI8658Component::configure_accelerometer_(uint8_t range, uint8_t odr, uint8_t lpf_mode) {
  uint8_t ctl_data = range | odr;
  this->write_register(QMI8658Register_Ctrl2, &ctl_data, 1);
  this->read_register(QMI8658Register_Ctrl5, &ctl_data, 1);
  ctl_data &= 0xf0;
  if (lpf_mode == LPF_DISABLED) {
    ctl_data &= ~0x01;
  } else {
    ctl_data |= 0x01 | A_LSP_MODE_3;
  }
  this->write_register(QMI8658Register_Ctrl5, &ctl_data, 1);
}

void QMI8658Component::configure_gyro_(uint8_t range, uint8_t odr, uint8_t lpf_mode) {
  uint8_t ctl_data = range | odr;
  this->write_register(QMI8658Register_Ctrl3, &ctl_data, 1);
  this->read_register(QMI8658Register_Ctrl5, &ctl_data, 1);
  ctl_data &= 0x0f;
  if (lpf_mode == LPF_DISABLED) {
    ctl_data &= ~0x10;
  } else {
    ctl_data |= 0x10 | G_LSP_MODE_3;
  }
  this->write_register(QMI8658Register_Ctrl5, &ctl_data, 1);
}

float QMI8658Component::get_setup_priority() const { return setup_priority::PROCESSOR; }

void QMI8658Component::enable_sensors_(bool accel_en, bool gyro_en, bool mag_en) {
  uint8_t ctl_data = QMI8658_CTRL7_DISABLE_ALL;
  if (this->accel_x_sensor_ != nullptr || this->accel_y_sensor_ != nullptr || this->accel_z_sensor_ != nullptr) {
    ctl_data |= QMI8658_CTRL7_ACC_ENABLE;
  }
  if (this->gyro_x_sensor_ != nullptr || this->gyro_y_sensor_ != nullptr || this->gyro_z_sensor_ != nullptr) {
    ctl_data |= QMI8658_CTRL7_GYR_ENABLE;
  }

  ctl_data &= QMI8658_CTRL7_ENABLE_MASK;
  this->write_register(QMI8658Register_Ctrl7, &ctl_data, 1);
}

}  // namespace qmi8658
}  // namespace esphome
