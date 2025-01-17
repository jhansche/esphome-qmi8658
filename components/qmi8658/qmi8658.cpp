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

  // Configure accelerometer:
  uint8_t ctl_data = 0x00;
  ctl_data = this->accel_range_ | this->accel_odr_;
  this->write_register(QMI8658Register_Ctrl2, &ctl_data, 1);
  this->read_register(QMI8658Register_Ctrl5, &ctl_data, 1);
  ctl_data &= 0xf0;
  if (this->accel_lpf_mode_ == LPF_DISABLED) {
    ctl_data &= ~0x01;
  } else {
    ctl_data |= 0x01 | A_LSP_MODE_3;
  }
  this->write_register(QMI8658Register_Ctrl5, &ctl_data, 1);

  // Configure gyro:
  ctl_data = this->gyro_range_ | this->gyro_odr_;
  this->write_register(QMI8658Register_Ctrl3, &ctl_data, 1);
  this->read_register(QMI8658Register_Ctrl5, &ctl_data, 1);
  ctl_data &= 0x0f;
  if (this->accel_lpf_mode_ == LPF_DISABLED) {
    ctl_data &= ~0x10;
  } else {
    ctl_data |= 0x10 | G_LSP_MODE_3;
  }
  this->write_register(QMI8658Register_Ctrl5, &ctl_data, 1);

  ctl_data = 0;
  if (this->accel_x_sensor_ != nullptr || this->accel_y_sensor_ != nullptr || this->accel_z_sensor_ != nullptr) {
    ctl_data |= QMI8658_CTRL7_ACC_ENABLE;
  }
  if (this->gyro_x_sensor_ != nullptr || this->gyro_y_sensor_ != nullptr || this->gyro_z_sensor_ != nullptr) {
    ctl_data |= QMI8658_CTRL7_GYR_ENABLE;
  }

  ctl_data &= QMI8658_CTRL7_ENABLE_MASK;
  this->write_register(QMI8658Register_Ctrl7, &ctl_data, 1);
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

  if (this->interrupt_pin_1_ != nullptr) {
    bool interrupt = this->interrupt_pin_1_->digital_read();
    if (interrupt)
      this->update();
  }
  if (this->interrupt_pin_2_ != nullptr) {
    bool interrupt = this->interrupt_pin_2_->digital_read();
    if (interrupt)
      this->update();
  }
}

void QMI8658Component::update() {
  // Debugging
  {
    uint8_t read_data = 0x00;
    this->read_register(QMI8658Register_Ctrl1, &read_data, 1);
    ESP_LOGI(TAG, "QMI8658Register_Ctrl1 = %x", read_data);
    this->read_register(QMI8658Register_Ctrl2, &read_data, 1);
    ESP_LOGI(TAG, "QMI8658Register_Ctrl2 = %x", read_data);
    this->read_register(QMI8658Register_Ctrl3, &read_data, 1);
    ESP_LOGI(TAG, "QMI8658Register_Ctrl3 = %x", read_data);
    this->read_register(QMI8658Register_Ctrl4, &read_data, 1);
    ESP_LOGI(TAG, "QMI8658Register_Ctrl4 = %x", read_data);
    this->read_register(QMI8658Register_Ctrl5, &read_data, 1);
    ESP_LOGI(TAG, "QMI8658Register_Ctrl5 = %x", read_data);
    this->read_register(QMI8658Register_Ctrl6, &read_data, 1);
    ESP_LOGI(TAG, "QMI8658Register_Ctrl6 = %x", read_data);
    this->read_register(QMI8658Register_Ctrl7, &read_data, 1);
    ESP_LOGI(TAG, "QMI8658Register_Ctrl7 = %x", read_data);
    this->read_register(QMI8658Register_StatusInt, &read_data, 1);
    ESP_LOGI(TAG, "QMI8658Register_StatusInt = %x", read_data);
    this->read_register(QMI8658Register_Status0, &read_data, 1);
    ESP_LOGI(TAG, "QMI8658Register_Status0 = %x", read_data);
    this->read_register(QMI8658Register_Status1, &read_data, 1);
    ESP_LOGI(TAG, "QMI8658Register_Status1 = %x", read_data);
  }

  // Read temperature
  if (this->temperature_sensor_ != nullptr) {
    uint8_t buf[2];
    int16_t temp = 0;
    float temp_f = 0;

    this->read_bytes(QMI8658Register_Tempearture_L, buf, 2);
    temp = ((int16_t) buf[1] << 8) | buf[0];
    temp_f = (float) temp / 256.0f;
    ESP_LOGD(TAG, "Temperature: %d °C", temp_f);
    temperature_sensor_->publish_state(temp_f);
  }

  // Read accelerometer
  {
    uint8_t buf_reg[2];
    int16_t raw_acc_xyz[3];

    this->read_register(QMI8658Register_Ax_L, &buf_reg[0], 1);
    this->read_register(QMI8658Register_Ax_H, &buf_reg[1], 1);
    ESP_LOGD(TAG, "Accel registers (Ax_LH +2): %02x%02x", buf_reg[0], buf_reg[1]);
    raw_acc_xyz[0] = (int16_t) ((uint16_t) (buf_reg[1] << 8) | (buf_reg[0]));
    accel_data.x = (raw_acc_xyz[0] * ONE_G) / acc_lsb_div;

    this->read_register(QMI8658Register_Ay_L, &buf_reg[0], 1);
    this->read_register(QMI8658Register_Ay_H, &buf_reg[1], 1);
    ESP_LOGD(TAG, "Accel registers (Ay_LH +2): %02x%02x", buf_reg[0], buf_reg[1]);
    raw_acc_xyz[1] = (int16_t) ((uint16_t) (buf_reg[1] << 8) | (buf_reg[0]));
    accel_data.y = (raw_acc_xyz[1] * ONE_G) / acc_lsb_div;

    this->read_register(QMI8658Register_Az_L, &buf_reg[0], 1);
    this->read_register(QMI8658Register_Az_H, &buf_reg[1], 1);
    ESP_LOGD(TAG, "Accel registers (Az_LH +2): %02x%02x", buf_reg[0], buf_reg[1]);
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

  // Read gyro
  {
    uint8_t buf_reg[6];
    int16_t raw_gyro_xyz[3];

    this->read_bytes(QMI8658Register_Gx_L, buf_reg, 6);  // 0x1f, 31
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
}

float QMI8658Component::get_setup_priority() const { return setup_priority::PROCESSOR; }

}  // namespace qmi8658
}  // namespace esphome
