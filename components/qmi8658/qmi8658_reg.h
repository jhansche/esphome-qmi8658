#pragma once

#include "qmi8658_types.h"
#include <cstdint>

namespace esphome {
namespace qmi8658 {

// CTRL1 (0x02)
typedef union {
  // 0x60 = 0110 0000: addr_ai=1, endianness=1
  struct {                // default 0x20
    bool disable : 1;     // 0=normal, 1=disable
    bool : 1;             // bit 1 is unused
    bool fifo_sel : 1;    // 0=int2, 1=int1
    bool int1_en : 1;     // int1: 0=highz, 1=output
    bool int2_en : 1;     // int2: 0=highz, 1=output
    bool endianness : 1;  // 0=little endian, 1=big endian
    bool addr_ai : 1;     // address auto increment (needed for burst reads)
    bool spi_3wire : 1;   // 0=4-wire, 1=3-wire
  };
  uint8_t packed[1];
} ctrl1_reg_t;

// CTRL2 (0x03)
typedef union {
  struct {
    QMI8658_AccOdr odr : 4;      // output data rate: 0=8000Hz, 1=4000Hz, 2=2000Hz, 3=1000Hz, 4=500Hz, 5=250Hz, 6=125Hz,
                                 //  7=62.5Hz, 8=31.25Hz,
                                 //  9-11: n/a
                                 //  12=128Hz, 13=21Hz, 14=11Hz, 15=3Hz
    QMI8658_AccRange scale : 3;  // full scale: 0=2g, 1=4g, 2=8g, 3=16g
    uint8_t test : 1;            // self test
  };
  uint8_t packed[1];
} ctrl2_reg_t;

// CTRL3 (0x04)
typedef union {
  struct {
    QMI8658_GyrOdr odr : 4;      // output data rate: 0=8000Hz, 1=4000Hz, 2=2000Hz, 3=1000Hz, 4=500Hz, 5=250Hz, 6=125Hz,
                                 //  7=62.5Hz, 8=31.25Hz,
                                 //  9-11: n/a
                                 //  12=128Hz, 13=21Hz, 14=11Hz, 15=3Hz
    QMI8658_GyrRange scale : 3;  // full scale: 0=16dps, 1=32dps, 2=64dps, 3=128dps, 4=256dps, 5=512dps, 6=1024dps,
                                 //  7=2048dps
    uint8_t test : 1;            // self test
  };
  uint8_t packed[1];
} ctrl3_reg_t;

// CTRL5 (0x06)
typedef union {
  struct {
    bool accel_lpf_en : 1;
    QMI8658_LpfMode accel_lpf_mode : 2;
    uint8_t : 1;
    bool gyro_lpf_en : 1;
    QMI8658_LpfMode gyro_lpf_mode : 2;
    uint8_t : 1;
  };
  uint8_t packed[1];
} ctrl5_reg_t;

// CTRL7 (0x08)
typedef union {
  struct {
    bool accel_en : 1;
    bool gyro_en : 1;
    uint8_t : 2;
    bool gyro_snooze : 1;
    bool data_ready_disable : 1;  // 0=enable DRDY with INT2, 1=disable
    uint8_t : 1;
    bool sync_sample : 1;
  };
  uint8_t packed[1];
} ctrl7_reg_t;

// CTRL8 (0x09)
// Motion detection and tap detection both _require_ sync_sample=0.
// Requires MOTION_MODE_CTRL (with CTRL_CMD_CONFIGURE_MOTION) or
// TAP_CTRL (with CTRL_CMD_CONFIGURE_TAP)
// both via CTRL9 cmd, while A&G are disabled.
typedef union {
  struct {
    bool tap_en : 1;         // tap detection
    bool any_motion_en : 1;  // any-motion
    bool no_motion_en : 1;   // no-motion
    bool sig_motion_en : 1;  // significant motion (requires any+no also enabled)
    bool ped_en : 1;         // pedometer
    uint8_t : 1;
    QMI8658_Interrupt int_sel : 1;  // Interrupt for the motion/tap detections 0=INT2, 1=INT1
    uint8_t handshake_type : 1;     // Ctrl9 handshake type 0=INT1, 1=statusint.done
  };
  uint8_t packed[1];
} ctrl8_reg_t;

// TODO: CTRL9 (0x0A)

// STATUSINT (0x2d)
typedef union {
  struct {
    bool int2_mirror : 1;  // mirrors int2
    bool int1_mirror : 1;  // mirrors int1
    uint8_t : 5;
    bool : 1;
  };
  struct {
    bool is_available : 1;  // ctrl7.sync_sample: data avilable; else mirrors int2
    bool is_locked : 1;     // ctrl7.sync_sample: data locked; else mirrors int1
    uint8_t : 5;
    bool done : 1;  // ctrl9 command is done
  };
  uint8_t packed[1];
} statusint_reg_t;

// STATUS0 (0x2e)
typedef union {
  struct {
    bool accel_data_ready : 1;
    bool gyro_data_ready : 1;
    uint8_t : 6;
  };
  uint8_t packed[1];
} status0_reg_t;

// STATUS1 (0x2f)
typedef union {
  struct {
    uint8_t : 1;
    bool tap_detected : 1;
    bool wom : 1;  // wake on motion detected
    uint8_t : 1;
    bool pedometer_step_detected : 1;
    bool any_motion_detected : 1;
    bool no_motion_detected : 1;
    bool sig_motion_detected : 1;
  };
  uint8_t packed[1];
} status1_reg_t;

// IMU data (Accel: Ax_L .. Az_H, Gyro: Gx_L .. Gz_H)
typedef union {
  struct {
    int16_t raw_x : 16;
    int16_t raw_y : 16;
    int16_t raw_z : 16;
  };
  int16_t raw[3];
  uint8_t packed[6];
} imu_axis_data_t;

// Temperature data (Tempearture_L .. Tempearture_H)
typedef union {
  struct {
    uint8_t deg_c_fraction : 8;  // fractional portion (float) = deg_c_fraction / 256.0f
    uint8_t deg_c : 8;           // degC
  };
  int16_t raw;
  uint8_t packed[2];
} temperature_data_t;

// Ctrl9 payload structures
typedef struct {
  uint8_t : 4;
  uint8_t cmd_page : 4;  // usually 1 or 2 depending on the command
} ctrl9_cmd_info_t;

#pragma region Motion detection
typedef struct {
  int8_t int_part : 3;    // integer portion
  uint8_t dec_32nds : 5;  // + n/32.0 for the decimal
} slope_threshold_t;

typedef struct {
  slope_threshold_t x /* : 8 */;
  slope_threshold_t y /* : 8 */;
  slope_threshold_t z /* : 8 */;
} slope_thresholds_t;

enum MotionModeAxisLogic {
  OR = 0,
  AND = 1,
};

typedef struct {
  bool any_motion_x_en : 1;
  bool any_motion_y_en : 1;
  bool any_motion_z_en : 1;
  MotionModeAxisLogic any_motion_axis_logic : 1;
  bool no_motion_x_en : 1;
  bool no_motion_y_en : 1;
  bool no_motion_z_en : 1;
  MotionModeAxisLogic no_motion_axis_logic : 1;
} ctrl9_motion_mode_ctrl_t;

typedef struct /* 8 bytes */ {
  slope_thresholds_t any_motion_thr /* : 24 */;
  slope_thresholds_t no_motion_thr /* : 24 */;
  ctrl9_motion_mode_ctrl_t motion_mode_ctrl /* : 8 */;
  ctrl9_cmd_info_t cmd_info{.cmd_page = 1};  // NB: this won't be set automatically!
} ctrl9_cmd_motion_config_page1_t;

typedef struct /* 8 bytes */ {
  uint8_t any_motion_window : 8;
  uint8_t no_motion_window : 8;
  uint16_t sig_motion_wait_window : 16;
  uint16_t sig_motion_confirm_window : 16;
  // reserve the last 2 bytes for cmd_info
  ctrl9_cmd_info_t cmd_info{.cmd_page = 2};  // NB: this won't be set automatically!
} ctrl9_cmd_motion_config_page2_t;

#pragma endregion

#pragma region Tap detection

typedef struct {
  union {
    struct {
      TapStatusType type : 2;  // 0=none, 1=single, 2=double
      uint8_t : 2;
      TapStatusAxis axis : 2;  // 0=none, 1=x, 2=y, 3=z
      uint8_t : 1;
      TapStatusDirection polarity : 1;  // 1=negative
    };
    uint8_t packed[1];
  };
} qmi8658_tap_status_t;

#define QMI8658_1F7(x) \
  (one_byte_7bit_fraction_t) { \
    .div_128 = static_cast<uint8_t>(std::round((x - std::floor(x)) * 128)), \
    .int_part = static_cast<uint8_t>(std::floor(x)) \
  }

typedef struct {
  uint8_t div_128 : 7;   // + n/128.0 for the decimal
  uint8_t int_part : 1;  // integer portion
} one_byte_7bit_fraction_t;

#define QMI8658_2F10(x) \
  (two_byte_10bit_fraction_t) { \
    .div_1024 = static_cast<uint16_t>(std::round((x - std::floor(x)) * 1024)), \
    .int_part = static_cast<uint8_t>(std::floor(x)) \
  }

typedef struct {
  uint16_t div_1024 : 10;  // + n/1024.0 for the decimal
  uint8_t int_part : 6;    // integer portion
} two_byte_10bit_fraction_t;

// First command page for [QMI8658_Ctrl9_Cmd_Configure_Tap], written to CAL1-4
typedef struct {
  uint8_t peak_window : 8;  // This depends on ODR!
  TapAxisPriorityOrder priority : 8;
  uint16_t tap_window : 16;         // This depends on ODR!
  uint16_t double_tap_window : 16;  // This depends on ODR!
  uint8_t : 8;
  ctrl9_cmd_info_t cmd_info = {.cmd_page = 1};
} ctrl9_cmd_tap_config_page1_t;

// Second command page for [QMI8658_Ctrl9_Cmd_Configure_Tap], written to CAL1-4
typedef struct {
  one_byte_7bit_fraction_t alpha;
  one_byte_7bit_fraction_t gamma;
  two_byte_10bit_fraction_t peak_mag_thr /* : 16 */;
  two_byte_10bit_fraction_t udm_thr /* : 16 */;
  uint8_t : 8;
  ctrl9_cmd_info_t cmd_info = {.cmd_page = 2};
} ctrl9_cmd_tap_config_page2_t;

#pragma endregion

#pragma region Wake on Motion

typedef struct {
  uint8_t wom_threshold : 8;                         // mg
  QMI8658_Interrupt target_interrupt : 1;            // 0=INT2, 1=INT1
  QMI8658_InterruptState interrupt_start_level : 1;  // 0=low, 1=high
  uint8_t blanking_time : 6;                         // number of samples to ignore after enabling
  uint8_t : 8;
  uint32_t : 32;  // Not used for anything, just filling up the 8 bytes
  ctrl9_cmd_info_t cmd_info = {.cmd_page = 0};
} ctrl9_cmd_wake_on_motion_config_page_t;

#pragma endregion

// CTRL9 command parameters. These should be written to the CAL1-4 registers (8 bytes),
// along with the corresponding command into CTRL9 register.
// Many commands require multiple pages to be written in sequence.
typedef union {
  ctrl9_cmd_motion_config_page1_t motion_config_page1;
  ctrl9_cmd_motion_config_page2_t motion_config_page2;

  ctrl9_cmd_tap_config_page1_t tap_config_page1;
  ctrl9_cmd_tap_config_page2_t tap_config_page2;

  ctrl9_cmd_wake_on_motion_config_page_t wom_config_page;

  uint8_t packed[8];     // 8 bytes total
  uint16_t packed16[4];  // WARNING: endianness can be an issue here!
  uint32_t packed32[2];  // here too
} ctrl9_cmd_parameters_t;

}  // namespace qmi8658
}  // namespace esphome
