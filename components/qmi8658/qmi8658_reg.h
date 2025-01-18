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
    bool _reserved : 1;   // bit 1 is unused
    bool fifo_sel : 1;    // 0=int2, 1=int1
    bool int1_en : 1;     // int1: 0=highz, 1=output
    bool int2_en : 1;     // int2: 0=highz, 1=output
    bool endianness : 1;  // 0=little endian, 1=big endian
    bool addr_ai : 1;     // address auto increment (needed for burst reads)
    bool spi_3wire : 1;   // 0=4-wire, 1=3-wire
  } bits;
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
    uint8_t _reserved1 : 1;
    bool gyro_lpf_en : 1;
    QMI8658_LpfMode gyro_lpf_mode : 2;
    uint8_t _reserved2 : 1;
  };
  uint8_t packed[1];
} ctrl5_reg_t;

// CTRL7 (0x08)
typedef union {
  struct {
    bool accel_en : 1;
    bool gyro_en : 1;
    uint8_t _reserved1 : 2;
    bool gyro_snooze : 1;
    bool data_ready_disable : 1;  // 0=enable DRDY with INT2, 1=disable
    uint8_t _reserved2 : 1;
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
    uint8_t _reserved : 1;
    uint8_t int_sel : 1;         // Interrupt for the motion/tap detections 0=INT2, 1=INT1
    uint8_t handshake_type : 1;  // Ctrl9 handshake type 0=INT1, 1=status_reg_t.bit7
  };
  uint8_t packed[1];
} ctrl8_reg_t;

// TODO: CTRL9 (0x0A)

// STATUSINT (0x2d)
typedef union {
  struct {
    bool is_available : 1;  // ctrl7.sync_sample: data avilable; else mirrors int2
    bool is_locked : 1;     // ctrl7.sync_sample: data locked; else mirrors int1
    uint8_t _reserved : 5;
    bool done : 1;  // ctrl9 command is done
  };
  uint8_t packed[1];
} statusint_reg_t;

// STATUS0 (0x2e)
typedef union {
  struct {
    bool accel_data_ready : 1;
    bool gyro_data_ready : 1;
    uint8_t _reserved : 6;
  };
  uint8_t packed[1];
} status0_reg_t;

// STATUS1 (0x2f)
typedef union {
  struct {
    uint8_t _reserved1 : 1;
    bool tap_detected : 1;
    bool wom : 1;  // wake on motion detected
    uint8_t _reserved2 : 1;
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
    int8_t deg_c_fraction : 8;  // fractional portion (float) = deg_c_fraction / 256.0f
    int8_t deg_c : 8;           // degC
  };
  int16_t raw;
  uint8_t packed[2];
} temperature_data_t;

}  // namespace qmi8658
}  // namespace esphome
