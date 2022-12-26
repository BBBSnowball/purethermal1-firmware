#ifndef MLX90614_I2C_H_
#define MLX90614_I2C_H_

#if defined(MLX90614)

#include <stdbool.h>
#include <stdint.h>

#define MLX90614_TAMBIENT   0x06
#define MLX90614_TOBJ       0x07
#define MLX90614_EEPROM0    0x20
#define MLX90614_EEPROM1    0x21

#define MLX90614_I2CADDR (0x5a<<1)

#define MLX90614_COMM_TIMEOUT 500

#define MLX90614_INVALID_MILLI_CELSIUS -420000  // -420 is physically impossible.

extern bool want_mlx90614_value;

bool has_last_milli_celsius(void);
long get_last_milli_celsius(void);
long get_milli_celsius(void);
int convert_C_to_F(int C);
int read_mlx90614_regs(bool force);
#endif

#endif

