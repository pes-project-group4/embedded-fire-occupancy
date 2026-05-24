#ifndef _MLX90614_H_
#define _MLX90614_H_

#include <stdint.h>

struct mlx90614_sample {
    int32_t ambient_centi_c;
    int32_t object_centi_c;
};

int mlx90614_init(void);

int mlx90614_read(struct mlx90614_sample *sample);

#endif
