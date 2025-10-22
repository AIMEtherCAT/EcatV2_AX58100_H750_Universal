//
// Created by Hang XU on 20/04/2025.
//

#ifndef DSHOT_H
#define DSHOT_H

#include "tim.h"



#ifdef __cplusplus
extern "C" {
#endif

void init_dshot1();

void init_dshot2();

void write_dshot1(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4);

void write_dshot2(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4);

#ifdef __cplusplus
}
#endif

#endif //DSHOT_H