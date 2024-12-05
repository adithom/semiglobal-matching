#ifndef SGM_GPU__COSTS_H_
#define SGM_GPU__COSTS_H_

#include <stdint.h>
#include "sgm_gpu/configuration.h"

namespace sgm_gpu
{

__global__ void CenterSymmetricCensusKernelSM2(const uint8_t *im, const uint8_t *im2, cost_t *transform, cost_t *transform2, const uint32_t rows, const uint32_t cols);

}

#endif // SGM_GPU__COSTS_H_

