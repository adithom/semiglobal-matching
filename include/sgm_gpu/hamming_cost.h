#ifndef SGM_GPU__HAMMING_COST_H_
#define SGM_GPU__HAMMING_COST_H_

#include "sgm_gpu/configuration.h"
#include "sgm_gpu/util.h"
#include <stdint.h>

namespace sgm_gpu
{

__global__ void
HammingDistanceCostKernel(const cost_t *d_transform0, const cost_t *d_transform1, uint8_t *d_cost, const int rows, const int cols );

}

#endif // SGM_GPU__HAMMING_COST_H_ 

