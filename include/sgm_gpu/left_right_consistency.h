#ifndef SGM_GPU__LEFT_RIGHT_CONSISTENCY_H_
#define SGM_GPU__LEFT_RIGHT_CONSISTENCY_H_

#include <stdint.h>

namespace sgm_gpu
{

__global__ void ChooseRightDisparity(uint8_t *right_disparity, const uint16_t *smoothed_cost, const uint32_t rows, const uint32_t cols);
__global__ void LeftRightConsistencyCheck(uint8_t *disparity, const uint8_t *disparity_right, const uint32_t rows, const uint32_t cols);

}

#endif // SGM_GPU__LEFT_RIGHT_CONSISTENCY_H_

