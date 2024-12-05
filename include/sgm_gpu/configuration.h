#ifndef SGM_GPU__CONFIGURATION_H_
#define SGM_GPU__CONFIGURATION_H_

#include <stdint.h>

#define	MAX_DISPARITY 128
#define CENSUS_WIDTH  9
#define CENSUS_HEIGHT 7

#define TOP  (CENSUS_HEIGHT-1)/2
#define LEFT (CENSUS_WIDTH-1)/2

namespace sgm_gpu
{

typedef uint32_t cost_t;

}

#define COSTAGG_BLOCKSIZE       GPU_THREADS_PER_BLOCK
#define COSTAGG_BLOCKSIZE_HORIZ GPU_THREADS_PER_BLOCK

#endif // SGM_GPU__CONFIGURATION_H_
