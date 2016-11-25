//
//  arm_math.h
//  BassSlab
//
//  Created by Chris Reed on 11/10/16.
//  Copyright © 2016 Immo Software. All rights reserved.
//

#ifndef arm_math_h
#define arm_math_h

#include <math.h>
#include <stdint.h>

#define PI (M_PI)

typedef float float32_t;

static inline float32_t arm_sin_f32(float32_t x)
{
    return sinf(x);
}

static inline float32_t arm_cos_f32(float32_t x)
{
    return cosf(x);
}

static inline void arm_sin_cos_f32(
  float32_t theta,
  float32_t * pSinVal,
  float32_t * pCosVal)
{
    *pSinVal = sinf(theta);
    *pCosVal = cosf(theta);
}

static inline void arm_fill_f32(
  float32_t value,
  float32_t * pDst,
  uint32_t blockSize)
{
    while (blockSize--)
    {
        *pDst++ = value;
    }
}

static inline void arm_scale_f32(
  float32_t * pSrc,
  float32_t scale,
  float32_t * pDst,
  uint32_t blockSize)
{
    while (blockSize--)
    {
        *pDst++ = *pSrc++ * scale;
    }
}

static inline void arm_mult_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  float32_t * pDst,
  uint32_t blockSize)
{
    while (blockSize--)
    {
        *pDst++ = (*pSrcA++) * (*pSrcB++);
    }
}

#endif /* arm_math_h */
