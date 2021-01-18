/**
 * @file  tracking_loop_filter.h
 * @date  18-January-2021
 * @brief Implementation of tracking loop filter.
 *
 * Inspiration: https://www.embeddedrelated.com/showarticle/530.php 
 *
 * @author
 * @author
 */


#ifdef __cplusplus
extern "C" {
#endif


#ifndef TRACKING_LOOP_FILTER_H
#define TRACKING_LOOP_FILTER_H


#include <stdint.h>


typedef struct
{
  float Kp;         /*!< Proportional gain */
  float Ki;         /*!< Integral gain */
  float Dt;         /*!< Time between samples */
  float tpulses;
  float fpulses;
  float Error;      /*!< Loop error */
  float AccError;   /*!< Cumulative error */
  float DiffOutAcc; /*!< Output from accumulator */
  float DiffOutput; /*!< Derivative of filter output */
  float Output;     /*!< Filter output */
  float Input;      /*!< Unfiltered input */
}TrackingLoopFilter_t;

typedef struct
{
  float Kp;         /*!< Proportional gain */
  float Ki;         /*!< Integral gain */
  float Dt;         /*!< Time between samples */
  int64_t tpulses;
  int64_t fpulses;
  int64_t Error;    /*!< Loop error */
  int64_t AccError; /*!< Cumulative error */
  float DiffOutAcc; /*!< Output from accumulator */
  float DiffOutput; /*!< Derivative of filter output */
  int64_t Output;   /*!< Filter output */
  int64_t Input;    /*!< Unfiltered input */
}TrackingLoopFilterInt_t;


void TrackingLoopFilter_Init(TrackingLoopFilter_t *Parameters, float Kp, float Ki, float Dt);
float TrackingLoopFilter_Run(TrackingLoopFilter_t *Parameters, float Input);

void TrackingLoopFilterInt_Init(TrackingLoopFilterInt_t *Parameters, float Kp, float Ki, float Dt);
float TrackingLoopFilterInt_Run(TrackingLoopFilterInt_t *Parameters, int64_t Input);



#ifdef __cplusplus
}
#endif

#endif /* TRACKING_LOOP_FILTER_H */
