/* 
 * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
 * Created by Denys Sene, January 1, 2017.
 * Released under MIT License - see LICENSE file for details.
 */

#include "Arduino.h"
#include "SimpleKalmanFilter.h"
#include <cmath> // Changed from math.h to cmath

SimpleKalmanFilter::SimpleKalmanFilter(float mea_e, float est_e, float q)
{
  _err_measure = mea_e;
  _err_estimate = est_e;
  _q = q;
}

float SimpleKalmanFilter::updateEstimate(float mea)
{
  _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
  _err_estimate = (1.0 - _kalman_gain) * _err_estimate + std::fabs(_last_estimate - _current_estimate) * _q; // Changed from fabs to std::fabs
  _last_estimate = _current_estimate;

  return _current_estimate;
}

void SimpleKalmanFilter::setMeasurementError(float mea_e)
{
  _err_measure = mea_e;
}

void SimpleKalmanFilter::setEstimateError(float est_e)
{
  _err_estimate = est_e;
}

void SimpleKalmanFilter::setProcessNoise(float q)
{
  _q = q;
}

float SimpleKalmanFilter::getKalmanGain() {
  return _kalman_gain;
}
