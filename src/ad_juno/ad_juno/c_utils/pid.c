#include <stdint.h>

#define MAX_RANGE 19911
#define INTEGRAL_THRESHOLD 9955.0 // around %50 of full microstep range
// we should also add a MAX_INTEGRAL after tuning ki
// MAX_INTEGRAL = MAX_RANGE / ki

/*
  10 degrees MAX ANGLE, 1.8 per full step, Transmission ratio =  14
  microstep per degre:
  (1/1.8)*256*14 ~= 19911
*/

typedef struct {
  double kp, ki, kd;
  double integral;
  double last_error;

} PIDController;

void pid_init(PIDController *pid, double kp, double ki, double kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0.0;
  pid->last_error = 0.0;
}

double pid_compute(PIDController *pid, double target, double actual,
                   double dt) {
  double err = target - actual;
  if (err > INTEGRAL_THRESHOLD || err < -INTEGRAL_THRESHOLD) {
    pid->integral = 0;
  }
  pid->integral += err * dt;
  double derivative = (dt > 0.0) ? (err - pid->last_error) / dt : 0.0;
  pid->last_error = err;
  double result =
      pid->kp * err + pid->ki * pid->integral + pid->kd * derivative;
  return result;
}
