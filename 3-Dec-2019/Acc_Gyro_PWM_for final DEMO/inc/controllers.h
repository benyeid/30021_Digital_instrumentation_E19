#include "stm32f30x_conf.h"
#include "gpio.h"

/// \brief The goal of this function is to balance the measurement system
/// using only a P-controller.
/// Depends on the \a accel_getValues() \a function for input data
/// \todo Kp value is hard-coded in the system, should be moved to parameter
/// \param Kp value of the controller function
/// \return (kp * error) adjusted. If larger than 128 than convert
///  it to fixed max value of 128. Same for smaller than -128
int16_t balance_P();

/// \brief This function shall balance the system using PI controller.
/// At the moment this seems to be the most effective.
/// Depends on the \a accel_getValues() \a function for input data (current_position)
/// \todo Move Kp and Ki values to parameters. Currently they are hard-coded:
///  Kp = 0.6, Ki = 5 (to be decreased).
/// \return (kp * error) * (Ki * integral) where
/// - error = current_position - target_position
/// - integral = integral * error;
/// Result is adjusted: If larger than 128 than convert it to fixed max value of 128.
/// Same for smaller than -128
int16_t balance_PI();

/// \brief This function shall balance the system using PID controller.
/// Depends on the \a accel_getValues() \a function for input data (current_position)
/// \todo Move Kp, Ki and Kd values to parameters. Currently they are hard-coded: Kp = 0.6, Ki = 2, Kd = 1.0 .
/// \return (kp * error) * (Ki * integral) + (Kd * derivative) where
/// - error = current_position - target_position
/// - integral = integral * error;
/// - derivative = error - last_error;
/// Resutlt is adusted: If larger than 128 than convert it to fixed max value of 128.
/// Same for smaller than -128
int16_t balance_PID();

/// \brief This function is used to balance the measurement sytem using PID controller
/// It calls the \a balance_PID() \a function and calculates the following two duty cycles:
///     -dutycykle1 = 128 + final_data;
///    - dutycykle2 = 255 - dutycykle1;
/// Then Timer 16 gets dutycykle1  and Timer 17 gets dutycykle2
void balance_weight();
