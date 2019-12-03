#include "stm32f30x_conf.h"
#include "gpio.h"

int16_t balance_P(){
    int16_t current_position = 0;
    int16_t target_position = 0;
    int16_t error;
    int16_t final_data;
    double kp = 0.6;
    current_position = accel_getValues();
    error = current_position - target_position;

    final_data = (kp * error);
    if (final_data > 128) final_data = 128;
    else if (final_data < -128) final_data = -128;
    return final_data;
}

int16_t balance_PI(){
    int16_t current_position = 0;
    int16_t target_position = 0;
    int16_t error;
    int16_t final_data;
    double kp = 0.6;
    double Ki = 5;
    int16_t integral = 1;

    current_position = accel_getValues();
    error = current_position - target_position;

    integral = integral * error;
    final_data = (kp * error) * (Ki * integral);
    if (final_data > 128) final_data = 128;
    else if (final_data < -128) final_data = -128;
    return final_data;
}

int16_t balance_PID(){
    int16_t current_position = 0;
    int16_t target_position = 0;
    int16_t error = 128;
    int16_t last_error = 128;
    int16_t final_data;
    double kp = 0.6;
    double Ki = 2.0;
    double Kd = 1.0;
    int16_t integral = 1;
    int16_t derivative = 0;

   /*while(error>50){
        current_position = accel_getValues();
        error = current_position - target_position;

        integral = integral * error;
        derivative = error - last_error;
        final_data = (kp * error) * (Ki * integral) + (Kd * derivative);

        last_error = error;

    }*/

    for(int i = 0; i<2; i++){
        current_position = accel_getValues();
        if(i == 0) last_error = current_position - target_position;
        if(i == 1) error = current_position - target_position;

        integral = integral * error;
        derivative = error - last_error;
    }
    final_data = (kp * error) * (Ki * integral) + (Kd * derivative);


    if (final_data > 128) final_data = 128;
    else if (final_data < -128) final_data = -128;
    return final_data;
}

void balance_weight()
{
    int16_t final_data, dutycykle1, dutycykle2;

    //final_data = accel_average();
    final_data = balance_PID();


    dutycykle1 = 128 + final_data;
    dutycykle2 = 255 - dutycykle1;

    TIM16->CCR1 = dutycykle1;
    TIM17->CCR1 = dutycykle2;
}
