/*
    Copyright 2019 Benjamin Vedder  benjamin@vedder.se

    This file is part of the VESC firmware.

    The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"


// Some useful includes
#include "mc_interface.h"
#include "utils.h"
#include "encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

#include <math.h>
#include <string.h>
#include <stdio.h>


// Custom ressources

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 2048);

// Private functions
static void pwm_callback(void);
static void terminal_test(int argc, const char **argv);
// Private custom functions
float min(float num1, float num2);
float max(float num1, float num2);
float constrain(float input, float val1, float val2);
float erpmToKmH (float erpm);

void deck_led_lightBlue(void);
void deck_led_green(void);
void deck_led_yellow(void);
void deck_led_white(void);

float compute_pid(float error);
void reset_pid(void);

// ------------------ Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

 // --- Control settings

#ifndef SPEED_GAP_FACTOR
#define SPEED_GAP_FACTOR 1.5
#endif

#ifndef SPEED_MINI_ASSIST
#define SPEED_MINI_ASSIST 4
#endif

#ifndef SPEED_MAX_ASSIST
#define SPEED_MAX_ASSIST 18
#endif

#ifndef SPEED_MAX_CONTROL
#define SPEED_MAX_CONTROL 50
#endif

 int twoFeetOn;
 const float operatingAngleWindow[2] = {-15,15};

 const unsigned long timingDelayBeforeAssist = 500;  // How long do we wait before using the motor again after a braking session
 unsigned long lastPowerInterrupt;  // That could be due to braking session or feet removal

 const unsigned long easeOutDuration = 400; //smooth the signal when suddenly switching to 0 assistance
 unsigned long lastAssistTime;  // Store the ms value of when was the last time we where in assist mode

 int last_AUTOMATIC_Duty_PPM_PID;

 // -- PID init

 float speedTarget=0;
 float currentSpeed;

 float pid_current_control;

 const float Kp_cur_cntrl = 2;
 const float Kd_cur_cntrl = 0.4;
 const float Ki_cur_cntrl = 0.015;
 const int limits = MCCONF_L_CURRENT_MAX;  // Take the setting from motor config
 const float maxi_I = 5*MCCONF_L_CURRENT_MAX;

 float _sumI = 0;
 float _prevError = 0;

 // -- Smoothing ERPM Input Value
 float prevErpm = 0;
 const float erpm_rc = 0.5; //

 // Smoothing angle Input Value with lpf struct
// float angle_rc = 0.5;
// LPF angle_lpf1(&angle_rc);
// bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
 float currentBoardAngle;


// Called when the custom application is started. Start our
// threads here and set up callbacks.

// ------- PIN DEFINITION

 // We use SERVO as a digital input for the foot sensor (GPIOB 5)
 #define PORT_FOOTSENSOR GPIOB
 #define PIN_FOOTSENSOR 5

 // THE Green LED is plugged to 3V and is always on

 // We use ADC EXT as an output to drive the led BLUE (GPIOA 5)
 #define PORT_LED_BLUE_DECK HW_SPI_PORT_SCK
 #define PIN_LED_BLUE_DECK HW_SPI_PIN_SCK

// We use ADC EXT 2 as an output for RED LED (GPIOA 6)
#define PORT_LED_RED_DECK HW_SPI_PORT_MISO
#define PIN_LED_RED_DECK HW_SPI_PIN_MISO


void app_custom_start(void) {


   // Set the MISO pin as an input with pulldown
    palSetPadMode(PORT_FOOTSENSOR, PIN_FOOTSENSOR, PAL_MODE_INPUT_PULLDOWN);

    palSetPadMode(PORT_LED_BLUE_DECK, PIN_LED_BLUE_DECK,
                PAL_MODE_OUTPUT_PUSHPULL |
                PAL_STM32_OSPEED_HIGHEST);

    palSetPadMode(PORT_LED_RED_DECK, PIN_LED_RED_DECK,
                    PAL_MODE_OUTPUT_PUSHPULL |
                    PAL_STM32_OSPEED_HIGHEST);

    deck_led_lightBlue();

    mc_interface_set_pwm_callback(pwm_callback);

    stop_now = false;

    // Start the example thread
    chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
            NORMALPRIO, my_thread, NULL);

    // Terminal commands for the VESC Tool terminal can be registered.
    terminal_register_command_callback(
            "custom_cmd",
            "Print the number d",
            "[d]",
            terminal_test);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.

void app_custom_stop(void) {
    mc_interface_set_pwm_callback(0);
    terminal_unregister_callback(terminal_test);

    stop_now = true;
    while (is_running) {
        chThdSleepMilliseconds(1);
    }
}

void app_custom_configure(app_configuration *conf) {
    (void)conf;
}

static THD_FUNCTION(my_thread, arg) {
    (void)arg;

    chRegSetThreadName("App SensorStableSpeed");

    is_running = true;

    // USEFUL FUNCTION
    // float rpm_local = mc_interface_get_rpm();

//    void mc_interface_set_duty(float dutyCycle);
//    void mc_interface_set_duty_noramp(float dutyCycle);
//    void mc_interface_set_    pid_speed(float rpm);
//    void mc_interface_set_pid_pos(float pos);
//    void mc_interface_set_current(float current);
//    void mc_interface_set_brake_current(float current);
//    void mc_interface_set_current_rel(float val);
//    void mc_interface_set_brake_current_rel(float val);
//    void mc_interface_set_handbrake(float current);
//    void mc_interface_set_handbrake_rel(float val);
//    int mc_interface_set_tachometer_value(int steps);
//    void mc_interface_brake_now(void);
//    void mc_interface_release_motor(void);

    commands_init_plot("Speed", "PID");
    commands_plot_add_graph("Temp Fet");
    commands_plot_add_graph("Input Voltage");
    commands_plot_add_graph("Target speed");

    float samp = 0.0;

    for(;;) {
        // Check if it is time to stop.
        if (stop_now) {
            is_running = false;
            return;
        }

        timeout_reset(); // Reset timeout if everything is OK.

        // Run your logic here. A lot of functionality is available in mc_interface.h.

        // -- 0 CHECK THE Current status

        unsigned long now = chVTGetSystemTimeX();
        twoFeetOn = palReadPad(PORT_FOOTSENSOR, PIN_FOOTSENSOR);
        currentBoardAngle = 0;

        // -- 1 WE RUN THE SIGNAL PROCESSING AND SMOOTHING

        prevErpm = prevErpm * (1-erpm_rc) + mc_interface_get_rpm()*erpm_rc; // ERPM smoothing
        currentSpeed = erpmToKmH(prevErpm);

        // - 2 detect if acceleration  and update target speed value if necessary
        if (((currentSpeed)>(speedTarget+SPEED_GAP_FACTOR))&&(currentSpeed>SPEED_MINI_ASSIST)&&twoFeetOn&&(currentSpeed<SPEED_MAX_CONTROL)&&((currentBoardAngle>operatingAngleWindow[0])&&(currentBoardAngle<operatingAngleWindow[1]))){  // analyse + safety coef
              speedTarget=min(currentSpeed,SPEED_MAX_ASSIST);

//              Duty_PPM_PID = forward_pid.compute(speedTarget-currentSpeed);
//              Duty_PPM_PID=constrain(Duty_PPM_PID,0,limits);
            }

        pid_current_control = compute_pid(speedTarget-currentSpeed);
        pid_current_control = constrain(pid_current_control,0,limits);

        //        commands_printf("%.2f", (pid_current_control));
        //        commands_printf("-------");

        // - 4 We COntrol or not the motor
        // - 4.1 FREE WHEEL when sensor is not triggered


        if (!twoFeetOn){
          speedTarget=0;
          reset_pid();
          mc_interface_release_motor();
          LED_RED_OFF();
          deck_led_lightBlue();
          // LED PINK
        }

        // - 4.2 Two feet on and in the control window of speed and angle => Current delivery
        else if((currentSpeed<SPEED_MAX_ASSIST)&&(speedTarget>SPEED_MINI_ASSIST)){
          // LED GREEN
          LED_RED_OFF();
          mc_interface_set_current(pid_current_control);
          deck_led_green();

        }
        // - 3.3 Two feet on but one condition to deliver power is missing => We warn the user the feet os on (led color)
        //      but no power delivered
        else{
          // LED BLUE
          LED_RED_ON();
          mc_interface_release_motor();
          deck_led_yellow();
        }

        // -- END PROG

        // --- Graph debug

        commands_plot_set_graph(0);
        commands_send_plot_points(samp, currentSpeed);
        commands_plot_set_graph(1);
        commands_send_plot_points(samp, pid_current_control);
        commands_plot_set_graph(2);
        commands_send_plot_points(samp, speedTarget);
        samp++;

        //

        chThdSleepMilliseconds(2);
    }
}

// ---------- CUSTOM FUNCTION FOR THE PROGRAM

float compute_pid(float error)
{
  float out = error *Kp_cur_cntrl + (error - _prevError) *Kd_cur_cntrl + _sumI *Ki_cur_cntrl;

  _prevError = error;
  _sumI = constrain(_sumI + error, -maxi_I, maxi_I);

  return out;
}

void reset_pid(void)
{
  _sumI = 0;
  _prevError = 0;
}

void deck_led_lightBlue(void)
{
  palClearPad(PORT_LED_RED_DECK, PIN_LED_RED_DECK);
  palSetPad(PORT_LED_BLUE_DECK, PIN_LED_BLUE_DECK);
}

void deck_led_green(void)
{
  palClearPad(PORT_LED_BLUE_DECK, PIN_LED_BLUE_DECK);
  palClearPad(PORT_LED_RED_DECK, PIN_LED_RED_DECK);
}

void deck_led_yellow(void)
{
  palClearPad(PORT_LED_BLUE_DECK, PIN_LED_BLUE_DECK);
  palSetPad(PORT_LED_RED_DECK, PIN_LED_RED_DECK);
}

void deck_led_white(void)
{
  palSetPad(PORT_LED_BLUE_DECK, PIN_LED_BLUE_DECK);
  palSetPad(PORT_LED_RED_DECK, PIN_LED_RED_DECK);
}

float max(float num1, float num2)
{
    return (num1 > num2 ) ? num1 : num2;
}

float min(float num1, float num2)
{
    return (num1 < num2 ) ? num1 : num2;
}

float constrain (float input, float val1, float val2){
  float low_val = min(val1,val2);
  float high_val = max(val1,val2);

  if (input>low_val){
    return min(input,high_val);
  }
  else{
    return low_val;
  }
}

float erpmToKmH (float erpm){
  //Round the number at 0.1
  //float speedValue = ((DiamRoue*3.1415/1000000)*((erpm/MagnetFactor)*gearRatio)*60); // convert erpm motor in km/h of the wheel
  float speedValue = ((MCCONF_SI_WHEEL_DIAMETER*3.1415/1000)*((erpm/MCCONF_SI_MOTOR_POLES)*MCCONF_SI_GEAR_RATIO)*60);
  speedValue = (round(speedValue*100));
  speedValue = speedValue/100;
  return speedValue;
}

// END of Custom function

static void pwm_callback(void) {
    // Called for every control iteration in interrupt context.
}

// Callback function for the terminal command with arguments.
static void terminal_test(int argc, const char **argv) {
    if (argc == 2) {
        int d = -1;
        sscanf(argv[1], "%d", &d);

        commands_printf("You have entered %d", d);

        // For example, read the ADC inputs on the COMM header.
        commands_printf("ADC1: %.2f V ADC2: %.2f V",
                (double)ADC_VOLTS(ADC_IND_EXT), (double)ADC_VOLTS(ADC_IND_EXT2));
    } else {
        commands_printf("This command requires one argument.\n");
    }
}
