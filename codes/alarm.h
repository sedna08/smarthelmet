#include <stdio.h>
#include <iostream>
#include <pigpio.h>

#define PIN_PWM 12
#define PIN_BTN 23
#define PIN_TEMP_GND 24
#define PI_TIME_RELATIVE 0
#define PI_DUTYCYCLE 128 // set duty cycle here (0-255)

#define BUZZ_DURATION 20 // set buzz duration here (in seconds)
#define MUTE_DURATION_AFTER_BUZZ 5 // set mute duration after buzz here (in seconds)

#define DUTYCYCLE_INCR 0.9 // set increment amount for duty cycle
#define TRANSITION_TIME_S 1 // set transition time from low to high volume (in seconds)

using namespace std;

class Alarm
{
public:
    int state;
    Alarm()
    {
        state = 0;
        secs = 0, mics = 0;
        start_time = 0.0;
        stop_time = -99.0;
        is_start = true;
        dutycycle = 0;
    }

    void init()
    {
        if (gpioInitialise() < 0)
        {
            cout << "PWM initialization failed" << endl;
        }
        else
        {
            cout << "PWM initialization successfull" << endl;
        }
        gpioSetMode(PIN_BTN, 0);         // set pin for button (input)
        gpioSetMode(PIN_PWM, 1);         // set PWM pin as output
        gpioWrite(PIN_PWM, 0);           // write 0 to PWM output
        gpioSetPWMfrequency(PIN_PWM, 3); // set PWM freq to (1/300ms)

        gpioSetMode(PIN_TEMP_GND, 1);
        gpioWrite(PIN_TEMP_GND, 0);
    }

    // Method 1: Control volume using dutycycle (setting it from 0 to 128 or 0% to 50% dutycycle) 
    // (during vol fade in)
    //   - Duty cycle of 0% will produce no sound
    //   - Duty cycle of >=50% will produce max volume
    int sound1()
    {   
        gpioTime(PI_TIME_RELATIVE, &secs, &mics);
        float time_passed_since_start = (secs + (mics / (float)1000000)) - start_time;
        float time_passed_since_stop = (secs + (mics / (float)1000000)) - stop_time;

        // Check if button is pressed
        if (state == 0)
        {
            state = 1;
            //cout << "Action: Button Push" << endl;
            stop_time = secs + (mics / (float)1000000);
            is_start = true;
            gpioPWM(PIN_PWM, 0);
            return 0;
        }

        if (time_passed_since_stop >= MUTE_DURATION_AFTER_BUZZ && state == 1)
        {
            // Check if awake-drowsy pattern
            if (is_start == true)
            {
                // Start alarm
                //cout << "\t<Buzz>\t<Starts>" << endl;
                start_time = secs + (mics / (float)1000000);
                is_start = false;
                dutycycle = 0;
            }
            else if (is_start == false)
            {
                if (time_passed_since_start < BUZZ_DURATION)
                {
                    // Fade in volume
                    if (dutycycle < PI_DUTYCYCLE)
                    {
                        gpioPWM(PIN_PWM, (int) dutycycle);
                        dutycycle = dutycycle + DUTYCYCLE_INCR; // increment by N
                    }
                    // Sound alarm
                    else if (dutycycle >= PI_DUTYCYCLE)
                    {
                        gpioPWM(PIN_PWM, PI_DUTYCYCLE);
                    }
                }
                else if (time_passed_since_start >= BUZZ_DURATION)
                {
                    // Stop 
                    //cout << "\t<Buzz>\t<Stops>" << endl;
                    stop_time = secs + (mics / (float)1000000);
                    is_start = true;
                    gpioPWM(PIN_PWM, 0);
                    return 0;
                }
            }
        }

        if (time_passed_since_stop >= MUTE_DURATION_AFTER_BUZZ && state == 1)
        {
            //printf("s passed = %0.6f", time_passed_since_start);
            if (is_start == true)
            {
                
            }
            else if (is_start == false)
            {
                if (time_passed_since_start < BUZZ_DURATION)
                {
                    if (dutycycle < PI_DUTYCYCLE)
                    {
                        //cout << "\t<Buzz>\t<Fading in...>" << endl;
                    }
                    else if (dutycycle >= PI_DUTYCYCLE)
                    {
                        //cout << "\t<Buzz>" << endl;
                    }
                }
                else if (time_passed_since_start >= BUZZ_DURATION)
                {

                }
            }
        }
        else
        {
           // printf("s passed = %0.6f", time_passed_since_stop);
            //cout << "\t<Waiting...>" << endl;
        }

        return 0;
    }
    
    // Method 2: Use GPIO pin in-between resistors to control volume (during vol fade in)
    int sound2(int state)
    {
        gpioTime(PI_TIME_RELATIVE, &secs, &mics);
        float time_passed_since_start = (secs + (mics / (float)1000000)) - start_time;
        float time_passed_since_stop = (secs + (mics / (float)1000000)) - stop_time;

        // Check if button is pressed
        if (gpioRead(23) == 1)
        {
            //cout << "Action: Button Push" << endl;
            state = 0;
            stop_time = secs + (mics / (float)1000000);
            is_start = true;
            gpioPWM(PIN_PWM, 0);
            return 0;
        }

        if (time_passed_since_stop >= MUTE_DURATION_AFTER_BUZZ && state == 1)
        {
            if (is_start == true)
            {
                //cout << "<First 'drowsy' state after last 'awake' state is detected.>" << endl;
                start_time = secs + (mics / (float)1000000);
                is_start = false;
                gpioSetMode(PIN_TEMP_GND, 0); // set to high impedance
                gpioPWM(PIN_PWM, PI_DUTYCYCLE); // sound buzzer
            }
            else if (is_start == false)
            {
                if (time_passed_since_start < BUZZ_DURATION)
                {
                    // Low to high vol transition time of N seconds
                    if (time_passed_since_start < TRANSITION_TIME_S)
                    {
                        gpioSetMode(PIN_TEMP_GND, 0); // set GPIO to high impedance
                        gpioPWM(PIN_PWM, PI_DUTYCYCLE); // sound buzzer
                    }
                    else if (time_passed_since_start >= TRANSITION_TIME_S)
                    {
                        gpioSetMode(PIN_TEMP_GND, 1); // set GPIO as input
                        gpioWrite(PIN_TEMP_GND, 0); // output GPIO as ground (0)
                        gpioPWM(PIN_PWM, PI_DUTYCYCLE); // sound buzzer
                    }
                }
                else if (time_passed_since_start >= BUZZ_DURATION)
                {
                    stop_time = secs + (mics / (float)1000000);
                    is_start = true;
                    gpioPWM(PIN_PWM, 0);
                    return 0;
                }
            }
        }

        //printf("s passed = %0.6f", time_passed_since_start);
        if (time_passed_since_stop >= MUTE_DURATION_AFTER_BUZZ && state == 1)
        {
           // cout << "\t<Buzz>" << endl;
        }
        else
        {
            //cout << "\t<Waiting...>" << endl;
        }

        return 0;
    }

    void loop()
    {
        gpioTime(PI_TIME_RELATIVE, &secs, &mics);
        float time_passed_since_start = (secs + (mics / (float)1000000)) - start_time;
        gpioPWM(PIN_PWM, PI_DUTYCYCLE);
        
        for (int brightness = 0; brightness <= PI_DUTYCYCLE; brightness += 1) {
            gpioPWM(PIN_PWM, brightness);
            time_sleep(0.03); // Wait for 30 millisecond(s)
        }
        for (int brightness = PI_DUTYCYCLE; brightness >= 0; brightness -= 1) {
            gpioPWM(PIN_PWM, brightness);
            time_sleep(0.03); // Wait for 30 millisecond(s)
        }
        
        printf("s passed = %0.6f\n", time_passed_since_start);
    }
    
private:
int secs, mics;
float start_time; // timestamp when an 'awake-drowsy' sequence happens
float stop_time; //  timestamp when a 'drowsy-awake' sequence happens
bool is_start;
float dutycycle; // can be used to control volume
};