#!/usr/bin/env python3

import bluerobotics_navigator as navigator


def test_set_pwm_channel_0():
    """Test script to set PWM signal for channel 0 to 0.5 using bluerobotics_navigator."""
    
    # Initialize the navigator
    print("Initializing Blue Robotics Navigator...")
    navigator.init()
    
    # Set PWM frequency (50 Hz is typical for servos/thrusters)
    pwm_frequency = 50
    navigator.set_pwm_freq_hz(pwm_frequency)
    print(f"PWM frequency set to {pwm_frequency} Hz")
    
    # Enable PWM output
    navigator.set_pwm_enable(True)
    print("PWM output enabled")
    
    # Set channel 0 to 0.5 (50% duty cycle) and all other channels to 0.0
    channels = [0, 1, 2, 3, 4, 5, 6, 7]
    pwm_values = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    print(f"Setting PWM values: {pwm_values}")
    navigator.set_pwm_channels_duty_cycle_values(channels, pwm_values)
    print("Channel 0 set to 0.5 (50% duty cycle)")
    print("All other channels set to 0.0")
    
    print("\nPWM signal successfully set!")
    print("Press Ctrl+C to stop and disable PWM...")
    
    try:
        # Keep the script running to maintain the PWM signal
        import time
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nDisabling PWM...")
        navigator.set_pwm_enable(False)
        print("PWM disabled. Exiting.")


if __name__ == '__main__':
    test_set_pwm_channel_0()
