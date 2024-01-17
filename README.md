# Project Overview
A constant challenge in the field of technology that covers several sectors is constantly
developing - the design and implementation of mechatronic systems for industrial equipment
or process automation. The challenge for this project was to design and program a supplementary 5th wheel system for a wheelchair, featuring a brushless motor for enhanced mobility

# Requirements
![fw1](https://github.com/GerardoDC14/5thWheel_5thSemester/assets/123440177/c327ff9d-fa48-490c-ad37-0654b138b9e4)

# Solution 
Developed a wheel mechanism that includes a brushless DC motor, which is attached to the wheelchair. This setup is controlled by a user-friendly controller that uses sensors to help steer the wheelchair, making it easier for people with lower body mobility issues to move around. The mechanism also features a display that shows the current speed, and it has an encoder to adjust and control the speed more accurately.
![fw2](https://github.com/GerardoDC14/5thWheel_5thSemester/assets/123440177/32165e3b-c78e-42e4-8f03-f944379d6138)

# Block Diagram 
![image](https://github.com/GerardoDC14/5thWheel_5thSemester/assets/123440177/36439088-edaf-432a-93c4-470f54049488)

The STM32F0Discovery microcontroller is at the center of the operation. With 64KB of flash
memory and 8KB of RAM, this device should perform most motor control jobs, albeit the
code size and libraries to be built must be considered. Its ARM Cortex-M0 core ensures
efficiency and the capacity to manage real-time operations, which is critical in motor control
applications. It operates at a maximum frequency of 48MHz. I2C, SPI, and USART
communication interfaces provide flexibility for sensor and other module integration.
Furthermore, features such as the ADC and PWM outputs are critical for exact sensor
readings and comprehensive motor control.
The selection of a 48V battery is in accordance with the controller's specifications. Other
system components, on the other hand, demand various voltage values. The step-down
converters come into play here. The first cuts the voltage from 48V to 12V, making it
acceptable for devices that operate in this range. The second step-down, from 12V to 3.5V,
is used for powering the STM32F0Discovery microcontroller and guaranteeing steady and
safe operation.

# Mechanical Elements
![image](https://github.com/GerardoDC14/5thWheel_5thSemester/assets/123440177/916a852a-03e1-4da3-b7b4-406fd9321c66)

The base structure of the motorized wheel consists of two 1/4-inch thick aluminum plates,
which serve as the primary framework for the assembly. These plates are securely fastened together using aluminum or iron spacers distributed around their perimeter. The purpose of
these spacers is to provide added structural support and to serve as attachment points for
the covering plates that enclose the entire design.
In addition to the aluminum plates, two acrylic side plates are included in the design to
safeguard the internal components from environmental factors such as water infiltration and
potential damage. These acrylic plates are strategically placed to cover and protect the inner
workings of the motorized wheel.

# Development of control systems and features 
## Flowchart 
![image](https://github.com/GerardoDC14/5thWheel_5thSemester/assets/123440177/69b865aa-6c43-4b1e-8efa-d1b823a7f296)

## PWM control
In the code the motor's speed is adjusted based on the duty cycle of the PWM signal.
When the user interacts with the rotary encoder, it modifies the duty cycle of the PWM,
thereby changing the speed of the motor. A higher duty cycle means the motor runs faster,
while a lower duty cycle slows it down.
● Timer 1 (TIM1) is configured for PWM functionality in the setupTimer1PWM()
function.
● The auto-reload value (TIM1->ARR) is set to 79, which means the PWM signal will
complete its cycle every 80 counts (from 0 to 79).
● The capture/compare register (TIM1->CCR2) for channel 2 is set to determine the
duty cycle.
● The duty cycle is adjusted based on the rotary encoder value in the main loop.

## Speed reading 
Every rotation or fraction of a rotation produces a specific number of pulses. By counting
these pulses over a set period, we can determine the speed of the motor. RPM (Revolutions
Per Minute) is a standard measure of rotational speed. It denotes how many times the motor
completes a full rotation within a minute. By using the number of pulses counted within a
known time and knowing the pulses per revolution, the RPM can be calculated.
● Pulses are generated for each revolution of the motor, and these pulses are counted.
● Timer 2 (TIM2) is configured to generate periodic interrupts. In the
TIM2_IRQHandler() function, the RPM is calculated based on the number of pulses
counted.
