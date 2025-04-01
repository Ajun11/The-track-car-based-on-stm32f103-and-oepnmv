

# :car:A Vision-Based Tracking Vehicle Utilizing OpenMV And STM32

This project pertains to a class design that I originally published on GitHub three years ago. Recently, I observed that both my peers and I might benefit from this small-scale demonstration, prompting me to update it while reflecting on the joy and fulfillment I experienced during its initial development. 

Note: This demonstration represents a fundamental project that is straightforward to implement, from this little project, you can get:

- **Gain a deeper understanding of OpenMV and STM32**, particularly the communication mechanisms that facilitate data exchange between these platforms.
- **Know more about control,** by tuning the PD control algorithm, develop an intuitive understanding of control theory, such as how a larger Kp value may induce jitter in a two-axis gimbal, thereby affecting system stability.
- **Make it!** Finally, I would like to say that starting with foundational concepts can foster greater confidence in your abilities. By actively engaging in problem-solving and developing solutions independently, you can enhance your coding skills. I hope this demo inspires you to create your own intelligent small-scale vehicle.

## 📋 Getting Started

This project requires hardware components, and prior to commencing, you may need to prepare the following hardware:

1. An OpenMV camera module (e.g., OpenMV Cam H7) for vision-based target detection and control for two-axes gimbal with sg90 (you can but two-axes gimbal with OpenMV together).

2. A microcontroller development board (e.g., STM32F103C8T6 is enough, you can select arundio or others) to serve as the main control unit for the vehicle.

3. A chassis kit (contain motor is better, DC motor is ok and you better choose a motor driver like L298N to achieve contorl for motor) for assembling the small-scale tracking vehicle.

4. A power supply, such as a rechargeable LiPo battery (e.g., 7.4V, 1000mAh), to power the system.

   dupont jumper wires, a breadboard, and other basic electronic components for circuit connections.

### Installation 

You can clone this repo from github or download zip file directly:

```shell
git clone https://github.com/Ajun11/The-track-car-based-on-stm32f103-and-oepnmv.git
```

The downloaded file have the following structure:

```shell
 openmv_part\  --- target detection and control for two-axes gimbal
 stm32_part\   --- samll vehicle control
    |_  Basic_Header\  --- include important header file such as motor and usart header
    |_  Libraries\     --- stm32 header file...
    |_  project\       --- include the keil proj, you can open it in keil 5
    |_  user\main.c    --- the main file of running
```

After addressing the software implementation, the next step is to establish the connections between the STM32 and OpenMV modules, with detailed instructions to be provided in the subsequent sections.

```shell
STM32 PA2 <---> OpenMV P5
STM32 PA3 <---> OpenMV P4
```

Subsequently, you can program the respective boards by uploading the code: the STM32 can be programmed using the Keil IDE, while the OpenMV module can be programmed via the OpenMV IDE. I think there will be some problem, the best way is understand the principle and slove the problem.

## :question: You can know more in `more.pdf` file. 
