/**
 * SC2107 RSLK Robot - COMPLETE ASSESSMENT REFERENCE
 *
 * This file contains EVERYTHING from all previous code snippets
 * organized and ready to use for your lab assessment.
 *
 * TABLE OF CONTENTS:
 * 1. Global Variables & Configurations
 * 2. Interrupt Service Routines (ISRs)
 * 3. Complete System Initialization
 * 4. Basic Helper Functions (LEDs, Motors, Sensors)
 * 5. L-Task Functions (Simple, Single Module)
 * 6. M-Task Functions (Medium, Two Modules)
 * 7. H-Task Functions (Complex, Algorithms)
 * 8. Interrupt-Based Solutions
 * 9. Test & Debug Functions
 * 10. Main Function with Menu System
 */

#include "msp.h"
#include <stdint.h>
#include <math.h>
#include "../inc/Clock.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/PWM.h"
#include "../inc/Reflectance.h"
#include "../inc/Bump.h"
#include "../inc/BumpInt.h"
#include "../inc/IRDistance.h"
#include "../inc/ADC14.h"
#include "../inc/LPF.h"
#include "../inc/TimerA1.h"
#include "../inc/SysTickInts.h"
#include "../inc/CortexM.h"
#include "../inc/Tachometer.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/UART0.h"
#include "../inc/EUSCIA0.h"
#include "../inc/FIFO0.h"

//=========================================================================================
// SECTION 1: GLOBAL VARIABLES & CONFIGURATIONS
//=========================================================================================

// Interrupt flags and data
volatile uint8_t bump_triggered = 0;
volatile uint8_t bump_value = 0;
volatile uint32_t bump_count = 0;
volatile uint8_t emergency_stop = 0;

// SysTick timing
volatile uint32_t systick_counter = 0;
volatile uint8_t systick_10ms_flag = 0;
volatile uint8_t systick_100ms_flag = 0;
volatile uint8_t systick_1s_flag = 0;
volatile uint32_t time_ms = 0;

// Sensor data (updated by interrupts or polling)
volatile uint8_t reflectance_data = 0;
volatile uint8_t new_reflectance_data = 0;
volatile uint32_t ir_left = 0, ir_center = 0, ir_right = 0;
volatile uint8_t line_detected = 0;
volatile uint8_t obstacle_detected = 0;

// Tachometer data
volatile uint16_t left_tach = 0, right_tach = 0;
volatile int32_t left_steps = 0, right_steps = 0;

// State machine variables
typedef enum {
    STATE_IDLE,
    STATE_FORWARD,
    STATE_TURNING,
    STATE_BACKING,
    STATE_STOPPED,
    STATE_LINE_FOLLOW,
    STATE_OBSTACLE_AVOID
} RobotState;

volatile RobotState current_state = STATE_IDLE;
volatile uint32_t state_timer = 0;

// Data collection buffer
#define BUFFER_SIZE 256
volatile uint8_t data_buffer[BUFFER_SIZE];
volatile uint16_t write_index = 0;
volatile uint16_t read_index = 0;
volatile uint16_t data_count = 0;

// Constants for robot dimensions and control
#define WHEEL_CIRCUMFERENCE 220  // mm
#define STEPS_PER_REV 360
#define WHEELBASE 150  // mm distance between wheels
#define BASE_SPEED 1000
#define MAX_SPEED 7000
#define MIN_SPEED 0

//=========================================================================================
// SECTION 2: INTERRUPT SERVICE ROUTINES
//=========================================================================================

/**
 * BUMP SWITCH ISR - Emergency stop and collision handling
 * TO ENABLE: BumpInt_Init(&Bump_ISR)
 * TO DISABLE: P4->IE &= ~0xED or don't call BumpInt_Init()
 */
void Bump_ISR(uint8_t bumps){
    Motor_Stop();
    bump_triggered = 1;
    bump_value = bumps;
    bump_count++;
    emergency_stop = 1;
    P2->OUT |= 0x01;  // Red LED on
}

/**
 * SYSTICK ISR - Periodic tasks every 1ms
 * TO ENABLE: SysTick_Init(48000, 2)
 * TO DISABLE: SysTick->CTRL = 0
 */
void SysTick_Handler(void){
    systick_counter++;
    time_ms++;

    // State machine timer
    if(state_timer > 0){
        state_timer--;
    }

    // 10ms tasks
    if(systick_counter % 10 == 0){
        systick_10ms_flag = 1;

        // Start reflectance reading
        Reflectance_Start();
    }

    // 11ms - complete reflectance reading
    if(systick_counter % 10 == 1){
        reflectance_data = Reflectance_End();
        new_reflectance_data = 1;

        // Check for line
        if(reflectance_data & 0x18){
            line_detected = 1;
        } else {
            line_detected = 0;
        }
    }

    // 50ms tasks
    if(systick_counter % 50 == 0){
        // Read IR sensors
        uint32_t raw17, raw12, raw16;
        ADC_In17_12_16(&raw17, &raw12, &raw16);
        ir_right = LPF_Calc(raw17);
        ir_center = LPF_Calc2(raw12);
        ir_left = LPF_Calc3(raw16);

        // Check for obstacles
        uint32_t center_dist = CenterConvert(ir_center);
        if(center_dist < 200){
            obstacle_detected = 1;
        } else {
            obstacle_detected = 0;
        }
    }

    // 100ms tasks
    if(systick_counter % 100 == 0){
        systick_100ms_flag = 1;
    }

    // 1 second tasks
    if(systick_counter >= 1000){
        systick_counter = 0;
        systick_1s_flag = 1;
        P2->OUT ^= 0x02;  // Toggle green LED heartbeat
    }
}

/**
 * TIMER A1 ISR - Custom period timer
 */
void TimerA1_Task(void){
    static uint32_t timer_count = 0;
    timer_count++;

    // Add your periodic task here
}

//=========================================================================================
// SECTION 3: COMPLETE SYSTEM INITIALIZATION
//=========================================================================================

/**
 * Initialize ALL hardware - call once at start
 */
void System_Init(void){
    DisableInterrupts();

    // Core system
    Clock_Init48MHz();

    // User interface
    LaunchPad_Init();
    UART0_Init();

    // Motors
    Motor_Init();
    Motor_Stop();

    // Sensors
    Reflectance_Init();
    Bump_Init();
    ADC0_InitSWTriggerCh17_12_16();

    // Initialize filters for IR sensors
    uint32_t raw17, raw12, raw16;
    ADC_In17_12_16(&raw17, &raw12, &raw16);
    LPF_Init(raw17, 256);
    LPF_Init2(raw12, 256);
    LPF_Init3(raw16, 256);

    // Optional: Tachometer
    // Tachometer_Init();

    EnableInterrupts();
}

/**
 * Initialize with interrupts enabled
 */
void System_Init_With_Interrupts(void){
    DisableInterrupts();

    System_Init();  // Basic initialization

    // Enable interrupts
    BumpInt_Init(&Bump_ISR);
    SysTick_Init(48000, 2);  // 1ms period
    // TimerA1_Init(&TimerA1_Task, 50000);  // Optional timer

    EnableInterrupts();
}

//=========================================================================================
// SECTION 4: BASIC HELPER FUNCTIONS
//=========================================================================================

// === LED Control Functions ===
void RedLED_On(void)    { P2->OUT |= 0x01; }
void RedLED_Off(void)   { P2->OUT &= ~0x01; }
void RedLED_Toggle(void){ P2->OUT ^= 0x01; }

void GreenLED_On(void)  { P2->OUT |= 0x02; }
void GreenLED_Off(void) { P2->OUT &= ~0x02; }
void GreenLED_Toggle(void){ P2->OUT ^= 0x02; }

void BlueLED_On(void)   { P2->OUT |= 0x04; }
void BlueLED_Off(void)  { P2->OUT &= ~0x04; }
void BlueLED_Toggle(void){ P2->OUT ^= 0x04; }

void RGB_LED_Control(uint8_t red, uint8_t green, uint8_t blue){
    if(red) P2->OUT |= 0x01; else P2->OUT &= ~0x01;
    if(green) P2->OUT |= 0x02; else P2->OUT &= ~0x02;
    if(blue) P2->OUT |= 0x04; else P2->OUT &= ~0x04;
}

void All_LEDs_Off(void) { P2->OUT &= ~0x07; }
void All_LEDs_On(void)  { P2->OUT |= 0x07; }

// === Basic Motor Movement Functions ===
void Move_Forward_Timed(uint16_t speed, uint32_t time_ms){
    Motor_Forward(speed, speed);
    Clock_Delay1ms(time_ms);
    Motor_Stop();
}

void Move_Backward_Timed(uint16_t speed, uint32_t time_ms){
    Motor_Backward(speed, speed);
    Clock_Delay1ms(time_ms);
    Motor_Stop();
}

void Turn_Right_90_Degrees(void){
    Motor_Right(3000, 3000);
    Clock_Delay1ms(500);  // Calibrate this value
    Motor_Stop();
}

void Turn_Left_90_Degrees(void){
    Motor_Left(3000, 3000);
    Clock_Delay1ms(500);  // Calibrate this value
    Motor_Stop();
}

void Turn_180_Degrees(void){
    Motor_Right(3000, 3000);
    Clock_Delay1ms(1000);  // Calibrate this value
    Motor_Stop();
}

void Pivot_Right(uint16_t speed, uint32_t time_ms){
    Motor_Right(speed, speed);
    Clock_Delay1ms(time_ms);
    Motor_Stop();
}

void Pivot_Left(uint16_t speed, uint32_t time_ms){
    Motor_Left(speed, speed);
    Clock_Delay1ms(time_ms);
    Motor_Stop();
}

// === Smooth Acceleration/Deceleration ===
void Smooth_Accelerate_Forward(void){
    uint16_t speed;
    for(speed = 0; speed < 5000; speed += 100){
        Motor_Forward(speed, speed);
        Clock_Delay1ms(10);
    }
}

void Smooth_Decelerate_Stop(void){
    uint16_t speed;
    for(speed = 5000; speed > 0; speed -= 100){
        Motor_Forward(speed, speed);
        Clock_Delay1ms(10);
    }
    Motor_Stop();
}

// === Reflectance Sensor Functions ===
uint8_t Is_On_Line(void){
    uint8_t data = Reflectance_Read(1000);
    return ((data & 0x18) != 0);  // Check center sensors
}

int32_t Get_Line_Position(void){
    uint8_t data = Reflectance_Read(1000);
    return Reflectance_Position(data);
}

uint8_t Count_Sensors_On_Line(void){
    uint8_t data = Reflectance_Read(1000);
    uint8_t count = 0;
    for(int i = 0; i < 8; i++){
        if(data & (1 << i)) count++;
    }
    return count;
}

// === Bump Switch Functions ===
void Wait_For_Bump(void){
    while(Bump_Read() == 0x3F);
}

uint8_t Is_Bump_Pressed(uint8_t bump_num){
    uint8_t bumps = Bump_Read();
    return ((bumps & (1 << bump_num)) == 0);  // Negative logic
}

uint8_t Get_Bump_Count(void){
    uint8_t bumps = Bump_Read();
    uint8_t count = 0;
    for(int i = 0; i < 6; i++){
        if((bumps & (1 << i)) == 0) count++;
    }
    return count;
}

uint8_t Read_Binary_From_Bumps(void){
    uint8_t bumps = Bump_Read();
    uint8_t binary = 0;

    // Convert negative logic to positive binary
    if(!(bumps & 0x01)) binary |= 0x01;
    if(!(bumps & 0x02)) binary |= 0x02;
    if(!(bumps & 0x04)) binary |= 0x04;
    if(!(bumps & 0x08)) binary |= 0x08;
    if(!(bumps & 0x10)) binary |= 0x10;
    if(!(bumps & 0x20)) binary |= 0x20;

    return binary;
}

// === IR Distance Sensor Functions ===
void Read_IR_Sensors(uint32_t *left, uint32_t *center, uint32_t *right){
    uint32_t raw17, raw12, raw16;
    ADC_In17_12_16(&raw17, &raw12, &raw16);
    *right = LPF_Calc(raw17);
    *center = LPF_Calc2(raw12);
    *left = LPF_Calc3(raw16);
}

void Get_IR_Distances_mm(int32_t *left_mm, int32_t *center_mm, int32_t *right_mm){
    uint32_t left_raw, center_raw, right_raw;
    Read_IR_Sensors(&left_raw, &center_raw, &right_raw);
    *left_mm = LeftConvert(left_raw);
    *center_mm = CenterConvert(center_raw);
    *right_mm = RightConvert(right_raw);
}

uint8_t Is_Obstacle_Ahead(uint32_t threshold_mm){
    int32_t left, center, right;
    Get_IR_Distances_mm(&left, &center, &right);
    return (center < threshold_mm);
}

// === Tachometer Functions ===
void Read_Tachometer_Data(uint16_t *leftTach, uint16_t *rightTach,
                         int32_t *leftSteps, int32_t *rightSteps){
    enum TachDirection leftDir, rightDir;
    Tachometer_Get(leftTach, &leftDir, leftSteps,
                   rightTach, &rightDir, rightSteps);
}

void Move_Distance(int32_t distance_mm){
    uint16_t leftTach, rightTach;
    int32_t leftSteps_start, rightSteps_start;
    int32_t leftSteps_current, rightSteps_current;

    Read_Tachometer_Data(&leftTach, &rightTach,
                        &leftSteps_start, &rightSteps_start);

    int32_t required_steps = (distance_mm * STEPS_PER_REV) / WHEEL_CIRCUMFERENCE;

    Motor_Forward(3000, 3000);

    while(1){
        Read_Tachometer_Data(&leftTach, &rightTach,
                            &leftSteps_current, &rightSteps_current);

        int32_t traveled_steps = leftSteps_current - leftSteps_start;

        if(traveled_steps >= required_steps){
            Motor_Stop();
            break;
        }
        Clock_Delay1ms(10);
    }
}

void Rotate_Angle(int32_t angle_degrees){
    int32_t distance_per_wheel = (WHEELBASE * 314 * abs(angle_degrees)) / (100 * 360);
    int32_t required_steps = (distance_per_wheel * STEPS_PER_REV) / WHEEL_CIRCUMFERENCE;

    uint16_t leftTach, rightTach;
    int32_t leftSteps_start, rightSteps_start;
    int32_t leftSteps_current, rightSteps_current;

    Read_Tachometer_Data(&leftTach, &rightTach,
                        &leftSteps_start, &rightSteps_start);

    if(angle_degrees > 0){
        Motor_Right(2000, 2000);
    } else {
        Motor_Left(2000, 2000);
    }

    while(1){
        Read_Tachometer_Data(&leftTach, &rightTach,
                            &leftSteps_current, &rightSteps_current);

        int32_t traveled_steps = abs(leftSteps_current - leftSteps_start);

        if(traveled_steps >= required_steps){
            Motor_Stop();
            break;
        }
        Clock_Delay1ms(10);
    }
}

// === UART Display Functions ===
void Display_Sensor_Data(void){
    uint8_t reflectance = Reflectance_Read(1000);
    int32_t position = Reflectance_Position(reflectance);
    uint8_t bumps = Bump_Read();
    int32_t left_mm, center_mm, right_mm;

    Get_IR_Distances_mm(&left_mm, &center_mm, &right_mm);

    UART0_OutString("\n\r=== Sensor Data ===\n\r");

    UART0_OutString("Reflectance: 0x");
    UART0_OutUHex2(reflectance);
    UART0_OutString(" Position: ");
    if(position >= 0) UART0_OutChar('+');
    UART0_OutUDec(position);
    UART0_OutString(" (0.1mm)\n\r");

    UART0_OutString("Bump Switches: 0x");
    UART0_OutUHex2(bumps);
    UART0_OutString("\n\r");

    UART0_OutString("IR Distances - L:");
    UART0_OutUDec(left_mm);
    UART0_OutString("mm C:");
    UART0_OutUDec(center_mm);
    UART0_OutString("mm R:");
    UART0_OutUDec(right_mm);
    UART0_OutString("mm\n\r");
}

//=========================================================================================
// SECTION 5: L-TASK FUNCTIONS (Simple, Single Module)
//=========================================================================================

/**
 * L1: Blink RED LED when black line detected on sensor 1
 */
void L1_LED_Line_Detect(void){
    UART0_OutString("L1: LED responds to line sensor\n\r");

    while(1){
        uint8_t data = Reflectance_Read(1000);

        if(data & 0x01){  // Sensor 1 (rightmost)
            RedLED_On();
        } else {
            RedLED_Off();
        }

        Clock_Delay1ms(50);
    }
}

/**
 * L2: Blink LED count based on bump switches pressed
 */
void L2_Bump_LED_Count(void){
    UART0_OutString("L2: LED blinks = bump count\n\r");
    uint32_t count = 0;

    while(1){
        uint8_t bumps = Bump_Read();

        if(bumps != 0x3F){  // Any bump pressed
            Clock_Delay1ms(200);  // Debounce

            // Count pressed switches
            count = Get_Bump_Count();

            // Blink LED 'count' times
            for(uint32_t i = 0; i < count; i++){
                RedLED_On();
                Clock_Delay1ms(200);
                RedLED_Off();
                Clock_Delay1ms(200);
            }

            // Wait for release
            while(Bump_Read() != 0x3F);
        }

        Clock_Delay1ms(50);
    }
}

/**
 * L3: Display speed difference on terminal
 */
void L3_Display_Speed_ISR(void){
    UART0_OutString("L3: Speed difference display\n\r");

    // This would normally use tachometer interrupts
    uint16_t left_period, right_period;
    int32_t left_steps, right_steps;

    while(1){
        Read_Tachometer_Data(&left_period, &right_period,
                           &left_steps, &right_steps);

        int32_t speed_diff = left_period - right_period;

        UART0_OutString("Speed Diff: ");
        if(speed_diff >= 0) UART0_OutChar('+');
        UART0_OutUDec(abs(speed_diff));
        UART0_OutString("\n\r");

        Clock_Delay1ms(500);
    }
}

//=========================================================================================
// SECTION 6: M-TASK FUNCTIONS (Medium, Two Modules)
//=========================================================================================

/**
 * M1: Move forward, turn 90° right when line detected
 */
void M1_Line_Turn_Right(void){
    UART0_OutString("M1: Turn right on line detection\n\r");

    Motor_Forward(3000, 3000);

    while(1){
        uint8_t data = Reflectance_Read(1000);

        if(data & 0x18){  // Center sensors detect line
            Motor_Stop();
            Clock_Delay1ms(500);
            Turn_Right_90_Degrees();
            Clock_Delay1ms(500);
            Motor_Forward(3000, 3000);
        }

        Clock_Delay1ms(10);
    }
}

/**
 * M2: Blink RED LED based on bump count
 */
void M2_Bump_Blink_Count(void){
    UART0_OutString("M2: Bump counter with LED\n\r");
    uint32_t total_presses = 0;

    while(1){
        uint8_t bumps = Bump_Read();

        if(bumps != 0x3F){
            total_presses++;

            // Blink LED number of times equal to total presses
            for(uint32_t i = 0; i < total_presses; i++){
                RedLED_On();
                Clock_Delay1ms(200);
                RedLED_Off();
                Clock_Delay1ms(200);
            }

            // Wait for release
            while(Bump_Read() != 0x3F);
            Clock_Delay1ms(100);

            if(total_presses >= 5) total_presses = 0;  // Reset after 5
        }
    }
}

/**
 * M3: Stop when obstacle detected, resume when clear
 */
void M3_Obstacle_Stop_Resume(void){
    UART0_OutString("M3: Obstacle detection\n\r");

    while(1){
        int32_t left_mm, center_mm, right_mm;
        Get_IR_Distances_mm(&left_mm, &center_mm, &right_mm);

        if(center_mm < 200){  // Obstacle within 200mm
            Motor_Stop();
            RedLED_On();

            // Wait for obstacle to clear
            while(center_mm < 300){
                Get_IR_Distances_mm(&left_mm, &center_mm, &right_mm);
                Clock_Delay1ms(100);
            }

            RedLED_Off();
        } else {
            Motor_Forward(3000, 3000);
        }

        Clock_Delay1ms(50);
    }
}

//=========================================================================================
// SECTION 7: H-TASK FUNCTIONS (Complex, Algorithms)
//=========================================================================================

/**
 * H1: PD Line Following Algorithm
 */
void H1_Line_Following_PD(void){
    while(1){
        uint8_t data;
            int32_t position;

            // Read all 8 reflectance sensors
            data = Reflectance_Read(1000);

            // Calculate position (-332 to +332 in 0.1mm units)
            // Negative = line to the left, Positive = line to the right
            position = Reflectance_Position(data);

            if(data == 0x00){
                // No line detected - stop
                Motor_Stop();
            }
            else if(position < -100){
                // Line is far to the left - sharp spin turn left
                Motor_Left(BASE_SPEED, BASE_SPEED);
            }
            else if(position > 100){
                // Line is far to the right - sharp spin turn right
                Motor_Right(BASE_SPEED, BASE_SPEED);
            }
            else if(position < -20){
                // Line is slightly to the left - gentle turn left
                Motor_Forward(BASE_SPEED / 4, BASE_SPEED);
            }
            else if(position > 20){
                // Line is slightly to the right - gentle turn right
                Motor_Forward(BASE_SPEED, BASE_SPEED / 4);
            }
            else{
                // Line is centered - go straight
                Motor_Forward(BASE_SPEED, BASE_SPEED);
            }
    }
    }

/**
 * H2: Binary to Decimal/Hex Converter
 */
void H2_Binary_Converter(void){
    UART0_OutString("H2: Binary Converter\n\r");
    UART0_OutString("Use bump switches for binary input\n\r");

    uint8_t last_value = 0xFF;

    while(1){
        uint8_t binary = Read_Binary_From_Bumps();

        if(binary != last_value && binary != 0){
            UART0_OutString("\n\rBinary: ");
            for(int i = 5; i >= 0; i--){
                UART0_OutChar((binary & (1 << i)) ? '1' : '0');
            }

            UART0_OutString(" = Dec: ");
            UART0_OutUDec(binary);

            UART0_OutString(" = Hex: 0x");
            UART0_OutUHex2(binary);

            UART0_OutString("\n\r");
            last_value = binary;
        }

        Clock_Delay1ms(100);
    }
}

/**
 * H3: 360° Scan and Obstacle Approach
 */
void H3_360_Scan_Obstacles(void){
    UART0_OutString("H3: 360 Scan & Approach\n\r");

    uint32_t nearest = 1000;
    uint32_t farthest = 0;

    // Scan phase
    UART0_OutString("Scanning...\n\r");
    Motor_Right(1000, 1000);

    for(int i = 0; i < 600; i++){  // 2 seconds
        int32_t left, center, right;
        Get_IR_Distances_mm(&left, &center, &right);

        if(center > 50 && center < 400){
            if(center < nearest) nearest = center;
            if(center > farthest) farthest = center;
        }
        Clock_Delay1ms(10);
    }
    Motor_Stop();

    UART0_OutString("Nearest: ");
    UART0_OutUDec(nearest);
    UART0_OutString("mm, Farthest: ");
    UART0_OutUDec(farthest);
    UART0_OutString("mm\n\r");

    Clock_Delay1ms(1000);

    // Find nearest
    UART0_OutString("Finding nearest...\n\r");
    Motor_Right(1500, 1500);

    while(1){
        int32_t left, center, right;
        Get_IR_Distances_mm(&left, &center, &right);

        if(abs(center - nearest) < 30){
            Motor_Stop();
            break;
        }
    }

    // Approach to 100mm
    UART0_OutString("Approaching...\n\r");
    while(1){
        int32_t left, center, right;
        Get_IR_Distances_mm(&left, &center, &right);

        if(center <= 100){
            Motor_Stop();
            break;
        }
        Motor_Forward(1500, 1500);
    }

    UART0_OutString("At 100mm\n\r");
    Clock_Delay1ms(1500);

    // Return
    UART0_OutString("Returning...\n\r");
    while(1){
        int32_t left, center, right;
        Get_IR_Distances_mm(&left, &center, &right);

        if(center >= nearest){
            Motor_Stop();
            break;
        }
        Motor_Backward(500, 500);

    }
    Motor_Stop();
    Clock_Delay1ms(1000);
    UART0_OutString("Finding nearest...\n\r");
        Motor_Right(1500, 1500);

        while(1){
            int32_t left, center, right;
            Get_IR_Distances_mm(&left, &center, &right);

            if(abs(center - farthest) < 30){
                Motor_Stop();
                break;
            }
        }

        // Approach to 100mm
        UART0_OutString("Approaching...\n\r");
        while(1){
            int32_t left, center, right;
            Get_IR_Distances_mm(&left, &center, &right);

            if(center <= 100){
                Motor_Stop();
                break;
            }
            Motor_Forward(1500, 1500);
        }
        Motor_Stop();

        UART0_OutString("At 100mm\n\r");
        Clock_Delay1ms(1500);

        // Return
        UART0_OutString("Returning...\n\r");
        while(1){
            int32_t left, center, right;
            Get_IR_Distances_mm(&left, &center, &right);

            if(center >= farthest){
                Motor_Stop();
                break;
            }
            Motor_Backward(1500, 1500);
        }

    UART0_OutString("Task complete!\n\r");
}

/**
 * H4: Maze Navigation with Wall Following
 */
void H4_Maze_Navigation(void){
    UART0_OutString("H4: Maze Navigation\n\r");

    const int32_t wall_distance = 150;
    const int32_t front_threshold = 200;

    while(1){
        int32_t left_dist, center_dist, right_dist;
        Get_IR_Distances_mm(&left_dist, &center_dist, &right_dist);

        // Check for parking spot (all sensors black)
        uint8_t line_data = Reflectance_Read(1000);
        if(line_data == 0xFF){
            Motor_Stop();
            UART0_OutString("Parked!\n\r");
            break;
        }

        // Wall following logic (right wall)
        if(center_dist < front_threshold){
            // Wall ahead, turn left
            Turn_Left_90_Degrees();
        }
        else if(right_dist > wall_distance + 50){
            // Lost wall, turn right
            Motor_Forward(2000, 2000);
            Clock_Delay1ms(200);
            Turn_Right_90_Degrees();
            Motor_Forward(2000, 2000);
            Clock_Delay1ms(300);
        }
        else if(right_dist < wall_distance - 30){
            // Too close to wall
            Motor_Left(2000, 2000);
            Clock_Delay1ms(100);
        }
        else if(right_dist > wall_distance + 30){
            // Too far from wall
            Motor_Right(2000, 2000);
            Clock_Delay1ms(100);
        }
        else {
            // Good distance
            Motor_Forward(3000, 3000);
        }

        Clock_Delay1ms(50);
    }
}

/**
 * H5: Advanced Obstacle Avoidance
 */
void H5_Advanced_Obstacle_Avoidance(void){
    UART0_OutString("H5: Advanced Obstacle Avoidance\n\r");

    while(1){
        int32_t left_dist, center_dist, right_dist;
        Get_IR_Distances_mm(&left_dist, &center_dist, &right_dist);

        if(center_dist < 150){
            // Obstacle ahead - choose direction
            Motor_Stop();

            if(left_dist > right_dist && left_dist > 200){
                // Turn left
                Turn_Left_90_Degrees();
            }
            else if(right_dist > 200){
                // Turn right
                Turn_Right_90_Degrees();
            }
            else {
                // Both blocked, turn around
                Turn_180_Degrees();
            }
        }
        else if(left_dist < 100){
            // Too close on left
            Motor_Right(2000, 3000);  // Veer right
        }
        else if(right_dist < 100){
            // Too close on right
            Motor_Forward(3000, 2000);  // Veer left
        }
        else {
            // Clear path
            Motor_Forward(3500, 3500);
        }

        Clock_Delay1ms(50);
    }
}

//=========================================================================================
// SECTION 8: INTERRUPT-BASED SOLUTIONS
//=========================================================================================

/**
 * Interrupt-driven line follower
 */
void Interrupt_Line_Follower(void){
    // Enable interrupts
    BumpInt_Init(&Bump_ISR);
    SysTick_Init(48000, 2);

    UART0_OutString("Interrupt Line Follower\n\r");

    while(1){
        // Check emergency stop
        if(emergency_stop){
            Clock_Delay1ms(1000);
            Motor_Backward(2000, 2000);
            Clock_Delay1ms(500);
            Motor_Stop();
            emergency_stop = 0;
            P2->OUT &= ~0x01;
        }

        // Use interrupt-updated sensor data
        if(new_reflectance_data){
            int32_t position = Reflectance_Position(reflectance_data);

            // Simple P control
            int32_t correction = position * 5;
            int32_t left = BASE_SPEED - correction;
            int32_t right = BASE_SPEED + correction;

            // Limit
            if(left < 0) left = 0;
            if(left > MAX_SPEED) left = MAX_SPEED;
            if(right < 0) right = 0;
            if(right > MAX_SPEED) right = MAX_SPEED;

            Motor_Forward(left, right);
            new_reflectance_data = 0;
        }

        WaitForInterrupt();
    }
}

/**
 * State machine with interrupts
 */
void State_Machine_Control(void){
    BumpInt_Init(&Bump_ISR);
    SysTick_Init(48000, 2);

    UART0_OutString("State Machine Control\n\r");
    current_state = STATE_FORWARD;
    Motor_Forward(3000, 3000);

    while(1){
        switch(current_state){
            case STATE_FORWARD:
                if(bump_triggered){
                    current_state = STATE_BACKING;
                    state_timer = 500;
                    Motor_Backward(2000, 2000);
                    bump_triggered = 0;
                }
                else if(line_detected){
                    current_state = STATE_LINE_FOLLOW;
                }
                break;

            case STATE_BACKING:
                if(state_timer == 0){
                    current_state = STATE_TURNING;
                    state_timer = 300;
                    Motor_Right(2000, 2000);
                }
                break;

            case STATE_TURNING:
                if(state_timer == 0){
                    current_state = STATE_FORWARD;
                    Motor_Forward(3000, 3000);
                }
                break;

            case STATE_LINE_FOLLOW:
                // Line following logic
                if(!line_detected){
                    current_state = STATE_FORWARD;
                }
                break;

            default:
                current_state = STATE_IDLE;
                Motor_Stop();
                break;
        }

        WaitForInterrupt();
    }
}

//=========================================================================================
// SECTION 9: TEST & DEBUG FUNCTIONS
//=========================================================================================

/**
 * Test all motors
 */
void Test_Motors(void){
    UART0_OutString("=== Motor Test ===\n\r");

    UART0_OutString("Forward...\n\r");
    Motor_Forward(3000, 3000);
    Clock_Delay1ms(1000);

    UART0_OutString("Backward...\n\r");
    Motor_Backward(3000, 3000);
    Clock_Delay1ms(1000);

    UART0_OutString("Right...\n\r");
    Motor_Right(3000, 3000);
    Clock_Delay1ms(1000);

    UART0_OutString("Left...\n\r");
    Motor_Left(3000, 3000);
    Clock_Delay1ms(1000);

    Motor_Stop();
    UART0_OutString("Motor test complete\n\r");
}

/**
 * Test all sensors
 */
void Test_Sensors(void){
    UART0_OutString("=== Sensor Test ===\n\r");
    UART0_OutString("Press SW1 to exit\n\r");

    while((P1->IN & 0x02) != 0){
        Display_Sensor_Data();
        Clock_Delay1ms(500);
    }
}

/**
 * Test interrupts
 */
void Test_Interrupts(void){
    UART0_OutString("=== Interrupt Test ===\n\r");

    BumpInt_Init(&Bump_ISR);
    SysTick_Init(48000, 2);

    UART0_OutString("Bump switches trigger red LED\n\r");
    UART0_OutString("Green LED blinks every second\n\r");
    UART0_OutString("Press SW1 to exit\n\r");

    while((P1->IN & 0x02) != 0){
        if(bump_triggered){
            UART0_OutString("Bump: 0x");
            UART0_OutUHex2(bump_value);
            UART0_OutString("\n\r");
            bump_triggered = 0;
            Clock_Delay1ms(500);
            P2->OUT &= ~0x01;
        }

        if(systick_1s_flag){
            UART0_OutString("1 second tick\n\r");
            systick_1s_flag = 0;
        }

        WaitForInterrupt();
    }

    // Disable interrupts for other tests
    SysTick->CTRL = 0;
    P4->IE &= ~0xED;
}

/**
 * Calibrate 90 degree turns
 */
void Calibrate_Turns(void){
    UART0_OutString("=== Turn Calibration ===\n\r");
    UART0_OutString("Adjust timing for exact 90 degrees\n\r");

    uint32_t turn_time = 500;  // Start with 500ms

    while(1){
        UART0_OutString("Current time: ");
        UART0_OutUDec(turn_time);
        UART0_OutString("ms\n\r");
        UART0_OutString("SW1=test, SW2=done, Bump0=decrease, Bump5=increase\n\r");

        // Wait for input
        while(1){
            if((P1->IN & 0x02) == 0){  // SW1 - test
                Motor_Right(3000, 3000);
                Clock_Delay1ms(turn_time);
                Motor_Stop();
                Clock_Delay1ms(200);
                break;
            }
            if((P1->IN & 0x04) == 0){  // SW2 - done
                UART0_OutString("Calibration complete: ");
                UART0_OutUDec(turn_time);
                UART0_OutString("ms for 90 degrees\n\r");
                return;
            }

            uint8_t bumps = Bump_Read();
            if(!(bumps & 0x01)){  // Bump0 - decrease
                turn_time -= 10;
                Clock_Delay1ms(200);
                break;
            }
            if(!(bumps & 0x20)){  // Bump5 - increase
                turn_time += 10;
                Clock_Delay1ms(200);
                break;
            }
        }
    }
}

//=========================================================================================
// SECTION 10: MAIN FUNCTION WITH MENU SYSTEM
//=========================================================================================

/**
 * Interactive menu system
 */
void Menu_System(void){
    char choice;

    UART0_OutString("\n\r=== RSLK Test Menu ===\n\r");
    UART0_OutString("1. Test Motors\n\r");
    UART0_OutString("2. Test Sensors\n\r");
    UART0_OutString("3. Test Interrupts\n\r");
    UART0_OutString("4. Calibrate Turns\n\r");
    UART0_OutString("5. L-Tasks\n\r");
    UART0_OutString("6. M-Tasks\n\r");
    UART0_OutString("7. H-Tasks\n\r");
    UART0_OutString("8. Interrupt Examples\n\r");
    UART0_OutString("Select: ");

    choice = UART0_InChar();
    UART0_OutChar(choice);
    UART0_OutString("\n\r");

    switch(choice){
        case '1':
            Test_Motors();
            break;
        case '2':
            Test_Sensors();
            break;
        case '3':
            Test_Interrupts();
            break;
        case '4':
            Calibrate_Turns();
            break;
        case '5':
            UART0_OutString("Select L-Task (1-3): ");
            choice = UART0_InChar();
            if(choice == '1') L1_LED_Line_Detect();
            if(choice == '2') L2_Bump_LED_Count();
            if(choice == '3') L3_Display_Speed_ISR();
            break;
        case '6':
            UART0_OutString("Select M-Task (1-3): ");
            choice = UART0_InChar();
            if(choice == '1') M1_Line_Turn_Right();
            if(choice == '2') M2_Bump_Blink_Count();
            if(choice == '3') M3_Obstacle_Stop_Resume();
            break;
        case '7':
            UART0_OutString("Select H-Task (1-5): ");
            choice = UART0_InChar();
            if(choice == '1') H1_Line_Following_PD();
            if(choice == '2') H2_Binary_Converter();
            if(choice == '3') H3_360_Scan_Obstacles();
            if(choice == '4') H4_Maze_Navigation();
            if(choice == '5') H5_Advanced_Obstacle_Avoidance();
            break;
        case '8':
            UART0_OutString("Select Example (1-2): ");
            choice = UART0_InChar();
            if(choice == '1') Interrupt_Line_Follower();
            if(choice == '2') State_Machine_Control();
            break;
        default:
            UART0_OutString("Invalid selection\n\r");
            break;
    }
}

/**
 * Emergency stop - can be called anytime
 */
void Emergency_Stop(void){
    DisableInterrupts();
    Motor_Stop();
    All_LEDs_On();
    UART0_OutString("\n\r!!! EMERGENCY STOP !!!\n\r");
    while(1);
}

/**
 * Main function - Complete system with all options
 */
int main(void){
    // === INITIALIZATION ===
    System_Init();  // Basic init without interrupts
    // OR
    // System_Init_With_Interrupts();  // Init with interrupts enabled

    // === STARTUP ===
    UART0_OutString("\n\r");
    UART0_OutString("=====================================\n\r");
    UART0_OutString("    SC2107 RSLK Robot System V2.0   \n\r");
    UART0_OutString("=====================================\n\r");
    UART0_OutString("All code snippets integrated!\n\r");
    UART0_OutString("Press SW1 to continue\n\r");

    // Wait for button
    while((P1->IN & 0x02) != 0){
        P2->OUT ^= 0x04;  // Blink blue
        Clock_Delay1ms(200);
    }
    Clock_Delay1ms(200);
    All_LEDs_Off();

    // === MAIN OPERATION ===
    // Option 1: Run menu system
    /*while(1){
        Rotate_Angle(90);
        Clock_Delay1ms(1000);

    }*/

    // Option 2: Run specific task directly
    // Comment out Option 1 and uncomment one of these:

    // === L-TASKS ===
    // L1_LED_Line_Detect();
    // L2_Bump_LED_Count();
    // L3_Display_Speed_ISR();

    // === M-TASKS ===
    // M1_Line_Turn_Right();
    // M2_Bump_Blink_Count();
    // M3_Obstacle_Stop_Resume();

    // === H-TASKS ===
     //H1_Line_Following_PD();
    // H2_Binary_Converter();
    H3_360_Scan_Obstacles();
    // H4_Maze_Navigation();
    // H5_Advanced_Obstacle_Avoidance();

    // === INTERRUPT EXAMPLES ===
    // Interrupt_Line_Follower();
    // State_Machine_Control();

    // === TEST FUNCTIONS ===
    // Test_Motors();
    // Test_Sensors();
    // Test_Interrupts();
    // Calibrate_Turns();
}

/**
 * QUICK REFERENCE CHEAT SHEET
 *
 * === INITIALIZATION ===
 * System_Init();                    // Basic init
 * System_Init_With_Interrupts();    // With interrupts
 *
 * === MOTOR COMMANDS ===
 * Motor_Forward(3000, 3000);        // Both wheels forward
 * Motor_Backward(3000, 3000);       // Both wheels backward
 * Motor_Right(3000, 3000);          // Turn right
 * Motor_Left(3000, 3000);           // Turn left
 * Motor_Stop();                     // Stop both motors
 *
 * === SENSOR READING ===
 * Reflectance_Read(1000);           // Returns 8-bit value
 * Bump_Read();                      // Returns 6-bit value
 * Get_IR_Distances_mm(&l,&c,&r);   // Get all IR distances
 *
 * === LED CONTROL ===
 * RedLED_On/Off/Toggle();
 * GreenLED_On/Off/Toggle();
 * BlueLED_On/Off/Toggle();
 *
 * === INTERRUPTS ===
 * BumpInt_Init(&handler);           // Enable bump interrupts
 * SysTick_Init(48000, 2);          // 1ms periodic interrupt
 * EnableInterrupts();               // Global enable
 * DisableInterrupts();              // Global disable
 *
 * === TIMING ===
 * Clock_Delay1ms(time);             // Delay in milliseconds
 * Clock_Delay1us(time);             // Delay in microseconds
 *
 * === UART OUTPUT ===
 * UART0_OutString("text");
 * UART0_OutUDec(number);
 * UART0_OutUHex2(hex);
 *
 * === COMMON VALUES ===
 * Speed range: 0-7500
 * No bumps pressed: 0x3F
 * All reflectance black: 0xFF
 * Typical IR threshold: 200mm
 * 90° turn time: ~500ms at speed 3000
 */
