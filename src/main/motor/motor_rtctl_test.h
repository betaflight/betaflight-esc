#pragma once

/**
 * Perform the ESC self test.电调硬件自身自检
 * @return 0        - test OK,
 *         negative - unable to run the test at the current state,
 *         positive - test failed.
 */
int motor_rtctl_test_hardware(void);

/**
 * Test the connected motor (if any).测试电机是否接入
 * @return 0        - motor appears to be connected,
 *         negative - unable to run the test at the current state,
 *         positive - motor is not connected or went bananas.
 */
int motor_rtctl_test_motor(void);
