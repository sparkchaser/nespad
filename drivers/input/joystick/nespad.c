/**
 * @file nespad.c
 * @brief Use a NES controller as a joystick/gamepad.
 *
 * A SuperNES controller could use the same code, only
 *  with four additional buttons and a slightly different
 *  button reporting order.
 * It should also be possible to use this code (with slight tweaks) for any
 *  homebrew digital joystick/gamepad that uses a shift register for
 *  button reporting.
 */
/*
 *  nespad - driver for GPIO-attached NES controller
 *  Copyright (C) 2016  Ben Allen
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 *  NON INFRINGEMENT.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>  // kernel module API and helpers
#include <linux/types.h>   // data types used by the kernel
#include <asm/io.h>        // needed by platform.h
#include <mach/platform.h> // for __io_address
#include <linux/timer.h>   // timer API
#include <linux/input.h>   // input device (gamepad) API
#include <asm/delay.h>     // for udelay()


// General module information
#define NESPI_DRIVER_VERSION "0.1.0-000"
MODULE_AUTHOR("Ben Allen");
MODULE_DESCRIPTION("Driver for GPIO-attached NES Controller, version " NESPI_DRIVER_VERSION);
MODULE_VERSION(NESPI_DRIVER_VERSION);
MODULE_LICENSE("GPL");

// Configure GPIO pins by modifying these.
// All pin numbers in this code are based on the
//  Broadcom pin numbers, not the pin numbers on
//  the Raspberry Pi connector.
#define CLOCK_PIN  27    /**< pin connected to controller's "clock" line */
#define LATCH_PIN  17    /**< pin connected to controller's "latch" line */
#define DATA_PIN   18    /**< pin connected to controller's "data" line */


/**
 * @name Bitmasks for individual buttons
 * @note this is the actual order for NES controllers, but not for
 *       SNES controllers.  This shouldn't matter as long as the
 *       tables below are set up in the order that the controller uses.
 * @{
 */
#define BUTTON_A    (0x1 << 0)  // A
#define BUTTON_B    (0x1 << 1)  // B
#define BUTTON_SEL  (0x1 << 2)  // Select
#define BUTTON_STA  (0x1 << 3)  // Start
#define BUTTON_UP   (0x1 << 4)  // D-pad up
#define BUTTON_DN   (0x1 << 5)  // D-pad down
#define BUTTON_LT   (0x1 << 6)  // D-pad left
#define BUTTON_RT   (0x1 << 7)  // D-pad right
#define BUTTON_X    (0x1 << 8)  // X
#define BUTTON_Y    (0x1 << 9)  // Y
#define BUTTON_L    (0x1 << 10) // left shoulder
#define BUTTON_R    (0x1 << 11) // right shoulder
#define BUTTON_NONE  0
/** @} */

/** Number of time to read controller data each second */
#define SAMPLES_PER_SECOND  60
#define SAMPLE_INTERVAL     (HZ / (SAMPLES_PER_SECOND)) // in "jiffies", the kernel's unit for time
// Hardware-specific timing parameters (in ms).
// Adjust these based on the requirements of the specific shift
//  register that you're using.
#define NORMAL_DELAY            6 /**< delay (ms) between state transitions */
#define SETUP_DELAY            12 /**< special delay (ms) following the "latch" operation */
#define ACTIVE_LOW_SIGNALS        /**< use this definition if a pressed button is reported as "0" */

// Helper macro for calculating the length of a statically-allocated array.
#define COUNT_OF(ary)   (sizeof(ary) / sizeof(0[ary]))

static void set_clock   (int state);
static void set_latch   (int state);
static int  get_data    (void);

/** Timer used for controlling device polling */
static struct timer_list nes_timer;

/** Structure representing the controller as an input device to the kernel */
static struct input_dev* nes_dev = NULL;


/**
 * @name State machine for reading controller buttons.
 * @{
 */

/**
 * Initialize the state machine and I/O pins.
 */
static void __init state_machine_reset(void)
{
    set_latch(0);
    set_clock(0);
}


/**
 * Data structure representing one step in the process of reading button
 *  data from a shift register.
 *
 * This structure indicates which level to set the output lines to, which
 *  button (if any) to read, and the length of time to spend on this step.
 *  An array of these structures drives the button-reading state machine.
 */
struct controller_state_data
{
    int     latch;      /**< state of the 'latch' line (high/low) */
    int     clock;      /**< state of the 'clock' line (high/low) */
    u16     read_data;  /**< if non-zero, read data and OR this into the output value if pressed */
    int     state_time; /**< amount of time to spend in this stage, or '0' for end-of-table */
};
/**
 * Button order and timing information for a NES controller
 */
static const struct controller_state_data nes_controller[] =
{
    {1, 0, BUTTON_NONE, SETUP_DELAY},
    {0, 0, BUTTON_A,    NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_B,    NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_SEL,  NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_STA,  NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_UP,   NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_DN,   NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_LT,   NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_RT,   NORMAL_DELAY},
    {0, 0, BUTTON_NONE, 0}
};
/**
 * Button order and timing information for a SNES controller
 */
static const struct controller_state_data snes_controller[] =
{
    {1, 0, BUTTON_NONE, SETUP_DELAY},
    {0, 0, BUTTON_B,    NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_Y,    NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_SEL,  NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_STA,  NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_UP,   NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_DN,   NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_LT,   NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_RT,   NORMAL_DELAY},
    {1, 0, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_A,    NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_X,    NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_L,    NORMAL_DELAY},
    {0, 1, BUTTON_NONE, NORMAL_DELAY},
    {0, 0, BUTTON_R,    NORMAL_DELAY},
    {0, 0, BUTTON_NONE, 0}
};

/**
 * State machine that reads button data from a controller.
 * The code is generic and should work with any controller type.  The controller-
 *  specific details are handled by the structure array provided.
 *
 * @param data  Array of structures describing the process for reading a
 *              complete set of button data from the controller.
 * @param num_states  Length of array 'data'
 * @return bitmask indicating which buttons were pressed (see 'BUTTON_' macros above)
 */
static u16 read_controller(const struct controller_state_data* data, int num_states)
{
    int i;
    u16 button_data = 0;

    BUG_ON(data == NULL);
    BUG_ON(num_states < 3);

    for (i = 0; (i < num_states) && (data[i].state_time > 0); ++i)
    {
        // Update clock and latch lines
        set_latch(data[i].latch);
        set_clock(data[i].clock);

        // Read data line, if needed
        if (data[i].read_data != BUTTON_NONE)
        {
#ifdef ACTIVE_LOW_SIGNALS
            if (get_data() == 0)
#else
            if (get_data() != 0)
#endif
            {
                // If button was reported as pressed, add its bitmask to the output
                button_data |= data[i].read_data;
            }
        }

        // Wait for shift register hardware to respond
        udelay(data[i].state_time);
    }

    return button_data;
}

/** @} */

/**
 * @name Platform-specific helper functions.
 *
 * Each implementation must fill in these templates with whatever
 *  is appropriate for that specific platform.
 * @{
 */

/**
 * Structure representing the Broadcom chip's GPIO control registers.
 * See the Broadcom datasheet for details about using these registers.
 */
struct gpio_registers
{
    u32 GPFSEL[6];
    u32 reserved1;
    u32 GPSET[2];
    u32 reserved2;
    u32 GPCLR[2];
    u32 reserved3;
    u32 GPLEV[2];
};
volatile struct gpio_registers* gpio_ptr;

#define PIN_INPUT   0b000
#define PIN_OUTPUT  0b001

/**
 * Configure a GPIO pin to be either an input or an output.
 * @param pin    GPIO pin to set
 * @param state  PIN_INPUT or PIN_OUTPUT
 */
static void configure_gpio_pin(int pin, int state)
{
    // *** platform-specific code goes here ***
    u32 oldval;
    u32 bitmask;

    BUG_ON((state != PIN_INPUT) && (state != PIN_OUTPUT));

    oldval = gpio_ptr->GPFSEL[pin / 10];
    bitmask = 0b111 << (pin % 10) * 3;
    gpio_ptr->GPFSEL[pin / 10] = (oldval & ~bitmask) | ((state << ((pin % 10) * 3)) & bitmask);
}

/**
 * Set a GPIO output pin.
 * @param pin    GPIO pin to set
 * @param state  zero for 'low', non-zero for 'high'
 */
static void gpio_set_pin(int pin, int state)
{
    // *** platform-specific code goes here ***
    if (state)
        gpio_ptr->GPSET[pin / 32] = (1 << (pin % 32));
    else
        gpio_ptr->GPCLR[pin / 32] = (1 << (pin % 32));
}

/**
 * Read GPIO input pin.
 * @param pin    GPIO pin to read
 * @return  zero for 'low', non-zero for 'high'
 */
static int gpio_get_pin(int pin)
{
    // *** platform-specific code goes here ***
    if (gpio_ptr->GPLEV[pin / 32] & (1 << (pin % 32)))
        return 1; 
    return 0;
}

/** @} */


/**
 * Set clock line (output).
 * @param state  zero for 'low', non-zero for 'high'
 */
static void set_clock(int state) { gpio_set_pin(CLOCK_PIN, state); }

/**
 * Set latch line (output).
 * @param state  zero for 'low', non-zero for 'high'
 */
static void set_latch(int state) { gpio_set_pin(LATCH_PIN, state); }

/**
 * Read data line (input).
 * @return  zero for 'low', non-zero for 'high'
 */
static int get_data(void) { return gpio_get_pin(DATA_PIN); }


/**
 * @name Linux kernel-specific code.
 * Startup and shutdown routines, etc.
 * @{
 */

/**
 * Callback function to run whenever the timer goes off.
 * This function is responsible for reading data from the controller,
 *  reporting button presses back to the kernel's input subsystem, and
 *  re-arming the timer for next time.
 * @param dummy   currently unused
 */
static void timer_callback(unsigned long dummy)
{
    u16 current_state;

    // Re-arm the timer
    // It would make more sense to do this at the end of the
    // function, but we don't want to include the sample time
    // in the delay.
    mod_timer(&nes_timer, jiffies + SAMPLE_INTERVAL);

    // Read a sample from the controller
    current_state = read_controller(nes_controller, COUNT_OF(nes_controller));

    // Translate data into gamepad input messages
    if ((current_state & BUTTON_RT) && !(current_state & BUTTON_LT))
        input_report_abs(nes_dev, ABS_X, 1);
    else if (!(current_state & BUTTON_RT) && (current_state & BUTTON_LT))
        input_report_abs(nes_dev, ABS_X, -1);
    else
        input_report_abs(nes_dev, ABS_X, 0);
    if ((current_state & BUTTON_UP) && !(current_state & BUTTON_DN))
        input_report_abs(nes_dev, ABS_Y, 1);
    else if (!(current_state & BUTTON_UP) && (current_state & BUTTON_DN))
        input_report_abs(nes_dev, ABS_Y, -1);
    else
        input_report_abs(nes_dev, ABS_Y, 0);
    input_report_key(nes_dev, BTN_A, current_state & BUTTON_A);
    input_report_key(nes_dev, BTN_B, current_state & BUTTON_B);
    input_report_key(nes_dev, BTN_START, current_state & BUTTON_STA);
    input_report_key(nes_dev, BTN_SELECT, current_state & BUTTON_SEL);

    // Push gamepad events to the input subsystem
    input_sync(nes_dev);
}


/**
 * Callback executed when a program opens a connection to a controller.
 */
static int pad_open(struct input_dev* dev)
{
    // Start timer for polling controller state
    mod_timer(&nes_timer, jiffies + SAMPLE_INTERVAL);

    return 0;
}

/**
 * Callback executed when a program's connection to a controller is closed.
 */
static void pad_close(struct input_dev* dev)
{
    // Delete timer
    // This prevents the kernel from wasting time polling the controller
    // when nothing is using it.
    del_timer_sync(&nes_timer);
}

/**
 * Register as a gamepad device.
 * This function will allocate an input device structure, set it up,
 *  register it with the system, and store the structure pointer into
 *  the address provided by the caller.
 * @param d  the pointer to the allocated metadata structure will be stored here
 * @return  0 on success, or an error code otherwise
 */
static int __init register_gamepad(struct input_dev** d)
{
    int error;
    struct input_dev* p;

    BUG_ON(d == NULL);

    // allocate input device structure
    p = input_allocate_device();
    if (!p)
    {
        printk(KERN_ERR "%s: Not enough memory\n", __FILE__);
        return -ENOMEM;
    }
    *d = p;

    // Set basic input device parameters
    p->name = "NES controller";

    // These callback functions are completely optional.  They are called when
    //  a program opens or closes a connection to the controller' device node.
    //  They are used here to only start polling the controller when a program
    //  is using it, and then to stop polling when the controller is no longer
    //  in use.  This simply prevents the system from wasting time reading from
    //  the controller when it's not necessary.
    p->open = pad_open;
    p->close = pad_close;

    // Indicate which types of buttons this controller supports
    p->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS); // controller supports "absolute" and "key" type events
    input_set_abs_params(p, ABS_X, -1, 1, 0, 0); // X axis, registers from -1 (left) to 1 (right)
    input_set_abs_params(p, ABS_Y, -1, 1, 0, 0); // Y axis, registers from -1 (down) to 1 (up)
    __set_bit(BTN_A, p->keybit); // 'A' button
    __set_bit(BTN_B, p->keybit); // 'B' button
    __set_bit(BTN_SELECT, p->keybit); // 'Select' button
    __set_bit(BTN_START, p->keybit); // 'Start' button

    // Register with input device subsystem
    error = input_register_device(p);
    if (error)
    {
        printk(KERN_ERR "%s: Failed to register device\n", __FILE__);
        input_free_device(p);
        *d = NULL;
        return error;
    }

    return 0;
} 

/**
 * Driver initialization function.
 * Initialize hardware and register devices with the kernel.
 * This function will be called automatically when the driver is loaded.
 * @return  0 on success, non-zero on error
 */
static int __init NESpadModule_init(void)
{
    int retval;

    // Set up device register access by retrieving a pointer to
    //  the memory-mapped registers
    gpio_ptr = (struct gpio_registers*)__io_address(GPIO_BASE);

    // Configure GPIO pins
    configure_gpio_pin(CLOCK_PIN, PIN_OUTPUT);
    configure_gpio_pin(LATCH_PIN, PIN_OUTPUT);
    configure_gpio_pin(DATA_PIN,  PIN_INPUT);

    // Initialize state machine
    state_machine_reset();

    // Register with the input subsystem as a gamepad device
    retval = register_gamepad(&nes_dev);
    if (retval)
    {
        printk(KERN_ERR "%s: device registration failed [%i]\n", __FILE__, retval);
        return retval;
    }

    // Create a timer so that we can poll the controller
    setup_timer(&nes_timer, timer_callback, 0);
    retval = mod_timer(&nes_timer, jiffies + SAMPLE_INTERVAL);
    BUG_ON(retval < 0);

    return 0;
}

/**
 * Driver cleanup function.
 * Undo everything done by the init function.
 * This function will be called automatically when the driver unloads.
 */
static void __exit NESpadModule_exit(void)
{
    // Delete timer
    del_timer(&nes_timer);

    // Unregister the input device
    if (nes_dev != NULL)
    {
        input_unregister_device(nes_dev);
    }

    // Datasheet recommends setting unused pins as inputs
    configure_gpio_pin(CLOCK_PIN, PIN_INPUT);
    configure_gpio_pin(LATCH_PIN, PIN_INPUT);
    configure_gpio_pin(DATA_PIN,  PIN_INPUT);
}

// Register module initialization and cleanup functions.
module_init(NESpadModule_init);
module_exit(NESpadModule_exit);

/** @} */

