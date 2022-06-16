/* 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board.h"
#include "tusb.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/uart.h"

#include "usb_descriptors.h"
#include "SwiCC.h"

//--------------------------------------------------------------------
// Global variables
//--------------------------------------------------------------------

// Controller reports and ring buffer.
USB_ControllerReport_Input_t neutral_con, rt_con, current_con;
USB_ControllerReport_Input_t con_data_buff[CON_BUFF_LEN];
USB_ControllerReport_Input_t rec_data_buff[REC_BUFF_LEN];
unsigned int queue_head, queue_tail, rec_tail, stream_head;

unsigned int frame_delay_us = 10000;

bool xmitting = false;

uint8_t action_mode = A_PLAY;

//--------------------------------------------------------------------
// Main
//--------------------------------------------------------------------
int main(void) {
    board_init();

    feedback_GPIO_init();

    // zero-out the controller buffer
    buffer_init();

    // Set up USB
    tusb_init();

    // Set up GPIO interrupt
    gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // start serial comms
    uart_setup();

    // Forever loop
    while (1) {
        tud_task(); // tinyusb device task
        hid_task();
    }

    return 0;
}

static void feedback_GPIO_init(){
    gpio_init(FEEDBACK_VSYNC);
    gpio_set_dir(FEEDBACK_VSYNC, GPIO_OUT);

    gpio_init(FEEDBACK_RX);
    gpio_set_dir(FEEDBACK_RX, GPIO_OUT);

    gpio_init(FEEDBACK_TX);
    gpio_set_dir(FEEDBACK_TX, GPIO_OUT);

    gpio_init(FEEDBACK_PLAY);
    gpio_set_dir(FEEDBACK_PLAY, GPIO_OUT);

    gpio_init(FEEDBACK_REC);
    gpio_set_dir(FEEDBACK_REC, GPIO_OUT);
}

//--------------------------------------------------------------------
// Device callbacks
//--------------------------------------------------------------------

// Invoked when device is mounted
void tud_mount_cb(void) {
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
    (void) remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
}

//--------------------------------------------------------------------
// USB HID
//--------------------------------------------------------------------

void hid_task(void) {

    // Remote wakeup (unused?)
    if (tud_suspended() && false) {
        // Wake up host if we are in suspend mode
        // and REMOTE_WAKEUP feature is enabled by host
        tud_remote_wakeup();
    }

    // feedback
    gpio_put(FEEDBACK_PLAY, action_mode==A_PLAY);
    gpio_put(FEEDBACK_REC, action_mode==A_REC);

    // report controller data
    if (tud_hid_ready()) {
        switch(action_mode){
            case A_PLAY: // play from buffer
            case A_REC:  // play while recording
            tud_hid_report(0, &current_con, sizeof(USB_ControllerReport_Input_t));
            break;

            case A_RT: // play from immediate value
            tud_hid_report(0, &rt_con, sizeof(USB_ControllerReport_Input_t));
            break;

            case A_STOP: // output neutral
            tud_hid_report(0, &neutral_con, sizeof(USB_ControllerReport_Input_t));
            break;

            default:
            break;
        }
    }
}


//--------------------------------------------------------------------
// UART and buffer code
//--------------------------------------------------------------------

/* Initialize the buffer and other controller variables.
*/
void buffer_init(){
    // Configure a neutral controller state
    neutral_con.LX = 128;
    neutral_con.LY = 128;
    neutral_con.RX = 128;
    neutral_con.RY = 128;
    neutral_con.HAT = 0x08;
    neutral_con.Button = 0;

    // Copy to initial controller state
    memcpy(&current_con, &neutral_con, sizeof(USB_ControllerReport_Input_t));
    memcpy(&rt_con, &neutral_con, sizeof(USB_ControllerReport_Input_t));

    // Copy the neutral controller into all buffer entries
    for(int i=0; i<CON_BUFF_LEN; i++){
        memcpy(&(con_data_buff[i]), &neutral_con, sizeof(USB_ControllerReport_Input_t));
    }

}

void uart_setup(){
    // Set up UART with a basic baud rate.
    uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    // Turn off UART flow control CTS/RTS 
    uart_set_hw_flow(UART_ID, false, false);
    // Set data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    // Turn on FIFO's
    uart_set_fifo_enabled(UART_ID, false);
    // Set up a RX interrupt
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);
    // Eable the UART to send interrupts (on RX only)
    uart_set_irq_enables(UART_ID, true, false);

}

/* Each time a character is received, process it.
*  Uses state in cmd_state to track what is happening.
*/
void on_uart_rx() {
    static uint8_t rx_toggle = 0;
    static int cmd_state = C_IDLE;
    static char cmd_str[32]; // incoming command string
    static uint8_t cmd_str_ind = 0; // index into command string

    rx_toggle++;
    gpio_put(FEEDBACK_RX, rx_toggle&1);

//    board_led_write(1);
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);

        // hard force actionation on command character
        if (ch == CMD_CHAR) {
            // reset command string
            cmd_str_ind = 0;
            memset(cmd_str, 0, sizeof(cmd_str));
        }
        // complete parse on newline
        else if ((ch == '\r') || (ch == '\n')) {

            // Special treatment for queue commands
            if ((cmd_str[0] == 'Q') && (cmd_str_ind == 15)) {
                add_to_queue(cmd_str);
            }

            // Special treatment for immediate commands
            if ((cmd_str[0] == 'I') && (cmd_str_ind == 15)) {
                force_con_state(cmd_str);
            }

            if (strcmp(cmd_str, "MREC") == 0) {
                rec_tail = 0;
                action_mode = A_REC;
            }

            if (strcmp(cmd_str, "MSTOP") == 0) {
                action_mode = A_STOP;
            }

            if (strcmp(cmd_str, "GQF") == 0) { // get queue buffer fullness
                uart_resp_int(get_queue_fill());
            }
            if (strcmp(cmd_str, "GRF") == 0) { // get recording buffer fullness
                uart_resp_int(get_recording_fill());
            }

            if (strcmp(cmd_str, "SR0") == 0) { // send recording from 0
                stream_head = 0;
                send_recording();
                if (stream_head >= rec_tail) {
                    // end of stream
                    uart_puts(UART_ID, "+x\n");
                }
                else {
                    // end of stream
                    uart_puts(UART_ID, "+m\n");       
                }
            }
            if (strcmp(cmd_str, "SRC") == 0) { // send recording (continue)
                send_recording();
                if (stream_head >= rec_tail) {
                    // end of stream
                    uart_puts(UART_ID, "+x\n");
                }
                else {
                    // end of stream
                    uart_puts(UART_ID, "+m\n");       
                }
            }

        }
        else {
            // add chars to the string
            if (cmd_str_ind < (sizeof(cmd_str)-1)) { // -1 to leave null termination
                cmd_str[cmd_str_ind] = ch;
                cmd_str_ind++;
            }

        }

    }
}

/* Respond with an integer encoded in hex, starting with +, ending with newline.
*/
void uart_resp_int(unsigned int msg){
    static uint8_t tx_toggle = 0;
    char msgstr[5] = "00000";

    tx_toggle++;
    gpio_put(FEEDBACK_TX, tx_toggle&1);

    sprintf(msgstr, "%04X", msg);
    if (uart_is_writable(UART_ID)) {
        uart_putc(UART_ID, '+');
        uart_puts(UART_ID, msgstr);
        uart_putc(UART_ID, '\n');
    }
}

/* Send the entire recording over serial
*/
void send_recording(){

    char msgstr[5] = "00000";
    for (uint8_t i=0; i<30; i++) {
        if (stream_head < rec_tail) {
            // write all the data, pausing if needed
            uart_putc(UART_ID, '+');

            sprintf(msgstr, "%04X", rec_data_buff[stream_head].Button);
            uart_puts(UART_ID, msgstr);
            
            sprintf(msgstr, "%02X", rec_data_buff[stream_head].HAT);
            uart_puts(UART_ID, msgstr);
            
            sprintf(msgstr, "%02X", rec_data_buff[stream_head].LX);
            uart_puts(UART_ID, msgstr);
            
            sprintf(msgstr, "%02X", rec_data_buff[stream_head].LY);
            uart_puts(UART_ID, msgstr);
            
            sprintf(msgstr, "%02X", rec_data_buff[stream_head].RX);
            uart_puts(UART_ID, msgstr);
            
            sprintf(msgstr, "%02X", rec_data_buff[stream_head].RY);
            uart_puts(UART_ID, msgstr);

            uart_putc(UART_ID, '\n');

            stream_head++;
        }
    }
}

/* Set a new amount of delay from VSYNC to controller data change.
*/
int set_frame_delay(char* cstr){
    for(uint8_t i=0; i<4; i++) {
        // error on any non-hex characters
        if ( !( (cstr[i]>='0' && cstr[i]<='9') || (cstr[i]>='A' && cstr[i]<='F') ) ) return -1;
    }
    frame_delay_us = hex2int(cstr, 4);
    return 0;
}

/* Add a new controller state to the buffer.
*  Incoming data is a hex-encoded string.
*/
int add_to_queue(char* cstr){

    for(uint8_t i=1; i<15; i++) {
        // error on any non-hex characters
        if ( !( (cstr[i]>='0' && cstr[i]<='9') || (cstr[i]>='A' && cstr[i]<='F') ) ) return -1;
    }

    // Increment the tail, wrapping if needed.
    queue_tail = (queue_tail+1) % CON_BUFF_LEN;
    // Add the new data to the new tail
    con_data_buff[queue_tail].Button = hex2int(cstr+1, 4);
    con_data_buff[queue_tail].HAT = hex2int(cstr+5, 2);
    con_data_buff[queue_tail].LX = hex2int(cstr+7, 2);
    con_data_buff[queue_tail].LY = hex2int(cstr+9, 2);
    con_data_buff[queue_tail].RX = hex2int(cstr+11, 2);
    con_data_buff[queue_tail].RY = hex2int(cstr+13, 2);

    // Assume that adding to the queue means the user wants to play the queue
    action_mode = A_PLAY;

    return get_queue_fill();
}

/* Returns the amount of space currently used in the playback buffer.
*/
unsigned int get_queue_fill(){
    // Account for the fact that the buffer wraps around.
    if (queue_tail>=queue_head) {
        return (unsigned int)(queue_tail-queue_head);
    } else {
        return (unsigned int)(CON_BUFF_LEN-(queue_head-queue_tail));
    }
}

/* Returns the amount of space currently used in the recording buffer.
*/
unsigned int get_recording_fill(){
    // Account for the fact that the buffer wraps around.
    if (queue_tail>=queue_head) {
        return (unsigned int)(queue_tail-queue_head);
    } else {
        return (unsigned int)(CON_BUFF_LEN-(queue_head-queue_tail));
    }
}

/* Set a new forced controller state (aka an immediate state).
*  Data is a hex-encoded string.
*/
int force_con_state(char* cstr){
    for(uint8_t i=1; i<15; i++) {
        // error on any non-hex characters
        if ( !( (cstr[i]>='0' && cstr[i]<='9') || (cstr[i]>='A' && cstr[i]<='F') ) ) return -1;
    }

    // Write the data to the "static" controller state variable.
    rt_con.Button = hex2int(cstr+1, 4);
    rt_con.HAT = hex2int(cstr+5, 2);
    rt_con.LX = hex2int(cstr+7, 2);
    rt_con.LY = hex2int(cstr+9, 2);
    rt_con.RX = hex2int(cstr+11, 2);
    rt_con.RY = hex2int(cstr+13, 2);

    // Assume that writing an immediate means the user wants to enter a real-time mode
    // (which includes recording mode).
    if ( !( (action_mode == A_RT) || (action_mode == A_REC)) )
        action_mode = A_RT;

    return get_queue_fill();
}

//--------------------------------------------------------------------
// Timer code
//--------------------------------------------------------------------

/* Alarm interrupt handler.
*/
static void alarm_irq(void) {
    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << 0);

    // Increment head as long as buffer isn't empty, wrapping when needed
    if (queue_head != queue_tail) {
        queue_head = (queue_head+1) % CON_BUFF_LEN;
    }

    // Copy current buffer entry to USB data
    if (action_mode == A_REC) {
        // in record mode, assume input is coming from real-time value.
        memcpy(&current_con, &rt_con, sizeof(USB_ControllerReport_Input_t));
        // copy to record buffer
        memcpy(&(rec_data_buff[rec_tail]), &current_con, sizeof(USB_ControllerReport_Input_t));
        // increment index
        if (rec_tail < sizeof(rec_data_buff)) rec_tail++;
    }
    else {
        // else assume playback is ocurring
        memcpy(&current_con, &(con_data_buff[queue_head]), sizeof(USB_ControllerReport_Input_t));
    }
}

/* Set up an alarm in the future.
*/
static void alarm_in_us(uint32_t delay_us) {
    // Enable the interrupt for the alarm
    hw_set_bits(&timer_hw->inte, 1u << 0);
    // Set irq handler for alarm irq
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    // Enable the alarm irq
    irq_set_enabled(ALARM_IRQ, true);
    // Enable interrupt in block and at processor
   uint64_t target = timer_hw->timerawl + delay_us;

    // Write the lower 32 bits of the target time to the alarm which
    // will arm it
    timer_hw->alarm[0] = (uint32_t) target;
}

//--------------------------------------------------------------------
// GPIO code
//--------------------------------------------------------------------

/* GPIO interrupt handler
*/
void gpio_callback(uint gpio, uint32_t events) {
    static uint8_t vsync_toggle = 0;
    // set up an interrupt in the future to change controller data
    alarm_in_us(frame_delay_us);

    vsync_toggle++;
    gpio_put(FEEDBACK_VSYNC, (vsync_toggle>>2)&1);
}


//--------------------------------------------------------------------
// Unknown code -- existed in template and can't be removed, but
// doesn't seem to do anything important.
//--------------------------------------------------------------------

uint16_t tud_hid_get_report_cb(uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) bufsize;
}

