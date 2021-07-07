#include <stdint.h>

// Controller HID report structure. We have an input and an output.
typedef struct {
	uint16_t Button; // 16 buttons; see ControllerButtons_t for bit mapping
	uint8_t  HAT;    // HAT switch; one nibble w/ unused nibble
	uint8_t  LX;     // Left  Stick X
	uint8_t  LY;     // Left  Stick Y
	uint8_t  RX;     // Right Stick X
	uint8_t  RY;     // Right Stick Y
	uint8_t  VendorSpec;
} USB_ControllerReport_Input_t;

// The output is structured as a mirror of the input.
// This is based on initial observations of the Pokken Controller.
typedef struct {
	uint16_t Button; // 16 buttons; see ControllerButtons_t for bit mapping
	uint8_t  HAT;    // HAT switch; one nibble w/ unused nibble
	uint8_t  LX;     // Left  Stick X
	uint8_t  LY;     // Left  Stick Y
	uint8_t  RX;     // Right Stick X
	uint8_t  RY;     // Right Stick Y
} USB_ControllerReport_Output_t;

// Type Defines
// Enumeration for controller buttons.
typedef enum {
	KEY_Y       = 0x01,
	KEY_B       = 0x02,
	KEY_A       = 0x04,
	KEY_X       = 0x08,
	KEY_L       = 0x10,
	KEY_R       = 0x20,
	KEY_ZL      = 0x40,
	KEY_ZR      = 0x80,
	KEY_SELECT  = 0x100,
	KEY_START   = 0x200,
	KEY_LCLICK  = 0x400,
	KEY_RCLICK  = 0x800,
	KEY_HOME    = 0x1000,
	KEY_CAPTURE = 0x2000,
} ControllerButtons_t;

// Action state
enum {
	A_PLAY, // play from buffer
	A_RT,   // real-time
	A_REC,  // record
	A_STOP  // stop
};

// Serial control information
enum {
    C_IDLE,        // nothing happening
    C_ACTIVATED,   // activated by command character
    C_Q,           // receiving a controller state for queue
    C_I,           // receiving an immediate controller state
	C_F,           // request for queue buffer fill amount
	C_M,           // mode change
	C_R,           // request to read from record buffer 
	C_D            // receiving a new delay value
};

// Output pins
enum {
	FEEDBACK_VSYNC  = 6,
	FEEDBACK_RX = 7, 
	FEEDBACK_TX = 8,
	FEEDBACK_PLAY  = 9,
	FEEDBACK_REC  = 10
};

#define CMD_CHAR '+'

#define UART_ID uart0
#define BAUD_RATE 57600
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define VSYNC_IND_PIN 15
#define MODE_IND_PIN 14
#define ALARM_IRQ TIMER_IRQ_0

#define CON_BUFF_LEN 256
#define REC_BUFF_LEN 16000



void hid_task(void);
void buffer_init();
static void feedback_GPIO_init();
int set_frame_delay(char* cstr);
int add_to_queue(char* cstr);
int force_con_state(char* cstr);
unsigned int get_queue_fill();
unsigned int get_recording_fill();
void uart_setup();
void on_uart_rx();
void uart_resp_int(unsigned int msg);
void send_recording();
void gpio_callback(uint gpio, uint32_t events);

int hex2int(char* ch, uint8_t num) {
	if ( (num<1) || (num>4) ) return -1;
	int val = 0, tval = 0;
	// Convert hex digits to number
	for (uint8_t i=0; i<num; i++) {
		val = val<<4;
		tval = ch[i]-'0';
		if (tval > 9) tval -= ('A'-'0'-10);
		val += tval;
	}
	return val;
}