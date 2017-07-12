#ifndef EXTERNAL_CONTROL_H
#define EXTERNAL_CONTROL_H

#ifndef EXTERNAL_CONTROL_FILE
	#define EXTERNAL_CONTROL_EXT extern
#else 
	#define EXTERNAL_CONTROL_EXT
#endif

EXTERNAL_CONTROL_EXT uint8_t Chassis_Connected;
EXTERNAL_CONTROL_EXT uint8_t Gimbal_Connected;
EXTERNAL_CONTROL_EXT uint8_t DBUS_Connected;
EXTERNAL_CONTROL_EXT uint8_t Judge_Connected;
EXTERNAL_CONTROL_EXT int16_t chassis_ch2;
EXTERNAL_CONTROL_EXT int16_t last_ch_input[4];
EXTERNAL_CONTROL_EXT int16_t ch_input[4];
EXTERNAL_CONTROL_EXT int16_t mouse_input[2];
EXTERNAL_CONTROL_EXT int16_t last_mouse_input[2];
EXTERNAL_CONTROL_EXT uint8_t dancing;
EXTERNAL_CONTROL_EXT uint8_t rune;
EXTERNAL_CONTROL_EXT int16_t chassis_ch2_dancing_input;

void external_control_init(void);

void external_control(void);
void remote_control(void);
void computer_control(void);
void remote_buff_adjust(void);
void process_mouse_data(void);
void process_keyboard_data(void);

void dancing_mode(void);
void rune_mode(void);

void DBUS_disconnect_handler(void);
void gimbal_disconnect_handler(void);
void chassis_disconnect_handler(void);
void chassis_auto_stop(void);

#endif
