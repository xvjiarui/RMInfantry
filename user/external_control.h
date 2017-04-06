#ifndef EXTERNAL_CONTROL_H
#define EXTERNAL_CONTROL_H

void external_control(void);
void remote_control(void);
void computer_control(void);
void remote_buff_adjust(void);
void process_mouse_data(void);
void process_keyboard_data(void);

void dancing_mode(void);

void DBUS_disconnect_handler(void);
void gimbal_disconnect_handler(void);
void chassis_disconnect_handler(void);
void chassis_auto_stop(void);

#endif
