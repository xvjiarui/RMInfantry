#ifndef PARAM_H
#define PARAM_H

// gimbal speed
#define MOUSE_YAW_RATIO					6
#define MOUSE_PITCH_RATIO				2

#define CONTROLLER_YAW_RATIO			0.125f
#define CONTROLLER_PITCH_RATIO			1.0f

#define YAW_ACCELERATION				4
#define PITCH_ACCELERATION				4

// chassis speed
#define KEYBOARD_FWBW_RATIO				1.0f
#define KEYBOARD_RWLW_RATIO				0.833f

#define CONTROLLER_FWBW_RATIO			1.5
#define CONTROLLER_RWLW_RATIO			1.25

#define FWBW_ACCELERATION				1
#define RWLW_ACCELERATION				2

#define ROTATION_ACCELERATION 			150

#define DANCING_SPEED					900

#ifdef GOGOGO
#define FRICTION_WHEEL_PWM				1.0
#elif defined RUNRUNRUN
#define FRICTION_WHEEL_PWM				1.0
#else 
#define FRICTION_WHEEL_PWM				1.0
#endif

#endif
