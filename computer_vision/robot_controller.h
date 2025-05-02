#pragma once
#include <Windows.h>
struct WheelCmd {
	uint8 left, right;
};		//0-255 PWM
bool open_bt(const char* com);		//COM 7 port for my laptop
void close_bt();
void send_cmd(const WheelCmd& c);
WheelCmd decide_cmd(double fx, double fy, double bx, double by, double ox, double oy, double obx, double oby);

enum Mode { ATTACK, DEFENCE };
extern Mode g_mode;		//we choose mode at the start
WheelCmd decide_cmd(double fx, double fy, double bx, double by, double ox, double oy, double obx, double oby);

