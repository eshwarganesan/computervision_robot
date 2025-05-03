#pragma once

#include <Windows.h>
#include <cstdint>

//Startup choice
enum Mode { ATTACK, DEFENCE };
extern Mode g_mode;

struct WheelCmd {
	uint8_t left; uint8_t right;
};		//0-255 PWM

//Bluetooth communication and control
bool open_bt(const char* comPort);		//COM 7 port for my laptop
void close_bt();
void send_cmd(const WheelCmd &c);

//collision detection
WheelCmd decide_cmd(double fx, double fy, double bx, double by, double ox, double oy, double obx, double oby, bool collision);

