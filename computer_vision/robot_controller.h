#pragma once

#include <Windows.h>
#include <cstdint>

//Startup choice
enum Mode { ATTACK, DEFENCE };
extern Mode g_mode;

struct WheelCmd {
	uint8_t left; uint8_t right;
};		//0-255 PWM

enum class DriveCmd : char {  // transmitted byte
    STOP = 'X',
    FWD = 'W',
    REV = 'S',
    LEFT = 'A',
    RIGHT = 'D'
};

//Bluetooth communication and control
bool open_bt(const char* comPort);		//COM 7 port for my laptop
void close_bt();
void send_cmd(DriveCmd c);

//collision detection
DriveCmd decide_cmd(double fx, double fy, double bx, double by, double ofx, double ofy, double obx, double oby, double theta, const double obs_x[], const double obs_y[], const double obs_r[], int N_obs);

