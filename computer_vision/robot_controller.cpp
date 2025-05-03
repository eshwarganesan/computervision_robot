#include "robot_controller.h"
#include <cmath>
#include <algorithm>
#include <iostream>

Mode g_mode = ATTACK;		//default mode

static HANDLE hSer = INVALID_HANDLE_VALUE;
static constexpr uint8_t STOP = 128;		//neutral PWM

bool open_bt(const char* com) {
	hSer = CreateFileA(com, GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (hSer == INVALID_HANDLE_VALUE) {
		std::cerr << "Opening Bluetooth failed\n";
		return false;
	}
	//bluetooth baud rate, parity, and other comms settings
	DCB dcb{};
	dcb.DCBlength = sizeof(dcb);
	GetCommState(hSer, &dcb);
	dcb.BaudRate = CBR_9600;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	SetCommState(hSer, &dcb);
	return true;
}

void close_bt() {
	if (hSer != INVALID_HANDLE_VALUE) {
		CloseHandle(hSer);
		hSer = INVALID_HANDLE_VALUE;
	}
}
void send_cmd(const WheelCmd &c) {
	if (hSer == INVALID_HANDLE_VALUE) return;
	char pkt[3]{ char(c.left), char(c.right), '\n' };
	DWORD n; WriteFile(hSer, pkt, 3, &n, nullptr);
}

static uint8_t clamp_pwm(int v) {
	if (v < 0) return 0;
	if (v > 255) return 255;
	return uint8_t(v);
}

WheelCmd decide_cmd(double fx, double fy, double bx, double by, double ox, double oy, double obx, double oby, bool collision) {
	
	//OUR position
	double x = (fx + bx) *0.5, y = (fy + by) *0.5;
	double theta = atan2(fy - by, fx - bx);

	//OPPONENT position
	double ox2 = (ox + obx) *0.5, oy2 = (oy + oby) *0.5;
	double dx = ox2 - x, dy = oy2 - y;
	double dist = std::hypot(dx, dy);
	double err = atan2(sin((dy,dx) - theta), cos((dy,dx) - theta));		//- pi to pi range

	int vl = 0, vr = 0;
	const int MAX = 90;

	if(collision) {
		vl = -MAX;
		vr = MAX;
	}else if (g_mode == ATTACK) {
		double k = 1.5;
		vl = int(MAX - k*err*180);
		vr = int(MAX + k*err*180);

	}
	else {
		vl = vr = -MAX;		//reverse max speed
	}

	WheelCmd c;
	c.left = clamp_pwm(STOP + vl);
	c.right = clamp_pwm(STOP + vr);
	return c;
}
