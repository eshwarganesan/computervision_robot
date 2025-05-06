#include "robot_controller.h"
#include <cmath>
#include <algorithm>
#include <iostream>

Mode g_mode = ATTACK;		//default mode

static HANDLE hSer = INVALID_HANDLE_VALUE;
//static constexpr uint8_t STOP = 128;		//neutral PWM

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
void send_cmd(DriveCmd c) {
	if (hSer == INVALID_HANDLE_VALUE) return;
	char ch = static_cast<char>(c);
	DWORD n;
	WriteFile(hSer, &ch, 1, &n, nullptr);
}

/*static uint8_t clamp_pwm(int v) {
	if (v < 0) return 0;
	if (v > 255) return 255;
	return uint8_t(v);
}*/

static double normalize_angle(double angle) {
	// Normalize angle to the range [-pi, pi]
	while (angle > 3.14159) angle -= 2 * 3.14159;
	while (angle < -3.14159) angle += 2 * 3.14159;
	return angle;
}
DriveCmd decide_cmd(double fx, double fy, double bx, double by, double ofx, double ofy, double obx, double oby, double theta, const double obs_x[], const double obs_y[], const double obs_r[], int N_obs) {
	
	//get direction to opponent
	
	double cx = (fx + bx) / 2.0;
	double cy = (fy + by) / 2.0;
	double ox = (ofx + obx) / 2.0;
	double oy = (ofy + oby) / 2.0;

	double dx = ox - cx;
	double dy = oy - cy;

	double repulse_x = 0.0, repulse_y = 0.0;
	for (int i = 0; i < N_obs; ++i) {
		double odx = obs_x[i] - cx;
		double ody = obs_y[i] - cy;
		double dist2 = (odx * odx + ody * ody) - obs_r[i];

		if (dist2 < 15000) { //radius squared
			double scale = 10000 / dist2;
			repulse_x -= scale * odx;
			repulse_y -= scale * ody;
		}
	}
	double dist_left = cx - 0.0;
	if (dist_left > 1.0) {
		double scale = 10000.0 / (dist_left * dist_left);
		repulse_x += scale; //push rightt positive x
	}
	double dist_right = 640.0 - cx;
	if (dist_right > 1.0) {
		double scale = 10000.0 / (dist_right * dist_right);
		repulse_x -= scale; // push left negative x
	}
	double dist_top = cy - 0.0;
	if (dist_top > 1.0) {
		double scale = 10000.0 / (dist_top * dist_top);
		repulse_y += scale; // push down positive y
	}
	double dist_bottom = 480.0 - cy;
	if (dist_bottom > 1.0) {
		double scale = 10000.0 / (dist_bottom * dist_bottom);
		repulse_y -= scale; // push up negative y
	}
	

	double goal_theta = atan2(dy + repulse_y, dx + repulse_x);
	double angle_diff = normalize_angle(goal_theta - theta);

	/*
	//OUR position
	double x = (fx + bx) *0.5, y = (fy + by) *0.5;
	double theta = atan2(fy - by, fx - bx);

	//OPPONENT position
	double ox2 = (ox + obx) *0.5, oy2 = (oy + oby) *0.5;
	double dx = ox2 - x, dy = oy2 - y;
	double dist = std::hypot(dx, dy);
	double err = atan2(sin((dy,dx) - theta), cos((dy,dx) - theta));		//- pi to pi range
	*/
	int vl = 0, vr = 0;
	const int MAX = 90;
	/*
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

	
	
	double ox = (ofx + obx) * 0.5;
	double oy = (ofy + oby) * 0.5;
	double cx = (fx + bx) * 0.5;
	double cy = (fy + by) * 0.5;
	double dx = ox - cx;
	double dy = oy - cy;
	
	
	double goal_theta = atan2(dy, dx);
	double angle_diff = normalize_angle(goal_theta - theta);
	*/
	if (fabs(angle_diff) < 0.2) {
		return DriveCmd::FWD;	//move straight
	}
	else if (angle_diff > 0) {
		return DriveCmd::LEFT;
	}//rotate left
	else {
		return DriveCmd::RIGHT;
	}//rotate right
	//WheelCmd c;
	//c.left = clamp_pwm(STOP + vl);
	//c.right = clamp_pwm(STOP + vr);
	//return c;
}
