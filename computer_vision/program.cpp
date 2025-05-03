﻿
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <Windows.h>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <functional>
#include <filesystem>

using namespace std;

#include "image_transfer.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

#include "update_simulation.h"

#include "robot_controller.h"


extern robot_system S1;


int activate();
int deactivate();
int run_sim();
int run_vision();
int run_test();
int select_object(i2byte &nlabel, image &label, image &a, image &b);
int search_object(i2byte &nlabel, image &label, int is, int js);
int track_object(i2byte nlabel, double &ic, double &jc);
int label_objects(int tvalue);
void handle_keyboard_input(double dpw, int& pw_l, int& pw_r);
void calculate_HSV(int R, int G, int B, double& hue, double& sat, double& value);
int filter_color(image &a, image &b, double hue, double sat, double value, double htol, double stol, double vtol);
int object_area(image& label, int nlabel);
double get_orientation(double front_ic, double front_jc, double back_ic, double back_jc);
int get_front_centroid(double& front_x, double& front_y);
int get_back_centroid(double& back_x, double& back_y);
int get_opponent_front_centroid(double& front_x, double& front_y);
int get_opponent_back_centroid(double& back_x, double& back_y);
bool check_collision(double front_x, double front_y, double back_x, double back_y, double L, double W, const double* obstacle_x, const double* obstacle_y, const double* obstacle_r, int N_OBS);
bool has_line_of_sight(double x1, double y1, double x2, double y2);
int filter_obstacles();
void sobel_edge_detection(const image& input, image& output);

// declare some global image structures (globals are bad, but easy)
image a,b,rgb1;
image rgb0; // original image before processing
image label;

int	tvalue = 120; // threshold value

const int IMAGE_WIDTH = 640;
const int IMAGE_HEIGHT = 480;
int mod;
int cam_number = 0;		// for external cam on Ricky's laptop
double sim_robot_width = 52.5;
double sim_robot_length = 61.0;
//defining obstacles
const int N_OBS = 2;
double obs_x[N_OBS] = { 270.5, 135.0 };
double obs_y[N_OBS] = { 270.5, 135.0 };
double obs_r[N_OBS] = { 35.0, 35.0 };
const int BLACK_THRESH = 50;	//threshold for black color detection
const double OUTLINE_FRACTION = 0.3;	//fraction robot's colour circle thatt can be black (<0.3 currently)

extern robot_system S1;
int activate();
int deactivate();

int main()
{ 

	activate_vision();

	// initialize the program and
	// create some images for processing
	activate();
	 
	cout << "\nSelect module to run: \n1 - Simulator \n2 - Hardware \n3 - Test\n";
	cin >> mod;
	
	if (mod == 1) {
		cout << "\npress space key to begin program.";
		pause();

		run_sim();
		
	}
	else if (mod == 2) {
		int choice;
		cout << "\nSelect mode: 0 = ATTACK, 1 = DEFENCE :\n";
		cin >> choice;
		g_mode = (choice == 1) ? DEFENCE : ATTACK;

		if (!open_bt("COM7")) {
			cout << "Failed to open Bluetooth port.\n";
			return 1;
		}
		pause();
		run_vision();
		close_bt();
		
	}
	else if (mod == 3) {
		cout << "\npress space key to begin program.";
		pause();
		
		run_test();

	}
	

	// deactivate the program
	deactivate();

	deactivate_vision();

	cout << "\n\ndone.\n";
	pause();

 	return 0; // no errors
}

int run_test() {
	save_rgb_image("goodpic.png", rgb1);


	int nlabels;
	double ic, jc;

	load_rgb_image("goodpic.bmp", rgb1);
	view_rgb_image(rgb1);
	cout << "\ntest image rgb";
	pause();

	//sobel edge detection
	///*

	copy(rgb1, a);
	copy(a, rgb1);
	view_rgb_image(rgb1);
	cout << "\n image greyscale";
	pause();

	sobel_edge_detection(a, b);
	copy(b, rgb1);
	view_rgb_image(rgb1);
	cout << "\n sobel edge";
	pause();

	threshold(b, a, 200);
	copy(a, rgb1);
	view_rgb_image(rgb1);
	cout << "\nthreshold";
	pause();

	//*/

	//filter_color(rgb0, rgb1, 5.0, 0.7, 0.88, 5.0, 0.1, 0.1);
	//filter_color(rgb0, rgb1, 153.0, 0.6, 0.7, 5.0, 0.1, 0.1);
	/*
	
	view_rgb_image(rgb0);
	pause();
	nlabels = label_objects(tvalue);
	copy(a, rgb0);
	view_rgb_image(rgb0);

	for (int i = 1; i <= nlabels; i++)
	{
		int area = object_area(label, i);
		if (area >= 700 && area <= 4000) {

			centroid(a, label, i, ic, jc);
			cout << "\ncentroid: ic = " << ic << " jc = " << jc;

			// convert to RGB image format
			copy(a, rgb0);

			// mark the centroid point on the image with a blue point
			draw_point_rgb(rgb0, (int)ic, (int)jc, 0, 0, 255);

			view_rgb_image(rgb0);
			cout << "\nimage after a centroid is marked.";


			cout << "\nobject area = " << area;
			pause();
		}
	}


	

	filter_color(rgb1, rgb0, 5.0, 0.7, 0.88, 5.0, 0.1, 0.1);
	view_rgb_image(rgb0);
	pause();
	nlabels = label_objects(tvalue);
	copy(a, rgb0);
	view_rgb_image(rgb0);
	for (int i = 1; i <= nlabels; i++)
	{
		int area = object_area(label, i);
		if (area <= 700) {

			centroid(a, label, i, ic, jc);
			cout << "\ncentroid: ic = " << ic << " jc = " << jc;

			// convert to RGB image format
			copy(a, rgb0);

			// mark the centroid point on the image with a blue point
			draw_point_rgb(rgb0, (int)ic, (int)jc, 0, 0, 255);

			view_rgb_image(rgb0);
			cout << "\nimage after a centroid is marked.";


			cout << "\nobject area = " << area;
			pause();
		}
	}
	*/
	/*
	copy(rgb1, a);

	copy(a, rgb1);    // convert to RGB image format
	view_rgb_image(rgb1);
	cout << "\ntest image greyscale";

	scale(a, b);
	copy(b, a); // put result back into image a

	copy(a, rgb1);    // convert to RGB image format
	view_rgb_image(rgb1);
	cout << "\nimage scale function is applied";
	pause();

	lowpass_filter(a, b);
	copy(b, a);
	copy(a, rgb1);
	view_rgb_image(rgb1);
	cout << "\nimage after filter function is applied";
	pause();

	threshold(a, b, 200);
	copy(b, a);
	copy(a, rgb1); // convert to RGB image format
	view_rgb_image(rgb1);
	cout << "\nimage after threshold function is applied";
	pause();

	invert(a, b);
	copy(b, a);

	copy(a, rgb1);    // convert to RGB image format
	view_rgb_image(rgb1);
	cout << "\nimage after invert function is applied";
	pause();

	erode(a, b);
	copy(b, a);

	copy(a, rgb1);    // convert to RGB image format
	view_rgb_image(rgb1);
	cout << "\nimage after erosion function is applied";
	pause();

	dialate(a, b);
	copy(b, a);

	dialate(a, b);
	copy(b, a);

	copy(a, rgb1);    // convert to RGB image format
	view_rgb_image(rgb1);
	cout << "\nimage after dialation function is applied";
	pause();
	

	label_image(a, label, nlabels);
	*/
	/*
	for (int i = 1; i <= nlabels; i++)
	{

		centroid(a, label, i, ic, jc);
		cout << "\ncentroid: ic = " << ic << " jc = " << jc;

		// convert to RGB image format
		copy(a, rgb1);

		// mark the centroid point on the image with a blue point
		draw_point_rgb(rgb1, (int)ic, (int)jc, 0, 0, 255);

		view_rgb_image(rgb1);
		cout << "\nimage after a centroid is marked.";

		int area = object_area(label, i);
		cout << "\nobject area = " << area;
		pause();
	}
	*/
	///*
	
	
	//*/
	return 0;
}

int run_sim() {
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double width1, height1;
	int n_robot;
	double x_obs[50] = { 0.0 }, y_obs[50] = { 0.0 }, r_obs[50] = { 0.0 };
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;
	int capture = 0;
	int v_mode;
	i2byte nlabel{};
	double front_x, front_y, back_x, back_y, theta;

	//-----------------------------Initialization-------------------------------------------//
	width1 = 640;
	height1 = 480;

	const int N_obs = 2;

	char obstacle_file[N_obs][S_MAX] = {
		"obstacle_black.bmp" , "obstacle_green.bmp"
	};

	// obstacle locations
	// -- you must set one location for each obstacle

	x_obs[0] = 270.5; // pixels
	y_obs[0] = 270.5; // pixels

	x_obs[1] = 135; // pixels
	y_obs[1] = 135; // pixels

	r_obs[0] = 35;
	r_obs[1] = 35;

	D = 121.0;

	Lx = 31.0;
	Ly = 0.0;

	Ax = 37.0;
	Ay = 0.0;

	alpha_max = 3.14159 / 2;

	n_robot = 1;

	activate_simulation(width1, height1,
		x_obs, y_obs, N_obs,
		"robot_A.bmp", "robot_B.bmp", "background.bmp",
		obstacle_file, D, Lx, Ly,
		Ax, Ay, alpha_max, n_robot);

	mode = 0;
	set_simulation_mode(mode);

	x0 = 470;
	y0 = 170;
	theta0 = 0;
	set_robot_position(x0, y0, theta0);

	x0 = 150;
	y0 = 375;
	theta0 = 3.14159 / 4;
	set_opponent_position(x0, y0, theta0);

	pw_l = 0; // pulse width for left wheel servo (us)
	pw_r = 0; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; //

	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100;

	set_inputs(pw_l, pw_r, pw_laser, laser, max_speed);

	// opponent inputs
	pw_l_o = 0; // pulse width for left wheel servo (us)
	pw_r_o = 0; // pulse width for right wheel servo (us)
	pw_laser_o = 1500; // pulse width for laser servo (us)
	laser_o = 0; // laser input (0 - off, 1 - fire)

	// manually set opponent inputs for the simulation
	// -- good for testing your program
	set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
		opponent_max_speed);

	activate();

	tc0 = high_resolution_time();
	double dpw = 500;

	//-------------------------------------------------------------------------------//
	front_x, front_y, back_x, back_y = 0.0;
	double ic = 200.0, jc = 300.0;
	// initial simulation setup
	acquire_image_sim(rgb1);

	get_front_centroid(front_x, front_y);
	get_back_centroid(back_x, back_y);

	while (1) {

		//		update_background();
		//		update_obstacles();

				// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb0);

		//		update_image(rgb);

		tc = high_resolution_time() - tc0;

		// fire laser

		// change the inputs to move the robot around

		// pw_l -- pulse width of left servo (us) (from 1000 to 2000)
		// pw_r -- pulse width of right servo (us) (from 1000 to 2000)
		// pw_laser -- pulse width of laser servo (us) (from 1000 to 2000)
		// -- 1000 -> -90 deg
		// -- 1500 -> 0 deg
		// -- 2000 -> 90 deg
		// laser -- (0 - laser off, 1 - fire laser for 3 s)
		// max_speed -- pixels/s for right and left wheels


		handle_keyboard_input(dpw, pw_l_o, pw_r_o);

		// manually set opponent inputs for the simulation
		// -- good for testing your program

		set_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, max_speed);

		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
			opponent_max_speed);

		// * v_mode is an optional argument for view_rgb_image(...)
		// - adjusting it might improve performance / reduce delays
		// -- see "image_transfer.h" for more details
		v_mode = 1;
		
		track_object(nlabel, front_x, front_y);
		track_object(nlabel, back_x, back_y);
		theta = get_orientation(front_x, front_y, back_x, back_y);
		
		/*
		cout << "\rFront x: " << front_x
			<< "  Front y: " << front_y
			<< "  Back x: " << back_x
			<< "  Back y: " << back_y
			<< " Theta: " << theta
			<< flush;
		*/
		if (check_collision(front_x, front_y, back_x, back_y, sim_robot_length, sim_robot_width, x_obs, y_obs, r_obs, N_obs)) {
			cout << "\rCollision detected!" << flush;
		}
		else {
			cout << "\rNo Collision detected!" << flush;
		}
		view_rgb_image(rgb0, v_mode);

		if (KEY('X')) break;

		// * I removed the Sleep / delay function call below to 
		// improve performance / reduce delays, especially with 
		// player 1 / player 2 scenarios on laptops
		// -- it seems laptops tend to go into low CPU mode
		// when Sleep is called, which slows down the simulation
		// more than the requested sleep time
//		Sleep(10); // 100 fps max
	}

	return 0;
}

int run_vision() {

	i2byte nlabel;
	int nlabels = 0;
	double ic, jc;

	ic = 200;
	jc = 300;

	int cam_number = 0;			// Should be 0 for external camera
	int width = 640;
	int height = 480;

	activate_camera(cam_number, height, width);	// activate camera

	acquire_image(rgb0, cam_number); // acquire an image from a video source (RGB format)	

	//label objects in the image
	label_objects(tvalue);
	select_object(nlabel, label, a, b); // select an object to track

	// tracking
	//centroid(a, label, nlabel, ic, jc);
	double fx = 0, fy = 0, bx = 0, by = 0;
	get_front_centroid(fx, fy);
	get_back_centroid(bx, by);

	double ox = 0, oy = 0, obx = 0, oby = 0;
	//get_opponent_front_centroid(ox, oy);
	//get_opponent_back_centroid(obx, oby);


	while (1) {
		acquire_image(rgb0, cam_number);
		copy(rgb0, a);
		scale(a, a);
		threshold(a, a, tvalue);
		invert(a, a);
		label_image(a, label, nlabels);
		centroid(a, label, nlabel, fx, fy);
		draw_point_rgb(rgb0, fx, fy, 0, 0, 255);
		//bool avoid = check_collision(fx, fy, bx, by, sim_robot_length*0.5, sim_robot_width*0.5, obs_x, obs_y, obs_r, N_OBS);

		//WheelCmd cmd = decide_cmd(fx, fy, bx, by, ox, oy, obx, oby, avoid);

		//send_cmd(cmd);

		view_rgb_image(rgb0, 1);

		if (KEY('X')) break;
	}
	

	return 0;
}

int activate()
// initialize the program
{
	// set the type and size of the images
	a.type = GREY_IMAGE;
	a.width = IMAGE_WIDTH;
	a.height = IMAGE_HEIGHT;

	b.type = GREY_IMAGE;
	b.width = IMAGE_WIDTH;
	b.height = IMAGE_HEIGHT;

	rgb1.type = RGB_IMAGE;
	rgb1.width = IMAGE_WIDTH;
	rgb1.height = IMAGE_HEIGHT;

	rgb0.type = RGB_IMAGE;
	rgb0.width = IMAGE_WIDTH;
	rgb0.height = IMAGE_HEIGHT;

	label.type = LABEL_IMAGE;
	label.width = IMAGE_WIDTH;
	label.height = IMAGE_HEIGHT;

	// allocate memory for the images
	allocate_image(a);
	allocate_image(b);
	allocate_image(rgb1);
	allocate_image(rgb0);
	allocate_image(label);

	return 0; // no errors
}


int deactivate()
// terminate the program
{
	// free the image memory before the program completes
	free_image(a);
	free_image(b);
	free_image(rgb1);
	free_image(rgb0);
	free_image(label);

	return 0; // no errors
}


int select_object(i2byte &nlabel, image &label, image &a, image &b)
// select an object from a binary image
// a - image
// b - temp. image
{
	i2byte *pl;
	int i,j;

	// start in the image
	i = 200;
	j = 300;
	
	cout << "\nselect an object to track by moving the";
	cout << "\ngrey point to the desired object,";
	cout << "\nand then pressing the c key to continue";

	while(1) {
		
		// acquire image
		
		acquire_image(rgb0, cam_number); // acquire an image from a video source (RGB format)

		// label objects
		label_objects(tvalue);

		copy(a,b); // threshold image is in a
		draw_point(b,i,j,128); // draw the new point
		copy(b,rgb1);

		draw_point_rgb(rgb1,i,j,0,0,255);
		draw_point_rgb(rgb1,320,240,0,255,0);
		view_rgb_image(rgb1);

		// read the keyboard if a key is pressed
		if( KEY(VK_UP) ) j+=3; // up key
		if( KEY(VK_DOWN) ) j-=3; // down key
		if( KEY(VK_LEFT) ) i-=3; // left key
		if( KEY(VK_RIGHT) ) i+=3; // right key
			
		// limit (i,j) from going off the image
		if( i < 0 ) i = 0;
		if( i > b.width-1 ) i = b.width-1;
		if( j < 0 ) j = 0;
		if( j > b.height-1 ) j = b.height-1;			
			
		if( KEY('C') ) break;	

	} // end while

	// pointer to a label image
	pl = (i2byte *)label.pdata;

	// get the label value at co-ordinate (i,j)
	nlabel = *( pl + j*label.width + i );

	return 0; // no errors
}


int search_object(i2byte &nlabel, image &label, int is, int js)
// search for a labeled object in an outward spiral pattern
// and inital search location (is,js)
// *** Please study this function carefully
// -- more upcoming assignment and project problems 
// are related to this
{
	i2byte *pl;
	double r,rmax,dr,s,smax,ds,theta;
	int i,j;

	// pointer to a label image
	pl = (i2byte *)label.pdata;

	// check if an object exists at the current location
	nlabel = *( pl + js*label.width + is );
	if( nlabel != 0 ) return 0;

	rmax = 60.0; // maximum radius of search (pixels)
	dr = 3.0; // radius divisions (pixels)
	ds = 3.0; // arc-length divisions (pixels)

	// search for a labeled object in an outward concentic ring pattern
	for(r=1.0;r<=rmax;r+=dr) {
		smax = 2*3.1416*r; // maximum arc length
		for(s=0;s<=smax;s+=ds) {
			theta = s/r; // s = r*theta
			i = (int)(is + r*cos(theta));
			j = (int)(js + r*sin(theta));

			// limit (i,j) from going off the image
			if( i < 0 ) i = 0;
			if( i > label.width-1 ) i = label.width-1;
			if( j < 0 ) j = 0;
			if( j > label.height-1 ) j = label.height-1;

//			*( b.pdata + j*b.width + i ) = 128; // check pattern

			// check if there is an object at location (i,j)
			nlabel = *( pl + j*label.width + i );
			if( nlabel != 0 ) return 0;
		}
	}
	
	return 0; // no errors
}


int track_object(i2byte nlabel, double &ic, double &jc)
{

	label_objects(tvalue);
	search_object(nlabel, label, (int)ic, (int)jc);
	centroid(a, label, nlabel, ic, jc);
	draw_point_rgb(rgb0, ic, jc, 0, 0, 255);

	return 0; // no errors
}


int label_objects(int tvalue)
{
	int nlabels;

	// convert RGB image to a greyscale image
	copy(rgb0,a);

	// scale the image to enhance contrast
	scale(a,a);

	//filter
	lowpass_filter(a, b);
	copy(b, a);

	// use threshold function to make a binary image (0,255)
	threshold(a,a,tvalue);

	// invert the image
	invert(a,a);

	// perform an erosion function to remove noise (small objects)
	erode(a,b);

	// perform a dialation function to fill in 
	// and grow the objects
	dialate(b,a);

	// label the objects in a binary image
	// labels go from 1 to nlabels
	label_image(a,label,nlabels);

	return nlabels; // no errors
}

void handle_keyboard_input(double dpw, int& pw_l, int& pw_r) {
	if (KEY('I')) { pw_l = 1500 - dpw; pw_r = 1500 + dpw; }
	if (KEY('K')) { pw_l = 1500 + dpw; pw_r = 1500 - dpw; }
	if (KEY('J')) { pw_l = 1500 + dpw; pw_r = 1500 + dpw; }
	if (KEY('L')) { pw_l = 1500 - dpw; pw_r = 1500 - dpw; }
}

void calculate_HSV(int R, int G, int B, double& hue, double& sat, double& value)
{
	double rf = R / 255.0;
	double gf = G / 255.0;
	double bf = B / 255.0;

	double max_val = max(rf, max(gf, bf));
	double min_val = min(rf, min(gf, bf));
	double delta = max_val - min_val;

	value = max_val;

	if (delta < 1e-6) {
		hue = 0;
		sat = 0;
		return;
	}

	sat = delta / max_val;

	if (max_val == rf) {
		hue = 60.0 * (fmod(((gf - bf) / delta), 6.0));
	}
	else if (max_val == gf) {
		hue = 60.0 * (((bf - rf) / delta) + 2.0);
	}
	else { // max_val == bf
		hue = 60.0 * (((rf - gf) / delta) + 4.0);
	}

	if (hue < 0.0) {
		hue += 360.0;
	}

}

int filter_color(image& a, image& b, double hue, double sat, double value, double htol, double stol, double vtol) {
	i4byte size, i;
	ibyte* pa, * pb, min, max;
	double h, s, v;
	int red, green, blue;

	copy(a, b);

	pa = a.pdata;
	pb = b.pdata;

	if (a.height != b.height || a.width != b.width) {
		cout << "\nerror in scale: sizes of a, b are not the same!";
		return 1;
	}
	if (a.type != b.type) {
		cout << "\nerror in scale: types of a, b are not the same!";
		return 1;
	}
	if (a.type != RGB_IMAGE || b.type != RGB_IMAGE) {
		cout << "\nerror in scale: types of a, b are not the same!";
		return 1;
	}

	size = (i4byte)a.width * a.height * 3;
	for (i = 0; i < size; i += 3) {
		blue = pa[i];
		green = pa[i + 1];
		red = pa[i + 2];
		
		
		calculate_HSV(red, green, blue, h, s, v);

		if (h >= hue - htol && h <= hue + htol && s >= sat - stol && s <= sat + stol && v >= value - vtol && v <= value + vtol) {
			continue;
		}
		else {
			pb[i] = 255;
			pb[i + 1] = 255;
			pb[i + 2] = 255;
		}
		
	}

}

static void build_black_mask(image& blackmask) {
	copy(rgb0, blackmask);
	threshold(blackmask, blackmask, BLACK_THRESH);
	invert(blackmask, blackmask);
}

static bool has_black_outline(int lbl, image& labelImg, image& blackmask) {
	int W = labelImg.width, H = labelImg.height;

	image region;
	region.type = GREY_IMAGE;
	region.width = W;
	region.height = H;
	allocate_image(region);

	for (int i = 0, n = W * H; i < n; ++i) {
		region.pdata[i] = (((i2byte*)*labelImg.pdata)[i] == lbl ? 255 : 0);

		image dilated;
		dilated.type = GREY_IMAGE;
		dilated.width = W;
		dilated.height = H;
		allocate_image(dilated);
		dialate(region, dilated);

		image ring;		//ring = dilated - region
		ring.type = GREY_IMAGE;
		ring.width = W;
		ring.height = H;
		allocate_image(ring);

		invert(region, region);
		filter_color(dilated, ring, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);	//ring = pixels were dilated ==255 AND region(inverted) ==255
		int ringCount = 0, blackCount = 0;
		for (int i = 0, n = W * H; i < n; ++i) {	//counting ring pixels and verifying == to blackmask
			if (ring.pdata[i]) {
				++ringCount;
				if (blackmask.pdata[i]) {
					++blackCount;
				}
			}
		}
		free_image(region);
		free_image(dilated);
		free_image(ring);

		return ringCount > 0 && (double)blackCount / ringCount >= OUTLINE_FRACTION;

	}
}

int object_area(image& label, int nlabel) {
	ibyte* pl;
	i4byte size, i;

	int area = 0;
	pl = label.pdata;

	size = label.height * label.width;
	for (i = 0; i < size; i++) {
		if (pl[i] == nlabel) area++;
	}

	return area;
}

double get_orientation(double front_ic, double front_jc, double back_ic, double back_jc) {
	double dx = front_ic - back_ic;
	double dy = front_jc - back_jc;
	double theta = atan2(dy, dx);
	return theta;
}

int get_front_centroid(double &front_x, double &front_y) {
	filter_color(rgb1, rgb0, 153.0, 0.6, 0.7, 5.0, 0.1, 0.1); // GREEN
	int nlabels = label_objects(tvalue);
	image blackmask;
	blackmask.type = GREY_IMAGE;
	blackmask.width = IMAGE_WIDTH;
	blackmask.height = IMAGE_HEIGHT;
	allocate_image(blackmask);
	build_black_mask(blackmask);
	for (int i = 1; i <= nlabels; i++) {
		int area = object_area(label, i);
		if (area < 100 || area >4000) continue;
		if (!has_black_outline(i, label, blackmask)) continue;
		centroid(a, label, i, front_x, front_y);
		free_image(blackmask);
		return 0;
	}
	free_image(blackmask);
	return 1;
}

int get_back_centroid(double& back_x, double& back_y) {
	filter_color(rgb1, rgb0, 5.0, 0.65, 0.89, 2, 0.05, 0.05);  // RED
	int nlabels = label_objects(tvalue);
	image blackmask;
	blackmask.type = GREY_IMAGE;
	blackmask.width = IMAGE_WIDTH;
	blackmask.height = IMAGE_HEIGHT;
	allocate_image(blackmask);
	build_black_mask(blackmask);
	for (int i = 1; i <= nlabels; i++) {
		int area = object_area(label, i);
		if (area < 100 || area >4000) continue;
		if (!has_black_outline(i, label, blackmask)) continue;
		centroid(a, label, i, back_x, back_y);
		free_image(blackmask);
		return 0;
	}
	free_image(blackmask);
	return 1;
}

int get_opponent_front_centroid(double &front_x, double &front_y)
{
	filter_color(rgb1, rgb0, 30.0, 0.6, 0.8, 10.0, 0.2, 0.2); //ORANGE
	int nlabels = label_objects(tvalue);

	image blackmask = { GREY_IMAGE, IMAGE_WIDTH, IMAGE_HEIGHT, nullptr };
	allocate_image(blackmask);
	build_black_mask(blackmask);
	for (int i = 1; i <= nlabels; i++) {
		int area = object_area(label, i);
		if (area < 100 || area >4000) continue;
		if (!has_black_outline(i, label, blackmask)) continue;
		centroid(a, label, i, front_x, front_y);
		free_image(blackmask);
		return 0;
	}
	free_image(blackmask);
	return 1;
}

int get_opponent_back_centroid(double &back_x, double &back_y)
{
	filter_color(rgb1, rgb0, 210.0, 0.6, 0.7, 10.0, 0.2, 0.2); //BLUE
	int nlabels = label_objects(tvalue);

	image blackmask = { GREY_IMAGE, IMAGE_WIDTH, IMAGE_HEIGHT, nullptr };
	allocate_image(blackmask);
	build_black_mask(blackmask);
	for (int i = 1; i <= nlabels; i++) {
		int area = object_area(label, i);
		if (area < 100 || area >4000) continue;
		if (!has_black_outline(i, label, blackmask)) continue;
		centroid(a, label, i, back_x, back_y);
		free_image(blackmask);
		return 0;
	}
	free_image(blackmask);
}

bool check_collision(double front_x, double front_y, double back_x, double back_y, double half_L, double half_W, const double* obstacle_x, const double* obstacle_y, const double* obstacle_r, int N_OBS) {
	double x_center = (front_x + back_x) / 2.0;
	double y_center = (front_y + back_y) / 2.0;
	double theta = atan2(front_y - back_y, front_x - back_x);

	double cos_theta = cos(theta);
	double sin_theta = sin(theta);

	// Check edge collision using actual rectangle corners
	double corner_offsets[4][2] = {
		{ half_L,  half_W},
		{ half_L, -half_W},
		{-half_L, -half_W},
		{-half_L,  half_W}
	};

	for (int i = 0; i < 4; i++) {
		double dx = corner_offsets[i][0];
		double dy = corner_offsets[i][1];

		double x = x_center + dx * cos_theta - dy * sin_theta;
		double y = y_center + dx * sin_theta + dy * cos_theta;

		if (x < 0 || x >= IMAGE_WIDTH || y < 0 || y >= IMAGE_HEIGHT) {
			return true; // collision with edge
		}
	}

	// Check obstacle collisions using full rotated rectangle test
	for (int i = 0; i < N_OBS; ++i) {
		double xo = obstacle_x[i];
		double yo = obstacle_y[i];
		double ro = obstacle_r[i];

		// Translate obstacle into robot-centered coordinates
		double dx = xo - x_center;
		double dy = yo - y_center;

		// Rotate into robot's local (unrotated) frame
		double xr = dx * cos_theta + dy * sin_theta;
		double yr = -dx * sin_theta + dy * cos_theta;

		// Clamp circle center to rectangle bounds
		double x_closest = max(-half_L, min(xr, half_L));
		double y_closest = max(-half_W, min(yr, half_W));

		// Distance from obstacle to closest point
		double ddx = xr - x_closest;
		double ddy = yr - y_closest;

		if ((ddx * ddx + ddy * ddy) <= (ro * ro)) {
			return true; // collision with circular obstacle
		}
	}

	return false; // No collision
}

bool has_line_of_sight(double x1, double y1, double x2, double y2) {
	int steps = static_cast<int>(hypot(x2 - x1, y2 - y1));
	if (steps == 0) return true;

	double dx = (x2 - x1) / steps;
	double dy = (y2 - y1) / steps;

	ibyte* pdata = (ibyte*)label.pdata;
	int width = label.width;
	int height = label.height;

	for (int i = 0; i <= steps; ++i) {
		int xi = static_cast<int>(x1 + i * dx + 0.5);
		int yi = static_cast<int>(y1 + i * dy + 0.5);

		if (xi < 0 || xi >= width || yi < 0 || yi >= height) return false; // out of bounds

		if (pdata[yi * width + xi] > 0) return false; // obstacle in the way
	}
	return true; // clear path
}

void sobel_edge_detection(const image& input, image& output) {
	if (input.type != GREY_IMAGE || output.type != GREY_IMAGE ||
		input.width != output.width || input.height != output.height) {
		cout << "Error: input and output images must be GREY_IMAGE type and same size." << endl;
		return;
	}

	int w = input.width;
	int h = input.height;
	ibyte* in = (ibyte*)input.pdata;
	ibyte* out = (ibyte*)output.pdata;

	// Sobel kernels
	int Gx[3][3] = {
		{ -1,  0,  1 },
		{ -2,  0,  2 },
		{ -1,  0,  1 }
	};
	int Gy[3][3] = {
		{ -1, -2, -1 },
		{  0,  0,  0 },
		{  1,  2,  1 }
	};

	// Ignore border pixels
	for (int y = 1; y < h - 1; ++y) {
		for (int x = 1; x < w - 1; ++x) {
			int sumX = 0, sumY = 0;

			// Apply kernels
			for (int ky = -1; ky <= 1; ++ky) {
				for (int kx = -1; kx <= 1; ++kx) {
					int pixel = in[(y + ky) * w + (x + kx)];
					sumX += Gx[ky + 1][kx + 1] * pixel;
					sumY += Gy[ky + 1][kx + 1] * pixel;
				}
			}

			int magnitude = (int)sqrt(sumX * sumX + sumY * sumY);
			magnitude = min(255, max(0, magnitude));  // Clamp to 0�255
			out[y * w + x] = (ibyte)magnitude;
		}
	}
}
