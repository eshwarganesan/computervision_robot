
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <Windows.h>

using namespace std;

#include "image_transfer.h"

#include "vision.h"

#include "timer.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

// you should divide your project into functions like I did below
// -- I recommend you use less global variable than I do because
// they are less safe -- use call by reference function parameters
// instead.
int activate();
int deactivate();
int find_object(i2byte &nlabel);
int select_object(i2byte &nlabel, image &label, image &a, image &b);
int search_object(i2byte &nlabel, image &label, int is, int js);
int track_object(i2byte nlabel);
int label_objects(int tvalue);

// declare some global image structures (globals are bad, but easy)
image a,b,rgb;
image rgb0; // original image before processing
image label;

int	tvalue = 79; // threshold value

const int IMAGE_WIDTH = 640;
const int IMAGE_HEIGHT = 480;

int cam_number = 0;

int main()
{ 
	i2byte nlabel;
	int width, height;

	activate_vision();
	
	// set camera number (normally 0 or 1)
	cam_number = 0; 
	width  = 640;
	height = 480;

	activate_camera(cam_number,height,width);	// activate camera

	cout << "\npress space to begin.";
	pause();

	// initialize the program and
	// create some images for processing
	activate();

	// find an object
	// and select it from the keyboard
	find_object(nlabel);

	// track an object's centroid
	// if the object moves the program will track
	// the new position of the centroid
	track_object(nlabel);

	// deactivate the program
	deactivate();

	deactivate_vision();

	cout << "\n\ndone.\n";
	pause();

 	return 0; // no errors
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

	rgb.type = RGB_IMAGE;
	rgb.width = IMAGE_WIDTH;
	rgb.height = IMAGE_HEIGHT;

	rgb0.type = RGB_IMAGE;
	rgb0.width = IMAGE_WIDTH;
	rgb0.height = IMAGE_HEIGHT;

	label.type = LABEL_IMAGE;
	label.width = IMAGE_WIDTH;
	label.height = IMAGE_HEIGHT;

	// allocate memory for the images
	allocate_image(a);
	allocate_image(b);
	allocate_image(rgb);
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
	free_image(rgb);
	free_image(rgb0);
	free_image(label);

	return 0; // no errors
}


int find_object(i2byte &nlabel)
// find an object
{ 
	cout << "\npress space to get an image";
	pause();

	acquire_image(rgb0,cam_number); // acquire an image from a video source (RGB format)

	// label objects in the image
	label_objects(tvalue);

	// select an object from the binary image
	select_object(nlabel,label,a,b);

	cout << "\nobject # = " << nlabel;

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
		acquire_image(rgb0,cam_number);

		// label objects
		label_objects(tvalue);

		copy(a,b); // threshold image is in a
		draw_point(b,i,j,128); // draw the new point
		copy(b,rgb);

		draw_point_rgb(rgb,i,j,0,0,255);
		draw_point_rgb(rgb,320,240,0,255,0);
		view_rgb_image(rgb);

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


int track_object(i2byte nlabel)
{
	int i=0;
	double ic=200.0,jc=300.0;
	
	cout << "\n\nnow tracking the object.";
	cout << "\nif the object moves the centroid marker";
	cout << "\nwill follow the object.";

	// compute the centroid of the object
	centroid(a,label,nlabel,ic,jc);
	
	while(1) {
		i++;

		// search for an object at the last known centroid location
		search_object(nlabel,label,(int)ic,(int)jc);

		// compute the centroid of the object
		centroid(a,label,nlabel,ic,jc);
		cout << "\ncentroid: ic = " << ic << " " << jc;

		// draw a point at centroid location (ic,jc) with intensity 128
		copy(rgb0,b);
//		draw_point(b,(int)ic,(int)jc,255);
		copy(b,rgb);    // convert to RGB image format

		draw_point_rgb(rgb,(int)ic,(int)jc,0,0,255);
		draw_point_rgb(rgb,320,240,0,255,0);
		view_rgb_image(rgb);

		// read the keyboard if a key is pressed
		if( KEY('X') ) break;

		// acquire an image from a video source (RGB format)
		acquire_image(rgb0,cam_number);

		// label objects
		label_objects(tvalue);

	} // end while

	return 0; // no errors
}


int label_objects(int tvalue)
{
	int nlabels;

	// convert RGB image to a greyscale image
	copy(rgb0,a);

	// scale the image to enhance contrast
	scale(a,a);

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

	return 0; // no errors
}

