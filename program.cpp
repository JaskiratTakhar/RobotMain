
		//Project_1_Scale_Centroid

		#include <stdio.h>
		#include <stdlib.h>
		#include <conio.h>
		#include <math.h>
		#include <iostream>

		#include <windows.h>

		// serial communication functions
		#include "serial_com.h"

		// include this header file for basic image transfer functions
		#include "image_transfer2.h"

		// include this header file for computer vision functions
		#include "vision.h"

		#include "timer.h"

		using namespace std;

		int activate();
		int deactivate();
		int find_object(i2byte &nlabel);
		int find_Block(double Block[5+1], i2byte &nlabel);

		int activate_Object(image &a_RGB, i2byte &nlabel1, i2byte &nlabel2,
			ibyte ArrayColor[4 + 1][3 + 1], double ArrayHu[4 + 1][4 + 1], double ArrayPoint[3 + 1][3 + 1]);

		int select_object(i2byte &nlabel, image &label, image &a, image &b);

		int search_object(image &a, i2byte &nlabel, image &label,
			ibyte ArrayColor[4 + 1][3 + 1], double ArrayHu[4 + 1][4 + 1], double ArrayPoint[3 + 1][3 + 1], int &Flag, int Number);

		int centroid_Search(image &a, image &label, i2byte &nlabel,
			ibyte ArrayColor[4 + 1][3 + 1], double ArrayHu[4 + 1][4 + 1], double ArrayPoint[3 + 1][3 + 1], int &Flag, int Number);

		int pixel_color_Search(ibyte p1, ibyte p2, ibyte p3, ibyte ArrayColor[4 + 1][3 + 1],
			double ArrayHu[4 + 1][4 + 1], int &FlagColor, int Number);

		int label_objects(int tvalue);
		int draw_point_RGB(image &rgb, double ip1, double jp1, int R, int G, int B);

		int TETA_Object(double ArrayPoint[3 + 1][3 + 1], double &TETA);

		int TURN(double TETA, double TETA_NEW, unsigned char &COM, unsigned char DIR,
			unsigned char &PAR_LEFT, unsigned char &PAR_RIGHT, int &Flag_TETA);

		int DIR_TURN(double TETA, double TETA_NEW, unsigned char &DIR);

		int POSITION(double ArrayPoint_1[3 + 1][3 + 1], double ArrayPoint_2[3 + 1][3 + 1], double ArrayBlock[5+1], double &TETA_R,
			double &TETA_Tar, double &TETA_NEW, double DPoint[2 + 1], int &Flag_TETA, int &Flag_Point, int &Flag_SHOOT,
			double Size_Robot, double Size_Target);

		int MOVE(double ArrayPoint[3 + 1][3 + 1], double Array_DPoint[2 + 1], unsigned char &COM, unsigned char &DIR,
			unsigned char &PAR_LEFT, unsigned char &PAR_RIGHT, int &Flag_MOVE);

		int draw_point(image &a, double ip, double jp, int value);

		// declare some global image structures (globals are bad, but easy)
		image a,b,rgb, rgb1, rgb2;
		image rgb0; // original image before processing
		image label;		
		i4byte size;

		int	tvalue = 250; // threshold value
		double HSVtvalue = 0.4; //HSV treshhold value
		double t0;

		ibyte RobotColor[4+1][3+1], TargetColor [4+1][3+1];	// Bmax1, Gmax1, Rmax1 //Color max Obj #1
															// Bmin1, Gmin1, Rmin1 //Color min Obj #1
															// Bmax2, Gmax2, Rmax2 //Color max Obj #2
															// Bmin2, Gmin2, Rmin2 //Color min Obj #2
		
		double RobotHu[4+1][4+1], TargetHu[4+1][4+1];		//Hu_max1, Hu_min1 //Obj #1
															//Hu_max2, Hu_min2 //Obj #1
		int Obj_Number, Flag1 = 0, Flag2 = 0, Flag3 = 0, Flag4 = 0;
		int Flag_TETA=1, Flag_Point=1, Flag_SHOOT=0, Flag_COM=0;										// Object #
		
		double RobotPoint[3 + 1][3 + 1], TargetPoint[3 + 1][3 + 1]; //ic1, jc1 //center point of centroid #1
																	//ic2, jc2 //center point of centroid #2
																	//X, Y //center point of Object
		
		double Block[5+1];		// Obstacle parameters: icB, jcB, Mmin, Mmax, Rmax
		
		double TETA_Robot, TETA_Target, TETA_NEW;		//
		double Size_Robot, Size_Target, R_Robot, DPoint[2+1];		//DPoint [ic, jc], R_Robot - max Robot's Radius (for turning)
		
		// macro that check for a key press
		#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

		int main(int argc, char* argv[])
		{ 
			i2byte nlabel1, nlabel2, nlabel3, nlabel4, nlabelBlock; // 1,2 - Robot points, 3,4 - Target points
			int camera;

			HANDLE h1;
			const int NMAX = 64;
			char buffer_out[NMAX];
			int n, i;

			unsigned char COM, DIR, PAR_LEFT, PAR_RIGHT;
	
			camera = 0;
			activate_vision(camera);

			printf("\npress any key to begin");
			getch();
			
			activate();			
			acquire_image(rgb0);			
			copyRGB(rgb0, rgb1);
			scaleRGB(rgb1, rgb1, HSVtvalue);
			view_image(rgb1);

			cout << "\nPut your Objects on the fild";
			acquire_image(rgb0);
			copyRGB(rgb0, rgb1);			
			scaleRGB(rgb1, rgb1, HSVtvalue);
			getch();
			
			find_Block(Block, nlabelBlock);
			getch();
			//draw_point_RGB(rgb1, Block[1], Block[2], 255, 0, 0);
			//draw_point_RGB(rgb1, Block[1], (Block[2] + Block[5]), 255, 0, 0);
			//draw_point_RGB(rgb1, (Block[1] + Block[5]), Block[2], 255, 0, 0);

			activate_Object(rgb1, nlabel1, nlabel2, RobotColor, RobotHu, RobotPoint);		
			getch();

			activate_Object(rgb1, nlabel3, nlabel4, TargetColor,TargetHu, TargetPoint);
			getch();

			Size_Robot = sqrt(pow((RobotPoint[1][1] - RobotPoint[2][1]), 2.0) + pow((RobotPoint[1][2] - RobotPoint[2][2]), 2.0));
			R_Robot = 1.6*Size_Robot;
			Size_Target = sqrt(pow((TargetPoint[1][1] - TargetPoint[2][1]), 2.0) + pow((TargetPoint[1][2] - TargetPoint[2][2]), 2.0));

			cout << "\nSize_Robot ="<<Size_Robot;
			cout << "\nR_Robot =" << R_Robot;

			view_image(rgb1);

			open_serial("COM3", h1);

			cout << "\npress c key to continue";
			while (!KEY('C')) Sleep(1);

			// send start message to Arduino
			// note: setup() ends and loop() begins after this	
			n = 1;
			buffer_out[0] = 's';
			serial_send(buffer_out, n, h1);
			Sleep(100);

			cout << "\n\npress arrow keys to control robot (press x to exit)\n";
			getch();

			//t0 = high_resolution_time();
	
			while (1) {
				unsigned char ch;
				
				acquire_image(rgb0);
				copyRGB(rgb0, rgb1);
				scaleRGB(rgb1, rgb1, HSVtvalue);
				label_objects(tvalue);
				
				search_object (rgb1, nlabel1, label, RobotColor, RobotHu, RobotPoint, Flag1, 1);
				search_object (rgb1, nlabel2, label, RobotColor, RobotHu, RobotPoint, Flag2, 2);
				search_object(rgb1, nlabel3, label, TargetColor, TargetHu, TargetPoint, Flag3, 1);
				search_object(rgb1, nlabel4, label, TargetColor, TargetHu, TargetPoint, Flag4, 2);
				//centroid_color_object_Hu(rgb1, RobotColor, RobotHu, RobotPoint);
				
				///*
				//draw_point_RGB(rgb1, Block[1], Block[2], 255, 0, 0);
				
				draw_point_RGB(rgb1, RobotPoint[1][1], RobotPoint[1][2], 0, 255, 0);
				draw_point_RGB(rgb1, RobotPoint[2][1], RobotPoint[2][2], 0, 255, 0);

				draw_point_RGB(rgb1, TargetPoint[1][1], TargetPoint[1][2], 0, 255, 0);
				draw_point_RGB(rgb1, TargetPoint[2][1], TargetPoint[2][2], 0, 255, 0);
				draw_point_RGB(rgb1, TargetPoint[3][1], TargetPoint[3][2], 255, 0, 0);

				//draw_point_RGB(rgb1, DPoint[1], DPoint[2], 255, 0, 0);

				//draw_point(a, TargetPoint[1][1], TargetPoint[1][2], 180);
				//draw_point(a, TargetPoint[2][1], TargetPoint[2][2], 180);
				//copy(a, rgb1);				
				view_image(rgb1);
				//*/

				
					TETA_Object(RobotPoint, TETA_Robot);
					//cout << " TETA =" << TETA_Robot;
					//cout << " TETA_NEW =" << TETA_NEW;
					
					if (Flag_TETA == 1 && Flag_Point == 1 && Flag_SHOOT == 1 && Flag_COM == 2) {
						Sleep(500);
						COM = 1;
						PAR_LEFT = 0;
						PAR_RIGHT = 0;
						Flag_COM = 3;
					}
					if (Flag_TETA == 1 && Flag_Point == 1 && Flag_SHOOT == 1 && Flag_COM == 1) {
						Sleep(500);
						COM = 5;
						Flag_COM = 2;
					}
				
					if (Flag_TETA == 1 && Flag_Point == 1 && Flag_SHOOT == 0){
					
					POSITION(RobotPoint, TargetPoint, Block,
							TETA_Robot, TETA_Target, TETA_NEW, DPoint, Flag_TETA, Flag_Point, Flag_SHOOT, Size_Robot, Size_Target);
					
					if (Flag_TETA == 0) DIR_TURN(TETA_Robot, TETA_NEW, DIR);
					}

				if (Flag_TETA == 0) TURN(TETA_Robot, TETA_NEW, COM, DIR, PAR_LEFT, PAR_RIGHT, Flag_TETA);

				if (Flag_TETA == 1 && Flag_Point == 0 && Flag_SHOOT == 0) {
					
					MOVE(RobotPoint, DPoint, COM, DIR, PAR_LEFT, PAR_RIGHT, Flag_Point);				
				}				

				if (Flag_TETA == 1 && Flag_Point == 1 && Flag_SHOOT == 1 && Flag_COM == 0) { 
					COM = 4; 
					Flag_COM = 1;
				}				

				//cout << " \nFlag_TETA =" << Flag_TETA;
				//cout << " Flag_TPoint =" << Flag_Point;
				//cout << " Flag_SHOOT =" << Flag_SHOOT;
				//cout << " COM =" << (int)COM;
				//cout << " DIR =" << (int)DIR;

				buffer_out[0] = 254; // message start character
				buffer_out[1] = COM;
				buffer_out[2] = DIR; //1;// DIR = 1;
				buffer_out[3] = PAR_LEFT;
				buffer_out[4] = PAR_RIGHT;
				n = 5;
				serial_send(buffer_out, n, h1);
				Sleep(30); // 30 fps	

			} // end while
	
			getch();
			// turn off robot before ending program
			buffer_out[0] = 254; // message start character
			for (i = 1; i<n; i++) buffer_out[i] = 0;
			serial_send(buffer_out, n, h1);
			Sleep(30); // wait for transmission
			
			close_serial(h1);	
			deactivate();
			deactivate_vision();
			printf("\n\ndone.\n");
			getch();

 			return 0; // no errors
		}

		int draw_point_RGB(image &rgb, double ip1, double jp1, int R, int G, int B)
		{
			ibyte *pa;
			int i,j,pixel, ip, jp;
			int w = 2; // dot width

			ip = (int)ip1; jp = (int)jp1;
			// initialize pointer
			pa  = rgb.pdata;

			if ( rgb.type != RGB_IMAGE ) {
				printf("\nerror in draw_point_RGB: input type not valid!");
				return 1;
			}

			// limit out of range values
			if( ip < w ) ip = w;
			if( ip > rgb.width-w-1 ) ip = rgb.width-w-1;
			if( jp < w ) jp = w;
			if( jp > rgb.height-w-1 ) jp = rgb.height-w-1;

			for(i=-w;i<=w;i++) {
				for(j=-w;j<=w;j++) {
					pixel = rgb.width*(jp+j)+(ip+i);
					pa[3*pixel]   = B;
					pa[3*pixel+1] = G;
					pa[3*pixel+2] = R;
				}
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

			rgb.type = RGB_IMAGE;
			rgb.width = IMAGE_WIDTH;
			rgb.height = IMAGE_HEIGHT;

			rgb1.type = RGB_IMAGE;
			rgb1.width = IMAGE_WIDTH;
			rgb1.height = IMAGE_HEIGHT;

			rgb2.type = RGB_IMAGE;
			rgb2.width = IMAGE_WIDTH;
			rgb2.height = IMAGE_HEIGHT;
			
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
			allocate_image(rgb1);
			allocate_image(rgb2);			
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
			free_image(rgb1);
			free_image(rgb2);			
			free_image(rgb0);
			free_image(label);

			return 0; // no errors
		}


		int activate_Object(image &a_RGB, i2byte &nlabel1, i2byte &nlabel2,
			ibyte ArrayColor[4 + 1][3 + 1], double ArrayHu[4 + 1][4 + 1], double ArrayPoint[3 + 1][3 + 1]){

			int Obj_Number;

			cout << "\nenter Obj_Number #1 = ";
			cin >> Obj_Number;
			getch();
			
			label_objects(tvalue);

			// select an object from the binary image
			select_object(nlabel1, label, a, b);

			printf("\nobject # = %d", nlabel1);

			centroid_color(a_RGB, label, nlabel1, ArrayColor, ArrayHu, Obj_Number);
			centroid_color_object(a, label, nlabel1, ArrayPoint, Obj_Number);
						
			getch();

			cout << "\nBmax = " << (int)ArrayColor[1][1];
			cout << "   Bmin = " << (int)ArrayColor[2][1];
			cout << "\nGmax = " << (int)ArrayColor[1][2];
			cout << "   Gmin = " << (int)ArrayColor[2][2];
			cout << "\nRmax = " << (int)ArrayColor[1][3];
			cout << "   Rmin = " << (int)ArrayColor[2][3];
			cout << "\nHumax = " << ArrayHu[1][1];
			cout << "   Hu min = " << ArrayHu[1][2];
			cout << "\nMmax = " << ArrayHu[3][1];
			cout << "   Mmin = " << ArrayHu[3][2];
			cout << "\nic1 = " << ArrayPoint[1][1];
			cout << "   jc1 = " << ArrayPoint[1][2];

			cout << "\nenter Obj_Number #2 = ";
			cin >> Obj_Number;
			getch();

			label_objects(tvalue);

			// select an object from the binary image
			select_object(nlabel2, label, a, b);

			printf("\nobject # = %d", nlabel2);

			centroid_color(a_RGB, label, nlabel2, ArrayColor, ArrayHu, Obj_Number);
			centroid_color_object(a, label, nlabel2, ArrayPoint, Obj_Number);			

			cout << "\n\n";
			cout << "\nBmax = " << (int)ArrayColor[3][1];
			cout << "   Bmin = " << (int)ArrayColor[4][1];
			cout << "\nGmax = " << (int)ArrayColor[3][2];
			cout << "   Gmin = " << (int)ArrayColor[4][2];
			cout << "\nRmax = " << (int)ArrayColor[3][3];
			cout << "   Rmin = " << (int)ArrayColor[4][3];
			cout << "\nHumax = " << ArrayHu[2][1];
			cout << "   Hu min = " << ArrayHu[2][2];
			cout << "\nMmax = " << ArrayHu[4][1];
			cout << "   Mmin = " << ArrayHu[4][2];
			cout << "\nic2 = " << ArrayPoint[2][1];
			cout << "   jc2 = " << ArrayPoint[2][2];

			return 0; // no errors
		}

		int find_object(i2byte &nlabel)
		// find an object
		{ 
			
			printf("\npress any key to get an image");
			getch();
			
			label_objects(tvalue);

			// select an object from the binary image
			select_object(nlabel,label,a,b);

			printf("\nobject # = %d",nlabel);

			centroid_color(rgb1, label, nlabel, RobotColor, RobotHu, Obj_Number);
			centroid_color_object(a, label, nlabel, RobotPoint, Obj_Number);

			return 0; // no errors
		}
		
		int select_object(i2byte &nlabel, image &label, image &a, image &b)
		// select an object from a binary image
		// a - image
		// b - temp. image
		{
			i2byte *pl;
			char ch;
			int i,j;

			// start in the image
			i = 200;
			j = 300;
	
			printf("\nselect an object to track by moving the");
			printf("\ngrey point to the desired object,");
			printf("\nand then pressing the x key to continue");

			while(1) {
				
				label_objects(tvalue);

				copy(a,b); // threshold image is in a
				//draw_point(b,i,j,128); // draw the new point
				copy(b,rgb2);

				draw_point_RGB(rgb2,i,j,0,255,00);
				//draw_point_RGB(rgb2,320,240,255,0,255);
				view_image(rgb2);

				// read the keyboard if a key is pressed
				if( kbhit() ) {
					ch = getch();
					if(ch == -32) ch = getch();
					if(ch == 72) j+=3; // up key
					if(ch == 80) j-=3; // down key
					if(ch == 75) i-=3; // left key
					if(ch == 77) i+=3; // right key

					// limit (i,j) from going off the image
					if( i < 0 ) i = 0;
					if( i > b.width-1 ) i = b.width-1;
					if( j < 0 ) j = 0;
					if( j > b.height-1 ) j = b.height-1;

					// check if selection is complete
					if(ch == 'x') break; 
				}
			} // end while

			// pointer to a label image
			pl = (i2byte *)label.pdata;

			// get the label value at co-ordinate (i,j)
			nlabel = *( pl + j*label.width + i );

			return 0; // no errors
		}

		int search_object(image &a, i2byte &nlabel, image &label, 
			ibyte ArrayColor[4 + 1][3 + 1], double ArrayHu[4 + 1][4 + 1], double ArrayPoint[3 + 1][3 + 1], int &Flag, int Number)
		// search for a labeled object in an outward spiral pattern
		// and inital search location (is,js)
		{
			i2byte *pl;
			double r,rmax,dr,s,smax,ds,theta;
			double ic, jc;
			int i,j;
			
			Flag = 0;
			if (Number == 1){
				ic = ArrayPoint[1][1]; jc = ArrayPoint[1][2];
				}
			if (Number == 2){
				ic = ArrayPoint[2][1]; jc = ArrayPoint[2][2];
				}

			// pointer to a label image
			pl = (i2byte *)label.pdata;

			// check if an object exists at the current location
			nlabel = *( pl + ((int)jc)*label.width + ((int)ic) );

			if (nlabel == 0){
				rmax = 60.0; // maximum radius of search (pixels)
				dr = 3.0; // radius divisions (pixels)
				ds = 3.0; // arc-length divisions (pixels)

				// search for a labeled object in an outward spiral pattern
				for (r = 1.0; r <= rmax; r += dr) {
					smax = 3 * 3.1416*r; // maximum arc length
					for (s = 0; s <= smax; s += ds) {
						theta = s / r;
						i = (int)(ic + r*cos(theta));
						j = (int)(jc + r*sin(theta));

						// limit (i,j) from going off the image
						if (i < 0) i = 0;
						if (i > label.width - 1) i = label.width - 1;
						if (j < 0) j = 0;
						if (j > label.height - 1) j = label.height - 1;

						// check if there is an object at location (i,j)
						nlabel = *(pl + j*label.width + i);
						if (nlabel != 0) break;
					}
				}
			}
			
			if (nlabel!= 0) 
			centroid_Object_plus_color(a, label, nlabel, ArrayHu, ArrayPoint, Flag, Number);
			
			if (Flag != 1)
				centroid_Search(a, label, nlabel, ArrayColor, ArrayHu, ArrayPoint, Flag, Number);			 
			
			return 0; // no errors
		}		

		int label_objects(int tvalue)
		{
			int nlabels;

			copy(rgb1,a);
			
			scale(a,a);
				
			lowpass_filter(a, a);
				
			threshold(a,a,tvalue);
				
			invert(a,b);
	
			// perform an erosion function to remove noise (small objects)
			erode(b,a);
			erode(a, b);
			//copy(b, a);
			erode(b, a);
			//erode(a, b);
			//erode(b, a);
			
			//dialate(b,a);	
			//dialate(b, a);
	
			//copy(a, rgb2);
			///view_image(rgb2);
			//getch();
	
			label_image(a,label,nlabels);

			return 0; // no errors
		}

		int centroid_Search(image &a, image &label, i2byte &nlabel, 
			ibyte ArrayColor[4 + 1][3 + 1], double ArrayHu[4 + 1][4 + 1], double ArrayPoint[3 + 1][3 + 1], int &Flag, int Number)
			//ibyte *color_matrix [3][3], double *Hu_matrix[3],

			// calculate max and min RGBcolor, V,Hu
			// a - GREY_IMAGE type, b - RGB_IMAGE type
			// label - LABEL_IMAGE type (pixel values 0-65535)
			// nlabel - label number
			// color matrix to save [ Bmax Gmax Rmax ]
			//				        [ Bmin Gmin Rmin ]
			// Hu matrix to save [ Hu_max, Hu_min ]
			//
			// note: for image_transfer2.lib
			// the origin (0,0) of the image is in the lower left corner, ip is 
			// in the horizontal direction and jp is in the vertical direction.
		{
			ibyte *pa, *pa1, *pa2, *pa3, *pa4, *pa5;
			ibyte p1, p2, p3, Sum;
			i2byte *pl;
			i4byte i, width, height, size;
			int FlagColor=0, M=0, K=0, X=0,Y=0;


			// check for compatibility of a, label
			if (a.height != label.height || a.width != label.width) {
				printf("\nerror in centroid: sizes of a, label are not the same!");
				return 1;
			}

			if (a.type != RGB_IMAGE || label.type != LABEL_IMAGE) {
				printf("\nerror in centroid: input types are not valid!");
				return 1;
			}			
			
			// number of pixels
			width = a.width;
			height = a.height;
			size = (i4byte)(width*height * 3 - 2*width * 3 - 2*3);
			
			
			pa = a.pdata + width * 3 + 3;
			pl = (i2byte *)label.pdata;
									//			
			pa1 = pa;				//		 pa3
			pa2 = pa + 3;			//	 pa4 p1  pa2
			pa3 = pa + width * 3;	//		 pa5
			pa4 = pa - 3;			//			
			pa5 = pa - width * 3;
			
			
			 for (i = 0; i<size; i += 3) {	// i=+3points*3pixels

				Sum = 0;
				p1 = *pa1; p2 = *(pa1 + 1); p3 = *(pa1 + 2);
				pixel_color_Search(p1, p2, p3, ArrayColor, ArrayHu, FlagColor, Number);
				if (FlagColor == 1) Sum++;				
				
				p1 = *pa2; p2 = *(pa2 + 1); p3 = *(pa2 + 2);
				pixel_color_Search(p1, p2, p3, ArrayColor, ArrayHu, FlagColor, Number);
				if (FlagColor == 1) Sum++;

				p1 = *pa3; p2 = *(pa3 + 1); p3 = *(pa3 + 2);
				pixel_color_Search(p1, p2, p3, ArrayColor, ArrayHu, FlagColor, Number);
				if (FlagColor == 1) Sum++;

				p1 = *pa4; p2 = *(pa4 + 1); p3 = *(pa4 + 2);
				pixel_color_Search(p1, p2, p3, ArrayColor, ArrayHu, FlagColor, Number);
				if (FlagColor == 1) Sum++;

				p1 = *pa5; p2 = *(pa5 + 1); p3 = *(pa5 + 2);
				pixel_color_Search(p1, p2, p3, ArrayColor, ArrayHu, FlagColor, Number);
				if (FlagColor == 1) Sum++;
				
				
				if (Sum == 5){				
					
					K = (int)(i / 3 + width * 3 + 3);
					X = K%width; Y = (int)K / width;
					nlabel = *(pl + (i2byte)(K + Y*(int)width));
					//if (nlabel != 0) {						
						if (Number == 1){
							ArrayPoint[1][1] = X; ArrayPoint[1][2] = Y;
							}
						if (Number == 2){
							ArrayPoint[2][1] = X; ArrayPoint[2][2] = Y;
							}						
						//}
					}
				//if (Sum == 5 && nlabel != 0) break;
				if (nlabel!=0) break;

				// increment pointers
				pa1 += 3; pa2 += 3; pa3 += 3; pa4 += 3; pa5 += 3; 
				}
			//nlabel = 0;
			 
			//draw_point_RGB(rgb1, X, Y, 0, 255, 0);
			//cout << " L=" << (int)nlabel;
			
			return 0;
		}

		int pixel_color_Search(ibyte p1, ibyte p2, ibyte p3, ibyte ArrayColor[4+1][3+1], double ArrayHu [4+1][4+1], int &FlagColor, int Number)
			// calculate RGB and Hu parameters for 3 pixels
			// RGB color parameters of object - RGBmax, RGBmin, Hu_max, Hu_min
			//
			// note: for image_transfer2.lib
			// the origin (0,0) of the image is in the lower left corner, ip is 
			// in the horizontal direction and jp is in the vertical direction.
		{
			ibyte Sum, max, min, Delta;
			ibyte Bmax, Bmin, Gmax, Gmin, Rmax, Rmin;
			double H, Hu, Hu_max, Hu_min, Hu_maxCaunt1, Hu_minCaunt1, Hu_maxCaunt2, Hu_minCaunt2;
			int max1, min1, pp1, pp2, pp3, Delta1;

			FlagColor = 0;

			if (Number == 1){
				Bmax = ArrayColor[1][1]; Gmax = ArrayColor[1][2]; Rmax = ArrayColor[1][3];
				Bmin = ArrayColor[2][1]; Gmin = ArrayColor[2][2]; Rmin = ArrayColor[2][3];
				Hu_max = ArrayHu[1][1]; Hu_min = ArrayHu[1][2];
			}

			if (Number == 2){
				Bmax = ArrayColor[3][1]; Gmax = ArrayColor[3][2]; Rmax = ArrayColor[3][3];
				Bmin = ArrayColor[4][1]; Gmin = ArrayColor[4][2]; Rmin = ArrayColor[4][3];
				Hu_max = ArrayHu[2][1]; Hu_min = ArrayHu[2][2];
			} 

			Hu_maxCaunt1 = Hu_max;
			Hu_minCaunt1 = Hu_min;

			Hu_maxCaunt2 = 0;
			Hu_minCaunt2 = 360;

			if ((Hu_max > 340) && (Hu_min < 20)){
				Hu_maxCaunt1 = 360;
				Hu_minCaunt1 = Hu_max;

				Hu_maxCaunt2 = Hu_min;
				Hu_minCaunt2 = 0;
			}
			
			max = 0; min = 255;
			if (p1 < min) min = p1; // B
			if (p1 > max) max = p1; // B
			if (p2 < min) min = p2; // G
			if (p2 > max) max = p2; // G
			if (p3 < min) min = p3; // R
			if (p3 > max) max = p3; // R

			Sum = 0;
			//if (p1 >= Bmin) Sum++; // B
			//if (p1 <= Bmax) Sum++; // B
			//if (p2 >= Gmin) Sum++; // G
			//if (p2 <= Gmax) Sum++; // G
			//if (p3 >= Rmin) Sum++; // R
			//if (p3 <= Rmax) Sum++; // R

			Delta = max - min;
			if ((Delta > 0) && (Sum == 6)){

				Delta1 = (int)Delta;
				max1 = (int)max;
				min1 = (int)min;
				pp1 = (int)(p1);
				pp2 = (int)(p2);
				pp3 = (int)(p3);
				//cout << " Sum=" << (int)Sum;
				H = 5 - (double)(max1 - pp3) / (double)Delta1;

				if (pp3 == max1) {
					H = 3 + (double)(max1 - pp2) / (double)Delta1;
					if (pp2 == min1) H = 5 + (double)(max1 - pp1) / (double)Delta1;
					if (pp2 != min1) H = 1 - (double)(max1 - pp2) / (double)Delta1;
				}
				if ((pp2 == max1) && (pp1 == min1)) H = 1 + (double)(max1 - pp3) / (double)Delta1;

				if ((pp2 == max1) && (pp1 != min1)) H = 3 - (double)(max1 - pp1) / (double)Delta1;

				Hu = H * 60;				
				if (Hu < 0) Hu = Hu + 360;

				if ((Hu >= Hu_minCaunt1) && (Hu <= Hu_maxCaunt1)) Sum++;
				
				if ((Hu >= Hu_minCaunt2) && (Hu <= Hu_maxCaunt2)) Sum++;

				
				//if (Sum >= 7) FlagColor = 1;
				if (Sum >= 1) FlagColor = 1;
			}
			return 0;
		}
	
		int find_Block(double Block[5+1], i2byte &nlabel){

			double icB, jcB, m, Rmax;
			printf("\nSelect Block");
			getch();

			label_objects(tvalue);

			// select an object from the binary image
			select_object(nlabel, label, a, b);

			printf("\nobject # = %d", nlabel);

			centroid_M(a, label, nlabel, icB, jcB, m, Rmax);

			Block[1] = icB;		Block[2] = jcB;
			Block[3] = m*1.2;	Block[4] = m*0.8;
			Block[5] = Rmax;

			cout << "\nic = " << Block[1]; cout << "   jc = " << Block[2];
			cout << "\nMmax = " << Block[3]; cout << "   Mmin = " << Block[4];
			cout << "\nRmax = " << Block[5];

			return 0; // no errors

		}


		int TETA_Object(double ArrayPoint[3 + 1][3 + 1], double &TETA){

			double ic1, ic2, jc1, jc2;
			double Delta_X, Delta_Y;

			ic1 = ArrayPoint[1][1]; jc1 = ArrayPoint[1][2];
			ic2 = ArrayPoint[2][1]; jc2 = ArrayPoint[2][2];

			Delta_X = ic2 - ic1;
			Delta_Y = jc2 - jc1;
			
			TETA = atan2(Delta_Y, Delta_X);
			
			return 0; // no errors

		}

		int TURN(double TETA, double TETA_NEW, unsigned char &COM, unsigned char DIR, 
			unsigned char &PAR_LEFT, unsigned char &PAR_RIGHT, int &Flag_TETA){

			double  e, TETA_1, TETA_2;			
			
			if (TETA > 0) TETA_1 = TETA;
			if (TETA < 0) TETA_1 = (6.283+TETA);
			if (TETA == 0) TETA_1 = 6.283;
			
			if (TETA_NEW > 0) TETA_2 = TETA_NEW;
			if (TETA_NEW < 0) TETA_2 = (6.283 + TETA_NEW);
			if (TETA_NEW == 0) TETA_2 = 6.283;

			if ((TETA_1 > TETA_2) && (DIR == 2)) e = TETA_1 - TETA_2;
			if ((TETA_1 > TETA_2) && (DIR == 1)) e = 6.283 - TETA_2 + TETA_1;
			
			if ((TETA_1 < TETA_2) && (DIR == 2)) e = 6.283 - TETA_2 + TETA_1;
			if ((TETA_1 < TETA_2) && (DIR == 1)) e = TETA_2 - TETA_1;
						
			if (fabs(e) < 0.06) {
				COM = 3;				
				PAR_LEFT = 0;
				PAR_RIGHT = 0;
				Flag_TETA = 1;				
				}
			if ((fabs(e) >= 0.05) && (fabs(e) < 0.3)){
				COM = 3;				
				PAR_LEFT = 2;
				PAR_RIGHT = 2;
				}
			if ((fabs(e) >= 0.3) && (fabs(e) < 0.9)){
				COM = 3;				
				PAR_LEFT = 4;
				PAR_RIGHT = 4;
				}
			if (fabs(e) >= 0.9){				
				COM = 3;
				PAR_LEFT = 6;
				PAR_RIGHT = 6;
				}						
			return 0; // no errors
		}

		int DIR_TURN(double TETA, double TETA_NEW, unsigned char &DIR){	
					
				double TETA_1, TETA_2, A, B;

				if (TETA > 0) TETA_1 = TETA;
				if (TETA < 0) TETA_1 = (6.283 + TETA);  //6.283 = 2Pi
				if (TETA == 0)  TETA_1 = 6.283;

				if (TETA_NEW > 0) TETA_2 = TETA_NEW;
				if (TETA_NEW < 0) TETA_2 = (6.283 + TETA_NEW); //6.283 = 2Pi 
				if (TETA_NEW == 0) TETA_2 = 6.283;

				if (TETA_1 > TETA_2){
					A = TETA_1 - TETA_2; B = 6.283 - TETA_1 + TETA_2;
					if (A >= B) DIR = 1;
					if (A < B) DIR = 2;
					}
				if (TETA_1 < TETA_2){
					A = TETA_2 - TETA_1; B = 6.283 - TETA_2 + TETA_1;
					if (A > B) DIR = 2;
					if (A <= B) DIR = 1;
					}
				//Flag_TETA = 0;

			return 0;
		}

		int POSITION(double ArrayPoint_1[3 + 1][3 + 1], double ArrayPoint_2[3 + 1][3 + 1], double ArrayBlock[5+1], double &TETA_R,
			double &TETA_Tar, double &TETA_NEW, double DPoint[2 + 1], int &Flag_TETA, int &Flag_Point, int &Flag_SHOOT,
			double Size_Robot, double Size_Target){

			double ic1, ic2, ic3, ic4, jc1, jc2, jc3, jc4, icB, jcB;
			double X_Tar, Y_Tar, DisMin, DisTar, DTP;			//DTP - dictance to point, DisMin - min distance to take over the block;
			double Delta_X, Delta_Y, Delta_X_Tar, Delta_Y_Tar, Delta_X_DPoint, Delta_Y_DPoint, Delta_TETA;
			double Delta_X_Block, Delta_Y_Block, TETA_Block;
			double A, B, C;								//factors for equetion of line
			double TETA_DPoint, TETA_Robot_DPoint, K1, K2;
			double  e, TETA_1, TETA_2, A_Teta, B_Teta, D;

			ic1 = ArrayPoint_1[1][1]; jc1 = ArrayPoint_1[1][2];
			ic2 = ArrayPoint_1[2][1]; jc2 = ArrayPoint_1[2][2];

			ic3 = ArrayPoint_2[1][1]; jc3 = ArrayPoint_2[1][2];
			ic4 = ArrayPoint_2[2][1]; jc4 = ArrayPoint_2[2][2];

			icB = ArrayBlock[1];		jcB = ArrayBlock[2];			

			DisMin = 1.8*Size_Robot + 1.3*ArrayBlock[5];

			X_Tar = (ic3 + ic4) / 2;	Y_Tar = (jc3 + jc4) / 2;

			ArrayPoint_2[3][1] = X_Tar; ArrayPoint_2[3][2] = Y_Tar;

			Delta_X = ic2 - ic1;		Delta_Y = jc2 - jc1;
			Delta_X_Tar = X_Tar - ic1;	Delta_Y_Tar = Y_Tar - jc1;
			Delta_X_Block = ArrayBlock[1] - ic1;	Delta_Y_Block = ArrayBlock[2] - jc1;

			TETA_R = atan2(Delta_Y, Delta_X);
			TETA_Tar = atan2(Delta_Y_Tar, Delta_X_Tar);
			TETA_Block = atan2(Delta_Y_Block, Delta_X_Block);

			if (TETA_Tar>=TETA_Block) TETA_DPoint = TETA_Tar + (3.1415 / 2);
			if (TETA_Tar<TETA_Block) TETA_DPoint = TETA_Tar - (3.1415 / 2);
			//TETA_DPoint = TETA_Tar + (3.1415 / 2);
			
			if (TETA_DPoint > 3.1415) TETA_DPoint = -(3.1415-(TETA_DPoint - 3.1415));
			if (TETA_DPoint < -3.1415) TETA_DPoint = 3.1415 + (TETA_DPoint + 3.1415);
			
			
			A = jc1 - Y_Tar; B = X_Tar - ic1; C = ic1*Y_Tar - X_Tar*jc1;
			
			DTP = fabs ( (A*icB + B*jcB + C) / (sqrt((pow(A, 2.0) + pow(B, 2.0))))  );
	
			DPoint[1] = ArrayBlock[1] + DisMin * cos(TETA_DPoint); DPoint[2] = ArrayBlock[2] + DisMin * sin(TETA_DPoint);

			//if (DPoint[1] < (int)1.2*Size_Robot) DPoint[1] = (int)1.2*Size_Robot;
			//if (DPoint[1] > (int)(640-1.2*Size_Robot)) DPoint[1] = (int)(640-1.2*Size_Robot);

			//if (DPoint[2] < (int)1.2*Size_Robot) DPoint[2] = (int)1.2*Size_Robot;
			//if (DPoint[2] > (int)(480 - 1.2*Size_Robot)) DPoint[2] = (int)(480-1.2*Size_Robot);
			
			Delta_X_DPoint = DPoint[1] - ic1;	Delta_Y_DPoint = DPoint[2] - jc1;
			TETA_Robot_DPoint = atan2(Delta_Y_DPoint, Delta_X_DPoint);

			DisTar = sqrt(pow((X_Tar - ic1), 2.0) + pow((Y_Tar - jc1), 2.0));
			Delta_TETA = (0.3*Size_Target) / DisTar;
			if (Delta_TETA < 0.05) Delta_TETA = 0.05;

			if (DTP <= 1.5*ArrayBlock[5]) {
				Flag_TETA = 0;
				Flag_Point = 0;
				TETA_NEW = TETA_Robot_DPoint;
			}

			if (DTP > 1.5*ArrayBlock[5]){

			if (TETA_R > 0) TETA_1 = TETA_R;
			if (TETA_R < 0) TETA_1 = (6.283 + TETA_R);
			if (TETA_R == 0) TETA_1 = 6.283;

			if (TETA_Tar > 0) TETA_2 = TETA_Tar;
			if (TETA_Tar < 0) TETA_2 = (6.283 + TETA_Tar);
			if (TETA_Tar == 0) TETA_2 = 6.283;

			if (TETA_1 > TETA_2){
				A_Teta = TETA_1 - TETA_2; B_Teta = 6.283 - TETA_1 + TETA_2;
				if (A_Teta >= B_Teta) D = 1;
				if (A_Teta < B_Teta) D = 2;
			}
			if (TETA_1 < TETA_2){
				A_Teta = TETA_2 - TETA_1; B_Teta = 6.283 - TETA_2 + TETA_1;
				if (A_Teta > B_Teta) D = 2;
				if (A_Teta <= B_Teta) D = 1;
			}
			if ((TETA_1 > TETA_2) && (D == 2)) e = TETA_1 - TETA_2;
			if ((TETA_1 > TETA_2) && (D == 1)) e = 6.283 - TETA_2 + TETA_1;

			if ((TETA_1 < TETA_2) && (D == 2)) e = 6.283 - TETA_2 + TETA_1;
			if ((TETA_1 < TETA_2) && (D == 1)) e = TETA_2 - TETA_1;
			
			if (fabs(e) >= Delta_TETA){
				Flag_TETA = 0;
				TETA_NEW = TETA_Tar;
				}			

			if (fabs(e) < Delta_TETA){ 								
					Flag_Point = 1;
					Flag_TETA = 1;
					}				
				}

			if (Flag_TETA == 1 && Flag_Point == 1) Flag_SHOOT = 1;		
			
			return 0;
		}

		int MOVE(double ArrayPoint[3 + 1][3 + 1], double Array_DPoint[2 + 1], unsigned char &COM, unsigned char &DIR,
			unsigned char &PAR_LEFT, unsigned char &PAR_RIGHT, int &Flag_Point){

			double K1 = 10, K2 = 30, Distance;

			Distance = sqrt(pow((Array_DPoint[1] - ArrayPoint[1][1]), 2.0) + pow((Array_DPoint[2] - ArrayPoint[1][2]), 2.0));
				
			COM = 2;
			DIR = 2;
			PAR_LEFT = 7;
			PAR_RIGHT = 7;
			if (Distance < K2){
				PAR_LEFT =5;
				PAR_RIGHT =5;
				}
	
			//if ( fabs(ArrayPoint[1][1] - Array_DPoint[1]) <= K1 && fabs(ArrayPoint[1][2] - Array_DPoint[2]) <= K1 ){
			if (fabs(ArrayPoint[1][1] - Array_DPoint[1]) <= K1){
			COM = 2;
			DIR = 2;
			PAR_LEFT = 0;
			PAR_RIGHT = 0;
			Flag_Point = 1;
			}
			

		return 0;
		}


		int draw_point(image &a, double ip, double jp, int value)
			// draw a point at pixel location (ip,jp)
			// note: for image_transfer2.lib
			// the origin (0,0) of the image is in the lower left corner, ip is 
			// in the horizontal direction and jp is in the vertical direction.
		{
			ibyte *pa;
			int i, j, w = 2;

			// initialize pointer
			pa = a.pdata;

			if (a.type != GREY_IMAGE) {
				printf("\nerror in draw_point: input type not valid!");
				return 1;
			}

			// limit out of range values
			if (ip < w) ip = w;
			if (ip > a.width - w - 1) ip = a.width - w - 1;
			if (jp < w) jp = w;
			if (jp > a.height - w - 1) jp = a.height - w - 1;

			for (i = -w; i <= w; i++) {
				for (j = -w; j <= w; j++) {
					pa[a.width*((int)jp + j) + ((int)ip + i)] = value;
				}
			}

			return 0;
		}



		//Key Control

		/*
		// read keys and set robot inputs
		if (KEY(VK_UP)) {
			COM = 2;
			DIR = 1;
			if (PAR_LEFT < 5) PAR_LEFT = 5;
			if (PAR_LEFT > 25) PAR_LEFT = 25;
			if (PAR_RIGHT < 5) PAR_RIGHT = 5;
			if (PAR_RIGHT > 20) PAR_RIGHT = 20;

			PAR_LEFT++; // increase velocity input
			PAR_RIGHT++;
		}

		if (KEY(VK_DOWN)) {
			COM = 2;
			DIR = 2;
			if (PAR_LEFT < 5) PAR_LEFT = 5;
			if (PAR_LEFT > 25) PAR_LEFT = 25;
			if (PAR_RIGHT < 5) PAR_RIGHT = 5;
			if (PAR_RIGHT > 20) PAR_RIGHT = 20;

			PAR_LEFT++; // increase velocity input
			PAR_RIGHT++;
		}

		if (KEY(VK_LEFT)) { //Left U-Turn
			COM = 3;
			DIR = 1;
			PAR_LEFT = 5;
			PAR_RIGHT = 5;
		}

		if (KEY(VK_RIGHT)) { // Right U-Turn
			COM = 3;
			DIR = 2;
			PAR_LEFT = 5;
			PAR_RIGHT = 5;
		}

		//SHOOT
		if (KEY(0x5A)) { //Z
			COM = 4;
		}

		if (KEY(0x50)) {  //P
			COM = 5;
		}

		// stop robot
		if (KEY(VK_SPACE)) {
			COM = 1;
			DIR = 1;
			PAR_LEFT = 0;
			PAR_RIGHT = 0;
		}

		buffer_out[0] = 254; // message start character
		buffer_out[1] = COM;
		buffer_out[2] = DIR;
		buffer_out[3] = PAR_LEFT;
		buffer_out[4] = PAR_RIGHT;

		// send inputs to Arduino
		n = 5;

		if (COM == 5) {
			Sleep(500);
			COM = 1;
			DIR = 1;
			PAR_LEFT = 0;
			PAR_RIGHT = 0;
		}

		//open_serial("COM8", h1);
		serial_send(buffer_out, n, h1);

		Sleep(30); // 30 fps

		// read the keyboard if a key is pressed
		if (kbhit()) {
			ch = getch();
			// check if tracking is complete
			if (ch == 'x') break;
		}

		*/