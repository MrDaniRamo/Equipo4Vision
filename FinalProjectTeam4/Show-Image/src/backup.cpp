/*
 * Title: Etapa 1. Proyecto final
 * Class: Vision para Robots
 * Instructor: Dr. Jose Luis Gordillo (http://LabRob.mty.itesm.mx/)
 * Code: 
 * Institution: Tec de Monterrey, Campus Monterrey
 * Date: March 10, 2012
 *
 * Description: First stage of the final project
 *
 * This programs uses OpenCV http://opencv.willowgarage.com/wiki/
 */


/********************************************************************************************************************************
Libraries
*********************************************************************************************************************************/
#include <iostream>
#include <cstddef>
#include <queue>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/tracking.hpp>


using namespace std;
using namespace cv;
/********************************************************************************************************************************
Variable Declaration
*********************************************************************************************************************************/

// Images Matrixes
Mat currentImage;
Mat RoI;
Mat filteredImage;
Mat frame;
Mat convertedImage;
Mat segmentedImage;
Mat plot1;
Mat plot2;
Mat plot3;
Mat quads1;
Mat quads2;

//interface control
bool pause = 0;
bool fClick = false;
bool isQuads = 0;
int key = 0;
int colorSpace = 49;
int b,g,r;
int quadx, quady;

//segmentation vaiables
bool segDone[13] = {false, false, false, false, false, false, false, false, false, false, false, false, false};
bool isSegmented = false;
bool noSeed = true;
int k = 0;  //kolor
Point seed[7];   // punto semilla
Point q[4] = {Point (1,0), Point(0,1), Point(-1,0), Point(0,-1)};
double A[7]; // area
double m00[7];
double m01[7];
double m10[7];
double m11[7];
double m20[7];
double m02[7];

double X[7], Y[7];
double u00[7], u10[7], u01[7], u20[7], u02[7], u11[7];
double theta[7];
double n20[7], n02[7], n11[7];
double phi1[7], phi2[7];

//Entrenamiento
double AVGphi1[5] = {0.0, 0.197926, 0.169221, 0.187255 , 0.160513};
double AVGphi2[5] = {0.0, 0.0120475, 0.0031186, 0.00613623 ,  0.000144823};

double dPhi1[5] = { 0.0, 0.01688114792, 0.004659150757, 0.03481897302, 0.008912745084};
double dPhi2[5] = { 0.0, 0.003626741209, 0.002597481889, 0.01592588206, 0.000755941977};

int shape[5] = {0,0,0,0,0}; // 1 = pera, 2 = manzana, 3 = sandia, 4 = limon
int quad[5] = {0,0,0,0,0}; // 1 = pera y manzana, 2 = pera y limon, 3 = sandia y manzana, 4 = sandia y limon
	double maxphi1[5];
	double minphi1[5];
	double maxphi2[5];
	double minphi2[5];
int xLine, yLine, lineLength;

//color array
int paint[13][3] = {
		{0,0,0}, //empty
		{254, 0, 0 }, //blue
		{0, 254, 0 }, //Green
		{0, 0, 254 }, //Red
		{254, 0, 254 }, //Magenta
		{254, 254, 0 }, //Cyan
		{0, 254, 254 }, //Yellow
		{124, 254, 0 }, //Turqoise
		{254, 0, 124 }, //Violet
		{124, 0, 254 }, //Orange
		{0, 254, 124 }, //Spring Green
		{254, 124, 0 }, //Ocean
		{124, 0, 254}  //Raspberry
		};




//filter variables
const int max_sigmaN = 5;
int sigma_slider = 3;
int sigmaN;
bool firstFilter;
static void t_callback(int, void*){ // Trackbar callback function
}





/********************************************************************************************************************************
Function delcaration
*********************************************************************************************************************************/
void filter(const Mat &sourceImage, Mat &destinationImage, Mat &RoI, int sigmaN);
void filterGray(const Mat &sourceImage, Mat &destinationImage, Mat &RoI, int sigmaN);
//colorsapce conversions
void RGB2Gray(const Mat &sourceImage, Mat &destinationImage);
void RGB2YIQ(const Mat &sourceImage, Mat &destinationImage);
void segmentImg(Mat &sourceImage, Mat &destinationImage, Point s, int c);
Point findSeed(Point s,  Mat &destinationImage);
void seedFinder(int event, int x, int y, int flags, void* param);
void createPlot(Mat &sourceImage, Mat &destinationImage);
void modelPlot(Mat &sourceImage, Mat &destinationImage, int c);
void createQuadrants(Mat &destinationImage);
void FindQuadrant(Mat &sourceImage, Mat &destinationImage);

/********************************************************************************************************************************
Main
*********************************************************************************************************************************/

int main(int argc, char *argv[])
{
		/* First, open camera device */
	VideoCapture camera = VideoCapture(0);
	bool isCameraAvailable = camera.isOpened();

	while(true){

		key = waitKey(5);

		//check for number key press
		if (key > 48 && key < 58){
			colorSpace = key;
			destroyAllWindows();
			key = 0;
			firstFilter = false;
			convertedImage.release();
		}

		if (key == 'p')
			pause = !pause;
		
		// obtain new frame if pause is not enabled
		
		
		if (!pause){
			// check if camera available, if not exit
			if (isCameraAvailable){
				camera.read(currentImage);
			}

			else{
				cout << "Camera not found, check conection\n";
				break;
			}
		}
		
		
		

		/************** Choose Image ****************************************************************************************/
		switch(colorSpace){
			
			case 49:
				//currentImage = imread("pruebasvision0.png", IMREAD_COLOR);
				imshow("Original", currentImage);
				RGB2YIQ(currentImage, convertedImage);
				//imshow("YIQ", convertedImage);
				
				
			break;
		

			default:
			colorSpace = 49;
		}

		/************************************************************************************************************************/

		if(firstFilter){
			//pre-create window to attach trackbar
			namedWindow("Filtered image", WINDOW_AUTOSIZE);
			//create trackbar (trackbar name, window name, pointer to int to store trackbar value, max value, trackbar callback)
			createTrackbar("sigma", "Filtered image", &sigma_slider, max_sigmaN, t_callback);
			sigmaN = getTrackbarPos("sigma", "Filtered image");
			
			filter(convertedImage, filteredImage, RoI, sigmaN);
			imshow("Filtered image", filteredImage);
		}
		
		if(isSegmented){
			setMouseCallback("Segmented Image", seedFinder);
			
			if (fClick){
				for (int i = 1 ; i < k+1; i++){
					segmentImg(filteredImage, segmentedImage, seed[i], i);
				}
				//cout<<"here\n";
			}
			
			imshow("Segmented Image", segmentedImage);
			//segmentedImage = Mat(filteredImage.rows, filteredImage.cols, filteredImage.type()); 
			if (fClick){
			//calculo de momentos
			for (int i = 1; i < k+1; i ++){
			
			//Centroide
			X[i]=m10[i]/m00[i];
			Y[i]=m01[i]/m00[i];
			
			//Momentos centralizados (invariantes en traslacion)
			u00[i] = m00[i];
			u10[i] = 0;
			u01[i] = 0;
			u20[i] = m20[i]-X[i]*m10[i];
			u02[i] = m02[i]-Y[i]*m01[i];
			u11[i] = m11[i]-Y[i]*m10[i];
			//Theta
			theta[i] = 0.5*atan2(2*u11[i], (u20[i] - u02[i]));
			//theta[i]= 0.5*atan((2*u11[i])/(u20[i]-u02[i]));
			
			//Momentos normalizados (invariantes en escala)
			n20[i] = u20[i] / (m00[i] * m00[i]);
			n02[i] = u02[i] / (m00[i] * m00[i]);
			n11[i] = u11[i] / (m00[i] * m00[i]);
			
			//Momentos de Hu (invariantes en rotacion)
			phi1[i] = n20[i] + n02[i];
			phi2[i] = (n20[i] - n02[i])*(n20[i] - n02[i]) + 4*(n11[i]*n11[i]);
			
			lineLength = sqrt(m00[i]);
			xLine = lineLength*cos(-theta[i]);
			yLine = lineLength*sin(-theta[i]);
			
			/*
			cout << "Area of Region " << i << " is: "<< A[i] << " \n";
			cout << "m00 of Region " << i << " is: "<< m00[i] << " \n";
			cout << "m10 of Region " << i << " is: "<< m10[i] << " \n";
			cout << "m01 of Region " << i << " is: "<< m01[i] << " \n";
			cout << "m11 of Region " << i << " is: "<< m11[i] << " \n";
			cout << "m20 of Region " << i << " is: "<< m20[i] << " \n";
			cout << "m02 of Region " << i << " is: "<< m02[i] << " \n";
			cout << "Centroide of image " << i << " is: "<< X[i] << " , "<< Y[i] << " \n";
			*/	
			
			//centroide;
			circle(segmentedImage, Point(round(X[i]), round(Y[i])), 3, Scalar(255, 255, 255), 2, 8, 0);
			line(segmentedImage, 	Point(round(X[i] - xLine), round(Y[i] + yLine) ),Point(round(X[i] + xLine), round(Y[i] - yLine) ), Scalar( 0, 0, 255), 2, 8, 0);
			seed[i].y = X[i];
			seed[i].x = Y[i];
			}
			}
			imshow("Segmented Image", segmentedImage);
		segmentedImage = Mat(filteredImage.rows, filteredImage.cols, filteredImage.type()); 
		}
		
		if(isQuads){
			
			plot1 = imread("plot.png", IMREAD_COLOR);
			createPlot(plot1, plot2);
			createQuadrants(quads1);
			quads2 = imread("cuadrants.png", IMREAD_COLOR);
			imshow("Quadrants", quads2);
			for (int i = 1; i < 5; i++){
				maxphi1[i] = AVGphi1[i]+(dPhi1[i]);
				minphi1[i] = AVGphi1[i]-(dPhi1[i]);
				maxphi2[i] = AVGphi2[i]+(dPhi2[i]);
				minphi2[i] = AVGphi2[i]-(dPhi2[i]);
			}
			
			/*
			cout << "Pera: Max Phi1: "<< maxphi1[1] << " min Phi1: "<< minphi1[1] << " max Phi2 : "<< maxphi2[1]<< " min phi2 : "<< minphi2[1] << "\n";
			cout << "Manzana: Max Phi1: "<< maxphi1[2] << " min Phi1: "<< minphi1[2] << " max Phi2 : "<< maxphi2[2]<< " min phi2 : "<< minphi2[2] << "\n";
			cout << "Sandia: Max Phi1: "<< maxphi1[3] << " min Phi1: "<< minphi1[3] << " max Phi2 : "<< maxphi2[3]<< " min phi2 : "<< minphi2[3] << "\n";
			cout << "Limon: Max Phi1: "<< maxphi1[4] << " min Phi1: "<< minphi1[4] << " max Phi2 : "<< maxphi2[4]<< " min phi2 : "<< minphi2[4] << "\n";
			*/
			
			for (int i = 1; i < k+1; i ++){

			modelPlot(plot1,plot2, i);
			}

			FindQuadrant(quads1, quads2);

			imshow("Quadrants", quads2);
			

			
		}
		
		
		if( key == 'f'){  //select color to be filtered
			cout << "Select object to be filtered\n";
			Rect2d r = selectROI("Object select", convertedImage, 0, 0); // Select ROI
			frame = convertedImage.clone();
			RoI = frame(r);
			imshow("ROI", RoI);
			destroyWindow("Object select");
			firstFilter = true;
			key = 0;

		}
		
		
		if( key == 'w'){ //segment image
			segmentedImage = Mat(filteredImage.rows, filteredImage.cols, filteredImage.type()); 
			imshow("Segmented Image", segmentedImage);
			isSegmented = true;
		}

		if (key == 'e'){
			
			plot1 = imread("plot.png", IMREAD_COLOR);
			createPlot(plot1, plot2);
			imshow("Plot", plot2);
			for (int i = 1; i < 5; i++){
				maxphi1[i] = AVGphi1[i]+(dPhi1[i]);
				minphi1[i] = AVGphi1[i]-(dPhi1[i]);
				maxphi2[i] = AVGphi2[i]+(dPhi2[i]);
				minphi2[i] = AVGphi2[i]-(dPhi2[i]);
			}
			/*
			cout << "Pera: Max Phi1: "<< maxphi1[1] << " min Phi1: "<< minphi1[1] << " max Phi2 : "<< maxphi2[1]<< " min phi2 : "<< minphi2[1] << "\n";
			cout << "Manzana: Max Phi1: "<< maxphi1[2] << " min Phi1: "<< minphi1[2] << " max Phi2 : "<< maxphi2[2]<< " min phi2 : "<< minphi2[2] << "\n";
			cout << "Sandia: Max Phi1: "<< maxphi1[3] << " min Phi1: "<< minphi1[3] << " max Phi2 : "<< maxphi2[3]<< " min phi2 : "<< minphi2[3] << "\n";
			cout << "Limon: Max Phi1: "<< maxphi1[4] << " min Phi1: "<< minphi1[4] << " max Phi2 : "<< maxphi2[4]<< " min phi2 : "<< minphi2[4] << "\n";
			*/
			for (int i = 1; i < k+1; i ++){

			modelPlot(plot1,plot2, i);
			}
			imshow("Plot", plot2);
		}
		
		if (key == 'q'){
			
			isQuads = true;
			/*
			plot1 = imread("plot.png", IMREAD_COLOR);
			createPlot(plot1, plot2);
			createQuadrants(quads1);
			quads2 = imread("cuadrants.png", IMREAD_COLOR);
			imshow("Quadrants", quads2);
			for (int i = 1; i < 5; i++){
				maxphi1[i] = AVGphi1[i]+(dPhi1[i]);
				minphi1[i] = AVGphi1[i]-(dPhi1[i]);
				maxphi2[i] = AVGphi2[i]+(dPhi2[i]);
				minphi2[i] = AVGphi2[i]-(dPhi2[i]);
			}
			*/
			/*
			cout << "Pera: Max Phi1: "<< maxphi1[1] << " min Phi1: "<< minphi1[1] << " max Phi2 : "<< maxphi2[1]<< " min phi2 : "<< minphi2[1] << "\n";
			cout << "Manzana: Max Phi1: "<< maxphi1[2] << " min Phi1: "<< minphi1[2] << " max Phi2 : "<< maxphi2[2]<< " min phi2 : "<< minphi2[2] << "\n";
			cout << "Sandia: Max Phi1: "<< maxphi1[3] << " min Phi1: "<< minphi1[3] << " max Phi2 : "<< maxphi2[3]<< " min phi2 : "<< minphi2[3] << "\n";
			cout << "Limon: Max Phi1: "<< maxphi1[4] << " min Phi1: "<< minphi1[4] << " max Phi2 : "<< maxphi2[4]<< " min phi2 : "<< minphi2[4] << "\n";
			*/
			/*
			for (int i = 1; i < k+1; i ++){

			modelPlot(plot1,plot2, i);
			}

			FindQuadrant(quads1, quads2);

			imshow("Quadrants", quads2);
			
			*/
		}
		
		if (key == 'r'){
			k = 0;
			isSegmented = !isSegmented;
		
		}
			

		if (key == 'x')
			break;
	}
	
}


/************************************************************************************************
Function definition
*********************************************************************************************************************************/
void FindQuadrant(Mat &sourceImage, Mat &destinationImage){
	switch (shape[1]) {
		case 1:  //pera
			if (shape[2] == 2){ //manzana
				circle(destinationImage, Point(128,128), 20, Scalar(0,0,255), 2, 8, 0); //cuadrante 1
				//cout << "pera y manzana\n";
				
				
				
			}
			else { //limon
				circle(destinationImage, Point(384,128), 20, Scalar(0,0,255), 2, 8, 0); //cuadrante 2
				//cout << "pera y limon\n";
			}
			quadx = 200*cos(-theta[1]);
			quady = 200*sin(-theta[1]);
				line(destinationImage, Point(256 - quadx, 256+quady ),Point(256 + quadx, 256-quady ), Scalar( 0, 0, 255), 2, 8, 0);
			
		break;
		case 2:  //manzana
			if (shape[2] == 1){ //pera
				circle(destinationImage, Point(128,128), 20, Scalar(0,0,255), 2, 8, 0); //cuadrante 1
				//cout << "pera y manzana\n";
			}
			else { //sandia
				circle(destinationImage, Point(384,128), 20, Scalar(0,0,255), 2, 8, 0); //cuadrante 2
				//cout << "sandia y manzana\n";
			}
			quadx = 200*cos(-theta[2]);
			quady = 200*sin(-theta[2]);
				line(destinationImage, Point(256 - quadx, 256+quady ),Point(256 + quadx, 256-quady ), Scalar( 0, 0, 255), 2, 8, 0);
		break;
		case 3: // sandia
			if (shape[2] == 2){ //manzana
				circle(destinationImage, Point(128,384), 20, Scalar(0,0,255), 2, 8, 0); //cuadrante 3
				//cout << "sandia y manzana\n";
			}
			else { //limon
				circle(destinationImage, Point(384,384), 20, Scalar(0,0,255), 2, 8, 0); //cuadrante 4
				//cout << "sandia y limon\n";
			}
			quadx = 200*cos(-theta[1]);
			quady = 200*sin(-theta[1]);
				line(destinationImage, Point(256 - quadx, 256+quady ),Point(256 + quadx, 256-quady ), Scalar( 0, 0, 255), 2, 8, 0);
		break;
		case 4:  //limon
			if (shape[2] == 1){ //pera
				circle(destinationImage, Point(128,128), 20, Scalar(0,0,255), 2, 8, 0); //cuadrante 1
				//cout << "pera y limon\n";
			}
			else { //sandia
				circle(destinationImage, Point(384,128), 20, Scalar(0,0,255), 2, 8, 0); //cuadrante 2
				//cout << "sandia y limon\n";
			}
			quadx = 200*cos(-theta[2]);
			quady = 200*sin(-theta[2]);
				line(destinationImage, Point(256 - quadx, 256+quady ),Point(256 + quadx, 256-quady ), Scalar( 0, 0, 255), 2, 8, 0);
		break;
	}
}


void createQuadrants(Mat &destinationImage){
	destinationImage = imread("cuadrants.png", IMREAD_COLOR);
}




void modelPlot(Mat &sourceImage, Mat &destinationImage, int c){
      	double p1 = phi1[c];
      	double p2 = phi2[c]; 
      	int col = destinationImage.cols;
	int rows = destinationImage.rows;

			//cout << "Ph1 " << c << " is: "<< p1 << " \n";
			//cout << "Phi2 " << c << " is: "<< p2 << " \n";
			

      	shape[c] = 0;


       if( ((p1 < maxphi1[2])&& (p1 > minphi1[2])) && ((p2 < maxphi2[2]) && (p2 > minphi2[2]))){ //manzana
      		shape[c] = 2;
      		circle(plot2, Point(round(p1*3000)-rows+110, round(p2*3000)+col-165), 2, Scalar(paint[c][0],paint[c][1],paint[c][2]), 2, 8, 0);
      	}
  
      	else 	if( ((p1 < maxphi1[4]) && (p1 > minphi1[4])) && ((p2 < maxphi2[4]) && (p2 > minphi2[4]))){ //limon
      		shape[c] = 4;
      		circle(plot2, Point(round(p1*3000)-rows+102, round(p2*3000)+col-149), 2, Scalar(paint[c][0],paint[c][1],paint[c][2]), 2, 8, 0);
      	}
      	else if( ((p1 < maxphi1[1])&& (p1 > minphi1[1])) && ((p2 < maxphi2[1]) && (p2 > minphi2[1]))){ //pera
      		shape[c] = 1;
      		circle(plot2, Point(round(p1*3000)-rows+100, round(p2*3000)+col-230), 2, Scalar(paint[c][0],paint[c][1],paint[c][2]), 2, 8, 0);
      	}
  
      	
      	else if( ((p1 < maxphi1[3])&& (p1 > minphi1[3])) && ((p2 < maxphi2[3]) && (p2 > minphi2[3]))){ //sandia
      		shape[c] = 3;
      		circle(plot2, Point(round(p1*3000)-rows+100, round(p2*3000)+col-200), 2, Scalar(paint[c][0],paint[c][1],paint[c][2]), 2, 8, 0);
      	}
      	
      	//cout << "Region " << c << " is shape " << shape[c] <<"\n";

}

void createPlot(Mat &sourceImage, Mat &destinationImage){
	destinationImage = imread("plot.png", IMREAD_COLOR);
	int col = destinationImage.cols;
	int rows = destinationImage.rows;
	
	circle(destinationImage, Point(round(AVGphi1[1]*3000)-rows+120, -round(AVGphi2[1]*3000)+col-150), round(dPhi2[1]*3500), Scalar(255, 0, 0), 2, 8, 0);// pera
	circle(destinationImage, Point(round(AVGphi1[2]*3000)-rows+110,  -round(AVGphi2[2]*3000)+col-150), round(dPhi1[2]*2500), Scalar(0, 255, 0), 2, 8, 0);// manzana
	circle(destinationImage, Point(round(AVGphi1[3]*3000)-rows+100,  -round(AVGphi2[3]*3000)+col-150), round(dPhi2[3]*1250), Scalar(0, 0, 255), 2, 8, 0);// sandia
	circle(destinationImage, Point(round(AVGphi1[4]*3000)-rows+100,  -round(AVGphi2[4]*3000)+col-150), round(dPhi1[4]*1500), Scalar(255, 0, 255), 2, 8, 0);// limon
	
	
	
}

void segmentImg(Mat &sourceImage, Mat &destinationImage, Point s, int c){
	Point po, pe;
	int x,y;
	queue<Point> Fo; //
	Fo.push(s);
	//if (!segDone[c]){
	A[c] = 1;
	m00[c] = A[c];
	m01[c] = s.x;
	m10[c] = s.y;
	m11[c] = (s.x)*(s.y);
	m02[c] = (s.y)*(s.y);
	m20[c] = (s.x)*(s.x);
	//}
	//cout << "Segmentating Region\n";
	//cout << "K: "<< k <<"\n";
	while (!Fo.empty()){
	
		pe = Fo.front();
		Fo.pop();
		for (int i = 0; i < 4; i++){
			po = pe + q[i];
			x = po.x;
			y = po.y;
			if (destinationImage.at<Vec3b>(x, y)[0] == 255){
				//cout << "Coloring: "<< x << " "<< y<<"\n";
				destinationImage.at<Vec3b>(x,y)[0] = paint[c][0];
				destinationImage.at<Vec3b>(x,y)[1] = paint[c][1];
				destinationImage.at<Vec3b>(x,y)[2] = paint[c][2];
				
				A[c]   ++;
				m00[c] ++;
				m10[c] += y;
				m01[c] += x;
				m11[c] += x*y;
				m02[c] += x*x;
				m20[c] += y*y;
				
				
				Fo.push(po);

				
				
			}	
			
		}
	}
	segDone[c] = true;
	//cout << "Finishing Segmentation \n";
	//cout << "Area of Region " << k << " is: "<< A[k] << " \n";
}


void findSeed(int x, int y,  Mat &destinationImage){
	b=destinationImage.at<Vec3b>(y, x)[0];
	g=destinationImage.at<Vec3b>(y, x)[1];
	r=destinationImage.at<Vec3b>(y, x)[2];
	//cout << "Color at point: "<< r << ", "<< g << ", "<< b<<"\n";
	if(b == 255 ){
		k++;
		fClick = true;
		//cout << "seed "<< k<<" found at: "<< x << " "<< y<<"\n";
		seed[k].x = y;
           	seed[k].y = x;
		destinationImage.at<Vec3b>(y, x)[0] = paint[k][0];
		destinationImage.at<Vec3b>(y, x)[1] = paint[k][1];
		destinationImage.at<Vec3b>(y, x)[2] = paint[k][2];
		
	}		
}


void seedFinder(int event, int x, int y, int flags, void* param)
{
    switch (event)
    {
        case EVENT_LBUTTONDOWN:
		
           	/*  Find seed */
           	cout << "point: "<< x << ", "<< y<<"\n";
           	findSeed(x,y,segmentedImage);
           	//k++
           	//if(k>0)
           	//segmentImg(filteredImage, segmentedImage, seed[k]);
           	
		
            break;
        case EVENT_RBUTTONDOWN:
         
            break; 
            
        case EVENT_MOUSEMOVE:
            break;
        case EVENT_LBUTTONUP:
            break;
 	}
 }

void filter(const Mat &sourceImage, Mat &destinationImage, Mat &RoI, int sigmaN){
	/* this function takes a source image and ROI, calculates its mean, and standar deviation, and then outputs a filtered image
	using the number of sigmas especified to create a color filter*/
	Mat mean, sigma;
	bool pass;


	destinationImage = Mat(sourceImage.rows, sourceImage.cols, sourceImage.type()); 

	meanStdDev(RoI, mean, sigma);//obtain mean and standard deviation form ROI

	for (int y = 0; y < sourceImage.rows; ++y)
		for (int x = 0; x < sourceImage.cols; ++x){
			pass = true;
			for (int i = 0; i < sourceImage.channels(); ++i){

				double lowerBound = mean.at<double>(i) - (sigma.at<double>(i) * sigmaN);
				double upperBound = mean.at<double>(i) + (sigma.at<double>(i) * sigmaN);
				//check if all pixels pass the filter
				if(sourceImage.at<Vec3b>(y, x)[i] < lowerBound || sourceImage.at<Vec3b>(y, x)[i] > upperBound)
					pass = false;
			}
				
			if(pass){
				destinationImage.at<Vec3b>(y, x) = {255,255,255};
			}

			else{
				destinationImage.at<Vec3b>(y, x) = {0,0,0};
			}
		}
}

void filterGray(const Mat &sourceImage, Mat &destinationImage, Mat &RoI, int sigmaN){
	/* this function takes a source image and ROI, calculates its mean, and standar deviation, and then outputs a filtered image
	using the number of sigmas especified to create a color filter*/
	Mat mean, sigma;
	bool pass;


	destinationImage = Mat(sourceImage.rows, sourceImage.cols, sourceImage.type()); 

	meanStdDev(RoI, mean, sigma);//obtain mean and standard deviation form ROI

	for (int y = 0; y < sourceImage.rows; ++y)
		for (int x = 0; x < sourceImage.cols; ++x){
			pass = true;
			for (int i = 0; i < sourceImage.channels(); ++i){

				double lowerBound = mean.at<double>(i) - (sigma.at<double>(i) * sigmaN);
				double upperBound = mean.at<double>(i) + (sigma.at<double>(i) * sigmaN);
				//check if all pixels pass the filter
				if(sourceImage.at<u_char>(y, x) < lowerBound || sourceImage.at<u_char>(y, x) > upperBound)
					pass = false;
			}
				
			if(pass){
				destinationImage.at<u_char>(y, x) = 255;
			}

			else{
				destinationImage.at<u_char>(y, x) = 0;
			}
		}
}

void RGB2Gray(const Mat &sourceImage, Mat &destinationImage)
{
	double G;
	destinationImage = Mat(sourceImage.rows, sourceImage.cols, 0); //Mat type 0 = U8 1 channel Mat
// weighted sum of RGB channels into single G channel
	for (int y = 0; y < sourceImage.rows; ++y)
		for (int x = 0; x < sourceImage.cols; ++x){
			G = sourceImage.at<Vec3b>(y, x)[0] *.114 //Blue channel
			+   sourceImage.at<Vec3b>(y, x)[1] *.587 //Green channel
			+   sourceImage.at<Vec3b>(y, x)[2] *.299;//Red channel
			
			destinationImage.at<u_char>(y, x) = (u_char)G; // mat.at<dataType>(row, column)
		}
}

void RGB2YIQ(const Mat &sourceImage, Mat &destinationImage)
{
	int R;
	int G;
	int B;
	double Y, I, Q;

	destinationImage = Mat(sourceImage.rows, sourceImage.cols, sourceImage.type()); 
// weighted sum of RGB channels into YIQ channels
	for (int y = 0; y < sourceImage.rows; ++y)
		for (int x = 0; x < sourceImage.cols; ++x){
			B = sourceImage.at<Vec3b>(y, x)[0]; //Blue channel
			G = sourceImage.at<Vec3b>(y, x)[1]; //Green channel
			R = sourceImage.at<Vec3b>(y, x)[2]; //Red channel


			Y = 0.299 * R + 0.587 * G + 0.114 * B; //Y
			I = 0.596 * R - 0.257 * G - 0.321 * B; //I
			Q = 0.212 * R - 0.523 * G + 0.311 * B; //Q

			// I & Q normalization
			// I min = 255*(-0.257 - 0.321) = -147.39; max = 255(.596) = 151.98; range = 151.98 - (-147.39) =  299.37
			I = (I * 0.85) + 125;
			// I min = 255*(-0.523) = -133.36; max = 255(.212 + .311) = 133.36; range = 133.36 - (-133.36) =  266
			Q = (Q * 0.95) + 126.7;

			destinationImage.at<Vec3b>(y, x)[0] = (int)Y; //Y
			destinationImage.at<Vec3b>(y, x)[1] = (int)I; //I
			destinationImage.at<Vec3b>(y, x)[2] = (int)Q;  //Q
		}
}






