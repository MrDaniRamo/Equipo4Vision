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
#include <cmath>
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
Mat RoIP;
Mat filteredImage;
Mat frame;
Mat convertedImage;
Mat segmentedImage;
Mat plot1;
Mat plot2;
Mat plot3;
Mat quads1;
Mat quads2;
Mat parking1;
Mat parking2;
Mat parkingBin;
Mat parkingF;
Mat parkingSpot;
Mat parkingBW;
Mat PotField;
Mat parkingPath;

int tr_s = 0;
int es = 0;

bool park = 0;
int matrixSize = 3;
int matrixKSize = 7;
bool showField;
Mat element = getStructuringElement( 0,
				Size (matrixKSize,matrixKSize),
				Point (0,0));

vector<Point> emptySpace = 	{Point(188,433),
							Point(188,384),
							Point(283,384),
							Point(348,234),
							Point(446,266),
							Point(510,268),
							Point(446,418),							
							Point(510,422),
							Point(366,45),
							Point(417,45),
							Point(35,535)};



vector<Point> points;

//interface control
bool pause = 0;
bool fClick = false;
bool isQuads = 0;
bool findFP = 0;
int key = 0;
int colorSpace = 49;
int b,g,r;
int quadx, quady;
int cRadio = 7;

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
double XL[7], YL[7];
double u00[7], u10[7], u01[7], u20[7], u02[7], u11[7];
double theta[7];
double n20[7], n02[7], n11[7];
double phi1[7], phi2[7];

//Entrenamiento
double AVGphi1[5] = {0.0, 0.2536562, 0.175228, 0.5095453 , 0.173};
double AVGphi2[5] = {0.0, 0.0863752, 0.0058838, 0.2328423 ,  0.00278411 };

double dPhi1[5] = { 0.0, 0.090781413, 0.025968, 0.142286, 0.0159};
double dPhi2[5] = { 0.0, 0.08236762, 0.070154, 0.172015, 0.004217441285};

int shape[5] = {0,0,0,0,0}; // 1 = zanahoria, 2 = limon, 3 = banana, 4 = naranja
int quad = 0;// 1 = zanahoria y limon, 2 = zanahoria y naranja, 3 = banana y limon, 4 = banana y naranja
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
int sigma_sliderP = 3;
int sigmaN;
int sigmaP;
bool firstFilter;
bool parkingFilter;
static void t_callback(int, void*){ // Trackbar callback function
}

//path taking
Point pes[5] = {Point(0,0), Point (80,76), Point(558,97), Point(84,469), Point(561,495)}; //possible points of entry
Point pE; //point of entry
int inD; //direccion inicial, 1 = derecha, 2 = arriba, 3 = izquierda, 4 = abajo;
int xB, yB;

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
Point findParkspot(Point parkSpot,  Mat &destinationImage);
void seedFinder(int event, int x, int y, int flags, void* param);
void parkspotFinder(int EVENT, int X, int Y, int flags, void* param);
void modelPlot(Mat &sourceImage, Mat &destinationImage, int c);
void createQuadrants(Mat &destinationImage);
void FindQuadrant(Mat &sourceImage, Mat &destinationImage);
void FindEntryPoint(int qu, Point pE);
void createPotField(Mat &sourceImage, Mat &destinationImage, Point Pf);
void calcPath(Mat &destinationImage, Mat &potField, Point Pi);
void animatePath(Mat &destinationImage, Mat &potField, Point Pi);
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
				if(!firstFilter)
				imshow("Original", currentImage);
				RGB2YIQ(currentImage, convertedImage);
				parking1 = imread("parking.png", IMREAD_COLOR);
				if(!parkingFilter)
				imshow("Estacionamiento", parking1);
				

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
			if(!isSegmented)
			imshow("Filtered image", filteredImage);
		}
		
		
		if(parkingFilter){
			//pre-create window to attach trackbar
			//namedWindow("Est. Binarizado", WINDOW_AUTOSIZE);
			//create trackbar (trackbar name, window name, pointer to int to store trackbar value, max value, trackbar callback)
			//createTrackbar("sigma", "Est. Binarizado", &sigma_sliderP, 5, t_callback);
			//sigmaP = getTrackbarPos("sigma", "Est. Binarizado");
			filter(parking1, parkingBin, RoIP, 5);
			//imshow("Est. Binarizado", parkingBin);
			cvtColor(parkingBin, parkingF, COLOR_BGR2GRAY);
			threshold(parkingBin, parkingF, 100,255, THRESH_BINARY);
			medianBlur(parkingF, parkingF, matrixSize);
			erode(parkingF, parkingF, element, Point(-1,-1), 2);
			if (park == 0){
			imshow("Estacionamiento filtrado", parkingF);
			}
			RGB2Gray(parkingF, parkingBW);
			
			if(showField){
				parkingPath = parking1.clone();
				createPotField(parkingBW, PotField, emptySpace[es]);
				cout << "funcion terminada " << endl;
				calcPath(parkingPath, PotField, pes[quad]);
				imshow("Path", parkingPath);
				waitKey(2000);
				animatePath(parkingPath, PotField, pes[quad]);
				showField = 0;
			}
		}
		

		if(findFP==0)
		if(isSegmented){
			//cout <<" here 1\n";
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
			XL[i] = xLine;
			YL[i] = yLine;
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
		
		
		
		if( key == 'f'){  //select color to be filtered
			cout << "Select object to be filtered\n";
			Rect2d r = selectROI("Object select", convertedImage, 0, 0); // Select ROI
			frame = convertedImage.clone();
			RoI = frame(r);
			imshow("ROI", RoI);
			destroyWindow("Object select");
			
			if(!firstFilter);
			destroyWindow("Original");
			firstFilter = true;
			key = 0;

		}
		
		if( key == 'g'){  //select color to be filtered
			cout << "Select object to be filtered\n";
			Rect2d r2 = selectROI("Object select", parking1, 0, 0); // Select ROI
			frame = parking1.clone();
			RoIP = frame(r2);
			//imshow("ROI", RoI);
			destroyWindow("Object select");
			if(!parkingFilter)
			destroyWindow("Estacionamiento");
			parkingFilter = true;
			key = 0;
		}
		
		
		if( key == 'w'){ //segment image
			segmentedImage = Mat(filteredImage.rows, filteredImage.cols, filteredImage.type()); 
			imshow("Segmented Image", segmentedImage);
			isSegmented = true;
			destroyWindow("Filtered image");
		}
		
		
		if (key == 'e'){

			park = 1;
			destroyWindow("Estacionamiento filtrado");
			parkingSpot = parkingF.clone();
			imshow("Parking Spots", parkingSpot);		
			setMouseCallback("Parking Spots", parkspotFinder);
			imshow("Parking Spots", parkingSpot);
			
		}
		

		if (key == 'd'){
			
			//isQuads = true;
			
			plot1 = imread("plot.png", IMREAD_COLOR);
			createQuadrants(quads1);
			quads2 = imread("cuadrants.png", IMREAD_COLOR);
			//imshow("Quadrants", quads2);
			for (int i = 1; i < 5; i++){
				maxphi1[i] = AVGphi1[i]+(dPhi1[i]);
				minphi1[i] = AVGphi1[i]-(dPhi1[i]);
				maxphi2[i] = AVGphi2[i]+(dPhi2[i]);
				minphi2[i] = AVGphi2[i]-(dPhi2[i]);
			}
			
			/*
			cout << "zanahoria: Max Phi1: "<< maxphi1[1] << " min Phi1: "<< minphi1[1] << " max Phi2 : "<< maxphi2[1]<< " min phi2 : "<< minphi2[1] << "\n";
			cout << "limon: Max Phi1: "<< maxphi1[2] << " min Phi1: "<< minphi1[2] << " max Phi2 : "<< maxphi2[2]<< " min phi2 : "<< minphi2[2] << "\n";
			cout << "banana: Max Phi1: "<< maxphi1[3] << " min Phi1: "<< minphi1[3] << " max Phi2 : "<< maxphi2[3]<< " min phi2 : "<< minphi2[3] << "\n";
			cout << "naranja: Max Phi1: "<< maxphi1[4] << " min Phi1: "<< minphi1[4] << " max Phi2 : "<< maxphi2[4]<< " min phi2 : "<< minphi2[4] << "\n";
			*/
			
			for (int i = 1; i < k+1; i ++){

			modelPlot(plot1,plot2, i);
			}

			FindQuadrant(quads1, quads2);

			//imshow("Quadrants", quads2);
			FindEntryPoint(quad, pE);
			parking2 = imread("parking.png", IMREAD_COLOR);
			drawMarker(parking2, pes[quad], Scalar(255,0,0),2, 12, 8 );
			imshow("parking2", parking2);
			
			
		}
		if (key == 'm'){
			
			showField = 1;
			

		
		}
		if (key == 'z'){
			findFP = 1;
			
			//showField = 1;
			destroyWindow("Segmented Image");
		
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
void FindEntryPoint(int qu, Point pE){
	switch (qu) {
		case 1: //izq arriba
			pE = pes[1];
			if (abs(xB) > abs(yB)) {
				inD = 1;
				cout << "Entry direction is right \n";
			}
			else{
				inD = 4; 
				cout << "Entry direction is down\n";
				
			}
		break;
		case 2: //derecha arriba
			pE = pes[2];
			if (abs(xB) > abs(yB)) {
				inD = 3;
				cout << "Entry direction is left \n";
			}
			else{
				inD = 4; 
				cout << "Entry direction is down\n";
				
			}
		break;
		
		case 3: //izq abajo
			pE = pes[3];
			if (abs(xB) > abs(yB)) {
				inD = 1;
				cout << "Entry direction is right \n";
			}
			else{
				inD = 2; 
				cout << "Entry direction is up\n";
				
			}
		break;
		case 4:  //derecha arriba
			pE = pes[4];
			if (abs(xB) > abs(yB)) {
				inD = 3;
				cout << "Entry direction is left \n";
			}
			else {
				inD = 1; 
				cout << "Entry direction is up\n";
				
			}
		break;
	}
	//cout<< "quadrant: " << qu << "\n";
	cout<< "point of entry: " << pE.x << ", " << pE.y<<"\n";
};

void FindQuadrant(Mat &sourceImage, Mat &destinationImage){
	switch (shape[1]) {
		case 1:  //zanahoria
			if (shape[2] == 2 | shape[2] == 0){ //limon
				circle(destinationImage, Point(128,128), 20, Scalar(0,0,255), 2, 8, 0); //cuadrante 1
				//cout << "zanahoria y limon\n";
				quad = 1;
				
			}
			else { //naranja
				circle(destinationImage, Point(384,128), 20, Scalar(0,0,255), 2, 8, 0); //cuadrante 2
				//cout << "zanahoria y naranja\n";
				quad = 2;
			}
			xB = XL[1];
			yB = YL[1];
			quadx = 200*cos(-theta[1]);
			quady = 200*sin(-theta[1]);
				line(destinationImage, Point(256 - quadx, 256+quady ),Point(256 + quadx, 256-quady ), Scalar( 0, 0, 255), 2, 8, 0);
			
		break;
		case 2:  //limon
			if (shape[2] == 1){ //zanahoria
				circle(destinationImage, Point(128,128), 20, Scalar(0,0,255), 2, 8, 0); //cuadrante 1
				//cout << "zanahoria y limon\n";
				quad = 1;
			}
			else { //banana
				circle(destinationImage, Point(128,384), 20, Scalar(0,0,255), 2, 8, 0); //cuadrante 2
				//cout << "banana y limon\n";
				quad = 3;
			}
			xB = XL[2];
			yB = YL[2];
			quadx = 200*cos(-theta[2]);
			quady = 200*sin(-theta[2]);
				line(destinationImage, Point(256 - quadx, 256+quady ),Point(256 + quadx, 256-quady ), Scalar( 0, 0, 255), 2, 8, 0);
		break;
		case 3: // banana
			if (shape[2] == 2){ //limon
				circle(destinationImage, Point(128,384), 20, Scalar(0,0,255), 2, 8, 0); //cuadrante 3
				//cout << "banana y limon\n";
				quad = 3;
			}
			else { //naranja
				circle(destinationImage, Point(384,384), 20, Scalar(0,0,255), 2, 8, 0); //cuadrante 4
				//cout << "banana y naranja\n";
				quad = 4;
			}
			xB = XL[1];
			yB = YL[1];
			quadx = 200*cos(-theta[1]);
			quady = 200*sin(-theta[1]);
				line(destinationImage, Point(256 - quadx, 256+quady ),Point(256 + quadx, 256-quady ), Scalar( 0, 0, 255), 2, 8, 0);
		break;
		case 4:  //naranja
			if (shape[2] == 1){ //zanahoria
				circle(destinationImage, Point(384,128), 20, Scalar(0,0,255), 2, 8, 0); //cuadrante 1
				//cout << "zanahoria y naranja\n";
				quad = 2;
			}
			else { //banana
				circle(destinationImage, Point(384,384), 20, Scalar(0,0,255), 2, 8, 0); //cuadrante 2
				//cout << "banana y naranja\n";
				quad = 4;
			}
			xB = XL[2];
			yB = YL[2];
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
		//	cout << "Phi2 " << c << " is: "<< p2 << " \n";
			

      	shape[c] = 0;

	if( ((p1 < maxphi1[4]) && (p1 > minphi1[4])) && ((p2 < maxphi2[4]) && (p2 > minphi2[4]))){ //naranja
      		shape[c] = 4;
      		
      	}

	else if( ((p1 < maxphi1[2])&& (p1 > minphi1[2])) && ((p2 < maxphi2[2]) && (p2 > minphi2[2]))){ //limon
      		shape[c] = 2;
      		
      	}  
      	
      	
	else if( ((p1 < maxphi1[1])&& (p1 > minphi1[1])) && ((p2 < maxphi2[1]) && (p2 > minphi2[1]))){ //zanahoria
      		shape[c] = 1;
      		
      	}
      	else if( ((p1 < maxphi1[3])&& (p1 > minphi1[3])) && ((p2 < maxphi2[3]) && (p2 > minphi2[3]))){ //banana
      		shape[c] = 3;
      		
      	}

      //	cout << "Region " << c << " is shape " << shape[c] <<"\n";
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

void findParkspot(int X, int Y,  Mat &parkingSpot){
	if (X >= 3 && X <= 55){
		if (Y >= 523 && Y <= 548){
			drawMarker(parkingSpot, (Point)emptySpace[10], Scalar(150,0,100),2, cRadio, 8 );
			cout << "Parking location: "<< X << ", "<< Y<<"\n";
			es = 10;
		}
	}

	if (X >= 159 && X <= 210){
		if (Y >= 420 && Y <= 448){
			drawMarker(parkingSpot, (Point)emptySpace[0], Scalar(150,0,100),2, cRadio, 8 );
			cout << "Parking location: "<< X << ", "<< Y<<"\n";
			es = 0;
		}
	}

	if (X >= 159 && X <= 210){
		if (Y >= 371 && Y <= 394){
			drawMarker(parkingSpot, (Point)emptySpace[1], Scalar(150,0,100),2, cRadio, 8 );
			cout << "Parking location: "<< X << ", "<< Y<<"\n";
			es = 1;
		}
	}

	if (X >= 265 && X <= 318){
		if (Y >= 376 && Y <= 400){
			drawMarker(parkingSpot, (Point)emptySpace[2], Scalar(150,0,100),2, cRadio, 8 );
			cout << "Parking location: "<< X << ", "<< Y<<"\n";
			es = 2;
		}
	}

	if (X >= 317 && X <= 370){
		if (Y >= 225 && Y <= 250){
			drawMarker(parkingSpot, (Point)emptySpace[3], Scalar(150,0,100),2, cRadio, 8 );
			cout << "Parking location: "<< X << ", "<< Y<<"\n";
			es = 3;
		}
	}

	if (X >= 423 && X <= 479){
		if (Y >= 255 && Y <= 278){
			drawMarker(parkingSpot, (Point)emptySpace[4], Scalar(150,0,100),2, cRadio, 8 );
			cout << "Parking location: "<< X << ", "<< Y<<"\n";
			es = 4;
		}
	}

	if (X >= 483 && X <= 535){
		if (Y >= 255 && Y <= 282){
			drawMarker(parkingSpot, (Point)emptySpace[5], Scalar(150,0,100),2, cRadio, 8 );
			cout << "Parking location: "<< X << ", "<< Y<<"\n";
			es = 5;
		}
	}

	if (X >= 427 && X <= 481){
		if (Y >= 407 && Y <= 433){
			drawMarker(parkingSpot, (Point)emptySpace[6], Scalar(150,0,100),2, cRadio, 8 );
			cout << "Parking location: "<< X << ", "<< Y<<"\n";
			es = 6;
		}
	}

	if (X >= 486 && X <= 536){
		if (Y >= 410 && Y <= 436){
			drawMarker(parkingSpot, (Point)emptySpace[7], Scalar(150,0,100),2, cRadio, 8 );
			cout << "Parking location: "<< X << ", "<< Y<<"\n";
			es = 7;
		}
	}

	if (X >= 356 && X <= 380){
		if (Y >= 12 && Y <= 67){
			drawMarker(parkingSpot, (Point)emptySpace[8], Scalar(150,0,100),2, cRadio, 8 );
			cout << "Parking location: "<< X << ", "<< Y<<"\n";
			es = 8;
		}
	}

	if (X >= 405 && X <= 432){
		if (Y >= 12 && Y <= 67){
			drawMarker(parkingSpot, (Point)emptySpace[9], Scalar(150,0,100),2, cRadio, 8 );
			cout << "Parking location: "<< X << ", "<< Y<<"\n";
			es = 9;
		}
	}

	imshow ("Parking Spots", parkingSpot);
}

void parkspotFinder(int EVENT, int X, int Y, int flags, void* param)
{
    switch (EVENT)
    {
        case EVENT_LBUTTONDOWN:
		
           	/*  Find seed */
           	points.push_back(Point(X, Y));
           	findParkspot(X,Y,parkingSpot);
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

void createPotField(Mat &sourceImage, Mat &destinationImage, Point Pf){
	destinationImage = Mat::zeros(sourceImage.rows, sourceImage.cols, CV_16UC1); 
	Mat counted = Mat::zeros(sourceImage.rows, sourceImage.cols, sourceImage.type()); 
	Mat show = Mat(sourceImage.rows, sourceImage.cols, sourceImage.type());

	for (int i = 0; i < destinationImage.rows; i++){
		for (int j = 0; j < destinationImage.cols; j++){
			destinationImage.at<u_short>(i,j) = 1000;
		}
	}

	Point po, pe;
	int x,y;
	queue<Point> FoA; // Actual level Fo
	queue<Point> FoN; // Next Fo
	queue<Point> FoR; // Reset queue
	
	FoN.push(Pf);
	int level = 0;
	x = Pf.x;
	y = Pf.y;

	
	destinationImage.at<u_short>(y, x) = level;
	counted.at<u_char>(y, x) = 1;
	//cout << "Hola " << endl;

	while (!FoN.empty()){
		FoA = FoN;
		FoN = FoR;
		level++;
		//cout << "level" << level << endl;
		while (!FoA.empty()){
			pe = FoA.front();
			FoA.pop();
			for (int i = 0; i < 4; i++){
				po = pe + q[i];
				x = po.x;
				y = po.y;

				if (sourceImage.at<u_char>(y, x) == 255 && counted.at<u_char>(y, x) == 0){

					destinationImage.at<u_short>(y, x) = level;
					counted.at<u_char>(y, x) = 1;
					
					FoN.push(po);
					
					
				}	
				
			}
		}

	}
	//ofstream myfile;
	// myfile.open ("field.txt");
	// myfile << "PotFields = " << endl << " "  << destinationImage << endl << endl;
	// myfile.close();
	for (int i = 0; i < destinationImage.rows-1; i++){
		for (int j = 0; j < destinationImage.cols-1; j++){
			u_int val = (destinationImage.at<u_short>(i,j))/4.0;
			show.at<u_char>(i,j) = u_char(val);
		}
	}

	imshow("PotFields", show);
}

void calcPath(Mat &destinationImage, Mat &potField, Point Pi){
	Point Pa, Po, Pn;
	int xa, ya, xo, yo;
	Pa = Pi;
	xa = Pa.x;
	ya = Pa.y;
	//drawMarker(sourceImage, Pa, Scalar(255,0,0),2, 12, 8 );
	int offset = 0;
	int dir;
	

	while (potField.at<u_short>(ya, xa) != 0){

		//cout << "Pa: " << Pa << endl;
		for (int i = 0; i < 4; i++){

			dir = i + offset;

			if(dir > 3){
				dir = dir - 4;
			}
			//cout << "dir: " << dir << endl;
			Po = Pa + q[i];
			xo = Po.x;
			yo = Po.y;
	
			// cout << "Po Val " << potField.at<u_short>(yo, xo) << endl;
			// cout << "Pa Val " << potField.at<u_short>(ya, xa) << endl;

			if(potField.at<u_short>(yo, xo) < potField.at<u_short>(ya, xa)){
				Pn = Po;
				xa = Pn.x;
				ya = Pn.y;
				offset = i;
			}

		}
		Pa = Pn;
		circle(destinationImage, Pa, 1, Scalar(255,255,0),2);

	}

}

void animatePath(Mat &destinationImage, Mat &potField, Point Pi){
	Point Pa, Po, Pn;
	int xa, ya, xo, yo;
	Pa = Pi;
	xa = Pa.x;
	ya = Pa.y;
	
	int offset = 0;
	int dir;

	Mat tempMat; // temp mat to store original image
	tempMat = destinationImage.clone();
	

	while (potField.at<u_short>(ya, xa) != 0){
		drawMarker(destinationImage, Pa, Scalar(255,0,0),2, 12, 8 );
		imshow("Path", destinationImage);
		//cout << "Pa: " << Pa << endl;
		for (int i = 0; i < 4; i++){

			dir = i + offset;

			if(dir > 3){
				dir = dir - 4;
			}
			//cout << "dir: " << dir << endl;
			Po = Pa + q[i];
			xo = Po.x;
			yo = Po.y;
	
			// cout << "Po Val " << potField.at<u_short>(yo, xo) << endl;
			// cout << "Pa Val " << potField.at<u_short>(ya, xa) << endl;

			if(potField.at<u_short>(yo, xo) < potField.at<u_short>(ya, xa)){
				Pn = Po;
				xa = Pn.x;
				ya = Pn.y;
				offset = i;
			}

		}
		Pa = Pn;
		waitKey(10);
		destinationImage = tempMat.clone();


	}

}


