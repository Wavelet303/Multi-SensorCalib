#include <windows.h>
#include <math.h>
#include <iostream>
#include <vector>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "RGBDStreamer/IRGBDStreamer.h"
#include "Multi-SensorCalib.h"

#define IMG_ACQUISITION 0
#define IMG_SAVING 0
#define IMG_LOADING 1
#define IMG_DSIPLAY 0
#define CALIBRATION 1



using namespace std;
using namespace cv;
const string strImgFolder = "Kinect2CircleGridImgs";

int main()
{
	vector<vector<Mat>> imgList;
	#if IMG_ACQUISITION
	//imshow( "Sensor0", cv::Mat( Height, Width, CV_16UC1 ));
	IRGBDStreamer* pKinect2 = StreamFactory::createFromKinect2( true, true, true );
	pKinect2->StartStream();
	FrameData Frames[3];

	uint16_t ColorWidth, ColorHeight, InfraredWidth, InfraredHeight;
	do
	{
		pKinect2->GetColorReso( ColorWidth, ColorHeight );
	} while (ColorWidth == 0 || ColorHeight == 0);
	//cv::Mat colMat, colDisplayMat;

	do
	{
		pKinect2->GetInfraredReso( InfraredWidth, InfraredHeight);
	} while (InfraredWidth == 0 || InfraredHeight == 0);
	//cv::Mat infMat, infDisplayMat;

	////Code for Visualize live video streams
	//while (true)
	//{
	//	pKinect2->GetFrames( Frames[0], Frames[1], Frames[2] );

	//	if (Frames[0].pData == nullptr) continue;
	//	colMat = cv::Mat( ColorHeight, ColorWidth, CV_8UC4, Frames[0].pData );
	//	cvtColor( colMat, colDisplayMat, CV_BGRA2RGBA );

	//	if(Frames[2].pData == nullptr) continue;
	//	infMat = cv::Mat( InfraredHeight, InfraredWidth, CV_16UC1, reinterpret_cast<uint16_t*>(Frames[2].pData) );
	//	infMat.convertTo( infMat, CV_32F );
	//	infMat = infMat / 65535.f;
	//	cv::pow( infMat, 0.32, infMat );
	//	infMat = infMat * 256;
	//	infMat.convertTo( infMat, CV_8UC1 );
	//	cvtColor( infMat, infDisplayMat, CV_GRAY2BGR );

	//	imshow( "Sensor0", colDisplayMat);
	//	imshow( "Sensor1", infDisplayMat );
	//// waitKey is essential for imshow to work properly in loop
	//	waitKey( 10 );
	//}

	imgList = Calibration::ImagesAcquisition( 2, [&]( vector<Mat>& imgs )->bool {
		bool succeed;
		// To make sure Kinect is fully up and start stream data before processing input
		do 
		{
			pKinect2->GetFrames( Frames[0], Frames[1], Frames[2] );
			succeed = (Frames[0].pData != nullptr && Frames[2].pData != nullptr);
		} while (!succeed);

		// Use depth sensor (infrared camera) as master sensor
		cv::Mat infMat = cv::Mat( InfraredHeight, InfraredWidth, CV_16UC1, reinterpret_cast<uint16_t*>(Frames[2].pData) );
		// Convert infrared image to compatible format
		infMat.convertTo( infMat, CV_32F );
		infMat = infMat / 65535.f;
		cv::pow( infMat, 0.32, infMat );
		infMat = infMat * 256;
		infMat.convertTo( infMat, CV_8UC1 );
		cv::Mat infDisplayMat;
		cvtColor( infMat, infDisplayMat, CV_GRAY2BGR );
		imgs.push_back( infDisplayMat );

		// Process color camera data (as slave sensor)
		cv::Mat colMat = cv::Mat( ColorHeight, ColorWidth, CV_8UC4, Frames[0].pData );
		cv::Mat colDisplayMat;
		cvtColor( colMat, colDisplayMat, CV_BGRA2RGBA );
		cvtColor( colDisplayMat, colDisplayMat, CV_RGBA2RGB );
		imgs.push_back( colDisplayMat );

		return succeed;
	} );
	#endif

#if IMG_SAVING	
	// Save images
	Calibration::SaveImages( imgList, strImgFolder );
#endif

#if IMG_LOADING
	// Load images
	imgList = Calibration::LoadImages( strImgFolder + "\\syncimgList.yml" );
#endif

#if CALIBRATION
	// Calibration
	Calibration::BundleCalibration( strImgFolder + "\\calibration_result.yml", imgList );
#endif

#if IMG_DSIPLAY
	Calibration::DisplayImages( imgList );
#endif
	return 0;
}
