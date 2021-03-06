
// Include  IGSIO - VTK //
#include "igsioConfigure.h"
#include "igsioTrackedFrame.h"
#include "igsioXmlUtils.h"
#include "vtkImageData.h"
#include "vtkMatrix4x4.h"
#include "vtkMatrix3x3.h"
#include "vtkIGSIOSequenceIO.h"
#include "vtkIGSIOTrackedFrameList.h"
#include "vtkIGSIOTransformRepository.h"
#include "vtkIGSIOVolumeReconstructor.h"
#include "vtkXMLUtilities.h"
#include "vtksys/CommandLineArguments.hxx"
#include "igsioVideoFrame.h"
#include "vtkDataArray.h"
#include "vtkDataSetMapper.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include <vtkCamera.h>
#include <vtkColorTransferFunction.h>
#include <vtkFixedPointVolumeRayCastMapper.h>
#include <vtkMetaImageReader.h>
#include <vtkNamedColors.h>
#include <vtkPiecewiseFunction.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkVolume.h>
#include <vtkVolumeProperty.h>
#include <igsioCommon.h>
#include <vtkTransform.h>

// Include OpenCV
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

//Include Other
#include<fstream>
#include <string>
#include <windows.h>
#include <iostream>


#define INF 10000 


using namespace std;
using namespace cv;


int waitTime = 0;
Mat canny_output;
Mat drawing;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
Mat dst;
Mat drawing2 = Mat::zeros(230, 170, CV_8UC3);



// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr' 
bool onSegment(Point p, Point q, Point r)
{
	if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
		q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
		return true;
	return false;
}

// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are colinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int orientation(Point p, Point q, Point r)
{
	int val = (q.y - p.y) * (r.x - q.x) -
		(q.x - p.x) * (r.y - q.y);

	if (val == 0) return 0;  // colinear 
	return (val > 0) ? 1 : 2; // clock or counterclock wise 
}

// The function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool doIntersect(Point p1, Point q1, Point p2, Point q2)
{
	// Find the four orientations needed for general and 
	// special cases 
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// General case 
	if (o1 != o2 && o3 != o4)
		return true;

	// Special Cases 
	// p1, q1 and p2 are colinear and p2 lies on segment p1q1 
	if (o1 == 0 && onSegment(p1, p2, q1)) return true;

	// p1, q1 and p2 are colinear and q2 lies on segment p1q1 
	if (o2 == 0 && onSegment(p1, q2, q1)) return true;

	// p2, q2 and p1 are colinear and p1 lies on segment p2q2 
	if (o3 == 0 && onSegment(p2, p1, q2)) return true;

	// p2, q2 and q1 are colinear and q1 lies on segment p2q2 
	if (o4 == 0 && onSegment(p2, q1, q2)) return true;

	return false; // Doesn't fall in any of the above cases 
}

// Returns true if the point p lies inside the polygon[] with n vertices 
bool isInside(vector<Point> polygon, int n, Point p)
{
	// There must be at least 3 vertices in polygon[] 
	if (n < 3)  return false;
	
	// Create a point for line segment from p to infinite 
	Point extreme = { INF, p.y-50 };
	circle(drawing2, Point(extreme.x, extreme.y), 2, Scalar(0, 0, 255), FILLED, LINE_AA);

	// Count intersections of the above line with sides of polygon 
	int count = 0, i = 0;
	do
	{
		//int next = (i + 1) ;

		// Check if the line segment from 'p' to 'extreme' intersects 
		// with the line segment from 'polygon[i]' to 'polygon[next]' 

		if (doIntersect(polygon[i], polygon[i+1], p, extreme))
		{
			// If the point 'p' is colinear with line segment 'i-next', 
			// then check if it lies on segment. If it lies, return true, 
			// otherwise false 
			if (orientation(polygon[i], p, polygon[i+1]) == 0)
				return onSegment(polygon[i], p, polygon[i+1]);

			count++;
		}
		i = (i + 1) % n;
	} while (i !=0);

	// Return true if count is odd, false otherwise 
	return count & 1;  // Same as (count%2 == 1) 
}


// Read Camera Parameters
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	return true;
}


// Stereo tracking
void triangulation(bool flag) {

	if (flag == true) {
		float marker_size = 0.005;
		Mat distCoeffsL;
		Mat cameraMatrixL;
		Mat distCoeffsR;
		Mat cameraMatrixR;
		Mat projMatL;
		Mat projMatR;

		vector<int> idsL, idsR;
		vector<vector<Point2f>> cornersL, cornersR;
		//Ptr<Dictionary> dictionary = getPredefinedDictionary(DICT_4X4_50);
		Ptr<aruco::Dictionary> dictionary = aruco::generateCustomDictionary(6, 3);

		FileStorage calibrationL("..\\..\\..\\..\\data\\Calibration\\cameraL.yaml", FileStorage::READ);
		calibrationL["camera_matrix"] >> cameraMatrixL;
		calibrationL["distortion_coefficients"] >> distCoeffsL;
		calibrationL.release();
		cout << "camera_matrixL  = " << endl << cameraMatrixL << endl << endl;
		cout << "distCoeffsL  = " << endl << distCoeffsL << endl << endl;

		FileStorage calibrationR("..\\..\\..\\..\\data\\Calibration\\\cameraR.yaml", FileStorage::READ);
		calibrationR["camera_matrix"] >> cameraMatrixR;
		calibrationR["distortion_coefficients"] >> distCoeffsR;
		calibrationR.release();
		cout << "camera_matrixR  = " << endl << cameraMatrixR << endl << endl;
		cout << "distCoeffsR  = " << endl << distCoeffsR << endl << endl;

		FileStorage Stereo_calibration("..\\..\\..\\..\\data\\Calibration\\extrinsics.yml", FileStorage::READ);
		Stereo_calibration["P1"] >> projMatL;
		Stereo_calibration["P2"] >> projMatR;
		Stereo_calibration.release();
		cout << "ProjectionL  = " << endl << projMatL << endl << endl;
		cout << "ProjectionR  = " << endl << projMatR << endl << endl;

		VideoCapture videoL("..\\..\\..\\..\\data\\Videos\\EndoscopeImageMemory_0_Cropped_1310_1412.avi");
		VideoCapture videoR("..\\..\\..\\..\\data\\Videos\\EndoscopeImageMemory_1_Cropped_1310_1412.avi");

		int frameNum = 0;
		int validNum = 0;

		Ptr<aruco::DetectorParameters> parameter = aruco::DetectorParameters::create();
		//parameter->errorCorrectionRate = 0.8f;

		while (videoL.grab() && videoR.grab())
		{
			frameNum++;

			int num_markers = 0;

			Mat imageL, imageR, imageL_un, imageR_un, image_copyL, image_copyR;
			videoL.retrieve(imageL);
			videoR.retrieve(imageR);

			//undistort(image, image_distortion, camera_matrix, distortion_coefficients);
			imageL.copyTo(image_copyL);
			imageR.copyTo(image_copyR);


			Mat T_m = Mat::eye(4, 4, CV_64F);
			Mat T1_m = Mat::eye(4, 4, CV_64F);
			Mat Rot_m, Rot_m2;

			undistort(imageL, imageL_un, cameraMatrixL, distCoeffsL);
			undistort(imageR, imageR_un, cameraMatrixR, distCoeffsR);

			detectMarkers(imageL_un, dictionary, cornersL, idsL, parameter);
			detectMarkers(imageR_un, dictionary, cornersR, idsR, parameter);
			// if at least one marker detected
			vector<Vec3d> rvecs, tvecs;
			//Mat rvec, tvec;
			Vec3d rvec, tvec;

			if (idsL.size() > 0) {
				for (int i = 0; i < idsL.size(); i++) {
					if (idsL[i] == 4) {
						if (idsR.size() > 0) {
							for (int j = 0; j < idsR.size(); j++) {
								if (idsR[j] == 4) {
									aruco::estimatePoseSingleMarkers(cornersL, marker_size, cameraMatrixL, distCoeffsL, rvecs, tvecs);

									aruco::drawDetectedMarkers(imageL, cornersL, idsL);
									//imshow("Detected markers", imageL);

									rvec = rvecs[i];
									tvec = tvecs[i];

									cout << "CornersL  = " << endl << cornersL[i] << endl << endl;
									cout << "CornersR  = " << endl << cornersR[j] << endl << endl;


									Mat triangCoords4D;
									triangulatePoints(projMatL, projMatR, cornersL[i], cornersR[j], triangCoords4D);
									Vec4f triangCoords1 = triangCoords4D.col(0);
									Vec4f triangCoords2 = triangCoords4D.col(1);
									Vec4f triangCoords3 = triangCoords4D.col(2);
									Vec4f triangCoords4 = triangCoords4D.col(3);
									cout << "triangCoords4D  = " << endl << triangCoords4D << endl << endl;
									Vec3f Coords13D, Coords23D, Coords33D, Coords43D;
									for (unsigned int k = 0; k < 3; k++) {
										Coords13D[k] = triangCoords1[k] / triangCoords1[3];
										Coords23D[k] = triangCoords2[k] / triangCoords2[3];
										Coords33D[k] = triangCoords3[k] / triangCoords3[3];
										Coords43D[k] = triangCoords4[k] / triangCoords4[3];
									}
									cout << "Coord1  = " << endl << Coords13D << endl << endl;
									cout << "Coord2  = " << endl << Coords23D << endl << endl;
									cout << "Coord3  = " << endl << Coords33D << endl << endl;
									cout << "Coord4  = " << endl << Coords43D << endl << endl;

									validNum++;
									cout << "Valid Frames  = " << endl << validNum << endl << endl;
									cout << "Frames  = " << endl << frameNum << endl << endl;

								}
							}
						}
						break;
					}
				}
			}
			char key = (char)waitKey(10);
			if (key == 27)
				break;
		}
		videoL.release();
		videoR.release();
	}
}

Mat  readCSVfile() {
		
	// Sep
	///ifstream fin("..\\..\\..\\..\\data\\Kinematics\\DaVinciSiMemory_test.csv");

	// Oct
	ifstream fin("..\\..\\..\\..\\data\\SyncedWithTimestamps\\Together\\part5_kine_test.csv");

	if (!fin.is_open())
		cout << "Can not open the csv file" << endl;


	///string index,size, tag, timestamp, type, Timestamp, JointAngles_ECM, JointAngles_PSM1, JointAngles_PSM2,
	///	JointAngles_PSM3, Pose_Base, Pose_ECM, Pose_PSM, Pose_RCM, Pose_Mount, Pose_Workplace;

	string Column1, Timestamp, Pose_1, Pose_2, Pose_3, Pose_4, Pose_5, Pose_6, Pose_7, Pose_8, Pose_9, Pose_10, Pose_11, Pose_12;


	Mat traNsformations(399,12,CV_64F);
	int tr_i = 0;
	int i = 0;
	while (i<1000) {
		
		getline(fin, Column1, ',');
		getline(fin, Timestamp, ',');
		getline(fin, Pose_1, ',');
		getline(fin, Pose_2, ',');
		getline(fin, Pose_3, ',');
		getline(fin, Pose_4, ',');
		getline(fin, Pose_5, ',');
		getline(fin, Pose_6, ',');
		getline(fin, Pose_7, ',');
		getline(fin, Pose_8, ',');
		getline(fin, Pose_9, ',');
		getline(fin, Pose_10, ',');
		getline(fin, Pose_11, ',');
		getline(fin, Pose_12, '\n');
		

		if (i>550 && i<950) {
			///stringstream ss(Pose_PSM);
			vector<double> doubles;


			double number = stod(Pose_1.c_str());
			doubles.push_back(number);
			number = stod(Pose_2.c_str());
			doubles.push_back(number);
			number = stod(Pose_3.c_str());
			doubles.push_back(number);
			number = stod(Pose_4.c_str());
			doubles.push_back(number);
			number = stod(Pose_5.c_str());
			doubles.push_back(number);
			number = stod(Pose_6.c_str());
			doubles.push_back(number);
			number = stod(Pose_7.c_str());
			doubles.push_back(number);
			number = stod(Pose_8.c_str());
			doubles.push_back(number);
			number = stod(Pose_9.c_str());
			doubles.push_back(number);
			number = stod(Pose_10.c_str());
			doubles.push_back(number);
			number = stod(Pose_11.c_str());
			doubles.push_back(number);
			number = stod(Pose_12.c_str());
			doubles.push_back(number);

			for (int j = 0; j < 12; j++) {
				traNsformations.at<double>(tr_i,j) = doubles[j];
			}
			tr_i++;
			}
		i++;
		}
	
	return traNsformations;

}

int main(int argc, char** argv)
{

	// 	Read Kinematics from CSV file	
	Mat inputTranformations=readCSVfile();

	// Read Endoscopic Video
	cv::Mat InImageR, InImageL ,InImageCopy;
		
	// Read Ultrasound Video
	cv::Mat InImage_Ultra;
	VideoCapture vreader_Ultra("..\\..\\..\\..\\data\\SyncedWithTimestamps\\RenderedImage_550_950.avi");
	if (vreader_Ultra.isOpened())
		vreader_Ultra >> InImage_Ultra;
	else
	{
		cerr << "Could not open Ultra input" << endl;
		return -1;
	}

	char key = 0; 

    //_______Initialize OpenCV - Aruco_____//
	
	bool showRejected = false;
	bool estimatePose = true;
	float markerLength = 0.005;


	Mat camMatrix, distCoeffs;
	if (true) {
		bool readOk = readCameraParameters("..\\..\\..\\..\\data\\Camera_1_calibration.xml", camMatrix, distCoeffs);
		if (!readOk) {
			cerr << "Invalid camera 1 file" << endl;
			return 0;
		}
	}

	float marker_size = 0.005;
	Mat distCoeffsL;
	Mat cameraMatrixL;
	Mat distCoeffsR;
	Mat cameraMatrixR;
	Mat projMatL;
	Mat projMatR;

	vector<int> idsL, idsR;
	vector<vector<Point2f>> cornersL, cornersR;
	//Ptr<Dictionary> dictionary = getPredefinedDictionary(DICT_4X4_50);
	//Ptr<aruco::Dictionary> dictionary = aruco::generateCustomDictionary(6, 3);

	FileStorage calibrationL("..\\..\\..\\..\\data\\Calibration\\cameraL.yaml", FileStorage::READ);
	calibrationL["camera_matrix"] >> cameraMatrixL;
	calibrationL["distortion_coefficients"] >> distCoeffsL;
	calibrationL.release();
	cout << "camera_matrixL  = " << endl << cameraMatrixL << endl << endl;
	cout << "distCoeffsL  = " << endl << distCoeffsL << endl << endl;

	FileStorage calibrationR("..\\..\\..\\..\\data\\Calibration\\\cameraR.yaml", FileStorage::READ);
	calibrationR["camera_matrix"] >> cameraMatrixR;
	calibrationR["distortion_coefficients"] >> distCoeffsR;
	calibrationR.release();
	cout << "camera_matrixR  = " << endl << cameraMatrixR << endl << endl;
	cout << "distCoeffsR  = " << endl << distCoeffsR << endl << endl;

	FileStorage Stereo_calibration("..\\..\\..\\..\\data\\Calibration\\extrinsics.yml", FileStorage::READ);
	Stereo_calibration["P1"] >> projMatL;
	Stereo_calibration["P2"] >> projMatR;
	Stereo_calibration.release();
	cout << "ProjectionL  = " << endl << projMatL << endl << endl;
	cout << "ProjectionR  = " << endl << projMatR << endl << endl;


	// After Sept
	VideoCapture videoR("..\\..\\..\\..\\data\\SyncedWithTimestamps\\Endoscope_550_950.avi");
	VideoCapture videoL("..\\..\\..\\..\\data\\SyncedWithTimestamps\\Endoscope_550_950.avi");




	if (videoL.isOpened())
		videoL >> InImageL;
	else
	{
		cerr << "Could not open input" << endl;
		return -1;
	}
	if (videoR.isOpened())
		videoR >> InImageR;
	else
	{
		cerr << "Could not open input" << endl;
		return -1;
	}
	int frameNum = 0;
	int validNum = 0;



	Ptr<aruco::DetectorParameters> parameter = aruco::DetectorParameters::create();
	parameter->errorCorrectionRate = 0.5f;
	//Ptr<aruco::Dictionary> dictionary =
	//	aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_ARUCO_ORIGINAL);

	Ptr<aruco::Dictionary> dictionary =
		aruco::generateCustomDictionary(6, 3);


	//____End OpenCV Initialization______//


	//____Initializate  IGSIO___________ //
	std::string inputImgSeqFileName;// "C:/Users/Charalampos/Desktop/Elbow_Volume/ElbowUltrasoundSweep.mha";
	std::string inputConfigFileName = "..\\..\\..\\..\\data\\Kidney-VolRec.xml";
	std::string outputVolumeFileName = "..\\..\\..\\..\\data\\Volume_xaris_15_04_kinematics.mha";
	std::string outputVolumeAlphaFileNameDeprecated;
	std::string outputVolumeAccumulationFileName;
	std::string outputFrameFileName;
	std::string importanceMaskFileName;
	std::string inputImageToReferenceTransformName = "ImageToReference";


	int verboseLevel = vtkIGSIOLogger::LOG_LEVEL_UNDEFINED;

	bool disableCompression = false;
	vtkIGSIOLogger::Instance()->SetLogLevel(verboseLevel);

	vtkSmartPointer<vtkIGSIOVolumeReconstructor> reconstructor = vtkSmartPointer<vtkIGSIOVolumeReconstructor>::New();

	if (!importanceMaskFileName.empty())
	{
		reconstructor->SetImportanceMaskFilename(importanceMaskFileName);
	}


	LOG_INFO("Reading configuration file:" << inputConfigFileName);
	vtkSmartPointer<vtkXMLDataElement> configRootElement = vtkSmartPointer<vtkXMLDataElement>::New();
	if (igsioXmlUtils::ReadDeviceSetConfigurationFromFile(configRootElement, inputConfigFileName.c_str()) == IGSIO_FAIL)
	{
		LOG_ERROR("Unable to read configuration from file " << inputConfigFileName.c_str());
		//return EXIT_FAILURE;
	}

	if (reconstructor->ReadConfiguration(configRootElement) != IGSIO_SUCCESS)
	{
		LOG_ERROR("Failed to read configuration from " << inputConfigFileName.c_str());
		//return EXIT_FAILURE;
	}
		
	vtkSmartPointer<vtkIGSIOTransformRepository> transformRepository = vtkSmartPointer<vtkIGSIOTransformRepository>::New();
	if (configRootElement->FindNestedElementWithName("CoordinateDefinitions") != NULL)
	{
		if (transformRepository->ReadConfiguration(configRootElement) != IGSIO_SUCCESS)
		{
			LOG_ERROR("Failed to read transforms from CoordinateDefinitions");
			//return EXIT_FAILURE;
		}
	}
	else
	{
		LOG_DEBUG("No transforms were found in CoordinateDefinitions. Only the transforms defined in the input image will be available.");
	}

	// Print calibration transform
	std::ostringstream osTransformRepo;
	transformRepository->Print(osTransformRepo);
	LOG_DEBUG("Transform repository: \n" << osTransformRepo.str());

	// Read image sequence
	LOG_INFO("Reading image sequence " << inputImgSeqFileName);
	vtkSmartPointer<vtkIGSIOTrackedFrameList> trackedFrameList = vtkSmartPointer<vtkIGSIOTrackedFrameList>::New();
		
	double timestamp = 232.542071;
	int images_count = 0;


	//______End IGSIO initialization _________________//


	bool detectTumor = false; // true if image segmentation and save new images
	bool visual = true;     // true if visual tracking , false for kinematics
	bool visual_print = false;

	bool detection_happened = false;
	int frames_counter = 0;  // counter for frames
	int used_frames = 100;   // choose how many frames to use 
	int loops = 0; // counter for loops
		
	int first_key = 0;
	std::vector<KeyPoint> first_keypoints;
	int point_count = 0;
		
	
	

	do
	{
		loops++;
		Vec3d rvec, tvec;
		Mat imageL, imageR;
		videoL.retrieve(imageL); // Read frame from endoscopic 0
		videoR.retrieve(imageR); // Read frame from endoscopic 1

		if (visual) {
			frameNum++;

			// Open CV detection - Single Camera

			vector< int > ids;
			vector< vector< Point2f > > corners, rejected;
			vector< Vec3d > rvecs, tvecs;

			detectMarkers(imageR, dictionary, cornersR, idsR, parameter);
			//aruco::drawDetectedMarkers(imageR, cornersR, idsR);

			if (idsR.size() > 0) {

				for (unsigned int i = 0; i < idsR.size(); i++) {

					if (idsR[i] == 2) {

						detection_happened = true;
						aruco::estimatePoseSingleMarkers(cornersR, markerLength, cameraMatrixR, distCoeffsR, rvecs,
							tvecs);
						cout << tvecs[i];
						if (visual) {
							rvec = rvecs[i];
							tvec = tvecs[i];
						}
						aruco::drawAxis(imageR, cameraMatrixR, distCoeffsR, rvecs[i], tvecs[i],
							markerLength * 5.f);
					}

				}
				aruco::drawDetectedMarkers(imageR, cornersR, idsR);
				imshow("Detection of Aruco Markers", imageR);
			}
		}
			// End of  Detection //



		vreader_Ultra.retrieve(InImage_Ultra); // Read frame from Ultrasound video

		if (!visual)
			detection_happened = true;

		key = cv::waitKey(waitTime);  // wait for key to be pressed
		if (key == 'r')
			waitTime = waitTime == 0 ? 1 : 0;

		igsioVideoFrame& video = igsioVideoFrame();
		if (detection_happened) {

			char name[100];
			if (images_count < 10) {
				snprintf(name, sizeof name, "C:/IGSIO-master/IGSIO-master/data/Kidney_10_10/Kidney000%d.bmp", images_count);
			}
			else if (images_count < 100) {
				snprintf(name, sizeof name, "C:/IGSIO-master/IGSIO-master/data/Kidney_10_10/Kidney00%d.bmp", images_count);

			}
			else if (images_count < 1000) {
				snprintf(name, sizeof name, "C:/IGSIO-master/IGSIO-master/data/Kidney_10_10/Kidney0%d.bmp", images_count);


			}
			images_count++;


			//   IGSIO frame list //

			igsioTrackedFrame trackedframe = igsioTrackedFrame();
			Mat mm, R;
			Rodrigues(rvec, R);


			//-------------------------------------------------------------------------------------------------------------//
			//-----------------------------------------------TUMOR  SEGMENTATION-------------------------------------------//
			//----------------------------------------------------------------------------------------------------------- //

			Mat gray;
			if (detectTumor) {

				Rect Rec(1150, 280, 230, 210);
				Rect Rec2(1033, 314, 210, 160);

				Mat image1 = InImage_Ultra;// = imread("C:/IGSIO-master/IGSIO-master/build/Debug/Kidney_17_8/Kidney0024.bmp");
				Mat image = image1(Rec2);

				cvtColor(image, gray, cv::COLOR_BGR2GRAY);
				InImage_Ultra = gray;
				imshow("gray", image);

				Mat dst1, dst2, dst3, dst4;

				// medfilt2
				for (int i = 1; i < 30; i = i + 2)
				{
					medianBlur(gray, dst, i);

				}

				char* equalized_window = "Equalized Image";

				// imsharpen
				Mat blurred; double sigma = 20, threshold_ = 5, amount = 2;
				GaussianBlur(dst, blurred, Size(), sigma, sigma);
				Mat lowContrastMask = abs(dst - blurred) < threshold_;
				Mat sharpened = dst*(1 + amount) + blurred*(-amount);
				dst.copyTo(sharpened, lowContrastMask);

				// A_sharp - A3
				dst3 = sharpened - dst;
				//imshow("difference", dst3);


				cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
				threshold(dst3, dst4, 0, 255, cv::THRESH_BINARY);

				imshow("binary", dst4);

				Mat dst5;

				dst5 = dst4;
				// Set up the detector with default parameters.
				SimpleBlobDetector::Params params;
				params.blobColor = 255;

				// Filter by Area.
				params.filterByArea = true;
				params.minArea = 2000;
				params.maxArea = 100000;

				// Filter by Circularity
				params.filterByCircularity = true;
				params.minCircularity = 0.1;

				// Filter by Convexity
				params.filterByConvexity = true;
				params.minConvexity = 0.15;

				// Filter by Inertia
				params.filterByInertia = true;
				params.minInertiaRatio = 0.01;
				// Detect blobs.
				std::vector<KeyPoint> keypoints;
				std::vector<KeyPoint> interest_keypoints;


				cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
				detector->detect(dst4, keypoints);

				// Draw detected blobs as red circles.
				// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
				Mat im_with_keypoints = gray;

				if (keypoints.size() > 0) {
					if (keypoints.size() > 0 && first_key == 0) {
						first_keypoints = keypoints;
						first_key = 1;
					}

					if (keypoints.size() > 0) {
						for (int i = 0; i < keypoints.size(); i++) {


							if ((keypoints.at(i).pt.x < 220) && (keypoints.at(i).pt.x > 10) && (keypoints.at(i).pt.y < 110) && (keypoints.at(i).pt.y > 0))
								interest_keypoints.push_back(keypoints.at(i));

						}
					}

					if (interest_keypoints.size() > 0) {
						drawKeypoints(gray, interest_keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

					}
				}

				else {

					params.blobColor = 0;

					cv::Ptr<cv::SimpleBlobDetector> detector2 = cv::SimpleBlobDetector::create(params);
					detector2->detect(dst4, keypoints);

					if (keypoints.size() > 0) {
						for (int i = 0; i < keypoints.size(); i++) {


							if ((keypoints.at(i).pt.x < 220) && (keypoints.at(i).pt.x > 10) && (keypoints.at(i).pt.y < 110) && (keypoints.at(i).pt.y > 0))
								interest_keypoints.push_back(keypoints.at(i));

						}
					}
					if (interest_keypoints.size() > 0) {
						drawKeypoints(gray, interest_keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


					}


				}
				imshow("blobs", im_with_keypoints);



				vector<vector<Point> > contours2;

				if (interest_keypoints.size() > 0) {

					/// Detect edges using canny
					Canny(dst5, canny_output, 20, 20 * 2, 7);
					/// Find contours
					findContours(canny_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, Point(0, 0));

					Mat testdraw = Mat::zeros(canny_output.size(), CV_8UC3);
					for (int i = 0; i < contours.size(); i++)
					{
						Scalar color = Scalar(224, 224, 224);
						// contour
						drawContours(testdraw, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point());

					}
					imshow("all contours", testdraw);

					/// Draw contours
					Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
					Mat labels;

					std::vector<Point2f> centers;
					point_count = 0;
					Mat points = cv::Mat(contours.size(), 2, CV_32F);

					for (int i = 0; i < contours.size(); i++)
					{
						if (contours[i].at(0).x <interest_keypoints.at(0).pt.x + 90 && contours[i].at(0).x >interest_keypoints.at(0).pt.x - 90 && contours[i].at(0).y <+interest_keypoints.at(0).pt.y + 90 && contours[i].at(0).y >interest_keypoints.at(0).pt.y - 90)
						{

							points.at<float>(point_count, 0) = contours[i].at(0).x;
							points.at<float>(point_count, 1) = contours[i].at(0).y;

							point_count++;
						}
					}


					/// Find the rotated rectangles and ellipses for each contour

					vector<RotatedRect> minRect(contours.size());
					vector<RotatedRect> minEllipse(contours.size());
					RotatedRect tryEllipse;

					for (int i = 0; i < contours.size(); i++)
					{
						minRect[i] = minAreaRect(Mat(contours[i]));
						if (minRect[i].center.x <interest_keypoints.at(0).pt.x + 100 && minRect[i].center.x >interest_keypoints.at(0).pt.x - 30 && minRect[i].center.y <interest_keypoints.at(0).pt.y + 100 && minRect[i].center.y >interest_keypoints.at(0).pt.y - 20) {
							if (contours[i].size() > 5)
							{
								tryEllipse = fitEllipse(Mat(contours[i]));
								if (tryEllipse.center.x <interest_keypoints.at(0).pt.x + 100 && tryEllipse.center.x >interest_keypoints.at(0).pt.x - 30 && tryEllipse.center.y <interest_keypoints.at(0).pt.y + 100 && tryEllipse.center.y >interest_keypoints.at(0).pt.y - 20) {
									minEllipse[i] = tryEllipse;

								}
							}
						}
					}
					vector<vector<Point>>  best_contours(contours.size());
					RotatedRect bestEllipse;
					float max_area = 0;

					for (int i = 0; i < contours.size(); i++) {

						//Size2f wh = minEllipse[i].size;
						if (!(minEllipse[i].size.empty())) {
							if (minEllipse[i].size.area() > max_area) {

								max_area = minEllipse[i].size.area();
								best_contours[0] = contours[i];
								bestEllipse = minEllipse[i];
							}
						}
					}
					/// Draw contours + rotated rects + ellipses
					for (int i = 0; i < contours.size(); i++)
					{
						Scalar color = Scalar(224, 224, 224);
						// contour
						//drawContours(drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point());
						// ellipse
						ellipse(drawing, bestEllipse, color, 2, 8);
					}


					//imshow("contours", drawing);
					Mat gray_draw;
					cv::cvtColor(drawing, gray_draw, cv::COLOR_BGR2GRAY);

					Mat contrast;
					bitwise_or(gray, gray_draw, contrast);
					//imshow("contr", contrast);

					// Contours in ellipse
					vector<vector<Point> > contours3;
					Mat canny_output2;
					Mat contrast2;
					threshold(contrast, contrast2, 200, 255, cv::THRESH_BINARY);

					/// Detect edges using canny
					Canny(contrast2, canny_output2, 20, 20 * 2, 7);

					/// Find contours
					findContours(canny_output2, contours3, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, Point(0, 0));

					/// Draw contours

					drawing2 = Mat::zeros(canny_output2.size(), CV_8UC3);

					vector<vector<Point> >hull(contours3.size());
					for (size_t i = 0; i < contours3.size(); i++)
					{
						convexHull(contours3[i], hull[i]);
					}
					cout << contours3.size();
					for (size_t i = 0; i < contours3.size(); i++)
					{
						Scalar color = Scalar(192 + 10 * i, 192, 192);

						//drawContours(drawing2, contours3, (int)i, color);
						//drawContours(drawing2, hull, (int)1, color2,2,8,hierarchy,0,Point());

					}

					vector<Point> half_hull0;

					for (int i = (hull[0].size()) / 2; i < hull[0].size(); i++) {

						half_hull0.push_back(hull[0][i]);

					}
					for (int i = 0 / 2; i < hull[0].size() / 2.; i++) {

						half_hull0.push_back(hull[0][i]);

					}
					for (int i = 0; i < half_hull0.size(); i = i + 1) {

						circle(drawing2, half_hull0[i], 2, Scalar(0, 0, 255), FILLED, LINE_AA);



					}
					circle(drawing2, Point(0, 0), 2, Scalar(0, 0, 255), FILLED, LINE_AA);
					int n = half_hull0.size();

					Mat final_draw;
					//cv::cvtColor(contrast, final_draw, cv::COLOR_GRAY2RGB);
					if (frames_counter > 250 && frames_counter < 330) {
						for (int i = 0; i < 160; i++) {

							for (int j = 0; j < 210; j++) {
								if (isInside(half_hull0, n - 1, Point(j, i)))//? cout << "Yes \n" : cout << "No \n";
								{
									contrast.at<uchar>(i, j) = 224;
								}
								else {
									contrast.at<uchar>(i, j) = 0;

								}

							}
						}
					}
					//isInside(half_hull0, n-1, Point(0, 0)) ? cout << "Yes \n" : cout << "No \n";
					imshow("contr", contrast);

					InImage_Ultra = contrast;



					//----------------------------------------------------------------------------------------------------------------------------------//
					//-----------------------------------------------------------End of Image Segmentation---------------------------------------------//
					//--------------------------------------------------------------------------------------------------------------------------------//

				}
				
				//-------------------- Kidney -----------------------------------//
				if (true) {
					// Until sseptember
					///Rect rec4(787, 265, 900, 380);

					//After September
					///Rect rec4(787, 265, 900, 500);

					// October
					Rect rec4(688, 114, 1134 - 688, 838 - 114);


					Mat image_surf = image1(rec4);
					Mat bin_kidney, gray_kidney;
					cvtColor(image_surf, gray_kidney, cv::COLOR_BGR2GRAY);
					threshold(gray_kidney, bin_kidney, 150, 255, cv::THRESH_BINARY);
					//imshow("surface", bin_kidney);
					Mat another_kidney;
					cvtColor(bin_kidney, another_kidney, cv::COLOR_GRAY2RGB);
					Mat final_kidney = another_kidney;


					//----- Put tumor in a larger image ------//
					/// October 
					Mat black = Mat::zeros(786, 902, CV_8UC3);

					///Mat black = Mat::zeros(734, 1176, CV_8UC3);


					Mat another;
					Mat grayan;
					//cvtColor(InImage_Ultra, grayan, cv::COLOR_BGR2GRAY);
					grayan = InImage_Ultra;
					cvtColor(grayan, another, cv::COLOR_GRAY2RGB);


					Mat final_im = another;
					// Kidney
					///final_kidney.copyTo(black(cv::Rect(787 - 614, 265 - 72, 900, 500)));

					// October 
					final_kidney.copyTo(black(cv::Rect(688 - 656, 114 - 102, 1134 - 688, 838 - 114)));


					// Tumor - Rect Rec2(1033, 314, 210, 160);


					if (interest_keypoints.size() > 0 && frames_counter > 250 && frames_counter < 330) {

						///final_im.copyTo(black(cv::Rect(1120 - 614, 350 - 72, 200, 155)));
						final_im.copyTo(black(cv::Rect(1033 - 656, 314 - 102, 210, 160)));

					}

					cv::rotate(black, black, cv::ROTATE_90_CLOCKWISE);
					cv::rotate(black, black, cv::ROTATE_90_CLOCKWISE);
					cv::rotate(black, black, cv::ROTATE_90_CLOCKWISE);

					imshow("Only Ultra", black);
					Mat final_result;
					resize(black, final_result, Size(), 0.088, 0.094);

					imwrite(name, final_result);

				}
			}

			std::string tring = std::string(name);

			const char* fileName = tring.c_str();
			std::cout << fileName;
			igsioStatus stat = igsioVideoFrame::ReadImageFromFile(video, fileName);
			trackedframe.SetImageData(video);


			///Matrices
			vtkMatrix4x4 *matrix4_Probe_Ref = vtkMatrix4x4::New();


			if (visual) {
				for (int k = 0; k < 3; k++) {
					for (int j = 0; j < 3; j++) {
						matrix4_Probe_Ref->SetElement(k, j, *R.ptr<double>(k, j));
					}
				}
				matrix4_Probe_Ref->SetElement(0, 3, -1000.*tvec[0]);
				matrix4_Probe_Ref->SetElement(1, 3, -1000.*tvec[1]);
				matrix4_Probe_Ref->SetElement(2, 3, -1000.*tvec[2]);
				matrix4_Probe_Ref->SetElement(3, 3, 1.);

				matrix4_Probe_Ref->SetElement(3, 0, 0.);
				matrix4_Probe_Ref->SetElement(3, 1, 0.);
				matrix4_Probe_Ref->SetElement(3, 2, 0.);
			}
			else {

				Mat rotation = Mat(3, 3, CV_64FC1);
				rotation.at<double>(0, 0) = inputTranformations.at<double>(frames_counter, 0);
				rotation.at<double>(0, 1) = inputTranformations.at<double>(frames_counter, 1);
				rotation.at<double>(0, 2) = inputTranformations.at<double>(frames_counter, 2);
				rotation.at<double>(1, 0) = inputTranformations.at<double>(frames_counter, 4);
				rotation.at<double>(1, 1) = inputTranformations.at<double>(frames_counter, 5);
				rotation.at<double>(1, 2) = inputTranformations.at<double>(frames_counter, 6);
				rotation.at<double>(2, 0) = inputTranformations.at<double>(frames_counter, 8);
				rotation.at<double>(2, 1) = inputTranformations.at<double>(frames_counter, 9);
				rotation.at<double>(2, 2) = inputTranformations.at<double>(frames_counter, 10);

				int transf_counter = 0;
				for (int k = 0; k < 3; k++) {
					for (int j = 0; j < 4; j++) {

						if (k == 0 && j == 3) {

							matrix4_Probe_Ref->SetElement(k, j, 1000.*inputTranformations.at<double>(frames_counter, 7));
							cout << inputTranformations.at<double>(frames_counter, 3);

						}
						else if (k == 1 && j == 3) {
							matrix4_Probe_Ref->SetElement(k, j, 1000.*inputTranformations.at<double>(frames_counter, 11));
							cout << inputTranformations.at<double>(frames_counter, 11);

						}

						else if (k == 2 && j == 3) {

							double z_prev = inputTranformations.at<double>(frames_counter, 7);
							matrix4_Probe_Ref->SetElement(k, j, 1000.*inputTranformations.at<double>(frames_counter, 3));
							cout << z_prev;

						}
						else
						{

							// wrong : matrix4_Probe_Ref->SetElement(k, j, rotation_inverse.at<double>(k,j));
							matrix4_Probe_Ref->SetElement(k, j, inputTranformations.at<double>(frames_counter, transf_counter));

						}
						transf_counter++;

					}
				}


				matrix4_Probe_Ref->SetElement(3, 0, 0.);
				matrix4_Probe_Ref->SetElement(3, 1, 0.);
				matrix4_Probe_Ref->SetElement(3, 2, 0.);
				matrix4_Probe_Ref->SetElement(3, 3, 1.);

				vtkTransform *transf = vtkTransform::New();
				transf->RotateX(90);


			}

			const igsioTransformName  nameProbRef = igsioTransformName(std::string("Probe"), std::string("Reference"));

			trackedframe.SetFrameTransform(nameProbRef, matrix4_Probe_Ref);
			trackedframe.SetFrameField("Timestamp", to_string(timestamp));
			trackedframe.SetTimestamp(timestamp);
			trackedframe.SetFrameTransformStatus(nameProbRef, ToolStatus::TOOL_OK);
			trackedFrameList->AddTrackedFrame(&trackedframe);
			timestamp = timestamp + 0.00008;
			frames_counter++;

			detection_happened = false;
		}
		/////////// End of frames ////////////////////////////////

	} while (frames_counter < used_frames && key != 27 && videoL.grab() && videoR.grab() && vreader_Ultra.grab());  // wait for esc to be pressed

	

	//////////// Start of reconstruction algorithm /////////////////////

	igsioTransformName imageToReferenceTransformName;
	if (!inputImageToReferenceTransformName.empty())
	{
		// image to reference transform is specified at the command-line
		if (imageToReferenceTransformName.SetTransformName(inputImageToReferenceTransformName.c_str()) != IGSIO_SUCCESS)
		{
			LOG_ERROR("Invalid image to reference transform name: " << inputImageToReferenceTransformName);
			//return EXIT_FAILURE;
		}
		reconstructor->SetImageCoordinateFrame(imageToReferenceTransformName.From());
		reconstructor->SetReferenceCoordinateFrame(imageToReferenceTransformName.To());

	}

	const int numberOfFrames = trackedFrameList->GetNumberOfTrackedFrames();
	int numberOfFramesAddedToVolume = 0;


	LOG_INFO("Set volume output extent...");
	std::string errorDetail;

		
	if (reconstructor->SetOutputExtentFromFrameList(trackedFrameList, transformRepository, errorDetail) != IGSIO_SUCCESS)
	{
		LOG_ERROR("Failed to set output extent of volume!");
		cout << errorDetail;
		//return EXIT_FAILURE;
	}

	LOG_INFO("Reconstruct volume...");
		

	for (int frameIndex = 0; frameIndex < numberOfFrames; frameIndex += reconstructor->GetSkipInterval())
	{
		LOG_DEBUG("Frame: " << frameIndex);
		vtkIGSIOLogger::PrintProgressbar((100.0 * frameIndex) / numberOfFrames);

		igsioTrackedFrame* frame = trackedFrameList->GetTrackedFrame(frameIndex);
			
			

		if (transformRepository->SetTransforms(*frame) != IGSIO_SUCCESS)
		{
			LOG_ERROR("Failed to update transform repository with frame #" << frameIndex);
			continue;
		}


		// Insert slice for reconstruction
		bool insertedIntoVolume = false;
		if (reconstructor->AddTrackedFrame(frame, transformRepository, &insertedIntoVolume) != IGSIO_SUCCESS)
		{
			LOG_ERROR("Failed to add tracked frame to volume with frame #" << frameIndex);
			continue;
		}


		if (insertedIntoVolume)
		{
			numberOfFramesAddedToVolume++;
		}

			
		// Write an ITK image with the image pose in the reference coordinate system
		if (!outputFrameFileName.empty())
		{
			vtkSmartPointer<vtkMatrix4x4> imageToReferenceTransformMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
			if (transformRepository->GetTransform(imageToReferenceTransformName, imageToReferenceTransformMatrix) != IGSIO_SUCCESS)
			{
				std::string strImageToReferenceTransformName;
				imageToReferenceTransformName.GetTransformName(strImageToReferenceTransformName);
				LOG_ERROR("Failed to get transform '" << strImageToReferenceTransformName << "' from transform repository!");
				continue;
			}

			// Print the image to reference transform
			std::ostringstream os;
			imageToReferenceTransformMatrix->Print(os);
			cout << "print" << *imageToReferenceTransformMatrix;
			LOG_TRACE("Image to reference transform: \n" << os.str());
				
			// Insert frame index before the file extension (image.mha => image001.mha)
			std::ostringstream ss;
			size_t found;
			found = outputFrameFileName.find_last_of(".");
			ss << outputFrameFileName.substr(0, found);
			ss.width(3);
			ss.fill('0');
			ss << frameIndex;
			ss << outputFrameFileName.substr(found);
				
			//igsioCommon::WriteToFile(frame, ss.str(), imageToReferenceTransformMatrix);
		}
	}

	vtkIGSIOLogger::PrintProgressbar(100);

	trackedFrameList->Clear();

	LOG_INFO("Number of frames added to the volume: " << numberOfFramesAddedToVolume << " out of " << numberOfFrames);

	LOG_INFO("Saving volume to file...");
	reconstructor->SaveReconstructedVolumeToFile(outputVolumeFileName, false, !disableCompression);


	///////////// END OF RECONSTRUCTION ////////////////////////////////


	return EXIT_SUCCESS;	
}



