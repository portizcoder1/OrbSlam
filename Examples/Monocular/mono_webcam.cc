/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<string>
#include<filesystem>

#include<opencv2/core/core.hpp>

#include<System.h>

#include<time.h>

#include <corecrt_io.h>

using namespace std;

// From http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

BOOL exists(char* fname)
{
	int fd = open(fname, 0);
	if (fd < 0)         // Error (Mo existe el fichero).
	{
		return false;
	}
	close(fd); // Cerramos el fichero.
	return true;
}

int main(int argc, char **argv)
{
	string vocabPath = "ORBvoc.txt";
	string settingsPath = "webcam.yaml";
	string filePath = "process.txt";
	string KeyFrameTrajectoryPath = "KeyFrameTrajectory.txt";
	int monoWevcamIndex = 0;
	__int64 lastTime = 0;


	if (argc == 1)
	{

	}
	else if (argc == 2)
	{
		vocabPath = argv[1];
	}
	else if (argc == 3)
	{
		vocabPath = argv[1];
		settingsPath = argv[2];
	}
	else if (argc == 4)
	{
		vocabPath = argv[1];
		settingsPath = argv[2];
		filePath = argv[3];
	}
	else if (argc == 5)
	{
		vocabPath = argv[1];
		settingsPath = argv[2];
		filePath = argv[3];
		KeyFrameTrajectoryPath = argv[4];
	}
	else if (argc == 6)
	{
		vocabPath = argv[1];
		settingsPath = argv[2];
		filePath = argv[3];
		KeyFrameTrajectoryPath = argv[4];
		monoWevcamIndex = std::stoi(argv[5]);
	}
    else
    {
        cerr << endl << "Usage: mono_webcam.exe path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

	//cout << endl << "----------------------------------" << endl;
	//CvCapture* capture = cvCaptureFromCAM(0); 
	//CvCapture* capture = cvCaptureFromCAM(CV_CAP_DSHOW);
	//videoInput.listDevices();
	//cout << capture << endl;

	//cv::videoInput VI = new videoInput();


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(vocabPath, settingsPath, ORB_SLAM2::System::MONOCULAR, true);
	//cout << endl << "----------------------------------" << endl;
    cout << "START PROCESSING" << endl;

	// Conetamos la camara.
	/*if (monoWevcamIndex < 0)
	{
		cv::VideoCapture camera;
		int device_number = 0;
		while (true) {
			if (!camera.open(device_number++)) {
				break;
			}
		}
		camera.release();
		cout << endl << "----------------------------------" << endl;
		cout << "Device count : " << device_number - 1 << endl;
		monoWevcamIndex = (device_number - 2);
	}*/

	//cv::VideoCapture cap(0);
	cv::VideoCapture cap(monoWevcamIndex);
	//cap.set(cv::CAP_PROP_FPS, 2);
	//cout << int(cap.get(5));

	// From http://stackoverflow.com/questions/19555121/how-to-get-current-timestamp-in-milliseconds-since-1970-just-the-way-java-gets
	__int64 now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	// Main loop
	cv::Mat im;
	cv::Mat Tcw;
	char *cstr = &filePath[0];
    while (!exists(cstr))
    {
		cap.read(im);

		__int64 curNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		if (curNow < lastTime + 40) continue;
		else lastTime = curNow;

		// Pass the image to the SLAM system
		Tcw = SLAM.TrackMonocular(im, curNow / 1000.0);

		// This can write each image with its position to a file if you want
		/*if (!Tcw.empty())
		{
			cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
			cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);
			std::ostringstream stream;
			//stream << "imgs/" << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2) << " " << twc.at<float>(0) << " " <<
			//	Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " " << twc.at<float>(1) << " " <<
				//Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " " << twc.at<float>(2) << ".jpg";
			stream << "imgs/" << curNow << ".jpg";
			string fileName = stream.str();
			cv::imwrite(fileName, im);
		}*/

		// This will make a third window with the color images, you need to click on this then press any key to quit
		//cv::imshow("Image", im);


		//if (cv::waitKey(1) >= 0)
		//	break;

		//if(exists("process.txt"))
		//{
		//	break;
		//}

    }

    // Stop all threads
    SLAM.Shutdown();
	cap.release();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM(KeyFrameTrajectoryPath);

    return 0;
}
