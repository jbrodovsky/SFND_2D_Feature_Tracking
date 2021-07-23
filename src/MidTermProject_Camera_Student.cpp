/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

#include <numeric>
#include <fstream>

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */
    std::vector<int> match_count;
    std::vector<double> times;
    string detectorType = "SIFT"; //(SHITOMASI, HARRIS, AKAZE, BRISK, FAST, ORB, SIFT)
    string descriptorType = "AKAZE"; // BRIEF, ORB, FREAK, AKAZE, SIFT
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);
        if(dataBuffer.size() > dataBufferSize)
        {
            dataBuffer.erase(dataBuffer.begin());
        }
        //// EOF STUDENT ASSIGNMENT
        //std::cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        //detectorType = "AKAZE";
        //cout << "Detector type? (SHITOMASI, HARRIS, AKAZE, BRISK, FAST, ORB, SIFT)\n";
        //cin >> detectorType;

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        /*
        Keypoint Detector method signatures
        void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false);
        void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false);
        void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis=false);
        void descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, std::string descriptorType);
        void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                            std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType);
        */
        double t = (double) cv::getTickCount();
        
        if (detectorType.compare("SHITOMASI") == 0) { detKeypointsShiTomasi(keypoints, imgGray, false); }
        else if (detectorType.compare("HARRIS") == 0) { detKeypointsHarris(keypoints, imgGray, false); }
        else { detKeypointsModern(keypoints, imgGray, detectorType, false); }

        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            vector<cv::KeyPoint> filtered_keypoints;
            for(cv::KeyPoint kp : keypoints)
            {
                if (vehicleRect.contains(kp.pt)) { filtered_keypoints.push_back(kp); }
            }
            keypoints = filtered_keypoints;
        }
        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        //std::cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        //string descriptorType; // BRIEF, ORB, FREAK, AKAZE, SIFT

        // FAST, BRISK, ORB, AKAZE, SIFT
        if (detectorType.compare("SIFT") == 0) { descriptorType = "SIFT"; }
        /*
        else 
        { 
            descriptorType = "BRIEF";  // Specify desired descriptor type here.
        }
        */
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        //cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType;        // MAT_BF, MAT_FLANN
            string descriptorType2;    // DES_BINARY, DES_HOG
            string selectorType;       // SEL_NN, SEL_KNN

            if (descriptorType.compare("SIFT") == 0 )
            {
                // SIFT must use these methods
                matcherType = "MAT_FLANN";
                descriptorType2 = "DES_HOG";
                selectorType = "SEL_KNN";
            }
            else
            {
                // Choose prefered methods here.
                matcherType = "MAT_BF";
                descriptorType2 = "DES_BINARY";
                selectorType = "SEL_NN";
            }

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
            
            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType2, matcherType, selectorType);

            //std::cout<<"Marker!\n";
            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;
            t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
            times.push_back(t);
            //cout << "#4 : MATCH KEYPOINT DESCRIPTORS done. Match pair size: "<< matches.size() << endl;
            match_count.push_back(matches.size());
            // visualize matches between current and previous image
            bVis = true;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                //cout << "Press key to continue to next image" << endl;
                //cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }

    } // eof loop over all images
    std::cout<<"---------------------\n";
    float average = 0.0;
    
    //std::cout<<"Using detector type " << detectorType << " and descriptor type " << descriptorType<<"\n:Image counts: ";
    // Write results as a csv .txt file
    ofstream my_stream;
    my_stream.open("./results.txt", ios::app);
    my_stream << detectorType <<", "<<descriptorType<<",";
    for(auto& n : match_count) 
    { 
        my_stream<< n << ", ";
        average+=n; 
    }
    average /= match_count.size();
    my_stream<<average <<"\n";
    my_stream<<" , , ";
    float average_time = 0;
    for(auto&n : times)
    {
        my_stream<<n<<", ";
        average_time+=n;
    }
    average_time /= times.size();
    my_stream<<average_time<<",\n";
    my_stream.close();
    return 0;
}