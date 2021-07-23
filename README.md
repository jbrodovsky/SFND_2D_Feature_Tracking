# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
1. cmake >= 2.8
 * All OSes: [click here for installation instructions](https://cmake.org/install/)

2. make >= 4.1 (Linux, Mac), 3.81 (Windows)
 * Linux: make is installed by default on most Linux distros
 * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
 * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)

3. OpenCV >= 4.1 with additional modules 
 * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
 * Make sure to also install [opencv_contrib](https://github.com/opencv/opencv_contrib). Clone this repo into the opencv directory as well.
 * General install, all OSes: refer to the [official instructions](https://docs.opencv.org/master/df/d65/tutorial_table_of_content_introduction.html)
 * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors. If using [homebrew](https://brew.sh/): `$> brew install --build-from-source opencv` will install required dependencies and compile opencv with the `opencv_contrib` module by default (no need to set `-DOPENCV_ENABLE_NONFREE=ON` manually).

4. gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using either [MinGW-w64](http://mingw-w64.org/doku.php/start) or [Microsoft's VCPKG, a C++ package manager](https://docs.microsoft.com/en-us/cpp/build/install-vcpkg?view=msvc-160&tabs=windows). VCPKG maintains its own binary distributions of OpenCV and many other packages. To see what packages are available, type `vcpkg search` at the command prompt. For example, once you've _VCPKG_ installed, you can install _OpenCV 4.1_ with the command:
```bash
c:\vcpkg> vcpkg install opencv4[nonfree,contrib]:x64-windows
```
Then, add *C:\vcpkg\installed\x64-windows\bin* and *C:\vcpkg\installed\x64-windows\debug\bin* to your user's _PATH_ variable. Also, set the _CMake Toolchain File_ to *c:\vcpkg\scripts\buildsystems\vcpkg.cmake*.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

## Project Completion Overview

### MP.0 Provide a Writeup / README

Here is how I completed the project and satisfied the rubric points listed below.

### MP.1 Data Buffer Optimization

TASK: Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This can be achieved by pushing in new elements on one end and removing elements on the other end. 

I  added an `if` statement to check if the vector is over the size specified by `dataBufferSize` and removed the first item if true.

```
if(dataBuffer.size() > dataBufferSize)
{
    dataBuffer.erase(dataBuffer.begin());
}
```

### MP.2 Keypoint Detection

TASK: Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.

I implemented the Harris detector using the function `detKeypointsHarris()` in `matching2D_Student.cpp`. The remaining methods were implemented using built in OpenCV methods. The method `detKeypointsModern()` identifies the detector type and calls the corresponding OpenCV function. I also implemented the visualization in a seperate function `visualizer()` that handles the keypoint visualization for all the detection methods. 

### MP.3 Keypoint Removal
	
TASK: Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing. 

I achieved this by doing the following:

```
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
```

### MP.4 Keypoint Descriptors
	
TASK: Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.

I implemented this by using a conditional structure to match the ORB, AKAZE, and SIFT `detectorType` and `descriptorType`. This ensured that those methods worked properly. The `else` case can then be used to toggle between different keypoint descriptor methods.

```
if (detectorType.compare("ORB") == 0) { descriptorType = "ORB"; }
else if (detectorType.compare("AKAZE") == 0) { descriptorType = "AKAZE"; }
else if (detectorType.compare("SIFT") == 0) { descriptorType = "SIFT"; }
else 
{ 
    descriptorType = "BRIEF";  // Specify desired descriptor type here.
}
descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
```

### MP.5 Descriptor Matching
	
TASK: Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.

I started off by modifying `main()` to check to see if SIFT was being used as it requires a specific configuration. Again, the `else` block is used for customization.

```
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
```
I then added my implementations for MAT_FLANN and SEL_KNN in the `matchDescriptors` method.


### MP.6 Descriptor Distance Ratio

TASK: Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.

Building off of the previous task I added a loop that examines the matches' distance attribute. If it is less than 4/5ths the second best match, it keeps the first match.

```
for(int i=0; i<kPtsSource.size(); i++)
{
    if(knn_matches[i][0].distance < 0.8 * knn_matches[i][1].distance) { matches.push_back(knn_matches[i][0]); }
}
```

### Performance Evaluation
	
TASK 7: Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.
	
TASK 8: Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.

TASK 9: Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.

| Detector Type | Descriptor type | Image 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | Average |
| ------------- | --------------- | ------- | - | - | - | - | - | - | - | - | ------- |
|SHITOMASI| BRIEF| 125| 118| 123| 120| 120| 113| 114| 123| 111| 118.556
| | Times | 0.0123725| 0.0113318| 0.0107004| 0.0132807| 0.010495| 0.0110437| 0.0125044| 0.0112243| 0.0100679| 0.0114467|
|SHITOMASI| ORB| 125| 118| 123| 120| 120| 113| 114| 123| 111| 118.556
| | Times | 0.016903| 0.0124959| 0.0133369| 0.0148487| 0.0140202| 0.0124388| 0.0141622| 0.0150134| 0.0154723| 0.014299| 
|SHITOMASI| FREAK| 125| 118| 123| 120| 120| 113| 114| 123| 111| 118.556
| | Times | 0.0326576| 0.0354248| 0.0353961| 0.0334334| 0.0328565| 0.0354525| 0.0347177| 0.0342466| 0.0331409| 0.0341473| 
|HARRIS| BRIEF| 26940| 26937| 26905| 26981| 26951| 26982| 26933| 26963| 26913| 26945
| | Times | 1.97848| 1.95777| 2.00661| 1.92154| 1.9059| 1.9496| 1.95767| 1.93302| 1.93293| 1.94928| 
|HARRIS| ORB| 26940| 26937| 26905| 26981| 26951| 26982| 26933| 26963| 26913| 26945
| | Times | 1.90082| 1.91072| 1.90092| 1.96184| 1.894| 1.89569| 1.89442| 1.89654| 1.89605| 1.90567| 
|HARRIS| FREAK| 26940| 26937| 26905| 26981| 26951| 26982| 26933| 26963| 26913| 26945
| | Times | 2.23806| 2.23446| 2.23118| 2.24438| 2.27438| 2.17861| 2.23684| 2.25727| 2.30438| 2.2444| 
|AKAZE| AKAZE| 166| 157| 161| 155| 163| 164| 173| 175| 177| 165.667
| | Times | 0.0769876| 0.0828779| 0.0779143| 0.0779919| 0.0784943| 0.0798918| 0.0830369| 0.0832762| 0.0801428| 0.0800682| 
|AKAZE| BRIEF| 166| 157| 161| 155| 163| 164| 173| 175| 177| 165.667
| | Times | 0.0432075| 0.0450152| 0.0449155| 0.048369| 0.0466762| 0.0471293| 0.0471836| 0.0450072| 0.0458929| 0.0459329| 
|AKAZE| ORB| 166| 157| 161| 155| 163| 164| 173| 175| 177| 165.667
| | Times | 0.0479824| 0.0479375| 0.0535408| 0.0519086| 0.0529066| 0.0517863| 0.0539532| 0.0531916| 0.0514566| 0.0516293| 
|AKAZE| SIFT| 135| 136| 130| 138| 137| 147| 148| 155| 151| 141.889
| | Times | 0.0599864| 0.0592253| 0.060945| 0.0564897| 0.059768| 0.0598705| 0.0641947| 0.0598099| 0.0648443| 0.0605704| 
|BRISK| BRIEF| 264| 282| 282| 277| 297| 279| 289| 272| 266| 278.667
| | Times | 0.0568112| 0.0550632| 0.0546297| 0.0537886| 0.0531055| 0.0545869| 0.0550129| 0.0572316| 0.0552323| 0.0550513| 
|BRISK| ORB| 264| 282| 282| 277| 297| 279| 289| 272| 266| 278.667
| | Times | 0.0652198| 0.0624177| 0.0618589| 0.0628291| 0.0619113| 0.0624922| 0.0643508| 0.0626101| 0.0617048| 0.0628216| 
|BRISK| FREAK| 242| 260| 263| 264| 274| 256| 269| 255| 243| 258.444
| | Times | 0.0765743| 0.0751977| 0.0735587| 0.0733864| 0.0723028| 0.0736373| 0.0717674| 0.0715283| 0.0718941| 0.0733164| 
|BRISK| SIFT| 184| 193| 171| 183| 173| 196| 196| 178| 185| 184.333
| | Times | 0.0746276| 0.0739031| 0.0740902| 0.0745083| 0.0735302| 0.0743294| 0.0735304| 0.0723179| 0.0732958| 0.0737925| 
|FAST| BRIEF| 419| 427| 404| 423| 386| 414| 418| 406| 396| 410.333
| | Times | 0.0049926| 0.00448899| 0.00455268| 0.00396672| 0.00371632| 0.00368948| 0.00375566| 0.00465008| 0.00384307| 0.00418396| 
|FAST| ORB| 419| 427| 404| 423| 386| 414| 418| 406| 396| 410.333
| | Times | 0.0072419| 0.00556669| 0.00585538| 0.00570546| 0.00548022| 0.00561734| 0.00551866| 0.00564504| 0.00571688| 0.0058164| 
|FAST| FREAK| 419| 427| 404| 423| 386| 414| 418| 406| 396| 410.333
| | Times | 0.0272987| 0.0313255| 0.0231055| 0.0228782| 0.0231074| 0.0233509| 0.0232877| 0.0231671| 0.0232901| 0.0245346| 
|FAST| SIFT| 317| 326| 299| 311| 291| 326| 318| 301| 301| 310
| | Times | 0.0244604| 0.0231377| 0.0232635| 0.0216298| 0.0220403| 0.022002| 0.023519| 0.0213257| 0.0221476| 0.022614| 
|ORB| BRIEF| 92| 102| 106| 113| 109| 125| 130| 129| 127| 114.778
| | Times | 0.00731457| 0.00687815| 0.0070304| 0.00724352| 0.00843171| 0.00756791| 0.0072045| 0.00685043| 0.00671911| 0.00724892| 
|ORB| ORB| 92| 102| 106| 113| 109| 125| 130| 129| 127| 114.778
| | Times | 0.0156606| 0.0157332| 0.0148332| 0.0150834| 0.0148205| 0.0161535| 0.0157323| 0.0184809| 0.0153662| 0.0157626| 
|ORB| FREAK| 46| 53| 56| 65| 55| 64| 66| 71| 73| 61
| | Times | 0.0292143| 0.0267922| 0.0259183| 0.0262647| 0.0259339| 0.0258552| 0.0259973| 0.026455| 0.0265804| 0.0265568| 
|ORB| SIFT| 67| 79| 78| 80| 82| 95| 95| 94| 94| 84.8889
| | Times | 0.0277104| 0.0273354| 0.0286746| 0.025961| 0.0272706| 0.0267545| 0.0292484| 0.0302119| 0.0278891| 0.0278951| 
|SIFT| SIFT| 82| 81| 85| 95| 90| 82| 83| 103| 105| 89.5556
|    Times | | 0.103697| 0.101107| 0.111727| 0.11332| 0.119584| 0.10741| 0.109044| 0.113733| 0.108854| 0.109831| 

From this data, I found that the top three methods (the fastest) were:

| Detector | Descriptor | Ave Keypoints | Time (ms) |
| -------- | ---------- | ------------- | --------- |
| FAST | BRIEF | 410.333 | 0.004184 |
| FAST | ORB | 410.333 | 0.005816 |
| ORB | BRIEF | 114.778 | 0.007249 |
