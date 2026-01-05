#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>

#include "Kinect.h"

#include "stdlib.h"
#include "GL\glut.h"

#include "OpiraLibraryMT.h"
#include "OpiraLibrary.h"
#include "CaptureLibrary.h"
#include "RegistrationAlgorithms\OCVSurf.h"
#include "RegistrationAlgorithms\SIFT.h"

#include "global.h"
#include "spider.h"

#include "Renderer.h"

using namespace OPIRALibrary;

bool running = true;
bool bRegKinect = false;

Spider *spider;
KinectAR *kinect;

void main(int argc, char **argv) {
	_CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	_CrtSetReportMode ( _CRT_ERROR, _CRTDBG_MODE_DEBUG);
//	_CrtSetBreakAlloc(20226);

	//Initialise our Camera
	Capture* camera = new Camera(0,cvSize(640,480), "Data/camera.yml");
	((Camera*)camera)->setAutoWhiteBalance(false);

	//Initialise the Kinect
	kinect = new KinectAR("Data/SamplesConfig.xml", "Data/kinect.yml");

	//Initialise the Registration Class
	Registration *regAR = new RegistrationOPIRAMT(new OCVSurf()); 
	Registration *regKinect = new RegistrationOPIRAMT(new OCVSurf()); regKinect->addResizedMarker("media/celica.bmp", 400);

	//Initialise the Spider
	spider = new Spider("media/spider01.ive", "media/animations.xml");

	//Initialise the OpenSceneGraph Renderer
	Renderer *renderer = new Renderer(640, 480, calcProjection(camera->getParameters(), camera->getDistortion(), cvSize(640,480)));
	renderer->addModel("media/celica.bmp", spider->getModel());

	
	while (running) {
		//Grab a frame from the AR Camera
		IplImage *new_frame = camera->getFrame();

		//Grab a frame from the Kinect
		kinect->getNewFrame();
		IplImage *kinectColour = kinect->getColour();
		IplImage *kinectDepth = kinect->getDepth();
		IplImage *kinectDepthMask = kinect->getDepthMask();

		if (bRegKinect) {
			vector<MarkerTransform> mt = regKinect->performRegistration(kinectColour, kinect->getParameters(), kinect->getDistortion());
			if (mt.size()>0) kinect->calculateTransform(mt.at(0).marker.size, mt.at(0).homography);
			for (int i=0; i<mt.size(); i++) {mt.at(i).clear();} mt.clear(); 

			regAR->removeMarker("media/celica.bmp");
			regAR->addResizedScaledMarker("media/celica.bmp", 400, kinect->getRealMarkerSize().width);
			printf("load: %d\t %d\n", kinect->getRealMarkerSize().width, kinect->getRealMarkerSize().height);

			bRegKinect = false;
		}

		double minV, maxV; CvPoint minL, maxL;
		cvMinMaxLoc(kinectDepth, &minV, &maxV, &minL, &maxL, kinectDepthMask);
		//printf("%f - %f\n", minV, maxV);
		double scale = 255.0/float(maxV-minV), shift = minV*scale;
		IplImage *depthIm8 = cvCreateImage(cvGetSize(kinectDepth), 8, 1); IplImage *depthIm83 = cvCreateImage(cvGetSize(kinectDepth), 8, 3);
		
		cvConvertScale(kinectDepth, depthIm8, scale, -shift); cvMerge(depthIm8, depthIm8, depthIm8, 0, depthIm83);
		cvCircle(depthIm83, minL, 3, cvScalar(255,0,0), 2); cvCircle(depthIm83, maxL, 3, cvScalar(0,0,255), 2);
		CvPoint3D32f p = kinect->getTransformedPoint(minL); p.y = -p.y; osg::Vec3 sP = spider->getPosition();
		if (!spider->isAnimating()) {
			float dist = sqrt((p.x-sP.x())*(p.x-sP.x())+(p.y-sP.y())*(p.y-sP.y()));
			//printf("D: %f\n", dist);
			if (dist>50) spider->moveTo(p.x, p.y, 0);
		}
		//printf("%.2f, %.2f, %.2f\t%.2f, %.2f, %.2f\n", p.x, p.y, p.z, sP.x(), sP.y(), sP.z());
		cvShowImage("col", kinectColour); cvShowImage("depth", depthIm83); cvShowImage("depthMask", kinectDepthMask);
		cvReleaseImage(&depthIm8); cvReleaseImage(&depthIm83);

		if (new_frame!=0) {
			vector<MarkerTransform> mt = regAR->performRegistration(new_frame, camera->getParameters(), camera->getDistortion());

			/*if (kinect->getTransform()!=0) {
				CvPoint *p = (CvPoint *)malloc(640*480*sizeof(CvPoint));
				for (int y=0; y<480; y++) {
					for (int x=0; x<640; x++) {
						p[x+(y*640)].x = x; p[x+(y*640)].y = y;
					}
				}
				CvPoint3D32f *ground_grid = kinect->getTransformedPoints(p,640*480);
				renderer->updateHeightMap(ground_grid);
				free(p); free(ground_grid);
			}*/


			renderer->render(new_frame, mt);

			for (int i=0; i<mt.size(); i++) {mt.at(i).clear();} mt.clear();

			//Check for the escape key and give the computer some processing time
			checkKeyPress (cvWaitKey(1));
		
		}

		//Clean up
		cvReleaseImage(&new_frame);
		cvReleaseImage(&kinectColour);
		cvReleaseImage(&kinectDepth);
		cvReleaseImage(&kinectDepthMask);

	};

	delete renderer;
	delete spider;
	delete regAR; delete regKinect;
	delete camera; delete kinect;
}

void checkKeyPress(int key) {
	switch (key) {
		/*case 'q':
			spider->setPosition(0, 0, 0); break;
		case 'w':
			spider->setPosition(kinect->getRealMarkerSize().width, 0, 0); break;
		case 'e':
			spider->setPosition(kinect->getRealMarkerSize().width, -kinect->getRealMarkerSize().height, 0); break;
		case 'r':
			spider->setPosition(0, -kinect->getRealMarkerSize().height, 0); break;*/
		case 'q':
			spider->moveTo(0, 0, 0); break;
		case 'w':
			spider->moveTo(kinect->getRealMarkerSize().width, 0, 0); break;
		case 'e':
			spider->moveTo(kinect->getRealMarkerSize().width, -kinect->getRealMarkerSize().height, 0); break;
		case 'r':
			spider->moveTo(0, -kinect->getRealMarkerSize().height, 0); break;
		case 27:
			running = false; break;
			break;
		case ' ':
			bRegKinect = true; break;
		case '1':
			spider->setAnimation(1); break;
		case '2':
			spider->setAnimation(2); break;
		case '3':
			spider->setAnimation(3); break;
		case '4':
			spider->setAnimation(4); break;
		case '5':
			spider->setAnimation(5); break;
		case '6':
			spider->setAnimation(6); break;
		case '7':
			spider->setAnimation(7); break;
		case '8':
			spider->setAnimation(8); break;
		case '9':
			spider->setAnimation(9); break;
	}
}

float getSpiderHeight() {
	osg::Vec3 sP = spider->getPosition();
	CvPoint3D32f p = kinect->getInverseTransformedPoint(cvPoint3D32f(sP.x(), -sP.y(), sP.z()));
	return kinect->getTransformedPoint(cvPoint(p.x, p.y)).z;
}