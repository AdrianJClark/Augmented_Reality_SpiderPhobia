/*#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>

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

#include "TinyXml.h"

using namespace OPIRALibrary;

Renderer *renderer;

bool running = true;

void readXML(char *filename);

Spider *spider;

void main(int argc, char **argv) {
	_CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	_CrtSetReportMode ( _CRT_ERROR, _CRTDBG_MODE_DEBUG);
//	_CrtSetBreakAlloc(20226);

	//Initialise our Camera
	Capture* camera = new Camera(0,cvSize(640,480), "camera.yml");
	((Camera*)camera)->setAutoWhiteBalance(false);

	Registration *r = new RegistrationOPIRAMT(new OCVSurf());
	r->addResizedScaledMarker("media/celica.bmp", 350, 944);

	spider = new Spider("media/spider01.ive", "media/animations.xml");

	renderer = new Renderer(640, 480, calcProjection(camera->getParameters(), camera->getDistortion(), cvSize(640,480)));
	renderer->addModel("media/celica.bmp", spider->getModel());

	
	while (running) {
		//Grab a frame 
		IplImage *new_frame = camera->getFrame();

		if (new_frame!=0) {
			vector<MarkerTransform> mt = r->performRegistration(new_frame, camera->getParameters(), camera->getDistortion());
			
			renderer->render(new_frame, mt);

			for (int i=0; i<mt.size(); i++) {mt.at(i).clear();} mt.clear();

			//Check for the escape key and give the computer some processing time
			checkKeyPress (cvWaitKey(1));
		
			//Clean up
			cvReleaseImage(&new_frame);
		} else running =false;
	};

	//delete renderer;
	delete spider;
	delete r;
	delete camera;
}

void checkKeyPress(int key) {
	switch (key) {
		case 27:
			running = false; break;
			break;
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

*/