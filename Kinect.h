#ifndef KINECT
#define KINECT

#include <XnOS.h>
#include <XnCppWrapper.h>

#include <cv.h>
#include <highgui.h>

#include <leastsquaresquat.h>

class KinectAR {
public:
	KinectAR (char *initFile, char *paramsFile) {
		//Initialise the Kinect
		xn::EnumerationErrors errors; 
		switch (XnStatus rc = niContext.InitFromXmlFile(initFile, &errors)) {
			case XN_STATUS_OK:
				break;
			case XN_STATUS_NO_NODE_PRESENT:
				XnChar strError[1024];	errors.ToString(strError, 1024);
				printf("%s\n", strError);
				return; break;
			default:
				printf("Open failed: %s\n", xnGetStatusString(rc));
				return;
		}

		niContext.FindExistingNode(XN_NODE_TYPE_DEPTH, niDepth);
		niContext.FindExistingNode(XN_NODE_TYPE_IMAGE, niImage);
		niDepth.GetMirrorCap().SetMirror(false); 
//		niImage.GetMirrorCap().SetMirror(false);
		niDepth.GetAlternativeViewPointCap().SetViewPoint(niImage);

		//Load Kinect Intrinsics
		loadParams(paramsFile);
		params->data.db[2]=320.0; params->data.db[5]=240.0;
		cvReleaseMat(&distortion); distortion = 0;

		//Set transform to 0
		transform = 0; invTransform = 0;
	}

	void getNewFrame() {
		if (XnStatus rc = niContext.WaitAnyUpdateAll() != XN_STATUS_OK) {
			printf("Read failed: %s\n", xnGetStatusString(rc));
			return;
		}

		// Update MetaData containers
		niDepth.GetMetaData(niDepthMD); niImage.GetMetaData(niImageMD);
	}

	// Extract Colour Image
	IplImage *getColour() {
		IplImage *colourIm = cvCreateImage(cvSize(niImageMD.XRes(), niImageMD.YRes()), IPL_DEPTH_8U, 3);
		memcpy(colourIm->imageData, niImageMD.Data(), colourIm->imageSize); cvCvtColor(colourIm, colourIm, CV_RGB2BGR);
		cvFlip(colourIm, colourIm, 1);
		return colourIm;
	}

	// Extract Depth Image
	IplImage *getDepth() {
		IplImage *depthIm = cvCreateImage(cvSize(niDepthMD.XRes(), niDepthMD.YRes()), IPL_DEPTH_16U, 1);
		memcpy(depthIm->imageData, niDepthMD.Data(), depthIm->imageSize);
		return depthIm;
	}

	// Extract Depth Mask
	IplImage *getDepthMask() {
		IplImage *depthMask = cvCreateImage(cvSize(niDepthMD.XRes(), niDepthMD.YRes()), IPL_DEPTH_8U, 1);
		char *dMask = depthMask->imageData; const unsigned short *niDepth = niDepthMD.Data();
		for (int i=0; i<depthMask->height*depthMask->width; i++)
			dMask[i] = (niDepth[i]==0)?0:255;

		return depthMask;
	}
	
	bool calculateTransform(CvSize markerSize, CvMat *homography) {
		//Find the position of the corners on the image
		CvPoint2D32f *markerCorners = (CvPoint2D32f *)malloc(4*sizeof(CvPoint2D32f));
		markerCorners[0] = cvPoint2D32f(0,0); markerCorners[1] = cvPoint2D32f(markerSize.width,0); 
		markerCorners[2] = cvPoint2D32f(markerSize.width,markerSize.height); markerCorners[3] = cvPoint2D32f(0,markerSize.height);

		CvMat mCorners = cvMat(4,1,CV_32FC2, markerCorners);
		cvPerspectiveTransform(&mCorners, &mCorners, homography);

		for (int i=0; i<4; i++) {
			if (markerCorners[i].x<0 || markerCorners[i].x> niImageMD.XRes() || markerCorners[i].y<0 || markerCorners[i].y>niImageMD.YRes()) {
				free(markerCorners);
				return false;
			}
		}

		//Find the position of the corners in the real world wrt kinect
		XnPoint3D xnCorner[4], xnNewCorner[4]; CvPoint3D32f markCorn[4];

		inpaintDepth(true);

		for (int i=0; i<4; i++) {
			markCorn[i] = cvPoint3D32f(markerCorners[i].x, markerCorners[i].y, niDepthMD[int(markerCorners[i].y*niDepthMD.XRes() + markerCorners[i].x)]);
			xnCorner[i].X = markCorn[i].x; xnCorner[i].Y = markCorn[i].y; xnCorner[i].Z = markCorn[i].z;
		}
		niDepth.ConvertProjectiveToRealWorld(4, xnCorner, xnNewCorner);

		//Calculate width and height of marker in real world
		float width1 = sqrt((xnNewCorner[0].X - xnNewCorner[1].X)*(xnNewCorner[0].X - xnNewCorner[1].X) + (xnNewCorner[0].Y - xnNewCorner[1].Y)*(xnNewCorner[0].Y - xnNewCorner[1].Y) + (xnNewCorner[0].Z - xnNewCorner[1].Z)*(xnNewCorner[0].Z - xnNewCorner[1].Z));
		float width2 = sqrt((xnNewCorner[3].X - xnNewCorner[2].X)*(xnNewCorner[3].X - xnNewCorner[2].X) + (xnNewCorner[3].Y - xnNewCorner[2].Y)*(xnNewCorner[3].Y - xnNewCorner[2].Y) + (xnNewCorner[3].Z - xnNewCorner[2].Z)*(xnNewCorner[3].Z - xnNewCorner[2].Z));
		float height1 = sqrt((xnNewCorner[3].X - xnNewCorner[0].X)*(xnNewCorner[3].X - xnNewCorner[0].X) + (xnNewCorner[3].Y - xnNewCorner[0].Y)*(xnNewCorner[3].Y - xnNewCorner[0].Y) + (xnNewCorner[3].Z - xnNewCorner[0].Z)*(xnNewCorner[3].Z - xnNewCorner[0].Z));
		float height2 = sqrt((xnNewCorner[2].X - xnNewCorner[1].X)*(xnNewCorner[2].X - xnNewCorner[1].X) + (xnNewCorner[2].Y - xnNewCorner[1].Y)*(xnNewCorner[2].Y - xnNewCorner[1].Y) + (xnNewCorner[2].Z - xnNewCorner[1].Z)*(xnNewCorner[2].Z - xnNewCorner[1].Z));
		//markerSize.width = (width1+width2)/2.0; markerSize.height = markerSize.width * ((float)mt.at(0).marker.size.height/(float)mt.at(0).marker.size.width);
		realMarkerSize.height = (height1+height2)/2.0; realMarkerSize.width = realMarkerSize.height * (markerSize.width/markerSize.height);
		printf("Marker Size %dx%d\n", realMarkerSize.width, realMarkerSize.height);

		//Render the marker corners
		IplImage *kinectMarker = getColour();
		for (int i=0; i<4; i++) cvCircle(kinectMarker, cvPoint(markerCorners[i].x, markerCorners[i].y), 3, cvScalar(0,0,255), -1);
		cvShowImage("Kinect Found Marker", kinectMarker);
		cvReleaseImage(&kinectMarker);

		free(markerCorners);

		//Calculate Kinect to OpenGL Transform
		{
			std::vector <CvPoint3D32f> srcPoints3D, dstPoints3D; std::vector <CvPoint2D32f> srcPoints2D, dstPoints2D;
			srcPoints2D.resize(50); srcPoints3D.resize(50); dstPoints2D.resize(50); dstPoints3D.resize(50);
			float xStep = float(realMarkerSize.width)/9.0; float yStep = float(realMarkerSize.height)/4.0;
			float xStep1 = float(markerSize.width)/9.0; float yStep1 = float(markerSize.height)/4.0;
			for (int y=0; y<5; y++) {
				for (int x=0; x<10; x++) {
					int index = x+(y*10);
					srcPoints3D.at(index) = cvPoint3D32f(x*xStep, y*yStep, 0);
					srcPoints2D.at(index) = cvPoint2D32f(x*xStep1, y*yStep1);
				}
			}
			
			CvMat mSrcCorners = cvMat(50,1,CV_32FC2, &srcPoints2D[0]); CvMat mDstCorners = cvMat(50,1,CV_32FC2, &dstPoints2D[0]);
			cvPerspectiveTransform(&mSrcCorners, &mDstCorners, homography);

			XnPoint3D _xnCorner[50], _xnNewCorner[50]; 
			for (int i=0; i<50; i++) {_xnCorner[i].X = dstPoints2D[i].x; _xnCorner[i].Y = dstPoints2D[i].y; _xnCorner[i].Z =  niDepthMD[int(_xnCorner[i].Y*niDepthMD.XRes() + _xnCorner[i].X)]; }
			niDepth.ConvertProjectiveToRealWorld(50, _xnCorner, _xnNewCorner);
			for (int i=0; i<50; i++) {dstPoints3D[i] = cvPoint3D32f(_xnNewCorner[i].X, _xnNewCorner[i].Y, _xnNewCorner[i].Z);}

			if (transform) { cvReleaseMat(&transform); transform = 0; }
			transform = findTransform(dstPoints3D, srcPoints3D);

			if (invTransform) { cvReleaseMat(&invTransform); invTransform = 0; }
			invTransform = findTransform(srcPoints3D, dstPoints3D);

			for (int y=0; y<4; y++) {
				for (int x=0; x<4; x++) {
					printf("%.2f\t", CV_MAT_ELEM((*transform), float, y,x));
				}
				printf("\n");
			}

		}

		return true;
	}

	CvPoint3D32f getRealWorldPoint(CvPoint p) {
		XnPoint3D _xnPoint, _xnRWPoint; 
		_xnPoint.X = p.x; _xnPoint.Y = p.y; _xnPoint.Z =  niDepthMD[int(_xnPoint.Y*niDepthMD.XRes() + _xnPoint.X)];
		niDepth.ConvertProjectiveToRealWorld(1, &_xnPoint, &_xnRWPoint);
		return cvPoint3D32f(_xnRWPoint.X, _xnRWPoint.Y, _xnRWPoint.Z);
	}

	CvPoint3D32f getTransformedPoint(CvPoint3D32f p) {
		if (transform==0) return cvPoint3D32f(0,0,0);

		float _p[4]; CvMat mP = cvMat(4,1, CV_32FC1, &_p);
		_p[0]=p.x; _p[1]=p.y; _p[2]=p.z; _p[3]=1;
		cvMatMul(transform, &mP, &mP);
		return cvPoint3D32f(_p[0]/_p[3], _p[1]/_p[3], _p[2]/_p[3]);
	}

	CvPoint3D32f getTransformedPoint(CvPoint p) {
		if (transform==0) return cvPoint3D32f(0,0,0);

		CvPoint3D32f p2 = getRealWorldPoint(p);
		return getTransformedPoint(p2);
	}

	CvPoint3D32f getInverseTransformedPoint(CvPoint3D32f p) {
		if (invTransform==0) return cvPoint3D32f(0,0,0);

		float _p[4]; CvMat mP = cvMat(4,1, CV_32FC1, &_p); 
		_p[0]=p.x; _p[1]=p.y; _p[2]=p.z; _p[3]=1;
		cvMatMul(invTransform, &mP, &mP);
		
		XnPoint3D _xnRWPoint,_xnPoint; 
		_xnRWPoint.X = _p[0]/_p[3]; _xnRWPoint.Y = _p[1]/_p[3]; _xnRWPoint.Z = _p[2]/_p[3];

		niDepth.ConvertRealWorldToProjective(1, &_xnRWPoint, &_xnPoint);
		return cvPoint3D32f(_xnPoint.X, _xnPoint.Y, _xnPoint.Z);
	}

	CvPoint3D32f* getRealWorldPoints(CvPoint *p, int count) {
		XnPoint3D *_xnPoint = (XnPoint3D *)malloc(count*sizeof(XnPoint3D));
		for (int i=0; i<count; i++) { _xnPoint[i].X = p[i].x; _xnPoint[i].Y = p[i].y; _xnPoint[i].Z =  niDepthMD[int(_xnPoint[i].Y*niDepthMD.XRes() + _xnPoint[i].X)]; }
		niDepth.ConvertProjectiveToRealWorld(count, _xnPoint, _xnPoint);
		CvPoint3D32f *rp = (CvPoint3D32f *)malloc(count*sizeof(CvPoint3D32f));
		for (int i=0; i<count; i++) { rp[i].x = _xnPoint[i].X; rp[i].y = _xnPoint[i].Y; rp[i].z = _xnPoint[i].Z; }
		free(_xnPoint);
		return rp;
	}

	CvPoint3D32f* getTransformedPoints(CvPoint3D32f *p, int count) {
		//if (transform==0) return cvPoint3D32f(0,0,0);

		CvPoint3D32f* rp = (CvPoint3D32f*)malloc(count*sizeof(CvPoint3D32f));
		float _p[4], _RWP[4];
		CvMat mP = cvMat(4,1, CV_32FC1, &_p); 
		CvMat mRWP = cvMat(4,1, CV_32FC1, &_RWP);
		for (int i=0; i<count; i++) {
			_p[0]=p[i].x; _p[1]=p[i].y; _p[2]=p[i].z; _p[3]=1;
			cvMatMul(transform, &mP, &mP);
			rp[i].x = _p[0]/_p[3]; rp[i].y = _p[1]/_p[3]; rp[i].z = _p[2]/_p[3];
		}
		return rp;
	}

	CvPoint3D32f* getTransformedPoints(CvPoint *p, int count) {
		//if (transform==0) return cvPoint3D32f(0,0,0);

		CvPoint3D32f *p2 = getRealWorldPoints(p, count);
		CvPoint3D32f *p3 = getTransformedPoints(p2, count);
		free(p2);
		return p3;
	}

	CvMat *getParameters() { return params;}
	CvMat *getDistortion() { return distortion;}
	
	CvMat *getTransform() { return transform;}
	CvMat *getInverseTransform() { return invTransform;}

	CvSize getRealMarkerSize() { return realMarkerSize; }

private:
	bool gotColour, gotDepth;

	xn::Context niContext;
	xn::DepthGenerator niDepth; 
	xn::ImageGenerator niImage;
	xn::DepthMetaData niDepthMD; 
	xn::ImageMetaData niImageMD;

	CvMat *params, *distortion;
	CvMat *transform, *invTransform;

	CvSize realMarkerSize;

	bool loadParams(char *filename) {
		CvFileStorage* fs = cvOpenFileStorage( filename, 0, CV_STORAGE_READ );
		if (fs==0) return false; 

		CvFileNode* fileparams;
		//Read the Camera Parameters
		fileparams = cvGetFileNodeByName( fs, NULL, "camera_matrix" );
		params = (CvMat*)cvRead( fs, fileparams );

		//Read the Camera Distortion 
		fileparams = cvGetFileNodeByName( fs, NULL, "distortion_coefficients" );
		distortion = (CvMat*)cvRead( fs, fileparams );
		cvReleaseFileStorage( &fs );
		return true;
	}


	void inpaintDepth(bool halfSize) {
		IplImage *depthIm, *depthImFull;
		
		if (halfSize) {
			depthImFull = cvCreateImage(cvSize(niDepthMD.XRes(), niDepthMD.YRes()), IPL_DEPTH_16U, 1);
			depthImFull->imageData = (char*)niDepthMD.WritableData();
			depthIm = cvCreateImage(cvSize(depthImFull->width/2.0, depthImFull->height/2.0), IPL_DEPTH_16U, 1);
			cvResize(depthImFull, depthIm, 0);
		} else {
			depthIm = cvCreateImage(cvSize(niDepthMD.XRes(), niDepthMD.YRes()), IPL_DEPTH_16U, 1);
			depthIm->imageData = (char*)niDepthMD.WritableData();
		}
		
		IplImage *depthImMask = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_8U, 1);
		for (int y=0; y<depthIm->height; y++) {
			for (int x=0; x<depthIm->width; x++) {
				CV_IMAGE_ELEM(depthImMask, char, y, x)=CV_IMAGE_ELEM(depthIm, unsigned short,y,x)==0?255:0;
			}
		}

		IplImage *depthImMaskInv = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_8U, 1);
		cvNot(depthImMask, depthImMaskInv);

		double min, max; cvMinMaxLoc(depthIm, &min, &max, 0, 0, depthImMaskInv);
		
		IplImage *depthIm8 = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_8U, 1);
		float scale = 255.0/(max-min);
		cvConvertScale(depthIm, depthIm8, scale, -(min*scale));

		IplImage *depthPaint = cvCreateImage(cvGetSize(depthIm8), IPL_DEPTH_8U, 1);
		cvInpaint(depthIm8, depthImMask, depthPaint, 3, CV_INPAINT_NS);
		
		IplImage *depthIm16 = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_16U, 1);
		cvConvertScale(depthPaint, depthIm16, 1/scale, min);

		if (halfSize) {
			IplImage *depthPaintedFull = cvCreateImage(cvGetSize(depthImFull), IPL_DEPTH_16U, 1);
			cvResize(depthIm16, depthPaintedFull,0);
			IplImage *depthImMaskFull = cvCreateImage(cvGetSize(depthImFull), IPL_DEPTH_8U, 1);
			for (int y=0; y<depthImFull->height; y++) for (int x=0; x<depthImFull->width; x++)
				CV_IMAGE_ELEM(depthImMaskFull, char, y, x)=CV_IMAGE_ELEM(depthImFull, unsigned short,y,x)==0?255:0;
			cvCopy(depthPaintedFull, depthImFull, depthImMaskFull);
			cvReleaseImage(&depthPaintedFull); cvReleaseImage(&depthImMaskFull);
			cvReleaseImage(&depthImFull);
		} else {
			cvCopy(depthIm16, depthIm, depthImMask);
		}

		cvReleaseImage(&depthIm8); cvReleaseImage(&depthIm16);
		cvReleaseImage(&depthPaint);
		cvReleaseImage(&depthImMask); cvReleaseImage(&depthImMaskInv);
		cvReleaseImage(&depthIm);
	}

};

#endif