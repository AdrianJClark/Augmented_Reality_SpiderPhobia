#ifndef RENDERERMODEL_H
#define RENDERERMODEL_H

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Texture2D>
#include <osg/Geode>
#include <osg/Geometry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/io_utils>
#include <osg/Depth>

#include "Model.h"
#include "Global.h"

class keyboardEventHandler : public osgGA::GUIEventHandler {
    public:
		virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&) { 
			if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP) {
				if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Escape)
					checkKeyPress(27);
				else 
					checkKeyPress(ea.getKey());
			}
			return false;
		}
		virtual void accept(osgGA::GUIEventHandlerVisitor& v)   { v.visit(*this); };
};


class Renderer {
public:
	Renderer(int Width, int Height, double *projMat) {
		_width = Width; _height = Height;
		mVideoImage = new osg::Image();

		scaleImage = cvCreateImage(cvSize(512, 512), IPL_DEPTH_8U, 3);

		viewer.addEventHandler(new osgViewer::WindowSizeHandler());
		viewer.setUpViewInWindow(100, 100, _width, _height);

		viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
		viewer.setKeyEventSetsDone(0);
		viewer.addEventHandler(new keyboardEventHandler());

		osg::ref_ptr<osg::Group> root = new osg::Group();
		viewer.setSceneData(root.get());

	
		// ----------------------------------------------------------------
		// Video background
		// ----------------------------------------------------------------
		osg::ref_ptr<osg::Camera> bgCamera = new osg::Camera();
		bgCamera->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin");
		bgCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
		bgCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
		bgCamera->getOrCreateStateSet()->setMode(GL_LIGHTING, GL_FALSE);
		bgCamera->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, GL_FALSE);
		bgCamera->setProjectionMatrixAsOrtho2D(0, _width, 0, _height);
	
		osg::ref_ptr<osg::Geometry> geomBG = osg::createTexturedQuadGeometry(osg::Vec3(0, 0, 0), osg::X_AXIS * _width, osg::Y_AXIS * _height, 0, 1, 1, 0);
		geomBG->getOrCreateStateSet()->setTextureAttributeAndModes(0, new osg::Texture2D(mVideoImage));
	
		osg::ref_ptr<osg::Geode> geodeBG = new osg::Geode();
		geodeBG->addDrawable(geomBG.get());
		bgCamera->addChild(geodeBG.get());
		root->addChild(bgCamera.get());
	
		// ----------------------------------------------------------------
		// Foreground 3D content
		// ----------------------------------------------------------------
		fgCamera = new osg::Camera();
		fgCamera->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
		fgCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
		fgCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
		fgCamera->setProjectionMatrix(osg::Matrixf(projMat));

		// Create the Heightfield Geometry
		/*{
			HeightFieldPoints = new osg::Vec3Array;;
			HeightFieldGeometry = new osg::Geometry(); 
			HeightFieldTransform = new osg::MatrixTransform();

			//Create Height Field
			for (int i=0; i<639; i++) {
				for (int j=0; j<479; j++) {
					HeightFieldPoints->push_back(osg::Vec3(i-64, j-64, 0));
					HeightFieldPoints->push_back(osg::Vec3(i-63, j-64, 0));
					HeightFieldPoints->push_back(osg::Vec3(i-63, j-63, 0));
					HeightFieldPoints->push_back(osg::Vec3(i-64, j-63, 0));
				}
			}

			HeightFieldGeometry->setVertexArray(HeightFieldPoints); 
			HeightFieldGeometry->addPrimitiveSet(new osg::DrawArrays( GL_QUADS, 0, HeightFieldPoints->size()));
			HeightFieldGeometry->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

			//Set the Heightfield to be alpha invisible
			HeightFieldGeometry->setColorBinding(osg::Geometry::BIND_OVERALL); 
			osg::Vec4Array* col = new osg::Vec4Array(); HeightFieldGeometry->setColorArray(col); col->push_back(osg::Vec4(1,1,1,0.5));

			//Create the containing geode
			osg::ref_ptr< osg::Geode > geode = new osg::Geode(); geode->addDrawable(HeightFieldGeometry);

			//Create the containing transform
			float scale = 10; float x = 0; float y = 0; float z = 0;
			osg::ref_ptr< osg::PositionAttitudeTransform > mt = new osg::PositionAttitudeTransform();
			mt->setScale(osg::Vec3d(scale,scale,scale));
			mt->setAttitude(osg::Quat(0,0,0,1));       
			mt->setPosition(osg::Vec3d(x, y, z)); 
			mt->addChild( geode.get() );  

			//Set up the depth testing for the landscale
			osg::Depth * depth = new osg::Depth();
			depth->setWriteMask(true); depth->setFunction(osg::Depth::Function::LEQUAL);
			mt->getOrCreateStateSet()->setAttributeAndModes(depth, osg::StateAttribute::ON);

			//Set up the shadow masks
			//mt->setNodeMask( rcvShadowMask );
			//mt->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin");
			//pat->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
			//shadowedScene->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
			
			//At the heightmap twice, once for shadowing and once for occlusion
			HeightFieldTransform->addChild(mt);
			fgCamera.get()->addChild(HeightFieldTransform);
		}*/

		root->addChild(fgCamera.get());
	}

	~Renderer() {
		cvReleaseImage(&scaleImage);
	};

	void addModel(string markerName, osg::Node *model) {
		IplImage *markerIm = cvLoadImage(markerName.c_str());
		arModels[markerName] = new ARNode(cvGetSize(markerIm), model);
		cvReleaseImage(&markerIm);

		fgCamera->addChild(arModels[markerName]);
	}

	void updateHeightMap(CvPoint3D32f *ground_grid) {
		int index=0;
		for(int y = 0; y < 479; y++) {
			for(int x = 0; x < 639; x++) {
				int c1 = (y*640)+x;
				int c2 = (y*640)+x+1;
				int c3 = ((y+1)*640)+x+1;
				int c4 = ((y+1)*640)+x;
				HeightFieldPoints->at(index++) = osg::Vec3(ground_grid[c1].x, ground_grid[c1].y, ground_grid[c1].z); 
				HeightFieldPoints->at(index++) = osg::Vec3(ground_grid[c2].x, ground_grid[c2].y, ground_grid[c2].z);
				HeightFieldPoints->at(index++) = osg::Vec3(ground_grid[c3].x, ground_grid[c3].y, ground_grid[c3].z); 
				HeightFieldPoints->at(index++) = osg::Vec3(ground_grid[c4].x, ground_grid[c4].y, ground_grid[c4].z);
			}
		}
		HeightFieldGeometry->dirtyDisplayList();
	}

	void render(IplImage* frame_input, vector<MarkerTransform> mt) { 
		//Copy the frame into the background image
		cvResize(frame_input, scaleImage); cvCvtColor(scaleImage, scaleImage, CV_RGB2BGR);
		mVideoImage->setImage(scaleImage->width, scaleImage->height, 0, 3, GL_RGB, GL_UNSIGNED_BYTE, (unsigned char*)scaleImage->imageData, osg::Image::NO_DELETE);
	
		//Set the HeightFieldTransform
		//if (mt.size()>0) HeightFieldTransform->setMatrix(osg::Matrixd(mt.at(0).transMat));

		for (map<string, ARNode*>::iterator i = arModels.begin(); i!=arModels.end(); i++) {
			i->second->setModelVisible(false); i->second->setBoundaryVisible(false);
		}

		for (int i = 0; i<mt.size(); i++) {
			string filename = mt.at(i).marker.name;

			//Set up the transform and visibility
			arModels[filename]->setTransform(mt.at(i).transMat);
			arModels[filename]->setModelVisible(true);
			arModels[filename]->setBoundaryVisible(false);
		}

		viewer.frame();
	}

private:
	osg::Image* mVideoImage;
	osgViewer::Viewer viewer;
	osg::ref_ptr<osg::Camera> fgCamera;

	//HeightField
	osg::Vec3Array* HeightFieldPoints;
	osg::Geometry* HeightFieldGeometry;
	osg::ref_ptr<osg::MatrixTransform> HeightFieldTransform;

	map<string, ARNode*> arModels;

	IplImage *scaleImage;
	int _width, _height;
};

#endif