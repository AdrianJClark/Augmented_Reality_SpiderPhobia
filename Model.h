#ifndef MODEL_H
#define MODEL_H

class ARNode : public osg::Group {
private:
	std::string markerName;
	std::map<std::string, osg::ref_ptr<osg::MatrixTransform>> MarkerMap;
public:
	ARNode(CvSize markerSize, osg::Node *node): osg::Group() {
		this->addChild(createLights(this));

		//The structure is as follows:
		// ARNode "this" -> Switch "foundObject" -> MatrixTransform "mt" -> MatrixTransform "osgTrans" -> Node "node"

		// Rotate the transformation to match OSG coordinates, and translate to the centre of the marker
		osg::MatrixTransform* osgTrans = new osg::MatrixTransform(osg::Matrix::rotate(osg::DegreesToRadians(180.0f), osg::X_AXIS));

		osgTrans->getOrCreateStateSet()->setMode(GL_NORMALIZE, GL_TRUE);
		
		//Add the model and switch
		mSwitch = new osg::Switch(); mSwitch->setAllChildrenOff();
		mSwitch->addChild(node);
		osgTrans->addChild(mSwitch);

		//Add the Boundary Lines and switch
		bSwitch = new osg::Switch(); bSwitch->setAllChildrenOff();
		bSwitch->addChild(createBoundaryLines(markerSize));
		osgTrans->addChild(bSwitch);

		//Add the Matrix Transform
		mt = new osg::MatrixTransform();
		mt->addChild(osgTrans);

		this->addChild(mt);
	}

	void setTransform(double *mat) {
		mt->setMatrix(osg::Matrixd(mat));
	}

	void setModelVisible(bool visible) {
		if (visible) mSwitch->setAllChildrenOn(); else mSwitch->setAllChildrenOff();
	}

	void setBoundaryVisible(bool visible) {
		if (visible) bSwitch->setAllChildrenOn(); else bSwitch->setAllChildrenOff();
	}

	void setBoundaryColour(float r, float g, float b) {
		osg::Vec4Array* bCols = new osg::Vec4Array(); 
		geom1->setColorArray(bCols);  
		bCols->push_back(osg::Vec4(r, g, b, 0.5));
	}

private:
	osg::ref_ptr<osg::MatrixTransform> mt;
	
	osg::Geometry* geom1;

	osg::ref_ptr<osg::Switch>	mSwitch;
	osg::ref_ptr<osg::Switch>	bSwitch;
	

	osg::Node* createBoundaryLines(CvSize markerSize) {
		//Create the geode, geometry and switch objects
		osg::Geode* geode = new osg::Geode();
		geom1 = new osg::Geometry();
		osg::Geometry* geom2 = new osg::Geometry();

		//Add the geometry objects, set up lighting, blending and line smoothing
		geode->addDrawable(geom1); geode->addDrawable(geom2);
		geode->getOrCreateStateSet()->setMode(GL_LIGHTING, GL_FALSE);
		geom1->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		geom2->getOrCreateStateSet()->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);

		//Calculate marker corner co-ords
		double hw = markerSize.width * 0.5f;
		double hh = markerSize.height * 0.5f;

		//Create vertex data
		osg::Vec3Array* verts = new osg::Vec3Array(); 
		verts->push_back(osg::Vec3(-hw, -hh, 0)); verts->push_back(osg::Vec3( hw, -hh, 0));
		verts->push_back(osg::Vec3( hw,  hh, 0)); verts->push_back(osg::Vec3(-hw,  hh, 0));
		geom1->setVertexArray(verts); geom2->setVertexArray(verts); 
		
		//Set up colour data
		geom1->setColorBinding(osg::Geometry::BIND_OVERALL);
		geom2->setColorBinding(osg::Geometry::BIND_OVERALL);
		//osg::Vec4Array* bCols = new osg::Vec4Array(); geom1->setColorArray(bCols);  bCols->push_back(osg::Vec4(0, 0, 0, 0.5));
		osg::Vec4Array* black = new osg::Vec4Array(); geom2->setColorArray(black);	black->push_back(osg::Vec4(0,0,0,1));

		//Create the Quad and Line Loop
		geom1->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));
		geom2->addPrimitiveSet(new osg::DrawArrays(GL_LINE_LOOP, 0, 4));

		return geode;
	}

	osg::Node* createLights(osg::Node* scene) {
		osg::Group* lightGroup = new osg::Group;
	   
		osg::StateSet* stateset = scene->getOrCreateStateSet();

		// Create a local light.
		osg::Light* light = new osg::Light();
		light->setLightNum(0);
		light->setPosition(osg::Vec4(0.0f, 0.0f, 1.0, 0.0f));
		light->setAmbient(osg::Vec4(0.4f,0.4f,0.4f,1.0f));
		light->setDiffuse(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
		light->setSpecular(osg::Vec4(0.4f,0.4f,0.4f,1.0f));
		light->setConstantAttenuation(0.2f);
		light->setLinearAttenuation(0.05f);
		light->setQuadraticAttenuation(0.05f);

		
		osg::LightSource* lightSource = new osg::LightSource;	
		lightSource->setLight(light);
		lightSource->setLocalStateSetModes(osg::StateAttribute::ON); 
		lightSource->setStateSetModes(*stateset, osg::StateAttribute::ON);
		lightGroup->addChild(lightSource);
		
		return lightGroup;
	}
};

#endif