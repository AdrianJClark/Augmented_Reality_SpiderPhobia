#ifndef SPIDER_H
#define SPIDER_H

#include <osgDB/ReadFile>
#include <osg/Sequence>
#include <osg/PositionAttitudeTransform>
#include "tinyxml.h"
#include "global.h"

template <typename T>
class CollectTypeNodeVisitor : public osg::NodeVisitor {
protected:
	osg::NodeList mCollectedNodes;

public:
	CollectTypeNodeVisitor() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) { }
    
    virtual void apply(osg::Node& node) {
		if (T t = dynamic_cast< T >(&node)) {
			mCollectedNodes.push_back(&node);
		}
		traverse(node);
    }

	inline const osg::NodeList& getCollectedNodes() { return mCollectedNodes; }
};

class AnimationSequenceCallback : public osg::NodeCallback 
{
public:
	void setValues(int _endVal) { endVal = _endVal; loop = false; }
	void setValues(int _startVal, int _endVal) { endVal = _endVal; startVal = _startVal; loop = true; }

   virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
   {
	  osg::ref_ptr<osg::Sequence> seq = dynamic_cast<osg::Sequence*> (node);
	  if(seq->getValue() >= endVal)
		  if (loop) { seq->setValue(startVal); } else { seq->setValue(endVal); }

	  traverse(node, nv); 
   }

private:
	int endVal, startVal;
	bool loop;
};

class Spider {
public:
	Spider(char *modelFile, char *animationFile) {
		model = osgDB::readNodeFile(modelFile);
		osg::PositionAttitudeTransform *scale = new osg::PositionAttitudeTransform(); scale->setScale(osg::Vec3(0.4,0.4,0.4)); scale->addChild(model);
		transform = new osg::PositionAttitudeTransform(); transform->addChild(scale);
		
		// Get Animation Nodes and apply callback
		CollectTypeNodeVisitor<osg::Sequence*> ctnv; aniSeqCB = new AnimationSequenceCallback();
		model->accept(ctnv); animationNodes = ctnv.getCollectedNodes();
		for (osg::NodeList::iterator iter = animationNodes.begin(); iter != animationNodes.end(); iter++)
			if (osg::Sequence* seq = dynamic_cast<osg::Sequence*>((*iter).get()))
				seq->addUpdateCallback(aniSeqCB);

		readXML(animationFile);
		setAnimation(0);
		lX=lY=lZ=0; lAng = 0; 
		aniPathCB = new SpiderAnimationPathCallback(this); _isAnimating = false;
	}

	~Spider() {}

	osg::Node* getModel() {
		return transform;
	}

	void setPosition(float x, float y, float z) {
		transform->setPosition(osg::Vec3d(x,y,z));
	}

	osg::Vec3 getPosition() {
		return transform->getPosition();
	}

	bool isAnimating() {
		return _isAnimating;
	}

	void moveTo(float x, float y, float z) {
		float ang = atan2(lY-y, lX-x)-osg::PI_2;
		_isAnimating = true; aniPathCB->reset();
		aniPathCB->setAnimationPath(createAnimationPath(osg::Vec3(lX, lY, lZ), lAng, osg::Vec3(x, y, z), ang));
		transform->setUpdateCallback(aniPathCB);
		setAnimation(2);
		
		lX = x; lY=y; lZ=z; lAng = ang;
	}

	void setAnimation(int index) {
		aniSeqCB->setValues(Animations.at(index).start, Animations.at(index).end);

		for (osg::NodeList::iterator iter = animationNodes.begin(); iter != animationNodes.end(); iter++)
			if (osg::Sequence* seq = dynamic_cast<osg::Sequence*>((*iter).get())) {
				seq->setValue(Animations.at(index).start);
			}
	}

	bool _isAnimating;
private:
	osg::Node *model;
	osg::PositionAttitudeTransform *transform;

	AnimationSequenceCallback *aniSeqCB;
	osg::NodeList animationNodes;

	osg::AnimationPathCallback *aniPathCB;
	

	float lX, lY, lZ, lAng;

	struct Animation {
		Animation() {}
		Animation(std::string _name, int _start, int _end) {name = _name; start = _start; end = _end; }
		std::string name;
		int start, end;
	};
	std::vector <Animation> Animations;

	class SpiderAnimationPathCallback: public osg::AnimationPathCallback
	{
	public:
		SpiderAnimationPathCallback(Spider *_s) {spider = _s;}

		void SpiderAnimationPathCallback::operator() (osg::Node *node, osg::NodeVisitor *nv){ 
			osg::AnimationPathCallback::operator()(node,nv); // Call Original Method
			if (this->getAnimationTime()> this->getAnimationPath()->getLastTime()) {
				spider->setAnimation(0); spider->_isAnimating = false;
			}
			float z=getSpiderHeight();
			if (z>500) z = lastZ;
			osg::Vec3 p= spider->getPosition();
			spider->setPosition(p.x(),p.y(), z);
			lastZ = z;
		}

	private:
		Spider *spider;
		int lastZ;
	};

	void readXML(char *filename) {
		TiXmlDocument doc( filename ); doc.LoadFile();

		TiXmlElement* animation = doc.FirstChildElement();

		while (animation) {
			Animations.push_back(Animation(animation->Attribute("name"), atoi(animation->Attribute("start")), atoi(animation->Attribute("end"))));
			animation = animation->NextSiblingElement();
		}
	}

	osg::AnimationPath* createAnimationPath(osg::Vec3 begin, float lAng, osg::Vec3 end, float ang)
	{
		// set up the animation path 
		osg::AnimationPath* animationPath = new osg::AnimationPath;
		animationPath->setLoopMode(osg::AnimationPath::LoopMode::NO_LOOPING);
		
		float dist = sqrt((begin.x()-end.x())*(begin.x()-end.x())+(begin.y()-end.y())*(begin.y()-end.y()));
		animationPath->insert(0.0,osg::AnimationPath::ControlPoint(begin, osg::Quat(lAng, osg::Vec3f(0,0,1))));
		animationPath->insert(abs(ang-lAng)/2.0,osg::AnimationPath::ControlPoint(begin, osg::Quat(ang, osg::Vec3f(0,0,1))));
		animationPath->insert(abs(ang-lAng)/2.0+dist/8.0,osg::AnimationPath::ControlPoint(end, osg::Quat(ang, osg::Vec3f(0,0,1))));
		
		return animationPath;    
	}


};

#endif