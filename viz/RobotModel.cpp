#include "RobotModel.h"
#include <urdf_parser/urdf_parser.h>
#include "fstream"
#include "sstream"
#include "osg/Texture2D"
#include "osg/BlendFunc"
#include "osg/AlphaFunc"
#include "osg/Billboard"
#include "osg/PointSprite"
#include "osg/Point"
#include "osg/Geometry"
#include "osg/Image"
#include "osg/Material"
//#include <resource_retriever/retriever.h>
//#include "ros/ros.h"
#include "OSGHelpers.hpp"
#include <base/Logging.hpp>
#include <QFileInfo>

#include <fstream>
#include <streambuf>

OSGSegment::OSGSegment(osg::Node* node, KDL::Segment seg)
{
    isSelected_=false;
    seg_ = seg;
    jointPos_ = 0;
    label_ = 0;
    visual_ = 0;
    toTipOsg_ = node->asTransform()->asPositionAttitudeTransform();
    updateJoint();
}

osg::ref_ptr<osg::Group> OSGSegment::getGroup() const{
    return toTipOsg_;
}

void OSGSegment::updateJoint(){
    toTipKdl_ = seg_.pose(jointPos_);
    kdl_to_osg(toTipKdl_, *toTipOsg_);
}

void OSGSegment::attachVisuals(std::vector<boost::shared_ptr<urdf::Visual> > &visual_array, QDir prefix)
{
    std::vector<boost::shared_ptr<urdf::Visual> >::iterator itr = visual_array.begin();
    std::vector<boost::shared_ptr<urdf::Visual> >::iterator itr_end = visual_array.end();

    for(itr = visual_array.begin(); itr != itr_end; ++itr)
    {
        boost::shared_ptr<urdf::Visual> visual = *itr;
        attachVisual(visual, prefix);
    }
}

void OSGSegment::attachVisual(boost::shared_ptr<urdf::Visual> visual, QDir baseDir)
{
    osg::PositionAttitudeTransform* to_visual = new osg::PositionAttitudeTransform();
    if (visual)
        urdf_to_osg(visual->origin, *to_visual);
    toTipOsg_->addChild(to_visual);
    toTipOsg_->setName(seg_.getJoint().getName());

    osg::Node* osg_visual = 0;
    if(visual && visual->geometry->type == urdf::Geometry::MESH){
        urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(visual->geometry.get());
        to_visual->setScale(urdf_to_osg(mesh->scale));

        std::string prefix = "file://";
        std::string filename = "";
        if(mesh->filename.compare(0, prefix.length(), prefix) == 0){
            filename = mesh->filename.substr(prefix.length());
        }
        else
            filename = mesh->filename;
        LOG_DEBUG("Trying to load mesh file %s", filename.c_str());

        QString qfilename = QString::fromStdString(filename);
        if (QFileInfo(qfilename).isRelative())
            filename = baseDir.absoluteFilePath(qfilename).toStdString();

        osg_visual = osgDB::readNodeFile(filename);
        if(!osg_visual){
            LOG_ERROR("OpenSceneGraph did not succees loading the mesh file %s.", filename.c_str());
            throw std::runtime_error("Error loading mesh file.");
        }
        if(!osg_visual){
            LOG_ERROR("Unecpected error loading mesh file %s", mesh->filename.c_str());
            throw(std::runtime_error("Couldn't load mesh file."));
        }

        if(visual->material){
            osg::ref_ptr<osg::StateSet> nodess = osg_visual->getOrCreateStateSet();
            nodess->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

            osg::ref_ptr<osg::Material> nodematerial = new osg::Material;
            //Specifying the colour of the object
            nodematerial->setDiffuse(osg::Material::FRONT,osg::Vec4(visual->material->color.r,
                                                                    visual->material->color.g,
                                                                    visual->material->color.b,
                                                                    visual->material->color.a));
            nodematerial->setSpecular(osg::Material::FRONT,osg::Vec4(0.2,
                                                                     0.2,
                                                                     0.2,
                                                                     1));

            //Attaching the newly defined state set object to the node state set
            nodess->setAttribute(nodematerial.get());
        }
    }
    else
    {
        osg_visual = new osg::Geode;
    }

    to_visual->addChild(osg_visual);
    osg_visual->setUserData(this);
    osg_visual->setName(seg_.getName());
    visual_ = osg_visual->asGeode();
}

void OSGSegment::removeLabel(){
    if(label_)
        toTipOsg_->removeChild(label_);
    label_ = 0;
}

void OSGSegment::attachLabel(std::string name, std::string filepath){
    if(label_)
        removeLabel();

    osg::Geode *geode = new osg::Geode();
    osg::Geometry *geometry = new osg::Geometry();

    osg::Vec3Array* vertices = new osg::Vec3Array;
    vertices->push_back (osg::Vec3 (0, 0, 0.0));

    geometry->setVertexArray (vertices);

    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));

    geode->addDrawable(geometry);
    osg::StateSet *set = new osg::StateSet();

    /// Setup the point sprites
    osg::PointSprite *sprite = new osg::PointSprite();
    set->setTextureAttributeAndModes(0, sprite, osg::StateAttribute::ON);

    /// Give some size to the points to be able to see the sprite
    osg::Point *point = new osg::Point();
    point->setSize(50);
    set->setAttribute(point);

    /// Disable depth test to avoid sort problems and Lighting
    set->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    set->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::ref_ptr<osg::BlendFunc> texture_blending_function = new osg::BlendFunc();
    set->setAttributeAndModes(texture_blending_function.get(), osg::StateAttribute::ON);

    osg::ref_ptr<osg::AlphaFunc> alpha_transparency_function = new osg::AlphaFunc();
    alpha_transparency_function->setFunction(osg::AlphaFunc::GEQUAL, 0.05);
    set->setAttributeAndModes(alpha_transparency_function.get(), osg::StateAttribute::ON );

    /// The texture for the sprites
    osg::Texture2D *tex = new osg::Texture2D();
    osg::Image* image = osgDB::readImageFile(filepath);
    image->flipVertical();
    tex->setImage(image);

    set->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON);

    toTipOsg_->addChild(geode);

    geode->setStateSet(set);
    geode->setName(name);
    geode->setUserData(this);

    label_ = geode;
}

bool OSGSegment::toggleSelected(){
    isSelected_ = !isSelected_;

    osg::Group* parent = visual_->getParent(0);

    if(isSelected_){
        osgFX::Outline* scribe = new osgFX::Outline();
        scribe->setWidth(1);
        scribe->setColor(osg::Vec4(1,0,0,1));
        scribe->addChild(visual_);
        parent->replaceChild(visual_,scribe);
    }
    else{
        //node already picked so we want to remove marker to unpick it.
        osg::Node::ParentList parentList = parent->getParents();
        for(osg::Node::ParentList::iterator itr=parentList.begin();
            itr!=parentList.end();
            ++itr)
        {
            (*itr)->replaceChild(parent, visual_);
        }
    }

    //update_visual();
    return isSelected_;
}


RobotModel::RobotModel(){
    //Root is the entry point to the scene graph
    osg::Group* root = new osg::Group();
    root_ = root;
}

osg::Node* RobotModel::loadEmptyScene(){
    root_->removeChildren(0, root_->getNumChildren());
    jointNames_.clear();
    segmentNames_.clear();
    return root_;
}

osg::Node* RobotModel::makeOsg2(KDL::Segment kdl_seg, urdf::Link urdf_link, osg::Group* root){
    osg::PositionAttitudeTransform* joint_forward = new osg::PositionAttitudeTransform();
    OSGSegment* seg = new OSGSegment(joint_forward, kdl_seg);

    joint_forward->setUserData( seg );
    joint_forward->setUpdateCallback(new OSGSegmentCallback);

    root->addChild(joint_forward);

    //Attach one visual to joint
    if (urdf_link.visual_array.size() == 0)
    {
        boost::shared_ptr<urdf::Visual> visual = urdf_link.visual;
        seg->attachVisual(visual, rootPrefix);
    }
    //Attach several visuals to joint
    else
    {
         std::vector<boost::shared_ptr<urdf::Visual> > visual_array = urdf_link.visual_array;
         seg->attachVisuals(visual_array, rootPrefix);
    }

    return joint_forward;
}

osg::Node* RobotModel::makeOsg( boost::shared_ptr<urdf::ModelInterface> urdf_model ){
    //Parse also to KDL
    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(*urdf_model, tree);

    //
    // Here we perform a full traversal throu the kinematic tree
    // hereby we go depth first
    //
    boost::shared_ptr<const urdf::Link> urdf_link; //Temp Storage for current urdf link
    KDL::Segment kdl_segment; //Temp Storage for urrent KDL link (same as URDF, but already parsed to KDL)
    osg::Node* hook = 0; //Node (from previous segment) to hook up next segment to

    std::vector<boost::shared_ptr<const urdf::Link> > link_buffer; //Buffer for links we still need to visit
    //used after LIFO principle
    std::vector<osg::Node*> hook_buffer;                  //Same as above but for hook. The top most
    //element here corresponds to the hook of the
    //previous depth level in the tree.
    link_buffer.push_back(urdf_model->getRoot()); //Initialize buffers with root
    hook_buffer.push_back(root_);
    while(!link_buffer.empty()){
        //get current node in buffer
        urdf_link = link_buffer.back();
        link_buffer.pop_back();

        //expand node
        link_buffer.reserve(link_buffer.size() + std::distance(urdf_link->child_links.begin(), urdf_link->child_links.end()));
        link_buffer.insert(link_buffer.end(), urdf_link->child_links.begin(), urdf_link->child_links.end());

        //create osg link
        hook = hook_buffer.back();
        hook_buffer.pop_back();
        kdl_segment = tree.getSegment(urdf_link->name)->second.segment;
        osg::Node* new_hook = makeOsg2(kdl_segment,
                                       *urdf_link, hook->asGroup());

        //Also store names of links and joints
        segmentNames_.push_back(kdl_segment.getName());
        if(kdl_segment.getJoint().getType() != KDL::Joint::None)
            jointNames_.push_back(kdl_segment.getJoint().getName());

        //Append hooks
        for(uint i=0; i<urdf_link->child_links.size(); i++)
            hook_buffer.push_back(new_hook);
    }
    return root_;
}

osg::Node* RobotModel::load(QString path){

    root_ = loadEmptyScene()->asGroup();

    std::ifstream t( path.toStdString().c_str() );
    std::string xml_str((std::istreambuf_iterator<char>(t)),
	                     std::istreambuf_iterator<char>());
    //Parse urdf
    boost::shared_ptr<urdf::ModelInterface> model = urdf::parseURDF( xml_str );
    rootPrefix = QDir(QFileInfo(path).absoluteDir());
    if (!model)
        return NULL;
    return makeOsg(model);
}

OSGSegment* RobotModel::getSegment(std::string name)
{
    osg::Node* node = findNamedNode(name, root_);
    if(!node)
        return 0;

    OSGSegment* jnt = dynamic_cast<OSGSegment*>(node->getUserData());
    if(!jnt)
        return 0;
    return jnt;
}

bool RobotModel::setJointState(std::string jointName, double jointVal)
{
    osg::Node* node = findNamedNode(jointName, root_);
    if(!node)
        return false;

    OSGSegment* jnt = dynamic_cast<OSGSegment*>(node->getUserData());
    jnt->setJointPos(jointVal);
    return true;
}

bool RobotModel::setJointState(const std::map<std::string, double>& jointVals)
{
    for (std::map<std::string, double>::const_iterator it=jointVals.begin();
         it!=jointVals.end(); ++it){
        osg::Node* node = findNamedNode( it->first, root_);
        if(!node)
            return false;

        OSGSegment* jnt = dynamic_cast<OSGSegment*>(node->getUserData());
        jnt->setJointPos(it->second);
    }
    return true;
}

bool RobotModel::toggleHighlight(std::string name)
{
    OSGSegment* seg = getSegment(name);
    if(!seg)
        return false;
    seg->toggleSelected();
    return true;
}
