#include "RobotModel.h"
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
#include <urdf/collada_parser/collada_parser.h>
#include <base/Logging.hpp>


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

void OSGSegment::updateJoint(){
    toTipKdl_ = seg_.pose(jointPos_);
    kdl_to_osg(toTipKdl_, *toTipOsg_);
}

void OSGSegment::attachVisual(boost::shared_ptr<urdf::Visual> visual)
{
    osg::PositionAttitudeTransform* to_visual = new osg::PositionAttitudeTransform();
    urdf_to_osg(visual->origin, *to_visual);
    toTipOsg_->addChild(to_visual);
    toTipOsg_->setName(seg_.getJoint().getName());

    if(visual->geometry->type == urdf::Geometry::MESH){
        urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(visual->geometry.get());
        to_visual->setScale(urdf_to_osg(mesh->scale));
        osg::Node* osg_visual = 0;

        std::string prefix = "package://";
        //Is define relative to package, use resource retriever from ros
        if(mesh->filename.compare(0, prefix.length(), prefix) == 0)
        {
#if 0
            resource_retriever::Retriever r;
            resource_retriever::MemoryResource resource;

            try
            {
                resource = r.get(mesh->filename);
            }
            catch (resource_retriever::Exception& e)
            {
                ROS_ERROR("Failed to retrieve file: %s", e.what());
                return;
            }

            int pos = mesh->filename.find_last_of('.');
            std::string suffix = mesh->filename.substr(pos);
            std::string fname = (std::string("/tmp/some_fancy_mesh_file")+suffix);

            FILE* f = fopen(fname.c_str(), "w");
            fwrite(resource.data.get(), resource.size, 1, f);
            fclose(f);

            osg_visual = osgDB::readNodeFile(fname);
#endif
            throw(std::runtime_error("URDF files points to ROS resource. This is not supported. Change file to use absolute or relative files."));
        }
        else{ //Assume its an absolute path or relative to execution folder
            prefix = "file://";
            std::string filename = "";
            if(mesh->filename.compare(0, prefix.length(), prefix) == 0){
                filename = mesh->filename.substr(prefix.length());
            }
            else
                filename = mesh->filename;
            LOG_DEBUG("Trying to load mesh file %s", filename.c_str());
            osg_visual = osgDB::readNodeFile(filename);
            if(!osg_visual){
                LOG_ERROR("OpenSceneGraph did not succees loading the mesh file %s.", filename.c_str());
                throw std::runtime_error("Error loading mesh file.");
#if 0

                std::stringstream ss;
                ss << "assimp export " << filename << " /tmp/vizkit_robotmodel_mesh_conversion.obj";
                if(system(ss.str().c_str()) == 0){
                    osg_visual = osgDB::readNodeFile("/tmp/vizkit_robotmodel_mesh_conversion.obj");
                }
                else{
                    LOG_ERROR("Conversion failed");
                }
#endif
            }
        }
        if(!osg_visual){
            LOG_ERROR("Unecpected error loading mesh file %s", mesh->filename.c_str());
            throw(std::runtime_error("Couldn't load mesh file."));
        }

        if(visual->material){
            osg::ref_ptr<osg::StateSet> nodess = osg_visual->getOrCreateStateSet();
            osg::ref_ptr<osg::Material> nodematerial = new osg::Material;
            //Specifying the yellow colour of the object
            nodematerial->setDiffuse(osg::Material::FRONT,osg::Vec4(visual->material->color.r*mesh->scale.x,
                                                                    visual->material->color.g*mesh->scale.x,
                                                                    visual->material->color.b*mesh->scale.x,
                                                                    visual->material->color.a));
            nodematerial->setSpecular(osg::Material::FRONT,osg::Vec4(1.*mesh->scale.x,
                                                                     1.*mesh->scale.x,
                                                                     1.*mesh->scale.x,
                                                                     1));

            //Attaching the newly defined state set object to the node state set
            nodess->setAttribute(nodematerial.get());
        }

        to_visual->addChild(osg_visual);
        osg_visual->setUserData(this);
        osg_visual->setName(seg_.getName());
        visual_ = osg_visual->asGeode();
    }
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

    //Attach visual to joint
    boost::shared_ptr<urdf::Visual> visual = urdf_link.visual;
    if(visual){
        seg->attachVisual(visual);
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

osg::Node* RobotModel::loadCollada(QString path){

    root_ = loadEmptyScene()->asGroup();
    //Read the urdf file
    std::ifstream fin;
    std::stringstream xml;
    fin.open(path.toLatin1().data());
    xml << fin.rdbuf();
    fin.close();
    std::string xml_str = xml.str();

    //Parse urdf
    boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseCollada(xml_str);
    return makeOsg(urdf_model);
}

osg::Node* RobotModel::loadURDF(QString path){
    root_ = loadEmptyScene()->asGroup();
    //Read the urdf file
    std::ifstream fin;
    std::stringstream xml;
    fin.open(path.toLatin1().data());
    xml << fin.rdbuf();
    fin.close();
    std::string xml_str = xml.str();

    //Parse urdf
    boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF(xml_str);
    return makeOsg(urdf_model);
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
