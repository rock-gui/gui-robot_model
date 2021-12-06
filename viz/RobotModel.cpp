#include "RobotModel.h"
#include "OSGHelpers.hpp"

#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <sstream>

#include <base-logging/Logging.hpp>
#include <QFileInfo>
#include <osg/ShadeModel>
#include <osgUtil/SmoothingVisitor>

#include <fstream>
#include <streambuf>
#include <locale>




RobotModel::RobotModel() {
    //Root is the entry point to the scene graph
    root_ = osg::ref_ptr<osg::Group>(new osg::Group());
    original_root_ = osg::ref_ptr<osg::Group>(new osg::Group());
    useVBO_ = RobotModel::getVBODefault();
    loadEmptyScene();
}

osg::ref_ptr<osg::Node> RobotModel::loadEmptyScene(){
    original_root_->removeChildren(0, original_root_->getNumChildren());
    root_->removeChildren(0, root_->getNumChildren());
    jointNames_.clear();
    segmentNames_.clear();
    return root_;
}

void RobotModel::makeOsg2(KDL::Segment kdl_seg, const std::vector<urdf::VisualSharedPtr>& visuals, const std::vector<urdf::CollisionSharedPtr>& collisions, OSGSegment& seg)
{
    seg.createVisuals(visuals, rootPrefix);
    seg.createCollisions(collisions, rootPrefix);
}

void RobotModel::makeOsg2(KDL::Segment const& kdl_seg, std::vector<sdf::ElementPtr> const& visuals, std::vector<sdf::ElementPtr> const& collisions, std::vector<sdf::ElementPtr> const& inertias, OSGSegment& seg){

    if (visuals.size() > 0)
    {
        seg.createVisuals(visuals, rootPrefix);
        seg.createCollisions(collisions, rootPrefix);
        seg.createInertias(inertias);
    }
    else {
        //in sdf files it is possible to create links without visuals
        //the code below create a representation of visual to attach the segment
        //to keep the compatibility create the nodes to_visual and osg_visual
        //the segment is attached in the osg_visual
        osg::PositionAttitudeTransform* to_visual = new osg::PositionAttitudeTransform();
        seg.post_transform_->addChild(to_visual);
    }
}

osg::ref_ptr<osg::Node> RobotModel::makeOsg( urdf::ModelInterfaceSharedPtr urdf_model ){
    //typedef urdf::LinkConstSharedPtr::iterator URDFLinkIterator;
    typedef KDL::SegmentMap::const_iterator SegmentIterator;

    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(*urdf_model, tree);

    std::list<urdf::LinkConstSharedPtr > queue;     //FIFO Buffer for links we still need to visit
    std::list<osg::ref_ptr<osg::Group> > osg_hooks; //The parent node for each element in queue

    queue.push_back(urdf_model->getRoot());
    osg_hooks.push_back(root_);

    //Travers through kinematic tree add each link to OSG
    while(!queue.empty()){
        //Retrieve current link and the parent osg_node from queue and remove them
        urdf::LinkConstSharedPtr urdf_link = queue.front();
        osg::ref_ptr<osg::Group> parent_osg = osg_hooks.front();
        queue.pop_front();
        osg_hooks.pop_front();

        //FIXME: This was to avoid adding the same segment multiple times. Is this still necessary?
        if(std::find (segmentNames_.begin(), segmentNames_.end(), urdf_link->name) != segmentNames_.end())
            continue;

        //Create OSG representation of the link
        KDL::Segment const& kdl = tree.getSegment(urdf_link->name)->second.segment;

        osg::ref_ptr<OSGSegment> seg = new OSGSegment(kdl, useVBO_);
        parent_osg->addChild(seg->toTipOsg_);

        std::vector<urdf::VisualSharedPtr> visuals = urdf_link->visual_array;
        if(urdf_link->visual)
            visuals.push_back(urdf_link->visual);

        std::vector<urdf::CollisionSharedPtr> collisions = urdf_link->collision_array;
        if(urdf_link->collision)
            collisions.push_back(urdf_link->collision);

        makeOsg2(kdl, visuals, collisions, *seg);

        //Set name to the main osg node so it can be found by name in the OSG graph
        osg::ref_ptr<osg::Group> osg = seg->getGroup();
        osg->setName(urdf_link->name);

        //Store names of links and joints for get*Names-member functions
        segmentNames_.push_back(kdl.getName());
        if(kdl.getJoint().getType() != KDL::Joint::None)
            jointNames_.push_back(kdl.getJoint().getName());

        //Fill queue with children of current link
        for(std::vector<urdf::LinkSharedPtr>::const_iterator child_it = urdf_link->child_links.begin(); child_it != urdf_link->child_links.end(); child_it++){
            queue.push_back(*child_it);
            osg_hooks.push_back(osg);
        }
    }

    // Add mimic joints
    for(auto j : urdf_model->joints_){
        if(j.second->mimic != nullptr)
            mimic_joints_[j.second->name] = MimicJoint(j.second->mimic->joint_name,
                                                       j.second->mimic->multiplier,
                                                       j.second->mimic->offset);
    }

    original_root_ = root_->getChild(0)->asGroup();
    original_root_name_ = tree.getRootSegment()->first;

    return root_;
}

osg::Node* RobotModel::makeOsg( sdf::ElementPtr sdf_model )
{
    typedef std::map<std::string, sdf::ElementPtr>::const_iterator SDFLinkIterator;
    typedef KDL::SegmentMap::const_iterator SegmentIterator;

    KDL::Tree tree;
    kdl_parser::treeFromSdfModel(sdf_model, tree);

    std::map<std::string, sdf::ElementPtr> sdf_links = loadSdfModelLinks(sdf_model);

    std::list<SegmentIterator> queue;
    std::list< osg::ref_ptr<osg::Group> > queue_osg;
    queue.push_back(tree.getRootSegment());
    queue_osg.push_back(root_);

    while (!queue.empty()){
        SegmentIterator it = queue.front();
        osg::ref_ptr<osg::Group> parent_osg = queue_osg.front();
        queue.pop_front();
        queue_osg.pop_front();

        KDL::Segment const& kdl = it->second.segment;

        osg::ref_ptr<OSGSegment> seg = new OSGSegment(kdl, useVBO_);
        parent_osg->addChild(seg->toTipOsg_);

        SDFLinkIterator sdf = sdf_links.find(kdl.getName());

        std::vector<sdf::ElementPtr> visuals;
        std::vector<sdf::ElementPtr> collisions;
        std::vector<sdf::ElementPtr> inertials;
        if (sdf != sdf_links.end()){
            sdf::ElementPtr sdf_link = sdf->second;
            sdf::ElementPtr visualElem = sdf_link->GetElement("visual");
            while (visualElem){
                visuals.push_back(visualElem);
                visualElem = visualElem->GetNextElement("visual");
            }
            sdf::ElementPtr collisionElem = sdf_link->GetElement("collision");
            while (collisionElem){
                collisions.push_back(collisionElem);
                collisionElem = collisionElem->GetNextElement("collision");
            }
            sdf::ElementPtr inertialElem = sdf_link->GetElement("inertial");
            while (inertialElem){
                inertials.push_back(inertialElem);
                inertialElem = inertialElem->GetNextElement("inertial");
            }
        }

        makeOsg2(kdl, visuals, collisions, inertials, *seg);

        //Set name to the main osg node so it can be found by name in the OSG graph
        osg::ref_ptr<osg::Group> osg = seg->getGroup();
        osg->setName(kdl.getName());

        std::vector<SegmentIterator> const& children =
            it->second.children;
        for (std::vector<SegmentIterator>::const_iterator child_it = children.begin();
                child_it != children.end(); ++child_it) {
            queue.push_back(*child_it);
            queue_osg.push_back(osg);
        }

        segmentNames_.push_back(kdl.getName());
        if(kdl.getJoint().getType() != KDL::Joint::None)
            jointNames_.push_back(kdl.getJoint().getName());
    }

    // Since we inject the KDL tree root, root_ is guaranteed to have only one
    // element, and that it is the segment for the tree root
    original_root_ = root_->getChild(0)->asGroup();
    original_root_name_ = tree.getRootSegment()->first;

    return root_;
}

osg::ref_ptr<osg::Node> RobotModel::load(QString path){

    return loadFromFile(path);
}

void RobotModel::attachCollisions(bool value)
{
    for (size_t i=0; i<segmentNames_.size(); i++){
        OSGSegment* seg = getSegment(segmentNames_[i]);
        if(value)
            seg->attachCollision();
        else
            seg->removeCollision();
    }
}

void RobotModel::attachInertias(bool value)
{
    for (size_t i=0; i<segmentNames_.size(); i++){
        OSGSegment* seg = getSegment(segmentNames_[i]);
        if(value)
            seg->attachInertia();
        else
            seg->removeInertia();
    }
}

void RobotModel::attachVisuals(bool value)
{
    for (size_t i=0; i<segmentNames_.size(); i++){
        OSGSegment* seg = getSegment(segmentNames_[i]);
        if(value)
            seg->attachVisual();
        else
            seg->removeVisual();
    }
}

osg::ref_ptr<osg::Node> RobotModel::loadFromFile(QString path, ROBOT_MODEL_FORMAT format)
{
    if (format == ROBOT_MODEL_AUTO)
    {
        kdl_parser::ROBOT_MODEL_FORMAT kdl_format = 
            kdl_parser::guessFormatFromFilename(path.toStdString());
        LOG_INFO("file %s guessed to be of type %s", path.toStdString().c_str(),
                kdl_parser::formatNameFromID(kdl_format));
        format = static_cast<ROBOT_MODEL_FORMAT>(kdl_format);
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        throw std::invalid_argument("cannot open " + path.toStdString() + " for reading");
    return loadFromString(QString::fromUtf8(file.readAll()), format, QFileInfo(path).absoluteDir().path());
}

osg::ref_ptr<osg::Node> RobotModel::loadFromString(QString xml, ROBOT_MODEL_FORMAT format, QString _rootPrefix)
{
    rootPrefix = QDir(_rootPrefix);

    loadEmptyScene();

    if (format == ROBOT_MODEL_URDF)
        return loadFromURDFString(xml);
    else if (format == ROBOT_MODEL_SDF)
        return loadFromSDFString(xml);
    else
        throw std::invalid_argument(std::string("unknown robot model format ") + kdl_parser::formatNameFromID(format));
}

osg::ref_ptr<osg::Node> RobotModel::loadFromURDFString(QString xml)
{
    urdf::ModelInterfaceSharedPtr model = urdf::parseURDF( xml.toStdString() );
    if (!model)
        return NULL;

    return makeOsg(model);
}

osg::ref_ptr<osg::Node> RobotModel::loadFromSDFString(QString xml)
{
    sdf::SDFPtr sdf(new sdf::SDF);
    if (!sdf::init(sdf)){
        LOG_ERROR("unable to initialize sdf.");
        return NULL;
    }
    std::string xml_s = xml.toStdString();
    if (!sdf::readString(xml_s, sdf))
    {
        LOG_ERROR("unable to load sdf from string %s.\n", xml_s.c_str());
        return NULL;
    }

    if (!sdf->Root()->HasElement("model")){
        LOG_ERROR("the <model> tag not exists");
        return NULL;
    }

    return makeOsg(sdf->Root()->GetElement("model"));
}

std::map<std::string, sdf::ElementPtr> RobotModel::loadSdfModelLinks(sdf::ElementPtr sdf_model)
{
    std::map<std::string, sdf::ElementPtr> links;
    std::string model_name = sdf_model->Get<std::string>("name");

    if (sdf_model->HasElement("link")){
        sdf::ElementPtr linkElem = sdf_model->GetElement("link");
        while (linkElem){
            std::string link_name = linkElem->Get<std::string>("name");
            links.insert(std::make_pair(model_name + "::" + link_name, linkElem));
            linkElem = linkElem->GetNextElement("link");
        }
    }

    return links;
}


osg::ref_ptr<OSGSegment> RobotModel::getSegment(std::string name)
{
    osg::ref_ptr<osg::Node> node = findNamedNode(name, original_root_);
    if(!node){
        std::cerr << "Could not find segment with name: " << name << std::endl;
        return 0;
    }
    return getSegment(node);
}

osg::ref_ptr<OSGSegment> RobotModel::getSegment(osg::ref_ptr<osg::Node> node)
{
    osg::ref_ptr<OSGSegment> jnt = dynamic_cast<OSGSegment*>(node->getUserData());
    if(!jnt){
        throw std::invalid_argument("Could not retrieve user data from node " + node->getName());
    }
    return jnt;
}

bool RobotModel::relocateRoot(osg::ref_ptr<osg::Node> group)
{
    osg::ref_ptr<OSGSegment> seg = getSegment(group);
    root_->removeChildren(0, root_->getNumChildren());
    root_->addChild(seg->post_transform_);
    return true;
}

bool RobotModel::relocateRoot(std::string name){
    osg::ref_ptr<OSGSegment> seg = getSegment(name);
    if(seg){
        root_->removeChildren(0, root_->getNumChildren());
        root_->addChild(seg->post_transform_);
    }
    else{
        std::cerr << "Segment " << name << " is unknown! Could not relocate root!" << std::endl;
        std::cerr << "Known segments:\n";
        std::vector<std::string> names = getSegmentNames();
        for(uint i=0; i<names.size(); i++)
            std::cerr << "  - "<<names[i]<<"\n";
        std::cerr<<std::endl;
        return false;
    }
    current_root_name_ = name;

    return true;
}

bool RobotModel::setJointPos(std::string jointName, double jointVal)
{
    osg::ref_ptr<osg::Node> node = findNamedNode(jointName, original_root_);
    if(!node)
        return false;

    osg::ref_ptr<OSGSegment> jnt = dynamic_cast<OSGSegment*>(node->getUserData());
    jnt->setJointPos(jointVal);
    return true;
}

bool RobotModel::setJointState(std::string jointName, double jointVal)
{
    // Checks if the joint is a mimic joint, then ignore the call
    if( mimic_joints_.find( jointName ) != mimic_joints_.end() ) {
        std::cerr << "Cannot set joint state for a mimic joint ( " <<
            jointName << " ) directly, ignoring value" << std::endl;
        return true;
    }
    
    // Loops through all the mimic joints to check it the joint needs to be mimiced
    for( std::map< std::string, MimicJoint >::const_iterator it = mimic_joints_.begin();
         it != mimic_joints_.end(); ++it ) {
        if( it->second.jointToMimic == jointName ) {
            if( !setJointPos( it->first, ( jointVal * it->second.multiplier ) + it->second.offset ) ) {
                return false;
            }
        }
    }

    // Sets the original joint
    return setJointPos( jointName, jointVal );
}

bool RobotModel::setJointState(const std::map<std::string, double>& jointVals)
{
    for (std::map<std::string, double>::const_iterator it=jointVals.begin();
         it!=jointVals.end(); ++it){

        if( !setJointState( it->first, it->second ) ) {
            return false;
        }
    }
    return true;
}

bool RobotModel::toggleHighlight(std::string name)
{
    osg::ref_ptr<OSGSegment> seg = getSegment(name);
    assert(seg);

    seg->toggleSelected();
    return seg->isSelected_;
}

bool RobotModel::getVBODefault()
{
    const char* env = getenv("ROCK_VIZ_USE_VBO");
    return (env && *env == '1');
}

void RobotModel::setUseVBO(bool flag)
{
    useVBO_ = flag;
}

bool RobotModel::getUseVBO() const
{
    return useVBO_;
}

osg::Matrixd RobotModel::getRelativeTransform(std::string source_segment, std::string target_segment)
{
    return getTransformBetweenNodes(getSegment(source_segment)->post_transform_, getSegment(target_segment)->post_transform_);
}
