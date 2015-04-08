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
#include "osg/ShapeDrawable"
#include "osg/TextureRectangle"
#include "osg/TexMat"
#include "OSGHelpers.hpp"
#include <base/Logging.hpp>
#include <QFileInfo>
#include <osg/ShadeModel>
#include <osgUtil/SmoothingVisitor>

#include <fstream>
#include <streambuf>

#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/PluginQuery>

OSGSegment::OSGSegment(KDL::Segment seg)
{
    isSelected_=false;
    seg_ = seg;
    jointPos_ = 0;
    label_ = 0;
    visual_ = 0;
    post_transform_ = new osg::Group();
    toTipOsg_ = new osg::PositionAttitudeTransform();
    toTipOsg_->addChild(post_transform_);
    toTipOsg_->setName(seg_.getJoint().getName());
    post_transform_->setUserData( this );
    post_transform_->setUpdateCallback(new OSGSegmentCallback);

    toTipOsg_->setUserData(this);

    setupTextLabel();
    updateJoint();
}

osg::ref_ptr<osg::Group> OSGSegment::getGroup() const{
    return post_transform_;
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
    post_transform_->addChild(to_visual);

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
    }
    else if(visual && visual->geometry->type == urdf::Geometry::BOX){
        urdf::Box* box = dynamic_cast<urdf::Box*>(visual->geometry.get());
        osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Box(osg::Vec3d(0,0,0), box->dim.x, box->dim.y, box->dim.z));
        osg_visual = new osg::Geode;
        osg_visual->asGeode()->addDrawable(drawable);
    }
    else if(visual && visual->geometry->type == urdf::Geometry::CYLINDER){
        urdf::Cylinder* cylinder = dynamic_cast<urdf::Cylinder*>(visual->geometry.get());
        osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0,0,0), cylinder->radius, cylinder->length));

        osg_visual = new osg::Geode;
        osg_visual->asGeode()->addDrawable(drawable);
    }
    else if(visual && visual->geometry->type == urdf::Geometry::SPHERE){
        urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(visual->geometry.get());
         osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3d(0,0,0), sphere->radius));

        osg_visual = new osg::Geode;
        osg_visual->asGeode()->addDrawable(drawable);
    }
    else
    {
        osg_visual = new osg::Geode;
    }

    //Set material
    if(visual){
        if(visual->material){
            osg::ref_ptr<osg::StateSet> nodess = osg_visual->getOrCreateStateSet();
            nodess->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

            osg::ref_ptr<osg::Material> nodematerial = new osg::Material;

            std::string filename = visual->material->texture_filename;
            if(filename != ""){
                QString qfilename = QString::fromStdString(visual->material->texture_filename);
                if (QFileInfo(qfilename).isRelative())
                    filename = baseDir.absoluteFilePath(qfilename).toStdString();
                osg::ref_ptr<osg::Image> texture_img = osgDB::readImageFile(filename);
                if(!texture_img){
                    std::stringstream ss;
                    ss << "Could not load texture from file '"<<visual->material->texture_filename<<"'.";
                    throw(std::runtime_error(ss.str()));
                }
                osg::ref_ptr<osg::TextureRectangle> texture_rect = osg::ref_ptr<osg::TextureRectangle>(new osg::TextureRectangle(texture_img));
                osg::ref_ptr<osg::TexMat> texmat = new osg::TexMat;
                texmat->setScaleByTextureRectangleSize(true);
                nodess->setTextureAttributeAndModes(0, texture_rect, osg::StateAttribute::ON);
                nodess->setTextureAttributeAndModes(0, texmat, osg::StateAttribute::ON);
            }
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

    //The smooting visitor calculates surface normals for correct shading
    osgUtil::SmoothingVisitor sv;
    osg_visual->accept(sv);

    to_visual->addChild(osg_visual);
    osg_visual->setUserData(this);
    osg_visual->setName(seg_.getName());
    visual_ = osg_visual->asGeode();
}

void OSGSegment::attachVisual(sdf::ElementPtr sdf_visual, QDir baseDir){

    osg::PositionAttitudeTransform* to_visual = new osg::PositionAttitudeTransform();
    sdf_pose_to_osg(sdf_visual->GetElement("pose"), *to_visual);

//    toTipOsg_->addChild(to_visual);
    post_transform_->addChild(to_visual);

    osg::Node* osg_visual = 0;
    if (sdf_visual->HasElement("geometry")){
        sdf::ElementPtr sdf_geometry  = sdf_visual->GetElement("geometry");
        sdf::ElementPtr sdf_geom_elem = sdf_geometry->GetFirstElement();

        if (sdf_geom_elem->GetName() == "box"){
            osg::Vec3f size;
            sdf_size_to_osg(sdf_geom_elem->GetElement("size"), size);
            osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0,0,0), size.x(), size.y(), size.z()));
            osg_visual = new osg::Geode;
            osg_visual->asGeode()->addDrawable(drawable);
        }
        else if (sdf_geom_elem->GetName() == "cylinder"){
            double radius = sdf_geom_elem->GetElement("radius")->Get<double>();
            double length = sdf_geom_elem->GetElement("length")->Get<double>();
            osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0,0,0), radius, length));
            osg_visual = new osg::Geode;
            osg_visual->asGeode()->addDrawable(drawable);
        }
        else if (sdf_geom_elem->GetName() == "sphere"){
            double radius = sdf_geom_elem->GetElement("radius")->Get<double>();
            osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3d(0,0,0), radius));
            osg_visual = new osg::Geode;
            osg_visual->asGeode()->addDrawable(drawable);
        }
        else if  (sdf_geom_elem->GetName() == "mesh"){
            osg::Vec3 scale;
            sdf_scale_to_osg(sdf_geom_elem->GetElement("scale"), scale);

            to_visual->setScale(scale);

            std::string uri = sdf_geom_elem->GetElement("uri")->Get<std::string>();

            std::string model_prefix = "model://";

            std::string filename = "";

            if(uri.compare(0, model_prefix.length(), model_prefix) == 0){
                filename = uri.substr(model_prefix.length());
            }
            else {
                filename = uri;
            }

            QString qfilename = QString::fromStdString(filename);

            if (QFileInfo(qfilename).isRelative()){
                QDir modelPaths = baseDir;
                modelPaths.cdUp();
                filename = modelPaths.absoluteFilePath(qfilename).toStdString();
            }

            osg_visual = osgDB::readNodeFile(filename);

            if (!osg_visual) {
                LOG_ERROR("OpenSceneGraph did not succees loading the mesh file %s.", filename.c_str());
                throw std::runtime_error("Error loading mesh file.");
            }

        }
        else {
            osg_visual = new osg::Geode;
            LOG_ERROR("It was not possible to find a geometry type compatible.");
        }
    }

    if (sdf_visual->HasElement("material")){

        osg::ref_ptr<osg::Material> nodematerial = new osg::Material;

        nodematerial->setSpecular(osg::Material::FRONT,osg::Vec4(0.2,
                                                                 0.2,
                                                                 0.2,
                                                                 1));

        sdf::ElementPtr sdf_material = sdf_visual->GetElement("material");

        if (sdf_material->HasElement("ambient")){
            osg::Vec4 ambient;
            sdf_color_to_osg(sdf_material->GetElement("ambient"), ambient);
            nodematerial->setAmbient(osg::Material::FRONT,ambient);
        }

        if (sdf_material->HasElement("diffuse")){
            osg::Vec4 diffuse;
            sdf_color_to_osg(sdf_material->GetElement("diffuse"), diffuse);
            nodematerial->setDiffuse(osg::Material::FRONT,diffuse);
        }

        if (sdf_material->HasElement("specular")){
            osg::Vec4 specular;
            sdf_color_to_osg(sdf_material->GetElement("specular"), specular);
            nodematerial->setSpecular(osg::Material::FRONT, specular);

        }

        if (sdf_material->HasElement("emissive")){
            osg::Vec4 emissive;
            sdf_color_to_osg(sdf_material->GetElement("emissive"), emissive);
            nodematerial->setEmission(osg::Material::FRONT, emissive);
        }

        osg::ref_ptr<osg::StateSet> nodess = osg_visual->getOrCreateStateSet();
        nodess->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
        nodess->setAttribute(nodematerial.get());
    }

    //The smooting visitor calculates surface normals for correct shading
    osgUtil::SmoothingVisitor sv;
    osg_visual->accept(sv);

    to_visual->addChild(osg_visual);
    osg_visual->setUserData(this);
    osg_visual->setName(seg_.getName());
    visual_ = osg_visual->asGeode();
}

void OSGSegment::attachVisuals(std::vector<sdf::ElementPtr> &visual_array, QDir prefix){

    std::vector<sdf::ElementPtr>::iterator itr = visual_array.begin();
    std::vector<sdf::ElementPtr >::iterator itr_end = visual_array.end();

    for(itr = visual_array.begin(); itr != itr_end; ++itr)
    {
        sdf::ElementPtr visual = *itr;
        attachVisual(visual, prefix);
    }
}


void OSGSegment::removeLabel(){
    if(label_)
        post_transform_->removeChild(label_);
    label_ = 0;
}

void OSGSegment::attachLabel(std::string name, std::string filepath){
    if(label_)
        removeLabel();

    osg::ref_ptr<osg::Geode> geode = osg::ref_ptr<osg::Geode>(new osg::Geode());
    osg::ref_ptr<osg::Geometry> geometry = osg::ref_ptr<osg::Geometry>(new osg::Geometry());

    osg::ref_ptr<osg::Vec3Array> vertices = osg::ref_ptr<osg::Vec3Array>(new osg::Vec3Array);
    vertices->push_back (osg::Vec3 (0, 0, 0.0));

    geometry->setVertexArray (vertices);

    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));

    geode->addDrawable(geometry);
    osg::ref_ptr<osg::StateSet> set = geode->getOrCreateStateSet();

    /// Setup the point sprites
    osg::ref_ptr<osg::PointSprite> sprite = osg::ref_ptr<osg::PointSprite>(new osg::PointSprite());
    set->setTextureAttributeAndModes(0, sprite, osg::StateAttribute::ON);

    /// Give some size to the points to be able to see the sprite
    osg::ref_ptr<osg::Point> point = osg::ref_ptr<osg::Point>(new osg::Point());
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
    osg::ref_ptr<osg::Texture2D> tex = osg::ref_ptr<osg::Texture2D>(new osg::Texture2D());
    osg::ref_ptr<osg::Image> image = osgDB::readImageFile(filepath);
    image->flipVertical();
    tex->setImage(image);

    set->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON);

    post_transform_->addChild(geode);

    geode->setName(name);
    geode->setUserData(this);

    label_ = geode;
}

void OSGSegment::setupTextLabel(){
    text_label_ = osg::ref_ptr<osgText::Text>(new osgText::Text());
    text_label_geode_ = osg::ref_ptr<osg::Geode>(new osg::Geode());
    osg::ref_ptr<osg::StateSet> set = text_label_geode_->getOrCreateStateSet();
    /// Disable depth test and Lighting
    set->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    set->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    text_label_geode_->addDrawable(text_label_);

    //Text should be rather small and be always readable on the screen
    text_label_->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
    text_label_->setCharacterSize(20);

    osgText::Font* font = osgText::Font::getDefaultFont();
    font->setMinFilterHint(osg::Texture::NEAREST); // aliasing when zoom out, this doesnt look so ugly because text is small
    font->setMagFilterHint(osg::Texture::NEAREST); // aliasing when zoom in
    text_label_->setFont(font);

    text_label_->setAxisAlignment(osgText::Text::SCREEN);

    // Set the text to render with alignment anchor and bounding box around it:
    text_label_->setDrawMode(osgText::Text::TEXT |
                           osgText::Text::ALIGNMENT);
    text_label_->setAlignment(osgText::Text::CENTER_TOP);
    text_label_->setPosition( osg::Vec3(0,0,0) );
    text_label_->setColor( osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );

    text_label_->setBackdropType(osgText::Text::OUTLINE);
    text_label_->setBackdropColor(osg::Vec4(0, 0, 0, 1.0f));
}

void OSGSegment::attachTextLabel(std::string text){
    if(text == ""){
        text = seg_.getName();
    }
    text_label_->setText(text);
    post_transform_->addChild(text_label_geode_);
}

void OSGSegment::removeTextLabel()
{
    post_transform_->removeChild(text_label_geode_);
}

bool OSGSegment::toggleSelected(){
    isSelected_ = !isSelected_;

    if(!visual_){
        std::clog << "Tried to highlight " << seg_.getName() << ", but it has no visual." << std::endl;
        return false;
    }
    osg::ref_ptr<osg::Group> parent = visual_->getParent(0);

    if(isSelected_){
        osg::ref_ptr<osgFX::Outline> scribe = osg::ref_ptr<osgFX::Outline>(new osgFX::Outline());
        scribe->setWidth(1);
        scribe->setColor(osg::Vec4(1,0,0,1));
        scribe->addChild(visual_);
        parent->replaceChild(visual_, scribe);
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
    root_ = osg::ref_ptr<osg::Group>(new osg::Group());
    original_root_ = osg::ref_ptr<osg::Group>(new osg::Group());
    loadFunctions["urdf"] = &RobotModel::loadURDF;
    loadFunctions["sdf"] = &RobotModel::loadSDF;
    loadEmptyScene();
}

osg::ref_ptr<osg::Node> RobotModel::loadEmptyScene(){
    original_root_->removeChildren(0, original_root_->getNumChildren());
    root_->removeChildren(0, root_->getNumChildren());
    jointNames_.clear();
    segmentNames_.clear();
    return root_;
}

osg::ref_ptr<osg::Node> RobotModel::makeOsg2(KDL::Segment kdl_seg, urdf::Link urdf_link, osg::ref_ptr<osg::Group> root){
    osg::ref_ptr<OSGSegment> seg = osg::ref_ptr<OSGSegment>(new OSGSegment(kdl_seg));
    root->addChild(seg->toTipOsg_);

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

    return seg->getGroup();
}

osg::Node* RobotModel::makeOsg2(KDL::Segment kdl_seg, sdf::ElementPtr sdf_link, sdf::ElementPtr sdf_parent_link, osg::Group* root){

    std::vector<sdf::ElementPtr> visuals;

    if (sdf_link->HasElement("visual")){

        sdf::ElementPtr visualElem= sdf_link->GetElement("visual");
        while (visualElem){
            visuals.push_back(visualElem);
            visualElem = visualElem->GetNextElement("visual");
        }
    }


    //transform to parent link
    //gazebo uses model position as reference. It is necessary convert to relative position from parent
    //to create the same behavior of gazebo it is necessary transform link position to parent position
    osg::PositionAttitudeTransform *to_parent_link = new osg::PositionAttitudeTransform();

    //set the link position using relative position. the reference position is the parent.
    osg::PositionAttitudeTransform *to_link = new osg::PositionAttitudeTransform();

    osg::ref_ptr<OSGSegment> seg = osg::ref_ptr<OSGSegment>(new OSGSegment(kdl_seg));

    //if link doesn't have a parent then your position is relative to model
    if (!sdf_parent_link){
        if (sdf_link->HasElement("pose")){
            sdf_pose_to_osg(sdf_link->GetElement("pose"), *to_link);
        }
    }
    else { //link has a parent. your position is relative to the parent position

        osg::Vec3 link_position;
        osg::Quat link_rotation;
        osg::Vec3 link_parent_position;
        osg::Quat link_parent_rotation;

        //get link position and rotation
        if (sdf_link->HasElement("pose")){
            sdf_pose_to_osg(sdf_link->GetElement("pose"), link_position, link_rotation);
        }

        //get parent link position and rotation
        if (sdf_parent_link->HasElement("pose")){
            sdf_pose_to_osg(sdf_parent_link->GetElement("pose"), link_parent_position, link_parent_rotation);
        }

        //revert to the parent rotation
        to_parent_link->setAttitude(link_parent_rotation.inverse());
        //rotate link
        to_link->setAttitude(link_rotation);
        //set relative position
        to_link->setPosition(link_position-link_parent_position);
    }

    to_link->addChild(seg->toTipOsg_);
    to_parent_link->addChild(to_link);

    root->addChild(to_parent_link);

    if (visuals.size() > 0)
    {
        seg->attachVisuals(visuals, rootPrefix);
    }
    else {
        //in sdf files it is possible to create links without visuals
        //the code below create a representation of visual to attach the segment
        //to keep the compatibility create the nodes to_visual and osg_visual
        //the segment is attached in the osg_visual
        osg::PositionAttitudeTransform* to_visual = new osg::PositionAttitudeTransform();
        seg->post_transform_->addChild(to_visual);

        //create an invisible node only to represent the visual information and keep the compatibility
        osg::Node *node = new osg::Geode();
        node->setUserData(seg.get());
        node->setName(kdl_seg.getName());
        to_visual->addChild(node);
        seg->visual_ = node->asGeode();

    }

    return seg->getGroup();
}

osg::ref_ptr<osg::Node> RobotModel::makeOsg( boost::shared_ptr<urdf::ModelInterface> urdf_model ){
    //Parse also to KDL
    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(*urdf_model, tree);

    //
    // Here we perform a full traversal throu the kinematic tree
    // hereby we go depth first
    //
    boost::shared_ptr<const urdf::Link> urdf_link; //Temp Storage for current urdf link
    KDL::Segment kdl_segment; //Temp Storage for urrent KDL link (same as URDF, but already parsed to KDL)
    osg::ref_ptr<osg::Node> hook = 0; //Node (from previous segment) to hook up next segment to

    std::vector<boost::shared_ptr<const urdf::Link> > link_buffer; //Buffer for links we still need to visit
    //used after LIFO principle
    std::vector<osg::ref_ptr<osg::Node> > hook_buffer;                  //Same as above but for hook. The top most
    //element here corresponds to the hook of the
    //previous depth level in the tree.
    link_buffer.push_back(urdf_model->getRoot()); //Initialize buffers with root
    hook_buffer.push_back(original_root_);
    original_root_name_ = urdf_model->getRoot()->name;
    while(!link_buffer.empty()){
        //get current node in buffer
        urdf_link = link_buffer.back();
        link_buffer.pop_back();

        //FIXME: This is hacky solution to prevent from links being added twice. There should be a better one
        if(std::find (segmentNames_.begin(), segmentNames_.end(), urdf_link->name) != segmentNames_.end())
            continue;

        //expand node
        link_buffer.reserve(link_buffer.size() + std::distance(urdf_link->child_links.begin(), urdf_link->child_links.end()));
        link_buffer.insert(link_buffer.end(), urdf_link->child_links.begin(), urdf_link->child_links.end());

        //create osg link
        hook = hook_buffer.back();
        hook_buffer.pop_back();
        kdl_segment = tree.getSegment(urdf_link->name)->second.segment;
        osg::ref_ptr<osg::Node> new_hook = makeOsg2(kdl_segment,
                                       *urdf_link, hook->asGroup());

        //Also store names of links and joints
        segmentNames_.push_back(kdl_segment.getName());
        if(kdl_segment.getJoint().getType() != KDL::Joint::None)
            jointNames_.push_back(kdl_segment.getJoint().getName());

        //Append hooks
        for(uint i=0; i<urdf_link->child_links.size(); i++)
            hook_buffer.push_back(new_hook);
    }
    relocateRoot(urdf_model->getRoot()->name);

    return root_;
}

osg::Node* RobotModel::makeOsg( sdf::ElementPtr sdf_model )
{
    std::string model_name = sdf_model->GetAttribute("name")->GetAsString();
    std::map<std::string, sdf::ElementPtr> links = loadSdfModelLinks(sdf_model);

    KDL::Tree tree;
    kdl_parser::treeFromSdfModel(sdf_model, tree);

    KDL::SegmentMap::const_iterator root = tree.getRootSegment();

    std::vector<KDL::SegmentMap::const_iterator> segment_buffer;
    segment_buffer.insert(segment_buffer.end(), root->second.children.begin(), root->second.children.end());

    std::vector<osg::Node*> hook_buffer;
    for(uint i = 0; i < root->second.children.size(); i++){
        hook_buffer.push_back(original_root_);
    }

    original_root_name_ = root->second.segment.getName();

    osg::Node* hook = 0;
    while (!segment_buffer.empty()){

        KDL::SegmentMap::const_iterator it = segment_buffer.back();
        KDL::SegmentMap::const_iterator parent_it = it->second.parent;

        segment_buffer.pop_back();
        KDL::Segment kdl_segment = it->second.segment;
        KDL::Segment kdl_parent_segment = parent_it->second.segment;

        if (it->second.children.size() > 0){
            segment_buffer.insert(segment_buffer.end(), it->second.children.begin(), it->second.children.end());
        }

        std::map<std::string, sdf::ElementPtr>::iterator link_itr = links.find(kdl_segment.getName());
        std::map<std::string, sdf::ElementPtr>::iterator link_parent_itr = links.find(kdl_parent_segment.getName());

        //check if link exists
        if (link_itr == links.end())
          throw std::runtime_error("Internal error: expected to find a SDF link called " + kdl_segment.getName());

        sdf::ElementPtr sdf_link = link_itr->second;
        sdf::ElementPtr sdf_parent_link;

        //if did not find the parent, then the child it is a root link
        if (link_parent_itr != links.end()) {
            sdf_parent_link = link_parent_itr->second;
        }

        //create osg link
        hook = hook_buffer.back();
        hook_buffer.pop_back();

        osg::Node* new_hook = makeOsg2(kdl_segment, sdf_link, sdf_parent_link, hook->asGroup());

        //Append hooks
        for(uint i = 0; i < it->second.children.size(); i++){
            hook_buffer.push_back(new_hook);
        }

        segmentNames_.push_back(kdl_segment.getName());
        if(kdl_segment.getJoint().getType() != KDL::Joint::None)
            jointNames_.push_back(kdl_segment.getJoint().getName());

    }

    /**
     * in the SDF files it is not necessary one joint per two links
     * in the SDF files it is possible to create a file with multiple links without joints
     * original_root_ is not a segment, is only used to store the segments
     * if the links in the SDF doesn't have a joint, then this segment will be a direct child of original_root_
     * if all links doen't have a joint, then all segments will be children direct of original_root_
     */
    root_->addChild(original_root_);
    return root_;
}

osg::ref_ptr<osg::Node> RobotModel::load(QString path){

    loadPlugins();
    loadEmptyScene();

    QString suffix = QFileInfo(path).suffix().toLower();

    /*
     * call the function based in the file suffix
     */
    if (loadFunctions.contains(suffix)){
        //call member-to-pointer function
        return (this->*loadFunctions[suffix])(path);
    }
    else {
        LOG_ERROR("the %s type of file is not supported .", suffix.toStdString().c_str());
        throw std::runtime_error("Internal error: the " + suffix.toStdString() + " type of file is not supported .");
    }

    return new osg::Group();
}

osg::Node* RobotModel::loadURDF(QString path)
{
    std::ifstream t( path.toStdString().c_str() );
    std::string xml_str((std::istreambuf_iterator<char>(t)),
                       std::istreambuf_iterator<char>());

    rootPrefix = QDir(QFileInfo(path).absoluteDir());

    //Parse urdf
    boost::shared_ptr<urdf::ModelInterface> model = urdf::parseURDF( xml_str );
    if (!model)
        return NULL;

    return makeOsg(model);
}

osg::Node* RobotModel::loadSDF(QString path)
{
    rootPrefix = QDir(QFileInfo(path).absoluteDir());

    sdf::SDFPtr sdf(new sdf::SDF);

    if (!sdf::init(sdf)){
        LOG_ERROR("unable to initialize sdf.");
        return NULL;
    }

    if (!sdf::readFile(path.toStdString(), sdf)){
        LOG_ERROR("unabled to read sdf file %s.", path.toStdString().c_str());
        return NULL;
    }

    if (!sdf->root->HasElement("model")){
        LOG_ERROR("the <model> tag not exists");
        return NULL;
    }

    return makeOsg(sdf->root->GetElement("model"));
}

void RobotModel::loadPlugins()
{
    osgDB::FileNameList plugins = osgDB::listAllAvailablePlugins();
    for(osgDB::FileNameList::iterator itr = plugins.begin();
        itr != plugins.end();
        ++itr)
    {
        osgDB::ReaderWriterInfoList infoList;
        osgDB::queryPlugin(*itr, infoList);
    }
}

std::map<std::string, sdf::ElementPtr> RobotModel::loadSdfModelLinks(sdf::ElementPtr sdf_model)
{
    std::map<std::string, sdf::ElementPtr> links;

    if (sdf_model->HasElement("link")){
        sdf::ElementPtr linkElem = sdf_model->GetElement("link");
        while (linkElem){
            std::string link_name = linkElem->Get<std::string>("name");
            links.insert(std::make_pair<std::string, sdf::ElementPtr>(link_name, linkElem));
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

    osg::ref_ptr<OSGSegment> jnt = dynamic_cast<OSGSegment*>(node->getUserData());
    if(!jnt){
        std::cerr << "Could not retrieve user data from node "<<name<<std::endl;
    }
    assert(jnt);
    return jnt;
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

bool RobotModel::setJointState(std::string jointName, double jointVal)
{
    osg::ref_ptr<osg::Node> node = findNamedNode(jointName, original_root_);
    if(!node)
        return false;

    osg::ref_ptr<OSGSegment> jnt = dynamic_cast<OSGSegment*>(node->getUserData());
    jnt->setJointPos(jointVal);
    return true;
}

bool RobotModel::setJointState(const std::map<std::string, double>& jointVals)
{
    for (std::map<std::string, double>::const_iterator it=jointVals.begin();
         it!=jointVals.end(); ++it){
        osg::ref_ptr<osg::Node> node = findNamedNode( it->first, original_root_);
        if(!node)
            return false;

        osg::ref_ptr<OSGSegment> jnt = dynamic_cast<OSGSegment*>(node->getUserData());
        jnt->setJointPos(it->second);
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

osg::Matrixd RobotModel::getRelativeTransform(std::string source_segment, std::string target_segment)
{
    return getTransformBetweenNodes(getSegment(source_segment)->post_transform_, getSegment(target_segment)->post_transform_);
}
