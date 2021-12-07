#include "OSGSegment.h"
#include "OSGSegmentCallback.h"
#include "OSGHelpers.hpp"
#include "VBOVisitor.hpp"

#include <osg/Geometry>
#include <osg/Image>
#include <osg/ShapeDrawable>
#include <osg/TextureRectangle>
#include <osg/TexMat>
#include <osgUtil/Tessellator>
#include <osg/LineWidth>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/PluginQuery>
#include <osgUtil/Optimizer>

#include <Eigen/SVD>

#include <boost/bind/bind.hpp>

#include <base-logging/Logging.hpp>




OSGSegment::OSGSegment(KDL::Segment seg, bool useVBO)
{
    isSelected_=false;
    seg_ = seg;
    jointPos_ = 0;
    label_ = 0;
    visual_ = 0;
    useVBO_ = useVBO;
    post_transform_ = new osg::Group();
    toTipOsg_ = new osg::PositionAttitudeTransform();
    toTipOsg_->addChild(post_transform_);
    toTipOsg_->setName(seg_.getJoint().getName());
    post_transform_->setUserData( this );
    post_transform_->setUpdateCallback(new OSGSegmentCallback);

    toTipOsg_->setUserData(this);

    text_label_geode_ = createTextLabel(seg.getName(), text_label_);
    updateJoint();
}

std::map<std::string, osg::ref_ptr<osg::Node>> OSGSegment::meshCache;

void OSGSegment::clearMeshCache()
{
    meshCache.clear();
}

osg::ref_ptr<osg::Group> OSGSegment::getGroup() const{
    return post_transform_;
}

void OSGSegment::updateJoint(){
    toTipKdl_ = seg_.pose(jointPos_);
    kdl_to_osg(toTipKdl_, *toTipOsg_);
}


void OSGSegment::createCollisions(const std::vector<urdf::CollisionSharedPtr> &collision_array, QDir prefix)
{
    std::vector<urdf::CollisionSharedPtr >::const_iterator itr = collision_array.begin();
    std::vector<urdf::CollisionSharedPtr >::const_iterator itr_end = collision_array.end();

    for(itr = collision_array.begin(); itr != itr_end; ++itr)
    {
        urdf::CollisionSharedPtr elem = *itr;
        //Convert to visual, then call create visual and attach it as collision
        urdf::VisualSharedPtr visual = urdf::VisualSharedPtr(new urdf::Visual());
        visual->geometry = elem->geometry;
        visual->name = elem->name;
        visual->origin = elem->origin;
        visual->material = urdf::MaterialSharedPtr(new urdf::Material());
        visual->material->color.r = 0;
        visual->material->color.g = 1;
        visual->material->color.b = 0;
        visual->material->color.a = 0.7;
        collision_ = createVisual(visual, prefix);
    }
}

void OSGSegment::createCollisions(std::vector<sdf::ElementPtr> const& collision_array, QDir prefix)
{
    std::vector<sdf::ElementPtr >::const_iterator itr = collision_array.begin();
    std::vector<sdf::ElementPtr >::const_iterator itr_end = collision_array.end();

    for(itr = collision_array.begin(); itr != itr_end; ++itr)
    {
        sdf::ElementPtr elem = *itr;
        if(!elem->HasElement("material")){
            sdf::ElementPtr amb = sdf::ElementPtr(new sdf::Element());
            amb->SetName("ambient");
            amb->AddValue("color", "0.0 1.0 0.0 0.7", true);

            sdf::ElementPtr spec = sdf::ElementPtr(new sdf::Element());
            spec->SetName("specular");
            spec->AddValue("color", "0.0 1.0 0.0 0.7", true);

            sdf::ElementPtr diff = sdf::ElementPtr(new sdf::Element());
            diff->SetName("diffuse");
            diff->AddValue("color", "0.0 1.0 0.0 0.7", true);

            sdf::ElementPtr emi = sdf::ElementPtr(new sdf::Element());
            emi->SetName("emissive");
            emi->AddValue("color", "0.0 1.0 0.0 0.7", true);

            sdf::ElementPtr mat = sdf::ElementPtr(new sdf::Element());
            mat->SetName("material");
            mat->InsertElement(spec);
            mat->InsertElement(amb);
            mat->InsertElement(diff);
            mat->InsertElement(emi);

            elem->InsertElement(mat);
        }

        collision_ = createVisual(elem, prefix);
    }
}


void OSGSegment::createInertias(const std::vector<urdf::InertialSharedPtr> &inertia_array)
{
    std::vector<urdf::InertialSharedPtr>::const_iterator itr = inertia_array.begin();
    std::vector<urdf::InertialSharedPtr >::const_iterator itr_end = inertia_array.end();

    inertia_ = osg::ref_ptr<osg::Group>(new osg::Group());
    for(itr = inertia_array.begin(); itr != itr_end; ++itr)
    {
        urdf::InertialSharedPtr elem = *itr;
        osg::PositionAttitudeTransform* to_cog = new osg::PositionAttitudeTransform();
        urdf_to_osg(elem->origin, *to_cog);
        inertia_->addChild(to_cog);

        osg::ref_ptr<osg::Group> geode = make_ellispoid(elem->ixx, elem->ixy, elem->ixz,
                                                        elem->iyy, elem->iyz,
                                                        elem->izz,
                                                        elem->mass);
        to_cog->addChild(geode);
    }
}


void OSGSegment::createInertias(std::vector<sdf::ElementPtr> const& inertia_array)
{
    std::vector<sdf::ElementPtr >::const_iterator itr = inertia_array.begin();
    std::vector<sdf::ElementPtr >::const_iterator itr_end = inertia_array.end();


    inertia_ = osg::ref_ptr<osg::Group>(new osg::Group());
    for(itr = inertia_array.begin(); itr != itr_end; ++itr)
    {
        sdf::ElementPtr elem = *itr;
        //<inertial>
        //  <origin rpy="0.0 0.0 0.0" xyz="0.0079897 0.0 0.00019625"/>
        //  <mass value="1.1943"/>
        //  <inertia ixx="0.00089755" ixy="1.4484e-08" ixz="-2.6127e-07" iyy="0.00057565" iyz="-7.8027e-07" izz="0.00056347"/>
        //</inertial>
        osg::PositionAttitudeTransform* to_cog = new osg::PositionAttitudeTransform();
        ignition::math::Pose3d pose = elem->GetElement("pose")->Get<ignition::math::Pose3d>();
        sdf_to_osg(pose , *to_cog);
        inertia_->addChild(to_cog);

        double ixx, ixy, ixz, iyy, iyz, izz, m;
        elem->GetElement("inertia")->GetElement("ixx")->GetValue()->Get(ixx);
        elem->GetElement("inertia")->GetElement("ixy")->GetValue()->Get(ixy);
        elem->GetElement("inertia")->GetElement("ixz")->GetValue()->Get(ixz);
        elem->GetElement("inertia")->GetElement("iyy")->GetValue()->Get(iyy);
        elem->GetElement("inertia")->GetElement("iyz")->GetValue()->Get(iyz);
        elem->GetElement("inertia")->GetElement("izz")->GetValue()->Get(izz);
        elem->GetElement("mass")->GetValue()->Get(m);
        osg::ref_ptr<osg::Group> geode = make_ellispoid(ixx, ixy, ixz, iyy, iyz, izz, m);
        to_cog->addChild(geode);
    }
}



osg::ref_ptr<osg::Group> OSGSegment::createVisual(urdf::VisualSharedPtr visual, QDir baseDir)
{
    osg::ref_ptr<osg::PositionAttitudeTransform> to_visual(new osg::PositionAttitudeTransform());
    if (visual)
        urdf_to_osg(visual->origin, *to_visual);


    osg::ref_ptr<osg::Group> root = osg::ref_ptr<osg::Group>(new osg::Group());
    osg::Node* osg_visual;
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
            qfilename = baseDir.absoluteFilePath(qfilename);
        if (QFileInfo(qfilename + ".osgb").exists())
            qfilename = qfilename + ".osgb";

        //Force 'classic' ('C'-style) encoding before loading mesh files.
        //This allows loading of .obj files from within an Qt App on a german system.
        //Otherwise decimal delimeter confusion prevents loading of obj-files correctly
        std::locale::global(std::locale::classic());
        filename = qfilename.toStdString();
        std::map<std::string, osg::ref_ptr<osg::Node>>::iterator it = meshCache.find(filename);
        if (it == meshCache.end()) {
            osg_visual = osgDB::readNodeFile(filename);
            meshCache[filename] = osg_visual;
        }
        else {
            osg_visual = it->second;
        }

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
                if(texture_img){
                    osg::ref_ptr<osg::TextureRectangle> texture_rect = osg::ref_ptr<osg::TextureRectangle>(new osg::TextureRectangle(texture_img));
                    osg::ref_ptr<osg::TexMat> texmat = new osg::TexMat;
                    texmat->setScaleByTextureRectangleSize(true);
                    nodess->setTextureAttributeAndModes(0, texture_rect, osg::StateAttribute::ON);
                    nodess->setTextureAttributeAndModes(0, texmat, osg::StateAttribute::ON);
                }
                else{
                    std::cout << "Could not load texture from file '"<<visual->material->texture_filename<<"'." << std::endl;
                }
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

    useVBOIfEnabled(osg_visual);

    to_visual->addChild(osg_visual);
    root->addChild(to_visual);

    return root;
}

osg::ref_ptr<osg::Group> OSGSegment::createVisual(sdf::ElementPtr sdf_visual, QDir baseDir){

    osg::PositionAttitudeTransform* to_visual = new osg::PositionAttitudeTransform();
    sdf_to_osg(sdf_visual->GetElement("pose")->Get<ignition::math::Pose3d>(), *to_visual);

    osg::ref_ptr<osg::Group> root = osg::ref_ptr<osg::Group>(new osg::Group());
    osg::Node* osg_visual = 0;
    if (sdf_visual->HasElement("geometry")){

        sdf::ElementPtr sdf_geometry  = sdf_visual->GetElement("geometry");
        sdf::ElementPtr sdf_geom_elem = sdf_geometry->GetFirstElement();
        if (!sdf_geom_elem){
            LOG_WARN("SDF: no geometry element");
            osg_visual = new osg::Geode;
        }
        else if (sdf_geom_elem->GetName() == "box"){
            osg::Vec3f size;
            sdf_to_osg(sdf_geom_elem->GetElement("size")->Get<ignition::math::Vector3d>(), size);
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
        else if (sdf_geom_elem->GetName() == "mesh") {
            osg::Vec3 scale;
            sdf_to_osg(sdf_geom_elem->GetElement("scale")->Get<ignition::math::Vector3d>(), scale);

            to_visual->setScale(scale);

            std::string uri = sdf_geom_elem->GetElement("uri")->Get<std::string>();
            std::string filename = sdf::findFile(uri, true, false);
            if (!QFileInfo(QString::fromStdString(filename)).exists()) {
                QDir modelPaths = baseDir;
                std::string model_prefix = "model://";
                if(uri.compare(0, model_prefix.length(), model_prefix) == 0) {
                    filename = uri.substr(model_prefix.length());
                    // If model://-style reference is use we assume that the SDF is in a separate subfolder and
                    // such that we have to go up one level to resolve the path.
                    // If there's jsut a path, we assume the given path is right as it was given.
                    // FIXME: Comment by Malte: whats the rationale behind the going up one folder? This was here for
                    //        quite some time, and appraently is used udner this assumption, but it appears a bit random
                    //        to me. Is this a rock-specific assumption or is coming from SDFormat?
                    //
                    modelPaths.cdUp();
                }
                else {
                    filename = uri;
                }

                QString qfilename = QString::fromStdString(filename);
                std::cout << "qfilename: " << qfilename.toStdString() << std::endl;
                if (QFileInfo(qfilename).isRelative()){
                    std::cout << "relative" << std::endl;
                    std::cout << "modelPaths:"<< modelPaths.absolutePath().toStdString() << std::endl;

                    std::cout << "modelPaths2:"<< modelPaths.absolutePath().toStdString()<< std::endl;
                    filename = modelPaths.absoluteFilePath(qfilename).toStdString();
                    std::cout << "filename:"<< filename << std::endl;
                }
                else{
                    std::cout << "not relative" << std::endl;
                }
            }

            if (QFileInfo(QString::fromStdString(filename + ".osgb")).exists())
                filename = filename + ".osgb";

            LOG_INFO("loading visual %s", filename.c_str());
            auto it = meshCache.find(filename);
            if (it == meshCache.end()) {
                osg_visual = osgDB::readNodeFile(filename);
                meshCache[filename] = osg_visual;
            }
            else {
                LOG_INFO("visual already in cache");
                osg_visual = it->second;
            }

            if (!osg_visual) {
                LOG_WARN("OpenSceneGraph did not succeed in loading the mesh file %s.", filename.c_str());
                osg_visual = new osg::Geode;
            }
        }
        else {
            LOG_WARN("SDF: %s is not a supported geometry", sdf_geom_elem->GetName().c_str());
            osg_visual = new osg::Geode;
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
            sdf_to_osg(sdf_material->GetElement("ambient")->Get<sdf::Color>(), ambient);
            nodematerial->setAmbient(osg::Material::FRONT,ambient);
        }

        if (sdf_material->HasElement("diffuse")){
            osg::Vec4 diffuse;
            sdf_to_osg(sdf_material->GetElement("diffuse")->Get<sdf::Color>(), diffuse);
            nodematerial->setDiffuse(osg::Material::FRONT,diffuse);
        }

        if (sdf_material->HasElement("specular")){
            osg::Vec4 specular;
            sdf_to_osg(sdf_material->GetElement("specular")->Get<sdf::Color>(), specular);
            nodematerial->setSpecular(osg::Material::FRONT, specular);

        }

        if (sdf_material->HasElement("emissive")){
            osg::Vec4 emissive;
            sdf_to_osg(sdf_material->GetElement("emissive")->Get<sdf::Color>(), emissive);
            nodematerial->setEmission(osg::Material::FRONT, emissive);
        }

        osg::ref_ptr<osg::StateSet> nodess = osg_visual->getOrCreateStateSet();
        nodess->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
        nodess->setAttribute(nodematerial.get());
    }

    useVBOIfEnabled(osg_visual);

    to_visual->addChild(osg_visual);
    root->addChild(to_visual);
    return root;
}

void OSGSegment::useVBOIfEnabled(osg::Node* node)
{
    if (useVBO_)
    {
        LOG_DEBUG("Using VBOs to display meshes")
        VBOVisitor vbo;
        node->accept(vbo);
    }
    else {
        LOG_DEBUG("Not using VBOs to display meshes, enable globally with ROCK_VIZ_USE_VBO")
    }
}

void OSGSegment::createVisuals(const std::vector<urdf::VisualSharedPtr > &visual_array, QDir prefix)
{
    std::vector<urdf::VisualSharedPtr >::const_iterator itr = visual_array.begin();
    std::vector<urdf::VisualSharedPtr >::const_iterator itr_end = visual_array.end();

    visual_ = osg::ref_ptr<osg::Group>(new osg::Group());
    for(itr = visual_array.begin(); itr != itr_end; ++itr)
    {
        urdf::VisualSharedPtr visual = *itr;
        visual_->addChild(createVisual(visual, prefix));
    }
}

void OSGSegment::createVisuals(std::vector<sdf::ElementPtr> const &visual_array, QDir prefix){

    std::vector<sdf::ElementPtr>::const_iterator
        itr,
        itr_end = visual_array.end();

    visual_ = osg::ref_ptr<osg::Group>(new osg::Group());
    for(itr = visual_array.begin(); itr != itr_end; ++itr)
    {
        sdf::ElementPtr visual = *itr;
        visual_->addChild(createVisual(visual, prefix));
    }
}


void OSGSegment::removeIconLabel(){
    if(label_)
        post_transform_->removeChild(label_);
    label_ = 0;
}


void OSGSegment::attachIconLabel(std::string name, std::string filepath){
    if(label_)
        removeIconLabel();

    osg::ref_ptr<osg::Geode> geode = createIconLabel(name, filepath);
    geode->setUserData(this);
    post_transform_->addChild(geode);

    label_ = geode;
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

void OSGSegment::attachCollision(){
    if(collision_){
        post_transform_->addChild(collision_);
    }
}

void OSGSegment::removeCollision()
{
    if(collision_){
        post_transform_->removeChild(collision_);
    }
}

void OSGSegment::removeVisual()
{
    if(visual_){
        post_transform_->removeChild(visual_);
    }
}

void OSGSegment::attachVisual()
{
    if(visual_){
        post_transform_->addChild(visual_);
    }
}

void OSGSegment::removeInertia()
{
    if(inertia_){
        post_transform_->removeChild(inertia_);
    }
}

void OSGSegment::attachInertia()
{
    if(inertia_){
        post_transform_->addChild(inertia_);
    }
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
