#ifndef OSGHELPERS_HPP
#define OSGHELPERS_HPP
#include <osg/Matrix>
#include <osg/Vec3>
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osg/Texture2D>
#include <osg/BlendFunc>
#include <osg/AlphaFunc>
#include <osg/Billboard>
#include <osg/Material>
#include <osg/Point>
#include <osg/PointSprite>
#include <osg/ShapeDrawable>
#include<osgText/Text>
#include<osgText/Font>
#include <osg/Depth>

#include <kdl/chain.hpp>
#include <urdf_model/pose.h>

#if SDF_MAJOR_VERSION >= 14
#include <gz/math.hh>
namespace math = gz::math;
#else
#include <ignition/math.hh>
namespace math = ignition::math;
#endif

inline void sdf_to_osg(math::Pose3d const& in, osg::PositionAttitudeTransform& out) {
    out.setPosition(osg::Vec3(in.Pos().X(), in.Pos().Y(), in.Pos().Z()));
    out.setAttitude(osg::Quat(in.Rot().X(), in.Rot().Y(), in.Rot().Z(), in.Rot().W()));
}

inline void sdf_to_osg(math::Vector3d const& in, osg::PositionAttitudeTransform& out) {
    out.setPosition(osg::Vec3(in.X(), in.Y(), in.Z()));
}

inline void sdf_to_osg(math::Vector3d const& in, osg::Vec3& out) {
    out.set(in.X(), in.Y(), in.Z());
}

inline void sdf_to_osg(math::Color in, osg::Vec4& out) {
    out.set(in.R(), in.G(), in.B(), in.A());
}

//inline void sdf_pose_to_osg(sdf::ElementPtr pose, osg::Vec3& pos, osg::Quat& quat)
//{
//    double x, y, z;
//    double roll, pitch, yaw;
//    sscanf(pose->Get<std::string>().c_str(), "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &roll, &pitch, &yaw);
//    pos.set(x, y, z);
//    osg::Quat q = osg::Quat(roll, osg::Vec3d(1, 0, 0), pitch, osg::Vec3d(0, 1, 0), yaw, osg::Vec3d(0, 0, 1));
//    quat.set(q.x(), q.y(), q.z(), q.w());
//}
//
//
//inline void sdf_pose_to_osg(sdf::ElementPtr pose, osg::PositionAttitudeTransform& out)
//{
//    double x, y, z;
//    double roll, pitch, yaw;
//    sscanf(pose->Get<std::string>().c_str(), "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &roll, &pitch, &yaw);
//    out.setPosition(osg::Vec3(x, y, z));
//    out.setAttitude(osg::Quat(roll, osg::Vec3d(1, 0, 0), pitch, osg::Vec3d(0, 1, 0), yaw, osg::Vec3d(0, 0, 1)));
//}

inline void kdl_to_osg(KDL::Frame& in, osg::PositionAttitudeTransform& out){
    out.setPosition(osg::Vec3(in.p[0],in.p[1],in.p[2]));
    double x,y,z,w;
    in.M.GetQuaternion(x, y, z, w);
    out.setAttitude(osg::Quat(x,y,z,w));
}

inline osg::Vec3 urdf_to_osg(const urdf::Vector3 &in){
    return osg::Vec3(in.x, in.y, in.z);
}

inline void urdf_to_osg(urdf::Pose& in, osg::PositionAttitudeTransform& out){
    out.setPosition(urdf_to_osg(in.position));
    //std::cout << in.position.x << ","<< in.position.y<<","<< in.position.z<<std::endl;
    out.setAttitude(osg::Quat(in.rotation.x, in.rotation.y, in.rotation.z, in.rotation.w));
}

inline void printNodeStructureRecursive(osg::Node* node, int levelCount){

    for (int i = 0; i < levelCount; i++) std::cout << "--";
    std::cout << node->className() << "(" <<  node->getName() << ")" << std::endl;

    osg::Group* group = node->asGroup();

    if (group)
    {
        for (unsigned int i = 0 ; i < group->getNumChildren(); i ++)
        {
            printNodeStructureRecursive(group->getChild(i), ++levelCount);
        }
    }


}

inline void printNodeStructure(osg::Node *node){
    printNodeStructureRecursive(node, 0);
}

inline osg::Node* findNamedNode(const std::string& searchName,
                                osg::Node* currNode)
/// Taken From http://www.openscenegraph.org/projects/osg/wiki/Support/Tutorials/FindingNodes"
{
    osg::Group* currGroup;
    osg::Node* foundNode;

    // check to see if we have a valid (non-NULL) node.
    // if we do have a null node, return NULL.
    if ( !currNode)
    {
        return NULL;
    }

    // We have a valid node, check to see if this is the node we
    // are looking for. If so, return the current node.
    if (currNode->getName() == searchName)
    {
        return currNode;
    }

    // We have a valid node, but not the one we are looking for.
    // Check to see if it has children (non-leaf node). If the node
    // has children, check each of the child nodes by recursive call.
    // If one of the recursive calls returns a non-null value we have
    // found the correct node, so return this node.
    // If we check all of the children and have not found the node,
    // return NULL
    currGroup = currNode->asGroup(); // returns NULL if not a group.
    if ( currGroup )
    {
        for (unsigned int i = 0 ; i < currGroup->getNumChildren(); i ++)
        {
            foundNode = findNamedNode(searchName, currGroup->getChild(i));
            if (foundNode)
                return foundNode; // found a match!
        }
        return NULL; // We have checked each child node - no match found.
    }
    else
    {
        return NULL; // leaf node, no match
    }
}

// Visitor to return the world coordinates of a node.
// It traverses from the starting node to the parent.
// The first time it reaches a root node, it stores the world coordinates of
// the node it started from.  The world coordinates are found by concatenating all
// the matrix transforms found on the path from the start node to the root node.
class getWorldCoordOfNodeVisitor : public osg::NodeVisitor
{
public:
    getWorldCoordOfNodeVisitor():
        osg::NodeVisitor(NodeVisitor::TRAVERSE_PARENTS), done(false)
    {
        wcMatrix= new osg::Matrixd();
    }
    virtual void apply(osg::Node &node)
    {
        if (!done)
        {
            if ( 0 == node.getNumParents() ) // no parents
            {
                wcMatrix->set( osg::computeLocalToWorld(this->getNodePath()) );
                done = true;
            }
            traverse(node);
        }
    }
    osg::Matrixd* giveUpDaMat()
    {
        return wcMatrix;
    }
private:
    bool done;
    osg::Matrixd* wcMatrix;
};

// Given a valid node placed in a scene under a transform, return the
// world coordinates in an osg::Matrix.
// Creates a visitor that will update a matrix representing world coordinates
// of the node, return this matrix.
// (This could be a class member for something derived from node also.
inline osg::Matrixd* getWorldCoords( osg::Node* node)
{
    getWorldCoordOfNodeVisitor* ncv = new getWorldCoordOfNodeVisitor();
    if (node && ncv)
    {
        node->accept(*ncv);
        return ncv->giveUpDaMat();
    }
    else
    {
        return NULL;
    }
}

//
// Will return the transform required to transform a point described in coordinate frame 'source'
// to the frame 'target'.
//
// To calculate the world transform of both nodes is calculated. and afterwards:
// T_source_to_target = Source_to_world * target_to_world^-1
//
inline osg::Matrixd getTransformBetweenNodes(osg::ref_ptr<osg::Node> source, osg::ref_ptr<osg::Node> target)
{
    osg::Matrixd* source_to_world = getWorldCoords(source);
    osg::Matrixd* target_to_world = getWorldCoords(target);

    osg::Matrixd inv;
    inv = osg::Matrixd::inverse(*target_to_world);

    osg::Matrixd source_to_target = (*source_to_world)*inv;
    return source_to_target;
}


// Given a Camera, create a wireframe representation of its
// view frustum. Create a default representation if camera==NULL.
inline osg::Node* makeFrustumFromCamera( osg::Camera* camera )
{
    // Projection and ModelView matrices
    osg::Matrixd proj;
    osg::Matrixd mv;
    if (camera)
    {
        proj = camera->getProjectionMatrix();
        mv = camera->getViewMatrix();
    }
    else
    {
        // Create some kind of reasonable default Projection matrix.
        proj.makePerspective( 30., 1., 1., 10. );
        // leave mv as identity
    }

    // Get near and far from the Projection matrix.
    const double near = proj(3,2) / (proj(2,2)-1.0);
    const double far = proj(3,2) / (1.0+proj(2,2));

    // Get the sides of the near plane.
    const double nLeft = near * (proj(2,0)-1.0) / proj(0,0);
    const double nRight = near * (1.0+proj(2,0)) / proj(0,0);
    const double nTop = near * (1.0+proj(2,1)) / proj(1,1);
    const double nBottom = near * (proj(2,1)-1.0) / proj(1,1);

    // Get the sides of the far plane.
    const double fLeft = far * (proj(2,0)-1.0) / proj(0,0);
    const double fRight = far * (1.0+proj(2,0)) / proj(0,0);
    const double fTop = far * (1.0+proj(2,1)) / proj(1,1);
    const double fBottom = far * (proj(2,1)-1.0) / proj(1,1);

    // Our vertex array needs only 9 vertices: The origin, and the
    // eight corners of the near and far planes.
    osg::Vec3Array* v = new osg::Vec3Array;
    v->resize( 9 );
    (*v)[0].set( 0., 0., 0. );
    (*v)[1].set( nLeft, nBottom, -near );
    (*v)[2].set( nRight, nBottom, -near );
    (*v)[3].set( nRight, nTop, -near );
    (*v)[4].set( nLeft, nTop, -near );
    (*v)[5].set( fLeft, fBottom, -far );
    (*v)[6].set( fRight, fBottom, -far );
    (*v)[7].set( fRight, fTop, -far );
    (*v)[8].set( fLeft, fTop, -far );

    osg::Geometry* geom = new osg::Geometry;
    geom->setUseDisplayList( false );
    geom->setVertexArray( v );

    osg::Vec4Array* c = new osg::Vec4Array;
    c->push_back( osg::Vec4( 1., 1., 1., 1. ) );
    geom->setColorArray( c );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    GLushort idxLines[8] = {
        0, 5, 0, 6, 0, 7, 0, 8 };
    GLushort idxLoops0[4] = {
        1, 2, 3, 4 };
    GLushort idxLoops1[4] = {
        5, 6, 7, 8 };
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINES, 8, idxLines ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops0 ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops1 ) );

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( geom );

    geode->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );


    // Create parent MatrixTransform to transform the view volume by
    // the inverse ModelView matrix.
    osg::MatrixTransform* mt = new osg::MatrixTransform;
    mt->setMatrix( osg::Matrixd::inverse( mv ) );
    mt->addChild( geode );

    return mt;
}


inline osg::ref_ptr<osg::Group> drawEllipsoid(unsigned int uiStacks, unsigned int uiSlices,
                                              float fA, float fB, float fC)
{
    osg::ref_ptr<osg::Group> root(new osg::Group());
    osg::Vec4d color((float) rand()/INT_MAX, (float) rand()/INT_MAX, (float) rand()/INT_MAX, 0.5);

    //Create Vertices. Taken from https://www.gamedev.net/forums/topic/126624-generating-an-ellipsoid-in-opengl/
    float tStep = (M_PI) / (float)uiSlices;
    float sStep = (M_PI) / (float)uiStacks;
    for(float t = -M_PI/2; t <= (M_PI/2)+.0001; t += tStep)
    {
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
        osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array();
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();

        for(float s = -M_PI; s <= M_PI+.0001; s += sStep)
        {
            osg::Vec3d p( fA * cos(t) * cos(s),
                        fB * cos(t) * sin(s),
                        fC * sin(t));
            vertices->push_back( p );
            p.normalize();
            normals->push_back( p );

            osg::Vec3d q( fA * cos(t+tStep) * cos(s),
                        fB * cos(t+tStep) * sin(s),
                        fC * sin(t+tStep));
            vertices->push_back( q );
            q.normalize();
            normals->push_back( q );

            colors->push_back(osg::Vec4d(1.0, 1.0, 0.0, 0.5));
        }
        osg::ref_ptr<osg::Geometry> polygon = new osg::Geometry;
        polygon->setVertexArray( vertices.get() );
        polygon->setNormalArray( normals.get() );
        polygon->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
        polygon->addPrimitiveSet( new osg::DrawArrays(GL_TRIANGLE_STRIP, 0, vertices->size()) );

        polygon->setColorArray( colors.get() );

        osg::ref_ptr<osg::StateSet> nodess = polygon->getOrCreateStateSet();
        nodess->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
        osg::ref_ptr<osg::Material> nodematerial = new osg::Material;

        nodematerial->setAmbient(osg::Material::FRONT, color);
        nodematerial->setDiffuse(osg::Material::FRONT, color);
        nodematerial->setSpecular(osg::Material::FRONT, color);
        nodematerial->setEmission(osg::Material::FRONT, color);
        nodess->setAttribute(nodematerial.get());

        osg::ref_ptr<osg::Geode> geode(new osg::Geode);
        geode->addDrawable( polygon.get() );

        root->addChild(geode);
    }

    //Draw Principal axes as cross insde the ellisoid
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
    vertices->push_back(osg::Vec3d(-fA/2., 0, 0));
    vertices->push_back(osg::Vec3d(fA/2., 0, 0));
    vertices->push_back(osg::Vec3d(0, -fB/2., 0));
    vertices->push_back(osg::Vec3d(0, fB/2., 0));
    vertices->push_back(osg::Vec3d(0, 0, -fC/2.));
    vertices->push_back(osg::Vec3d(0, 0, fC/2.));

    osg::ref_ptr<osg::Geometry> cross = new osg::Geometry;
    cross->setVertexArray( vertices.get() );
    cross->addPrimitiveSet( new osg::DrawArrays(GL_LINES, 0, vertices->size()) );

    osg::ref_ptr<osg::StateSet> nodess = cross->getOrCreateStateSet();
    nodess->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    osg::ref_ptr<osg::Material> nodematerial = new osg::Material;

    color.set(color.r(), color.g(), color.b(), 1.);
    nodematerial->setAmbient(osg::Material::FRONT, color);
    nodematerial->setDiffuse(osg::Material::FRONT, color);
    nodematerial->setSpecular(osg::Material::FRONT, color);
    nodematerial->setEmission(osg::Material::FRONT, color);
    nodess->setAttribute(nodematerial.get());

    osg::ref_ptr<osg::Depth> depth(new osg::Depth());
    depth->setFunction(osg::Depth::ALWAYS);
    nodess->setAttributeAndModes(depth, osg::StateAttribute::ON);

    osg::ref_ptr<osg::Geode> cross_geode(new osg::Geode);
    cross_geode->addDrawable( cross.get() );

    root->addChild(cross_geode);

    return root;
}

inline osg::ref_ptr<osg::Group> drawBox(double fA, double fB, double fC)
{
    osg::Vec4d color((float) rand()/INT_MAX, (float) rand()/INT_MAX, (float) rand()/INT_MAX, 0.5);

    osg::ref_ptr<osg::Material> nodematerial = new osg::Material;
    nodematerial->setAmbient(osg::Material::FRONT, color);
    nodematerial->setDiffuse(osg::Material::FRONT, color);
    nodematerial->setSpecular(osg::Material::FRONT, color);
    nodematerial->setEmission(osg::Material::FRONT, color);
    osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Box(osg::Vec3d(0,0,0), fA, fB, fC));
    osg::ref_ptr<osg::StateSet> nodess = drawable->getOrCreateStateSet();
    nodess->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    nodess->setAttribute(nodematerial.get());
    osg::ref_ptr<osg::Geode> geode(new osg::Geode());
    geode->addDrawable(drawable);
    osg::ref_ptr<osg::Group> root(new osg::Group());
    root->addChild(geode);
    return root;
}

inline osg::ref_ptr<osg::Group> make_ellispoid(float xx, float xy, float xz, float yy,
                                               float yz, float zz, float m)
{
    osg::ref_ptr<osg::Group> root(new osg::Group());

    math::MassMatrix3d M;
#if SDF_MAJOR_VERSION >= 12
    M.SetMass(m);
    bool ok = M.SetInertiaMatrix(xx, yy, zz, xy, xz, yz);
    if(!ok){
        std::cerr << "Inertia matrix " << M.Moi() << " with mass " << M.Mass() << " is invalid" << std::endl;
        return root;
    }
#else
    M.Mass(m);
    bool ok = M.InertiaMatrix(xx, yy, zz, xy, xz, yz);
    if(!ok){
        std::cerr << "Inertia matrix " << M.MOI() << " with mass " << M.Mass() << " is invalid" << std::endl;
        return root;
    }
#endif
    math::Vector3d radii;
    math::Quaterniond rot;
    M.EquivalentBox(radii, rot);
    osg::ref_ptr<osg::PositionAttitudeTransform> T(new osg::PositionAttitudeTransform());
    T->setPosition(osg::Vec3d(0., 0., 0.));
    T->setAttitude(osg::Quat(rot.X(), rot.Y(), rot.Z(), rot.W()));

    root->addChild(T);

    //T->addChild(DrawBox(radii.X(), radii.Y(), radii.Z()));
    T->addChild(drawEllipsoid(25, 25, radii.X(), radii.Y(), radii.Z()));

    return root;
}


inline osg::ref_ptr<osg::Geode> createTextLabel(const std::string& text, osg::ref_ptr<osgText::Text>& text_label)
{
    osg::ref_ptr<osg::Geode> geode(new osg::Geode());
    text_label = osg::ref_ptr<osgText::Text>(new osgText::Text());
    text_label->setText(text);

    osg::ref_ptr<osg::StateSet> set = geode->getOrCreateStateSet();
    /// Disable depth test and Lighting
    set->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    set->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geode->addDrawable(text_label);

    //Text should be rather small and be always readable on the screen
    text_label->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
    text_label->setCharacterSize(20);

    osgText::Font* font = osgText::Font::getDefaultFont();
    font->setMinFilterHint(osg::Texture::NEAREST); // aliasing when zoom out, this doesnt look so ugly because text is small
    font->setMagFilterHint(osg::Texture::NEAREST); // aliasing when zoom in
    text_label->setFont(font);

    text_label->setAxisAlignment(osgText::Text::SCREEN);

    // Set the text to render with alignment anchor and bounding box around it:
    text_label->setDrawMode(osgText::Text::TEXT |
                           osgText::Text::ALIGNMENT);
    text_label->setAlignment(osgText::Text::CENTER_TOP);
    text_label->setPosition( osg::Vec3(0,0,0) );
    text_label->setColor( osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );

    text_label->setBackdropType(osgText::Text::OUTLINE);
    text_label->setBackdropColor(osg::Vec4(0, 0, 0, 1.0f));

    return geode;
}


inline osg::ref_ptr<osg::Geode> createIconLabel(const std::string& name, const std::string& filepath)
{
    osg::ref_ptr<osg::Geode> geode = osg::ref_ptr<osg::Geode>(new osg::Geode());
    osg::ref_ptr<osg::Geometry> geometry = osg::ref_ptr<osg::Geometry>(new osg::Geometry());

    osg::ref_ptr<osg::Vec3Array> vertices = osg::ref_ptr<osg::Vec3Array>(new osg::Vec3Array);
    vertices->push_back (osg::Vec3 (0, 0, 0.0));

    geometry->setVertexArray (vertices);

    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));

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
    geode->setName(name);

    return geode;
}

#endif
