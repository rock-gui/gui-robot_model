#ifndef OSGHELPERS_HPP
#define OSGHELPERS_HPP

inline void sdf_to_osg(ignition::math::Pose3d const& in, osg::PositionAttitudeTransform& out) {
    out.setPosition(osg::Vec3(in.Pos().X(), in.Pos().Y(), in.Pos().Z()));
    out.setAttitude(osg::Quat(in.Rot().X(), in.Rot().Y(), in.Rot().Z(), in.Rot().W()));
}

inline void sdf_to_osg(ignition::math::Vector3d const& in, osg::PositionAttitudeTransform& out) {
    out.setPosition(osg::Vec3(in.X(), in.Y(), in.Z()));
}

inline void sdf_to_osg(ignition::math::Vector3d const& in, osg::Vec3& out) {
    out.set(in.X(), in.Y(), in.Z());
}

inline void sdf_to_osg(sdf::Color in, osg::Vec4& out) {
    out.set(in.r, in.g, in.b, in.a);
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

void printNodeStructureRecursive(osg::Node* node, int levelCount){

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

void printNodeStructure(osg::Node *node){
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
    osg::Matrix* wcMatrix;
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


#endif
