#pragma once

/**
 * @brief This callback for user defined calculations before rendering.
 *
 */
class OSGSegmentCallback : public osg::NodeCallback
{
public:
    /**
     * @brief the function that gets called.
     *
     * This callback is attached to every joint transformation node. It is
     * executed before rendering every frame. The OSG joint transformation
     * node gets updated by new transformation (depending on it joint_position_).
     *
     * @param node: The corresponding node this Callback is attached to.
     * @param nv: A node visitor (a thing that traverses throu nodes)
     */
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        osg::ref_ptr<OSGSegment> segment =
                dynamic_cast<OSGSegment*> (node->getUserData() );
        if(segment)
        {
            segment->updateJoint();
        }
        traverse(node, nv);
    }
};
