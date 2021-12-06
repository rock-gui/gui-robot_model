#pragma once
#include <osg/NodeVisitor>
#include <osg/Geode>


class VBOVisitor : public osg::NodeVisitor
{
public:
    VBOVisitor()
    {
        setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
    }

    virtual void apply(osg::Node& node)
    {
        osg::Geode *geode = node.asGeode();
        if (geode != NULL)
        {
            for (unsigned int i = 0; i < geode->getNumDrawables(); i++)
            {
                osg::Drawable &drawable = *geode->getDrawable(i);
                drawable.setUseDisplayList(false);
                drawable.setUseVertexBufferObjects(true);
            }
        }
        traverse(node);
    }
};

