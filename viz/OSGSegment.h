#pragma once

#include <kdl/chain.hpp>
#include <urdf_model/model.h>
#include <sdf/sdf.hh>

#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osgText/Text>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgFX/Outline>

#include <QDir>

/**
 * @brief Data structure that is attached as 'User Data' to nodes within the RobotModel.
 *
 * This data structure provides access to the visualization joints and segments of the robot.
 * Joint posiiotns can beupdated, Segemetns can be highlighted and labels can be attached to
 * the robot visualization.
 * Hereby internally, the same instance of OSGSegment is shared between the corresponding segment-,
 * joint-, and label nodes.
 *
 * This class should only be constructed by RobotModel.
 */
class OSGSegment : public osg::Referenced
{
public:
/**
 * @brief This class should only be constructed by RobotModel.
 *
 * @param node: The Node this OSGSegment is attached to. Should be the joint transform
 * @param seg: KDL segment corresponding to the node.
 */
    OSGSegment(KDL::Segment seg, bool useVBO);

    /** Clear the internal mesh file loading cache */
    static void clearMeshCache();

    /**
     * @brief Set position of joint
     *
     * @param rad: Position of joint in radians
     */
    void setJointPos(double rad){jointPos_ = rad;}

    /**
     * @brief Remove curretn label from node
     *
     */
    void removeLabel();

    /**
     * @brief Attach a label to node
     *
     * @param name: A name of the label, currently not internally used.
     * @param filepath: Path to an image file for the label. Should 256x256 Pixel.
     */
    void attachLabel(std::string name, std::string filepath);


    void setupTextLabel();
    /**
     * Attach a text label to a segemnt. If en empty string is given, the segment name is used
     * as label.
     */
    void attachTextLabel(std::string text="");

    /**
     * Remove text label
     */
    void removeTextLabel();
    void attachCollision();
    void removeCollision();
    void removeVisual();
    void attachVisual();
    void removeInertia();
    void attachInertia();

    /**
     * @brief Switch on/off highlighting visualization
     *
     * @return bool: New state of highlight (on/off)
     */
    bool toggleSelected();

    /**
     * @brief Returns the corresponding KDL::Segment
     *
     * @return const KDL::Segment
     */
    const KDL::Segment& getSegment(){return seg_;}

    /**
     * @brief returns the osg Node that is the root of the visualization for
     *   this segment
     */
    osg::ref_ptr<osg::Group> getGroup() const;

protected:
    /**
     * @brief Updates the joint transformation corresponding to the value of joint_pos_.
     *
     */
    void updateJoint();

    /**
    * @brief Attach visual mesh to node.
    *
    * Should only be called during initial construction of the robot model.
    *
    * The graph strcuture:
    *    visual -> (to_visual) -> the mesh or alike
    * To be attached to post_transform_
    *
    * @param visual: Parsed URDF tag (using urdf_parser) of the visual.
    */
   osg::ref_ptr<osg::Group> createVisual(urdf::VisualSharedPtr visual, QDir prefix = QDir());

   /**
   * @brieg Create nodes with visual meshes
   *
   * Should only be called during initial construction of the robot model
   *
   * @param visual_array: Parsed URDF tag (using urdf_parser) of the visual.
   */
   void createVisuals(const std::vector<urdf::VisualSharedPtr> &visual_array, QDir prefix = QDir());

  /**
  * @brieg Create nodes with collsion meshes
  *
  * Should only be called during initial construction of the robot model
  *
  * @param collision_array: Parsed URDF tag (using urdf_parser) of the visual.
  */
  void createCollisions(const std::vector<urdf::CollisionSharedPtr> &collision_array, QDir prefix = QDir());
  void createCollisions(std::vector<sdf::ElementPtr> const& collision_array, QDir prefix);
  void createInertias(std::vector<sdf::ElementPtr> const& inertia_array);


   /**
   * @brief Attach visual mesh to node.
   *
   * Should only be called during initial construction of the robot model.
   *
   * @param visual: Parsed SDF tag (using sdformat) of the visual.
   */
   osg::ref_ptr<osg::Group> createVisual(sdf::ElementPtr visual, QDir prefix = QDir());

   /**
   * @brieg Create nodes with visual meshes
   *
   * Should only be called during initial construction of the robot model
   *
   * @param visual_array: Parsed SDF tag (using sdformat) of the visual.
   */
   void createVisuals(std::vector<sdf::ElementPtr> const&visual_array, QDir prefix = QDir());

   /** Attach the VBO visitor to the node if VBOs are enabled */
   void useVBOIfEnabled(osg::Node* node);

private:
    KDL::Segment seg_; /**< KDKL representation of the segment */
    KDL::Frame toTipKdl_; /**< Temp storage for the current joint pose */
    // Graph structure is like this:
    // toTipOsg_ -> post_transform_ -> visual -> (to_visual) -> the mesh or alike
    //                                 collision -> (to_visual) -> the mesh or alike
    //                                 label
    //                                 text_label_geode -> text_label
    osg::ref_ptr<osg::PositionAttitudeTransform> toTipOsg_; /**< The osg node for the joint pose */
    osg::ref_ptr<osg::Group> post_transform_;
    float jointPos_; /**< Current joint position in radians */
    osg::ref_ptr<osg::Group> visual_; /**< OSG node for visual element */
    osg::ref_ptr<osg::Geode> label_; /**< OSG node for label */
    osg::ref_ptr<osgText::Text> text_label_;
    osg::ref_ptr<osg::Geode> text_label_geode_;
    osg::ref_ptr<osg::Group> collision_; /**< OSG node for collision visualization */
    osg::ref_ptr<osg::Group> inertia_; /**< OSG node for inertia visualization */
    bool isSelected_; /**< Selection state */
    bool useVBO_; /**< Whether rendering should use VBOs */

    /** Caches each laoded mesh indexed by it's filename to prevent double
     *  loading the files and avoid duplicates in osg scene */
    static std::map<std::string, osg::ref_ptr<osg::Node>> meshCache;

    friend class InteractionHandler;
    friend class OSGSegmentCallback;
    friend class RobotModel;
    friend class URDFRobotWidget;
    friend class CameraFollowNodeCallback;
};
