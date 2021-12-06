#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include <urdf_model/model.h>
#include <urdf_world/types.h>
#include <kdl_parser/kdl_parser.hpp>
//#include <kdl/chain.hpp>
#include <sdf/sdf.hh>


//#include <QHash>
#include <QObject>
#include "OSGSegment.h"


/** 
 * Struct to hold mimic joint properties
 */
struct MimicJoint {
    std::string jointToMimic;
    double multiplier;
    double offset;

    MimicJoint( std::string jointToMimic_ = "",
            double multiplier_ = 1,
            double offset_ = 0 )
        : jointToMimic( jointToMimic_ ),
        multiplier( multiplier_ ),
        offset( offset_ ) { }
};

/**
 * @brief This class is used for creating a visualization of a robot model and access it.
 *
 */
class RobotModel
{
public:
    enum ROBOT_MODEL_FORMAT
    {
        ROBOT_MODEL_AUTO = kdl_parser::ROBOT_MODEL_AUTO,
        ROBOT_MODEL_URDF = kdl_parser::ROBOT_MODEL_URDF,
        ROBOT_MODEL_SDF  = kdl_parser::ROBOT_MODEL_SDF
    };

    /**
     * @brief Constructor, does nearly nothing.
     *
     */
    RobotModel();

    /**
     * @brief
     *
     */
    ~RobotModel(){}

    /**
     * @brief Create visualization of a robot
     *
     * The model file format is guessed based on the file's extension. To
     * provide it explicitely. use loadFromFile(path, format)
     *
     * @param path: Path to the file.
     * @param format: the file format (either ROBOT_MODEL_URDF or
     *   ROBOT_MODEL_SDF)
     * @return osg::Node: Root node of the constructed OSG scene for the robot.
     */
    osg::ref_ptr<osg::Node> loadFromFile(QString path, ROBOT_MODEL_FORMAT format = ROBOT_MODEL_AUTO);

    /**
     * @brief Create visualization of a robot from a XML string
     *
     * @param xml: the XML document, as a string
     * @param format: the file format (either ROBOT_MODEL_URDF or
     *   ROBOT_MODEL_SDF)
     * @return osg::Node: Root node of the constructed OSG scene for the robot.
     */
    osg::ref_ptr<osg::Node> loadFromString(QString xml, ROBOT_MODEL_FORMAT format, QString rootPrefix = "");

    /**
     * @deprecated use loadFromFile instead
     */
    osg::ref_ptr<osg::Node> load(QString path);

    void attachCollisions(bool value);
    void attachInertias(bool value);
    void attachVisuals(bool value);

    /**
     * @brief Create an empty scene.
     *
     * This function removes to an existing robot from the scene.
     *
     * @return osg::Node: Root to the empty scene.
     */
    osg::ref_ptr<osg::Node> loadEmptyScene();

    /**
     * @brief Retrieve an OSG Segment by Joint or Segment name.
     *
     * @param name: Name of joint or segment (from URDF file)
     * @return OSGSegment
     */
    osg::ref_ptr<OSGSegment> getSegment(std::string name);

    /**
     * @brief Retrieve an OSG Segment by the corresponding OSG node
     *
     * @param node: the OSG node
     * @return OSGSegment
     */
    osg::ref_ptr<OSGSegment> getSegment(osg::ref_ptr<osg::Node> node);

    /**
     * @brief Retrieve joint names of all joints defined in URDF file which are not of type KDL::Joint::None.
     *
     * @return const std::vector<std::string>
     */
    const std::vector<std::string>& getJointNames(){return jointNames_;}

    /**
     * @brief Retrieve all segment names defined in URDF file
     *
     * @return const std::vector<std::string>
     */
    const std::vector<std::string>& getSegmentNames(){return segmentNames_;}

    /**
     * @brief Calculate relative transform between two segments
     *
     * Will return the transform required to transform a point described in coordinate frame of
     * 'source_segment' to the frame 'target_segment'.
     *
     * @return osg::Matrixd
     */
    osg::Matrixd getRelativeTransform(std::string source_segment, std::string target_segment);

    /**
     * @brief Relocate the root to a given segment
     */
    bool relocateRoot(std::string name);

    /** 
     * @brief Finds the relevant node and sets the position value
     */
    bool setJointPos(std::string jointName, double jointVal);
    /**
     * @brief Relocate the root to a given segment
     */
    bool relocateRoot(osg::ref_ptr<osg::Node> group);

    /**
     * Enables or disables usage of VBO (might affect performance)
     */
    void setUseVBO(bool enabled);

    /**
     * Checks whether VBOs will be used or not
     */
    bool getUseVBO() const;

protected:
    void makeOsg2(KDL::Segment kdl_seg, const std::vector<urdf::VisualSharedPtr> &visuals, const std::vector<urdf::CollisionSharedPtr> &collisions, OSGSegment &seg);
    osg::ref_ptr<osg::Node> makeOsg( urdf::ModelInterfaceSharedPtr urdf_model );

    void makeOsg2(KDL::Segment const &kdl_seg,
            std::vector<sdf::ElementPtr> const& visuals,
            std::vector<sdf::ElementPtr> const& collisions, const std::vector<sdf::ElementPtr> &inertias,
            OSGSegment& seg);
    osg::Node* makeOsg( sdf::ElementPtr sdf );

    /**
     * @brief load from a string containing a URDF model
     *
     * This is a helper from loadFromString
     */
    osg::ref_ptr<osg::Node> loadFromURDFString(QString xml);

    /**
     * @brief load from a string containing a SDF model
     *
     * This is a helper from loadFromString
     */
    osg::ref_ptr<osg::Node> loadFromSDFString(QString xml);

    std::map<std::string, sdf::ElementPtr> loadSdfModelLinks(sdf::ElementPtr sdf_model);


protected:
    osg::ref_ptr<osg::Group> root_; /**< Root of the OSG scene containing the robot */
    osg::ref_ptr<osg::Group> original_root_; /**< The original root always corresponding to the root of the URDF. If relocateRoot was called this get's not affected */
    std::string current_root_name_;
    std::string original_root_name_;
    std::vector<std::string> jointNames_; /**< Joint names defined in URDF (joint of type none are NOT included) */
    std::vector<std::string> segmentNames_; /**< Segment names defined in URDF */
    QDir rootPrefix;

    std::map< std::string, MimicJoint > mimic_joints_;

    bool useVBO_;

    static bool getVBODefault();

public:
    //bool set_joint_state(std::vector<double> joint_vals);
    /**
     * @brief Set position of a single joint by its name.
     *
     * @param std::string: Name of the joint
     * @param joint_val: Joint position in degree
     * @return bool
     */
    bool setJointState(std::string, double jointVal);
    bool setJointState(const std::map<std::string,double>& jointVals);
    bool toggleHighlight(std::string);

    friend class InteractionHandler;
    friend class URDFRobotWidget;
};

#endif
