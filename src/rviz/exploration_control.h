#ifndef EXPLORATIONCONTROL_H
#define EXPLORATIONCONTROL_H

#include <rviz/tool.h>
#include <ros/ros.h>

namespace Ogre
{
class SceneNode;
class Vector3;
}  // namespace Ogre

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
}  // namespace rviz

namespace sbc15_fsm
{
class ExplorationControl : public rviz::Tool
{
    Q_OBJECT

public:
    ExplorationControl();
    ~ExplorationControl();

    virtual void onInitialize();

    virtual void activate();
    virtual void deactivate();

    virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

    void destroyFlag();

private:
    void makeFlag(const Ogre::Vector3& position);

    Ogre::SceneNode* flag_node_;
    Ogre::SceneNode* moving_flag_node_;
    std::string flag_resource_;
    rviz::VectorProperty* current_flag_property_;

    ros::NodeHandle nh_;
    ros::Publisher search_dir_pub_;
};

}  // namespace sbc15_fsm
#endif  // EXPLORATIONCONTROL_H
