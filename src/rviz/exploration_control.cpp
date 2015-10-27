#include "exploration_control.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include <geometry_msgs/Point.h>

namespace sbc15_fsm
{

ExplorationControl::ExplorationControl()
    : flag_node_(NULL),
      moving_flag_node_( NULL ),
      current_flag_property_( NULL )
{
    shortcut_key_ = 'e';

    search_dir_pub_ = nh_.advertise<geometry_msgs::Point>("/exploration_direction", 1, true);
}

ExplorationControl::~ExplorationControl()
{
    if(flag_node_) {
        scene_manager_->destroySceneNode( flag_node_ );
    }
}

void ExplorationControl::onInitialize()
{
    flag_resource_ = "package://sbc15_fsm/media/flag.dae";

    if( rviz::loadMeshFromResource( flag_resource_ ).isNull() )
    {
        ROS_ERROR( "PlantFlagTool: failed to load model resource '%s'.", flag_resource_.c_str() );
        return;
    }

    moving_flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
    moving_flag_node_->attachObject( entity );
    moving_flag_node_->setVisible( false );
}

void ExplorationControl::activate()
{
    if( moving_flag_node_ )
    {
        moving_flag_node_->setVisible( true );

        current_flag_property_ = new rviz::VectorProperty( "ExplorationPoint" );
        current_flag_property_->setReadOnly( true );
        getPropertyContainer()->addChild( current_flag_property_ );
    }
}

void ExplorationControl::deactivate()
{
    if( moving_flag_node_ )
    {
        moving_flag_node_->setVisible( false );
        delete current_flag_property_;
        current_flag_property_ = NULL;
    }
}


int ExplorationControl::processMouseEvent( rviz::ViewportMouseEvent& event )
{
    if(event.rightUp()) {
        destroyFlag();
        return Render | Finished;
    }

    if( !moving_flag_node_ )
    {
        return Render;
    }
    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
    if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                           ground_plane,
                                           event.x, event.y, intersection ))
    {
        moving_flag_node_->setVisible( true );
        moving_flag_node_->setPosition( intersection );
        current_flag_property_->setVector( intersection );

        if( event.leftDown() )
        {
            makeFlag( intersection );
            current_flag_property_ = NULL; // Drop the reference so that deactivate() won't remove it.
            return Render | Finished;
        }
    }
    else
    {
        moving_flag_node_->setVisible( false ); // If the mouse is not pointing at the ground plane, don't show the flag.
    }
    return Render;
}

// This is a helper function to create a new flag in the Ogre scene and save its scene node in a list.
void ExplorationControl::makeFlag( const Ogre::Vector3& position )
{
    Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
    node->attachObject( entity );
    node->setVisible( true );
    node->setPosition( position );

    if(flag_node_) {
        destroyFlag();
    }

    flag_node_ = node;

    geometry_msgs::Point msg;
    msg.x = position.x;
    msg.y = position.y;
    msg.z = position.z;
    search_dir_pub_.publish(msg);
}

void ExplorationControl::destroyFlag()
{
    if(flag_node_) {
        scene_manager_->destroySceneNode(flag_node_);
        flag_node_ = NULL;

        delete current_flag_property_;
        current_flag_property_ = NULL;

        getPropertyContainer()->removeChildren();
    }
}

void ExplorationControl::save( rviz::Config config ) const
{
    config.mapSetValue( "Class", getClassId() );

    // To read the positions and names of the flags, we loop over the
    // the children of our Property container:
    rviz::Property* container = getPropertyContainer();
    int num_children = container->numChildren();

    if(num_children == 0) {
        return;
    } else {
        // only import one child
        num_children = 1;
    }

    // The top level of this tool's Config is a map, but our flags
    // should go in a list, since they may or may not have unique keys.
    // Therefore we make a child of the map (``flags_config``) to store
    // the list.
    rviz::Config flags_config = config.mapMakeChild( "ExplorationPoint" );

    for( int i = 0; i < num_children; i++ )
    {
        rviz::Property* position_prop = container->childAt( i );
        // For each Property, we create a new Config object representing a
        // single flag and append it to the Config list.
        rviz::Config flag_config = flags_config.listAppendNew();
        // Into the flag's config we store its name:
        flag_config.mapSetValue( "Name", position_prop->getName() );
        // ... and its position.
        position_prop->save( flag_config );
    }
}

void ExplorationControl::load( const rviz::Config& config )
{
    // Here we get the "Flags" sub-config from the tool config and loop over its entries:
    rviz::Config flags_config = config.mapGetChild( "ExplorationPoint" );
    int num_flags = flags_config.listLength();
    for( int i = 0; i < num_flags; i++ )
    {
        rviz::Config flag_config = flags_config.listChildAt( i );
        // At this point each ``flag_config`` represents a single flag.
        //
        // Here we provide a default name in case the name is not in the config file for some reason:
        QString name = "ExplorationPoint";
        // Then we use the convenience function mapGetString() to read the
        // name from ``flag_config`` if it is there.  (If no "Name" entry
        // were present it would return false, but we don't care about
        // that because we have already set a default.)
        flag_config.mapGetString( "Name", &name );
        // Given the name we can create an rviz::VectorProperty to display the position:
        rviz::VectorProperty* prop = new rviz::VectorProperty( name );
        // Then we just tell the property to read its contents from the config, and we've read all the data.
        prop->load( flag_config );
        // We finish each flag by marking it read-only (as discussed
        // above), adding it to the property container, and finally making
        // an actual visible flag object in the 3D scene at the correct
        // position.
        prop->setReadOnly( true );
        getPropertyContainer()->addChild( prop );
        makeFlag( prop->getVector() );
    }
}

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sbc15_fsm::ExplorationControl,rviz::Tool)
