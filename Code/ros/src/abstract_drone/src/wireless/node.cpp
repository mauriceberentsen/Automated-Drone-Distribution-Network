#include "node.hpp"

namespace wireless
{
Node::Node( Vector3< float >& _position,
            std::shared_ptr< ros::NodeHandle >& _nodeHandle,
            std::string& _subtopicname )
    : moved( true )
    , position( _position )
    , nodeHandle( _nodeHandle )
    , subtopicname( _subtopicname )
    , on(true)
{
 rosPub =
     nodeHandle->advertise< abstract_drone::NRF24 >( subtopicname, 100, true );
}

Node::~Node( )
{
}

void Node::recieveMessage( abstract_drone::NRF24& msg )
{
 rosPub.publish( msg );
}

const Vector3< float >& Node::getPosition( )
{
 return this->position;
}

void Node::setPosition( Vector3< float >& position )
{
 this->position = position;
}

bool Node::getMoved( )
{
 return this->moved;
}

void Node::isMoved( bool moved )
{
 this->moved = moved;
}

}  // namespace wireless