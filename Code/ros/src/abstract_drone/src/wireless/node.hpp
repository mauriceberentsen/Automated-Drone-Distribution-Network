#ifndef NODEHPP
#define NODEHPP

#include <ignition/math/Vector3.hh>
#include "ros/ros.h"
#include "abstract_drone/NRF24.h"

using namespace ignition::math;
namespace wireless
{
    class Node
    {
    public:
        Node(Vector3<float> &_position, std::shared_ptr<ros::NodeHandle> _nodeHandle, std::string _subtopicname);
        ~Node();
        const Vector3<float>& getPosition();
        void setPosition(Vector3<float>& position);
        void isMoved(bool moved);
        bool getMoved();
        void recieveMessage(abstract_drone::NRF24& msg);
    private:
        ros::Publisher rosPub;
        std::shared_ptr<ros::NodeHandle> nodeHandle;
        Vector3<float> position;
        std::string subtopicname;
        bool moved;


    };
    
}
#endif //NODEHPP