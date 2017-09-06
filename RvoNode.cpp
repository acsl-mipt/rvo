#include "RvoNode.h"
#include <cstdlib>


AgentProperties::AgentProperties() : _r(), _v(), _vPref(){}

AgentProperties::AgentProperties(
        const Vector3 &r,
        const Vector3 &v,
        const Vector3 &vPref
        ) : _r(r)
  ,_v(v)
  ,_vPref(vPref)
{}

Vector3 AgentProperties::r() const{
    return _r;
}


Vector3 AgentProperties::v() const{
    return _v;
}

Vector3 AgentProperties::vPref() const{
    return _vPref;
}

float AgentProperties::defaultMaxSpeed() const
{
    return 5.0f;
}

float AgentProperties::defaultNeighborDist() const
{
    return 20.0f;
}

float AgentProperties::defaultRadius() const
{
    return 0.25;
//    return 0.4f;
    //return 1.0f;
}

float AgentProperties::defaultTimeHorizon() const
{
    return 2.0f;
    //return 0.75f;
}

RvoNode::RvoNode(char* pId) :
    _rvoInited(false)
  ,_pub()
  ,_rvo()
  ,id(atoi(pId))
  ,obstacles_written(false)
{
    std::string id_str = std::string(pId);
    std::string nodeName = "rvo_" + id_str;
    std::string pubName = "rvo_output" + id_str;
    std::string subName = "rvo_input" + id_str;
    std::string subNameObstacle = "rvo_input_obs" + id_str;

    int rate = 10; // hz
    char **argv = 0;
    int argc = 0;
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nodeHandle;
    _pub = nodeHandle.advertise<std_msgs::Float32MultiArray>(pubName, rate);
    ros::Subscriber sub = nodeHandle.subscribe(subName, rate, &RvoNode::inputMsgCb, this);
    ros::Subscriber subObstacle = nodeHandle.subscribe(subNameObstacle, rate, &RvoNode::inputMsgObstacleCb, this);
    std::cout << "rvo node" << id << " started" << std::endl;
    ros::spin();
}


void RvoNode::inputMsgObstacleCb(std_msgs::Float32MultiArray::ConstPtr input)
{
    std::cout << "Into OBSTACLES CALLBACK" << std::endl;
    _obstacles = parseInput(input);
    std::cout << "Amount of obstacles: " << _obstacles.size() << std::endl;
    std::cout << "END OBSTACLES CALLBACK" << std::endl;

    obstacles_written = true;
}

void RvoNode::inputMsgCb(std_msgs::Float32MultiArray::ConstPtr input)
{
    std::vector<AgentProperties> props = parseInput(input);
    if(!_rvoInited){
        _rvoInited = initRvo(props);
        if (!_rvoInited) return;
    }

    double dt = ros::Time::now().toSec() - t_prev;
    t_prev = ros::Time::now().toSec();

    std::vector<Vector3> result = getVelocities(dt, props);
    publishResult(result);
}

bool RvoNode::initRvo(const std::vector<AgentProperties> &props){

    std::cout << "Starting rvo init" << std::endl;
    if (obstacles_written == false) {
        return false;
    }

    size_t maxNeighbors = _obstacles.size() + props.size() - 1;
    std::cout << "Max neighbors: " << maxNeighbors << std::endl;

    // adding copters
    for(int i = 0; i < props.size(); i++){
        AgentProperties prop = props.at(i);

        _rvo.addAgent(prop.r(),
                      prop.defaultNeighborDist(),
                      maxNeighbors,
                      prop.defaultTimeHorizon() ,
                      prop.defaultRadius(),
                      prop.defaultMaxSpeed(),
                      prop.v()
                      );
    }

    // adding obstacles
    for(int i = 0; i < _obstacles.size(); i++){
        AgentProperties prop = _obstacles.at(i);

        _rvo.addAgent(prop.r(),
                      prop.defaultNeighborDist(),
                      maxNeighbors,
                      10.0f,
                      0.25f,
                      0.005f,
                      prop.v()
                      );
    }

    t_prev = ros::Time::now().toSec();
    std::cout << "End rvo inited" << std::endl;
    return true;
}


std::vector<Vector3> RvoNode::getVelocities(double dt, const std::vector<AgentProperties>& props)
{
    std::vector<Vector3> result;
    _rvo.setTimeStep(dt);
    for (int i = 0; i < props.size(); i++){
        _rvo.setAgentPosition(i, props.at(i).r());
        _rvo.setAgentVelocity(i, props.at(i).v());

        if (i+1 == id) {
            _rvo.setAgentPrefVelocity(i, props.at(i).vPref());
        }
    }
    _rvo.doStep();
    
//    for (int i = 0; i < props.size(); i++){
//        Vector3 resVel = _rvo.getAgentVelocity(i);
//        result.push_back(resVel);
//    }
    result.push_back(_rvo.getAgentVelocity(id - 1));
    
    return result;
}

std::vector<AgentProperties> RvoNode::parseInput(std_msgs::Float32MultiArray::ConstPtr input) {
    std::vector<AgentProperties> props;
    int size = input->data.size();
    int lineLen = 9;
    if (size == 0 || size % lineLen != 0){
        std::cout << "invalid input data" << std::endl;
        return props;
    }
    int linesNumber = size / lineLen;
    for (int i = 0; i < linesNumber; i++){
        int k = i*lineLen;
        Vector3 r(input->data[k+0], input->data[k+1], input->data[k+2]);
        Vector3 v(input->data[k+3], input->data[k+4], input->data[k+5]);
        Vector3 vPref(input->data[k+6], input->data[k+7], input->data[k+8]);
        props.push_back(AgentProperties(r, v, vPref));
    }
    return props;
}


void RvoNode::publishResult(const std::vector< Vector3 >& result)
{   
    std::vector<float> vecToPub;
    for(int i = 0; i < result.size(); i++){
        vecToPub.push_back(result.at(i).x());
        vecToPub.push_back(result.at(i).y());
        vecToPub.push_back(result.at(i).z());
    };
    std_msgs::Float32MultiArray msg;
    msg.data = vecToPub;
    _pub.publish(msg);
}


int main(int argc, char **argv) {
//    std::string id(argv[1]);
    RvoNode node(argv[1]);
}
