#include "RvoNode.h"

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
    return 0.4f;
    //return 1.0f;
}

float AgentProperties::defaultTimeHorizon() const
{
    return 0.40f;
    //return 0.75f;
}

RvoNode::RvoNode(std::string id) : 
         _rvoInited(false)
        ,_pub()
        ,_rvo()
       ,_first_copter_index(0)
       ,_last_copter_index(0)
{
    std::string nodeName = "rvo_" + id;
    std::string pubName = "rvo_output" + id;
    std::string subName = "rvo_input" + id;
    std::string subNameObstacle = "rvo_input_obs";

    int rate = 10; // hz
    char **argv = 0;
    int argc = 0;
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nodeHandle;
    _pub = nodeHandle.advertise<std_msgs::Float32MultiArray>(pubName, rate);
    ros::Subscriber sub = nodeHandle.subscribe(subName, rate, &RvoNode::inputMsgCb, this);
    ros::Subscriber subObstacle = nodeHandle.subscribe(subNameObstacle, rate, &RvoNode::inputMsgObstacleCb, this);

    t_prev = ros::Time::now().toSec();
    std::cout << "rvo node" << id << " started" << std::endl;
    ros::spin();
}


void RvoNode::inputMsgObstacleCb(std_msgs::Float32MultiArray::ConstPtr input)
{
    std::vector<AgentProperties> props = parseInput(input);
    initRvoObstacle(props);
}

void RvoNode::inputMsgCb(std_msgs::Float32MultiArray::ConstPtr input)
{   

    std::vector<AgentProperties> props = parseInput(input);
    if(!_rvoInited){
        initRvo(props);
        _rvoInited = true;
    }

    double dt = ros::Time::now().toSec() - t_prev;
    t_prev = ros::Time::now().toSec();

    std::vector<Vector3> result = getVelocities(dt, props);
    publishResult(result);
}

void RvoNode::initRvo(const std::vector<AgentProperties> &props){
    size_t maxNeighbors = _rvo.getNumAgents() + props.size() - 1;

    _first_copter_index = _rvo.getNumAgents();
    for(int i = 0; i < props.size(); i++){
        AgentProperties prop = props.at(i);

        _rvo.addAgent(prop.r(),
                      prop.defaultNeighborDist(),
                      maxNeighbors,
//                      prop.defaultTimeHorizon() + i * 0.10,
                      2.0f,
                      prop.defaultRadius() + i * 0.10,
                      prop.defaultMaxSpeed(),
                      prop.v()
                     );
    }

    // update max neighbors for obstacles
    for (int i = 0; i < _first_copter_index; ++i) {
        _rvo.setAgentMaxNeighbors(i, maxNeighbors);
    }

    _last_copter_index = _rvo.getNumAgents() - 1;
}

void RvoNode::initRvoObstacle(const std::vector<AgentProperties> &props){
    size_t maxNeighbors = _rvo.getNumAgents() + props.size() - 1;
    for(int i = 0; i < props.size(); i++){
        AgentProperties prop = props.at(i);

        _rvo.addAgent(prop.r(),
                      prop.defaultNeighborDist(),
                      maxNeighbors,
                      prop.defaultTimeHorizon(),
                      0.5f,
                      prop.defaultMaxSpeed(),
                      prop.v()
                     );
    }
}

std::vector<Vector3> RvoNode::getVelocities(double dt, const std::vector<AgentProperties>& props)
{
    int copters_num = _last_copter_index - _first_copter_index + 1;
    if (copters_num != props.size()) {
        std::cout << "Invalid properties size!" << std::endl;
    }

    std::vector<Vector3> result;
    _rvo.setTimeStep(dt);
    for (int i = _first_copter_index, j = 0; i <= _last_copter_index; i++, j++){
        _rvo.setAgentPosition(i, props.at(j).r());
        _rvo.setAgentVelocity(i, props.at(j).v());
        _rvo.setAgentPrefVelocity(i, props.at(j).vPref());
    }
    _rvo.doStep();
    
    for (int i = _first_copter_index; i <= _last_copter_index; i++){
        Vector3 resVel = _rvo.getAgentVelocity(i);
        result.push_back(resVel);
    }
    
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
    std::string id(argv[1]);
    RvoNode node(id);
}
