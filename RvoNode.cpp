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
    return 1.0f;
}

float AgentProperties::defaultTimeHorizon() const
{
    return 0.75f;
}

RvoNode::RvoNode(std::string id) : 
         _rvoInited(false)
        ,_pub()
        ,_rvo()
{
    std::string nodeName = "rvo" +id; 
    std::string pubName = "rvo_output"+ id;
    std::string subName = "rvo_input" + id;
    int rate = 10; // hz
    char **argv = 0;
    int argc = 0;
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nodeHandle;
    _pub = nodeHandle.advertise<std_msgs::Float32MultiArray>(pubName, rate);
    ros::Subscriber sub = nodeHandle.subscribe(subName, rate, &RvoNode::inputMsgCb, this);
    std::cout << "rvo node" << id << " started" << std::endl;
    ros::spin();
}


void RvoNode::inputMsgCb(std_msgs::Float32MultiArray::ConstPtr input)
{   
    float dt = input->data.back();

    std::vector<AgentProperties> props = parseInput(input);
    if(!_rvoInited){
        initRvo(props);
        _rvoInited = true;
    }

    std::vector<Vector3> result = getVelocities(dt, props);
    publishResult(result);
}

void RvoNode::initRvo(const std::vector<AgentProperties> &props){
    size_t maxNeighbors = props.size() - 1;
    for(int i = 0; i < props.size(); i++){
        AgentProperties prop = props.at(i);
        _rvo.addAgent(prop.r(),
                      prop.defaultNeighborDist(),
                      maxNeighbors,
                      prop.defaultTimeHorizon(),
                      prop.defaultRadius(),
                      prop.defaultMaxSpeed(),
                      prop.v()
                     );
    }
}

std::vector<Vector3> RvoNode::getVelocities(double dt, const std::vector<AgentProperties>& props)
{
    std::vector<Vector3> result;
    _rvo.setTimeStep(dt);
    for (int i = 0; i < props.size(); i++){
        _rvo.setAgentPosition(i, props.at(i).r());
        _rvo.setAgentVelocity(i, props.at(i).v());
        _rvo.setAgentPrefVelocity(i, props.at(i).vPref());
    }
    _rvo.doStep();
    
    for (int i = 0; i < props.size(); i++){
        Vector3 resVel = _rvo.getAgentVelocity(i);
        result.push_back(resVel);
    }
    
    return result;
}

std::vector<AgentProperties> RvoNode::parseInput(std_msgs::Float32MultiArray::ConstPtr input) {
    std::vector<AgentProperties> props;    
    int size = input->data.size() - 1; // substruct one because last value is dt
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
