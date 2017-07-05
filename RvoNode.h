#pragma once

#include <iostream>
#include <string>
#include <sstream>

#include <std_msgs/Float32MultiArray.h>

#include "ros/ros.h"
#include "RVO2-3D/src/RVO.h"



typedef RVO::Vector3 Vector3;
typedef RVO::RVOSimulator RVOSimulator;

class AgentProperties{
    
public:
    
    AgentProperties();
    
    AgentProperties(
                    const Vector3 &r,
                    const Vector3 &v,
                    const Vector3 &vPref
    );
    
    Vector3 r() const;
    Vector3 v() const;
    Vector3 vPref() const;
    float defaultNeighborDist() const;
    float defaultTimeHorizon() const;
    float defaultRadius() const;
    float defaultMaxSpeed() const;
        
private:
    Vector3 _r;
    Vector3 _v;
    Vector3 _vPref;
};


class RvoNode {
    
public:
    
    RvoNode(std::string id);
    
private:
    
    void inputMsgCb(std_msgs::Float32MultiArray::ConstPtr input);
    void inputMsgObstacleCb(std_msgs::Float32MultiArray::ConstPtr input);
    
    std::vector<AgentProperties> parseInput(std_msgs::Float32MultiArray::ConstPtr input);
    
    bool initRvo(const std::vector<AgentProperties> &agents);
    void initRvoObstacle(const std::vector<AgentProperties> &agents);
    
    std::vector<Vector3> getVelocities(double dt, const std::vector<AgentProperties>& props);
    
    void publishResult(const std::vector<Vector3>& result);
    
private:
    ros::Publisher _pub;
    bool _rvoInited;
    RVOSimulator _rvo;
    // time when last message came
    double t_prev;
    std::vector<AgentProperties> _obstacles;
};
