// ROS
#include <ros/ros.h>

#pragma once

namespace meta_skills {

class MetaSkills
{
public:
    MetaSkills();
    ~MetaSkills();
    
    void init();
    void update();
    
private:
    geometry_msgs::Vector3Stamped massage_reference_;
    geometry_msgs::PoseStamped massage_vector_;
    geometry_msgs::WrenchStamped massage_force_;
    geometry_msgs::Vector3 massage_velocity_;
    
    std::string node_name_;
protected:
};

class Screw : public MetaSkills
{
public:
    Screw();
    ~Screw();
    
    void init();
    void update();
}

class Circle : public MetaSkills
{
public:
    Circle();
    ~Circle();
    
    void init();
    void update();
}

class Press : public MetaSkills
{
public:
    Press();
    ~Press();
    
    void init();
    void update();
}

class Push : public MetaSkills
{
public:
    Push();
    ~Push();
    
    void init();
    void update();
}

class Knock : public MetaSkills
{
public:
    Knock();
    ~Knock();
    
    void init();
    void update();
}

} // namespace meta_skills
