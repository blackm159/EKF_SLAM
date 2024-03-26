#include "turtlelib/svg.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath> // contains math functions
#include <iostream> // contains iostream objects
#include <fstream>
#include <cstdlib> 
#include <vector>


using namespace turtlelib;

DiffDrive::DiffDrive()
{
    wheel_track_ = 0.0;
    wheel_radius_ = 0.0;

    wheel_positions_.phi_left_ = 0.0;
    wheel_positions_.phi_right_ = 0.0;

    configuration_.x_ = 0.0;
    configuration_.y_ = 0.0;
    configuration_.theta_ = 0.0;

}

DiffDrive::DiffDrive(double wheel_track, double wheel_radius)
{
    wheel_track_ = wheel_track;
    wheel_radius_ = wheel_radius;

    wheel_positions_.phi_left_ = 0.0;
    wheel_positions_.phi_right_ = 0.0;

    configuration_.x_ = 0.0;
    configuration_.y_ = 0.0;
    configuration_.theta_ = 0.0;

}

void DiffDrive::update_configuration(WheelPositions new_wheel_pos)
{

    double delta_phi_left = new_wheel_pos.phi_left_ - wheel_positions_.phi_left_;
    double delta_phi_right = new_wheel_pos.phi_right_ - wheel_positions_.phi_right_;


    // Twist2D twist;
    twist_.omega = wheel_radius_ * (delta_phi_right - delta_phi_left) / wheel_track_;
    twist_.x = wheel_radius_ * (delta_phi_right + delta_phi_left) / 2.0;
    twist_.y = 0.0;


    Transform2D Tbbp;
    Tbbp = integrate_twist(twist_);
    Vector2D v;
    v.x = configuration_.x_;
    v.y = configuration_.y_;
    Transform2D Twb(v, configuration_.theta_);

    Transform2D Twbp = Twb * Tbbp;

    configuration_.x_ = Twbp.translation().x;
    configuration_.y_ = Twbp.translation().y;
    configuration_.theta_ = Twbp.rotation();


    wheel_positions_ = new_wheel_pos;

}


WheelPositions DiffDrive::twist_to_wheel_velocities(Twist2D twist)
{
    WheelPositions wheel_velocities;

    if (almost_equal(twist.y, 0.0, 1.0e-12))
    {
        wheel_velocities.phi_left_ = (twist.x - (wheel_track_ * twist.omega / 2.0)) / wheel_radius_;
        wheel_velocities.phi_right_ = (twist.x + (wheel_track_ * twist.omega / 2.0)) / wheel_radius_;
    }
    else
    {
        throw std::logic_error("twist_to_wheel_velocities: y is not equal to 0.0");
    }

    

    return wheel_velocities;
}

void DiffDrive::set_configuration(Configuration configuration)
{
    configuration_ = configuration;
}
