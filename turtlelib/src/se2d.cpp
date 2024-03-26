#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath> // contains math functions
#include <iostream> // contains iostream objects
#include <cstdlib> 
#include <vector>



std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Twist2D & tw)
{
    os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
    return os;
}

std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Twist2D & tw)
{
    char c;
    is >> c;
    if (c == '[')
    {
        is >> tw.omega >> tw.x >> tw.y >> c;
    }
    else
    {
        is.putback(c);
        is >> tw.omega >> tw.x >> tw.y;
    }

    return is;
}

turtlelib::Transform2D::Transform2D()
{

    theta = 0.0;
    vect.x = 0.0;
    vect.y = 0.0;

}

turtlelib::Transform2D::Transform2D(turtlelib::Vector2D trans)
{
    vect.x = trans.x;
    vect.y = trans.y;
    theta = 0.0;
}

turtlelib::Transform2D::Transform2D(double radians)
{
    theta = radians;
    vect.x = 0.0;
    vect.y = 0.0;
}


turtlelib::Transform2D::Transform2D(turtlelib::Vector2D trans, double radians)
{

    theta = radians;
    vect.x = trans.x;
    vect.y = trans.y;
}


turtlelib::Point2D turtlelib::Transform2D::operator()(turtlelib::Point2D p) const
{
    turtlelib::Point2D newPoint;
    newPoint.x = cos(theta) * p.x - sin(theta) * p.y + vect.x;
    newPoint.y = sin(theta) * p.x + cos(theta) * p.y + vect.y;
    return newPoint;
}

turtlelib::Vector2D turtlelib::Transform2D::operator()(turtlelib::Vector2D v) const
{
    turtlelib::Vector2D newVector;
    newVector.x = cos(theta) * v.x - sin(theta) * v.y;
    newVector.y = sin(theta) * v.x + cos(theta) * v.y;
    return newVector;
}

turtlelib::Twist2D turtlelib::Transform2D::operator()(turtlelib::Twist2D v) const
{
    turtlelib::Twist2D newTwist;
    newTwist.x = v.omega * vect.y + cos(theta) * v.x - sin(theta) * v.y;
    newTwist.y = -v.omega * vect.x + sin(theta) * v.x + cos(theta) * v.y;
    newTwist.omega = v.omega;

    return newTwist;
}


turtlelib::Transform2D turtlelib::Transform2D::inv() const 
{
    turtlelib::Transform2D Tinv;
    Tinv.theta = -theta;
    Tinv.vect.x = -vect.x * cos(theta) - vect.y * sin(theta);
    Tinv.vect.y = vect.x * sin(theta) - vect.y * cos(theta);

    return Tinv;
}

turtlelib::Transform2D & turtlelib::Transform2D::operator*=(const turtlelib::Transform2D & rhs)
{
    vect.x = vect.x + rhs.vect.x * cos(theta) - rhs.vect.y * sin(theta);
    vect.y = vect.y + rhs.vect.x * sin(theta) + rhs.vect.y * cos(theta);
    theta = theta + rhs.theta;

    return *this;

}

turtlelib::Vector2D turtlelib::Transform2D::translation() const
{
    return vect;
}

double turtlelib::Transform2D::rotation() const
{
    return theta;
}


std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Transform2D & tf)
{
    os << "deg: " << turtlelib::rad2deg(tf.rotation()) << " x: " << tf.translation().x << " y: " << tf.translation().y;

    return os;
}

std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Transform2D & tf)
{
    turtlelib::Vector2D v;
    double omega = 0.0;
    is >> omega >> v.x >> v.y;
    omega = turtlelib::deg2rad(omega);
    tf = turtlelib::Transform2D(v, omega);

    return is;
}


turtlelib::Transform2D turtlelib::operator*(turtlelib::Transform2D lhs, const turtlelib::Transform2D & rhs)
{
    lhs *= rhs;
    return lhs;
}

turtlelib::Transform2D turtlelib::integrate_twist(const turtlelib::Twist2D & twist)
{
    turtlelib::Transform2D Tbbp;
    
    if (turtlelib::almost_equal(twist.omega, 0.0, 1.0e-12))
    {
        turtlelib::Vector2D v;
        v.x = twist.x;
        v.y = twist.y;

        Tbbp = turtlelib::Transform2D(v);
    }
    else if (turtlelib::almost_equal(twist.x, 0.0, 1.0e-12) && turtlelib::almost_equal(twist.y, 0.0, 1.0e-12))
    {
        Tbbp = turtlelib::Transform2D(twist.omega);
    }
    else
    {
        turtlelib::Transform2D Tsb;
        turtlelib::Vector2D v;
        v.x = twist.y / twist.omega;
        v.y = -1 * twist.x / twist.omega;
        Tsb = turtlelib::Transform2D(v);

        turtlelib::Transform2D Tssp;
        Tssp = turtlelib::Transform2D(twist.omega);

        Tbbp = Tsb.inv() * Tssp * Tsb;

    }

    return Tbbp;
}