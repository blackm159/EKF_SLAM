#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include <vector>

TEST_CASE("Test operator<<", "[se2d_operator<<]")
{
    turtlelib::Twist2D tw;
    tw.x = 0.0;
    tw.y = 0.0;
    tw.omega = 0.0;
    std::stringstream ss;

    ss << tw;
    REQUIRE(ss.str() == "[0 0 0]");


    turtlelib::Vector2D trans;
    trans.x = 1.0;
    trans.y = 2.0;
    double radians = turtlelib::deg2rad(90.0);
    turtlelib::Transform2D T;
    T = turtlelib::Transform2D(trans, radians);

    std::stringstream ss2;
    ss2 << T;
    REQUIRE(ss2.str() == "deg: 90 x: 1 y: 2");

}


TEST_CASE("Test operator>>", "[se2d_operator>>]")
{
    turtlelib::Twist2D tw;
    std::stringstream ss;

    ss << "[0 1 2]";
    ss >> tw;
    REQUIRE_THAT(tw.omega, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(tw.x, Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(tw.y, Catch::Matchers::WithinAbs(2.0, 1.0e-12));

    ss << "3 4 5";
    ss >> tw;
    REQUIRE_THAT(tw.omega, Catch::Matchers::WithinAbs(3.0, 1.0e-12));
    REQUIRE_THAT(tw.x, Catch::Matchers::WithinAbs(4.0, 1.0e-12));
    REQUIRE_THAT(tw.y, Catch::Matchers::WithinAbs(5.0, 1.0e-12));

    turtlelib::Transform2D tf;
    std::stringstream ss2;
    ss2 << "90 1 2";
    ss2 >> tf;
    double x = tf.translation().x;
    double y = tf.translation().y;
    double angle = turtlelib::rad2deg(tf.rotation());
    // angle = turtlelib::rad2deg(angle);

    REQUIRE_THAT(angle, Catch::Matchers::WithinAbs(90.0, 1.0e-12));
    REQUIRE_THAT(x, Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(y, Catch::Matchers::WithinAbs(2.0, 1.0e-12));
}

TEST_CASE("Test Transform2D", "[Transform2D]")
{
    turtlelib::Transform2D T;

    REQUIRE_THAT(T.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0e-12));

    turtlelib::Vector2D trans;
    trans.x = 1.0;
    trans.y = 2.0;
    T = turtlelib::Transform2D(trans);

    REQUIRE_THAT(T.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(2.0, 1.0e-12));

    double radians = turtlelib::PI / 2.0;
    T = turtlelib::Transform2D(radians);

    REQUIRE_THAT(T.rotation(), Catch::Matchers::WithinAbs(radians, 1.0e-12));
    REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0e-12));


    T = turtlelib::Transform2D(trans, radians);

    REQUIRE_THAT(T.rotation(), Catch::Matchers::WithinAbs(radians, 1.0e-12));
    REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(trans.x, 1.0e-12));
    REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(trans.y, 1.0e-12));

}

TEST_CASE("Test operator()", "[operator]")
{

    turtlelib::Point2D p;
    p.x = 1.0;
    p.y = 2.0;

    turtlelib::Transform2D T;
    turtlelib::Vector2D trans;
    trans.x = 1.0;
    trans.y = 2.0;
    double radians = turtlelib::deg2rad(90.0);
    T = turtlelib::Transform2D(trans, radians);

    turtlelib::Point2D newPoint = T(p);

    REQUIRE_THAT(newPoint.x, Catch::Matchers::WithinAbs(-1.0, 1.0e-12));
    REQUIRE_THAT(newPoint.y, Catch::Matchers::WithinAbs(3.0, 1.0e-12));


    turtlelib::Vector2D v;
    v.x = 1.0;
    v.y = 2.0;

    turtlelib::Vector2D newVector = T(v);

    REQUIRE_THAT(newVector.x, Catch::Matchers::WithinAbs(-2.0, 1.0e-12));
    REQUIRE_THAT(newVector.y, Catch::Matchers::WithinAbs(1.0, 1.0e-12));


    turtlelib::Twist2D v2;
    v2.x = 1.0;
    v2.y = 2.0;
    v2.omega = 0.0;

    double omega = turtlelib::deg2rad(90.0);
    T = turtlelib::Transform2D(omega);

    turtlelib::Twist2D newTwist = T(v2);

    REQUIRE_THAT(newTwist.x, Catch::Matchers::WithinAbs(-2.0, 1.0e-12));
    REQUIRE_THAT(newTwist.y, Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(newTwist.omega, Catch::Matchers::WithinAbs(0.0, 1.0e-12));

}

TEST_CASE("Test inv()", "[inv]")
{

    turtlelib::Transform2D T;
    double radians = turtlelib::deg2rad(90.0);
    turtlelib::Vector2D v;
    v.x = 1.0;
    v.y = 2.0;
    T = turtlelib::Transform2D(v, radians);



    turtlelib::Transform2D Tinv = T.inv();

    // double radians = T.rotation();
    // turtlelib::Vector2D trans = T.translation();

    REQUIRE_THAT(Tinv.rotation(), Catch::Matchers::WithinAbs(-1*radians, 1.0e-12));
    REQUIRE_THAT(Tinv.translation().x, Catch::Matchers::WithinAbs(-2.0, 1.0e-12));
    REQUIRE_THAT(Tinv.translation().y, Catch::Matchers::WithinAbs(1.0, 1.0e-12));


}

TEST_CASE("Test operator*=", "[operator*=]")
{
    turtlelib::Vector2D v;
    v.x = 1.0;
    v.y = 2.0;
    double theta = turtlelib::deg2rad(90.0);

    turtlelib::Transform2D T = turtlelib::Transform2D(v, theta);

    turtlelib::Vector2D v2;
    v2.x = 3.0;
    v2.y = 4.0;
    double theta2 = turtlelib::deg2rad(180.0);
    turtlelib::Transform2D T1 = turtlelib::Transform2D(v2, theta2);

    T*=T1;
    double gamma = turtlelib::rad2deg(T.rotation());
    REQUIRE_THAT(gamma, Catch::Matchers::WithinAbs(270.0, 1.0e-12));
    REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(-3.0, 1.0e-12));
    REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(5.0, 1.0e-12));

}


TEST_CASE("Test operator*", "[operator*]")
{
    turtlelib::Vector2D v;
    v.x = 1.0;
    v.y = 2.0;
    double theta = turtlelib::deg2rad(90.0);

    turtlelib::Transform2D T1 = turtlelib::Transform2D(v, theta);

    turtlelib::Vector2D v2;
    v2.x = 3.0;
    v2.y = 4.0;
    double theta2 = turtlelib::deg2rad(180.0);
    turtlelib::Transform2D T2 = turtlelib::Transform2D(v2, theta2);

    turtlelib::Transform2D T;
    T = T1 * T2;
    double gamma = turtlelib::rad2deg(T.rotation());
    REQUIRE_THAT(gamma, Catch::Matchers::WithinAbs(270.0, 1.0e-12));
    REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(-3.0, 1.0e-12));
    REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(5.0, 1.0e-12));
}


TEST_CASE("Test integrate_twist", "[integrade_twist]")
{
    turtlelib::Twist2D tw;
    tw.x = 1.0;
    tw.y = 2.0;
    tw.omega = 0.0;

    turtlelib::Transform2D T;
    T = turtlelib::integrate_twist(tw);

    REQUIRE_THAT(T.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(2.0, 1.0e-12));

    tw.x = 0.0;
    tw.y = 0.0;
    tw.omega = turtlelib::deg2rad(90.0);

    T = turtlelib::integrate_twist(tw);

    REQUIRE_THAT(T.rotation(), Catch::Matchers::WithinAbs(turtlelib::deg2rad(90.0), 1.0e-12));
    REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0e-12));

    tw.x = 1.0;
    tw.y = 2.0;
    tw.omega = turtlelib::deg2rad(90.0);

    T = turtlelib::integrate_twist(tw);

    REQUIRE_THAT(T.rotation(), Catch::Matchers::WithinAbs(turtlelib::deg2rad(90.0), 1.0e-12));
    REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(-2.0/turtlelib::PI, 1.0e-12));
    REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(6.0/turtlelib::PI, 1.0e-12));

}
