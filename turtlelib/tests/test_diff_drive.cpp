#include "turtlelib/svg.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath> // contains math functions
#include <iostream> // contains iostream objects
#include <fstream>
#include <cstdlib> 
#include <vector>

using namespace turtlelib;


TEST_CASE("Matts Office Hours", "OfficeHours")
{
    double wheel_track = 1.0;
    double wheel_radius = 0.5;

    DiffDrive dd(wheel_track, wheel_radius);

    WheelPositions wp;
    wp.phi_left_ = PI/4.0;
    wp.phi_right_ = PI/4.0;

    dd.update_configuration(wp);

    Configuration config;
    config.x_ = PI/8.0;
    config.y_ = 0.0;
    config.theta_ = 0.0;

    Configuration dd_config = dd.get_configuration();

    std::cout << "First step" << std::endl;
    std::cout << "dd_config.x_: " << dd_config.x_ << std::endl;
    std::cout << "dd_config.y_: " << dd_config.y_ << std::endl;
    std::cout << "dd_config.theta_: " << dd_config.theta_ << std::endl;

    REQUIRE_THAT(dd_config.x_, Catch::Matchers::WithinAbs(config.x_, 1.0e-12));
    REQUIRE_THAT(dd_config.y_, Catch::Matchers::WithinAbs(config.y_, 1.0e-12));
    REQUIRE_THAT(dd_config.theta_, Catch::Matchers::WithinAbs(config.theta_, 1.0e-12));


    wp.phi_left_ = PI/2.0;
    wp.phi_right_ = PI/2.0;

    dd.update_configuration(wp);

    config.x_ = PI/4.0;
    config.y_ = 0.0;
    config.theta_ = 0.0;

    dd_config = dd.get_configuration();

    std::cout << "Second step" << std::endl;
    std::cout << "dd_config.x_: " << dd_config.x_ << std::endl;
    std::cout << "dd_config.y_: " << dd_config.y_ << std::endl;
    std::cout << "dd_config.theta_: " << dd_config.theta_ << std::endl;

    REQUIRE_THAT(dd_config.x_, Catch::Matchers::WithinAbs(config.x_, 1.0e-12));
    REQUIRE_THAT(dd_config.y_, Catch::Matchers::WithinAbs(config.y_, 1.0e-12));
    REQUIRE_THAT(dd_config.theta_, Catch::Matchers::WithinAbs(config.theta_, 1.0e-12));


    wp.phi_left_ = 3.0*PI/4.0;
    wp.phi_right_ = 3.0*PI/4.0;

    dd.update_configuration(wp);

    config.x_ = 3.0*PI/8.0;
    config.y_ = 0.0;
    config.theta_ = 0.0;

    dd_config = dd.get_configuration();

    std::cout << "Third step" << std::endl;
    std::cout << "dd_config.x_: " << dd_config.x_ << std::endl;
    std::cout << "dd_config.y_: " << dd_config.y_ << std::endl;
    std::cout << "dd_config.theta_: " << dd_config.theta_ << std::endl;

    REQUIRE_THAT(dd_config.x_, Catch::Matchers::WithinAbs(config.x_, 1.0e-12));
    REQUIRE_THAT(dd_config.y_, Catch::Matchers::WithinAbs(config.y_, 1.0e-12));
    REQUIRE_THAT(dd_config.theta_, Catch::Matchers::WithinAbs(config.theta_, 1.0e-12));


    wp.phi_left_ = 3.0*PI/4.0;
    wp.phi_right_ = 3.0*PI/4.0;

    dd.update_configuration(wp);

    config.x_ = 3.0*PI/8.0;
    config.y_ = 0.0;
    config.theta_ = 0.0;

    dd_config = dd.get_configuration();

    std::cout << "Fourth step" << std::endl;
    std::cout << "dd_config.x_: " << dd_config.x_ << std::endl;
    std::cout << "dd_config.y_: " << dd_config.y_ << std::endl;
    std::cout << "dd_config.theta_: " << dd_config.theta_ << std::endl;

    REQUIRE_THAT(dd_config.x_, Catch::Matchers::WithinAbs(config.x_, 1.0e-12));
    REQUIRE_THAT(dd_config.y_, Catch::Matchers::WithinAbs(config.y_, 1.0e-12));
    REQUIRE_THAT(dd_config.theta_, Catch::Matchers::WithinAbs(config.theta_, 1.0e-12));


    wp.phi_left_ = PI/2.0;
    wp.phi_right_ = PI/2.0;

    dd.update_configuration(wp);

    config.x_ = PI/4.0;
    config.y_ = 0.0;
    config.theta_ = 0.0;

    dd_config = dd.get_configuration();

    std::cout << "Fifth step" << std::endl;
    std::cout << "dd_config.x_: " << dd_config.x_ << std::endl;
    std::cout << "dd_config.y_: " << dd_config.y_ << std::endl;
    std::cout << "dd_config.theta_: " << dd_config.theta_ << std::endl;

    REQUIRE_THAT(dd_config.x_, Catch::Matchers::WithinAbs(config.x_, 1.0e-12));
    REQUIRE_THAT(dd_config.y_, Catch::Matchers::WithinAbs(config.y_, 1.0e-12));
    REQUIRE_THAT(dd_config.theta_, Catch::Matchers::WithinAbs(config.theta_, 1.0e-12));


    // REQUIRE(0 == 1);
}


TEST_CASE("Test DiffDrive", "[DiffDrive]")
{
    double wheel_track = 1.0;
    double wheel_radius = 0.5;

    DiffDrive dd(wheel_track, wheel_radius);

    WheelPositions wp;
    wp.phi_left_ = 0.0;
    wp.phi_right_ = 0.0;

    Configuration config;
    config.x_ = 0.0;
    config.y_ = 0.0;
    config.theta_ = 0.0;

    dd.update_configuration(wp);

    REQUIRE_THAT(dd.get_configuration().x_, Catch::Matchers::WithinAbs(config.x_, 1.0e-12));
    REQUIRE_THAT(dd.get_configuration().y_, Catch::Matchers::WithinAbs(config.y_, 1.0e-12));
    REQUIRE_THAT(dd.get_configuration().theta_, Catch::Matchers::WithinAbs(config.theta_, 1.0e-12));

    Twist2D twist;
    twist.omega = config.theta_;
    twist.x = config.x_;
    twist.y = config.y_;

    dd = DiffDrive(wheel_track, wheel_radius);

    REQUIRE_THAT(dd.twist_to_wheel_velocities(twist).phi_left_, Catch::Matchers::WithinAbs(wp.phi_left_, 1.0e-12));
    REQUIRE_THAT(dd.twist_to_wheel_velocities(twist).phi_right_, Catch::Matchers::WithinAbs(wp.phi_right_, 1.0e-12));



    dd = DiffDrive(wheel_track, wheel_radius);

    wp.phi_left_ = PI/2.0;
    wp.phi_right_ = PI/2.0;
    dd.update_configuration(wp);
    
    config.x_ = PI/2.0*wheel_radius;
    config.y_ = 0.0;
    config.theta_ = 0.0;

    REQUIRE_THAT(dd.get_configuration().x_, Catch::Matchers::WithinAbs(config.x_, 1.0e-12));
    REQUIRE_THAT(dd.get_configuration().y_, Catch::Matchers::WithinAbs(config.y_, 1.0e-12));
    REQUIRE_THAT(dd.get_configuration().theta_, Catch::Matchers::WithinAbs(config.theta_, 1.0e-12));

    twist.omega = config.theta_;
    twist.x = config.x_;
    twist.y = config.y_;

    dd = DiffDrive(wheel_track, wheel_radius);

    REQUIRE_THAT(dd.twist_to_wheel_velocities(twist).phi_left_, Catch::Matchers::WithinAbs(wp.phi_left_, 1.0e-12));
    REQUIRE_THAT(dd.twist_to_wheel_velocities(twist).phi_right_, Catch::Matchers::WithinAbs(wp.phi_right_, 1.0e-12));



    dd = DiffDrive(wheel_track, wheel_radius);

    wp.phi_left_ = -PI/2.0;
    wp.phi_right_ = -PI/2.0;
    dd.update_configuration(wp);

    config.x_ = -PI/2.0*wheel_radius;
    config.y_ = 0.0;
    config.theta_ = 0.0;

    REQUIRE_THAT(dd.get_configuration().x_, Catch::Matchers::WithinAbs(config.x_, 1.0e-12));
    REQUIRE_THAT(dd.get_configuration().y_, Catch::Matchers::WithinAbs(config.y_, 1.0e-12));
    REQUIRE_THAT(dd.get_configuration().theta_, Catch::Matchers::WithinAbs(config.theta_, 1.0e-12));

    twist.omega = config.theta_;
    twist.x = config.x_;
    twist.y = config.y_;

    dd = DiffDrive(wheel_track, wheel_radius);

    REQUIRE_THAT(dd.twist_to_wheel_velocities(twist).phi_left_, Catch::Matchers::WithinAbs(wp.phi_left_, 1.0e-12));
    REQUIRE_THAT(dd.twist_to_wheel_velocities(twist).phi_right_, Catch::Matchers::WithinAbs(wp.phi_right_, 1.0e-12));



    dd = DiffDrive(wheel_track, wheel_radius);

    wp.phi_left_ = PI/2.0;
    wp.phi_right_ = -1.0 * PI/2.0;
    dd.update_configuration(wp);

    config.x_ = 0.0;
    config.y_ = 0.0;
    config.theta_ = -PI/2.0;

    REQUIRE_THAT(dd.get_configuration().x_, Catch::Matchers::WithinAbs(config.x_, 1.0e-12));
    REQUIRE_THAT(dd.get_configuration().y_, Catch::Matchers::WithinAbs(config.y_, 1.0e-12));
    REQUIRE_THAT(dd.get_configuration().theta_, Catch::Matchers::WithinAbs(config.theta_, 1.0e-12));

    twist.omega = config.theta_;
    twist.x = config.x_;
    twist.y = config.y_;

    dd = DiffDrive(wheel_track, wheel_radius);

    REQUIRE_THAT(dd.twist_to_wheel_velocities(twist).phi_left_, Catch::Matchers::WithinAbs(wp.phi_left_, 1.0e-12));
    REQUIRE_THAT(dd.twist_to_wheel_velocities(twist).phi_right_, Catch::Matchers::WithinAbs(wp.phi_right_, 1.0e-12));



    dd = DiffDrive(wheel_track, wheel_radius);

    wp.phi_left_ = -PI/2.0;
    wp.phi_right_ = PI/2.0;
    dd.update_configuration(wp);

    config.x_ = 0.0;
    config.y_ = 0.0;
    config.theta_ = PI/2.0;

    REQUIRE_THAT(dd.get_configuration().x_, Catch::Matchers::WithinAbs(config.x_, 1.0e-12));
    REQUIRE_THAT(dd.get_configuration().y_, Catch::Matchers::WithinAbs(config.y_, 1.0e-12));
    REQUIRE_THAT(dd.get_configuration().theta_, Catch::Matchers::WithinAbs(config.theta_, 1.0e-12));

    twist.omega = config.theta_;
    twist.x = config.x_;
    twist.y = config.y_;

    dd = DiffDrive(wheel_track, wheel_radius);

    REQUIRE_THAT(dd.twist_to_wheel_velocities(twist).phi_left_, Catch::Matchers::WithinAbs(wp.phi_left_, 1.0e-12));
    REQUIRE_THAT(dd.twist_to_wheel_velocities(twist).phi_right_, Catch::Matchers::WithinAbs(wp.phi_right_, 1.0e-12));



    dd = DiffDrive(wheel_track, wheel_radius);

    wp.phi_left_ = PI/2.0;
    wp.phi_right_ = PI/4.0;
    dd.update_configuration(wp);

    config.x_ = -3.0/2.0 * sin(-PI/8.0);
    config.y_ = -3.0/2.0 + 3.0/2.0 * cos(-PI/8.0);
    config.theta_ = -PI/8.0;

    REQUIRE_THAT(dd.get_configuration().x_, Catch::Matchers::WithinAbs(config.x_, 1.0e-12));
    REQUIRE_THAT(dd.get_configuration().y_, Catch::Matchers::WithinAbs(config.y_, 1.0e-12));
    REQUIRE_THAT(dd.get_configuration().theta_, Catch::Matchers::WithinAbs(config.theta_, 1.0e-12));

    twist.omega = -PI/8.0;
    twist.x = 3.0*PI/16.0;
    twist.y = 0.0;

    dd = DiffDrive(wheel_track, wheel_radius);

    REQUIRE_THAT(dd.twist_to_wheel_velocities(twist).phi_left_, Catch::Matchers::WithinAbs(wp.phi_left_, 1.0e-12));
    REQUIRE_THAT(dd.twist_to_wheel_velocities(twist).phi_right_, Catch::Matchers::WithinAbs(wp.phi_right_, 1.0e-12));



    dd = DiffDrive(wheel_track, wheel_radius);

    wp.phi_left_ = -PI/2.0;
    wp.phi_right_ = -PI/4.0;
    dd.update_configuration(wp);

    config.x_ = 3.0/2.0 * sin(-PI/8.0);
    config.y_ = -3.0/2.0 + 3.0/2.0 * cos(-PI/8.0);
    config.theta_ = PI/8.0;

    REQUIRE_THAT(dd.get_configuration().x_, Catch::Matchers::WithinAbs(config.x_, 1.0e-12));
    REQUIRE_THAT(dd.get_configuration().y_, Catch::Matchers::WithinAbs(config.y_, 1.0e-12));
    REQUIRE_THAT(dd.get_configuration().theta_, Catch::Matchers::WithinAbs(config.theta_, 1.0e-12));

    twist.omega = PI/8.0;
    twist.x = -3.0*PI/16.0;
    twist.y = 0.0;

    dd = DiffDrive(wheel_track, wheel_radius);

    REQUIRE_THAT(dd.twist_to_wheel_velocities(twist).phi_left_, Catch::Matchers::WithinAbs(wp.phi_left_, 1.0e-12));
    REQUIRE_THAT(dd.twist_to_wheel_velocities(twist).phi_right_, Catch::Matchers::WithinAbs(wp.phi_right_, 1.0e-12));


    // TODO: Add test for unfollowable twist

}
