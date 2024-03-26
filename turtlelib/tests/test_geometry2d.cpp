#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>

TEST_CASE("Test almost_equal", "[almost_equal]")
{
    REQUIRE(turtlelib::almost_equal(0.0, 0.0) == true);
    REQUIRE(turtlelib::almost_equal(0.0, 1.0) == false);
    REQUIRE(turtlelib::almost_equal(0.0, 0.0000000000001) == true);
}


TEST_CASE("Test deg2rad", "[deg2rad]")
{
    REQUIRE_THAT(turtlelib::deg2rad(0.0), Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(turtlelib::deg2rad(180.0), Catch::Matchers::WithinAbs(turtlelib::PI, 1.0e-12));
    REQUIRE_THAT(turtlelib::deg2rad(360.0), Catch::Matchers::WithinAbs(2*turtlelib::PI, 1.0e-12));

}

TEST_CASE("Test rad2", "[rad2deg]")
{
    REQUIRE_THAT(turtlelib::rad2deg(0.0), Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(turtlelib::rad2deg(turtlelib::PI), Catch::Matchers::WithinAbs(180.0, 1.0e-12));
    REQUIRE_THAT(turtlelib::rad2deg(2*turtlelib::PI), Catch::Matchers::WithinAbs(360.0, 1.0e-12));

}

TEST_CASE("Test normalize_angle", "[normalize_angle]")
{
    REQUIRE_THAT(turtlelib::normalize_angle(turtlelib::PI), Catch::Matchers::WithinAbs(turtlelib::PI, 1.0e-12));
    REQUIRE_THAT(turtlelib::normalize_angle(-1*turtlelib::PI), Catch::Matchers::WithinAbs(turtlelib::PI, 1.0e-12));
    REQUIRE_THAT(turtlelib::normalize_angle(0.0), Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(turtlelib::normalize_angle(-1*turtlelib::PI/4), Catch::Matchers::WithinAbs(-1*turtlelib::PI/4, 1.0e-12));
    REQUIRE_THAT(turtlelib::normalize_angle(3*turtlelib::PI/2), Catch::Matchers::WithinAbs(-1*turtlelib::PI/2, 1.0e-12));
    REQUIRE_THAT(turtlelib::normalize_angle(-5*turtlelib::PI/2), Catch::Matchers::WithinAbs(-1*turtlelib::PI/2, 1.0e-12));

}

TEST_CASE("Test operator<<", "[operator<<]")
{
    turtlelib::Point2D p;
    p.x = 0.0;
    p.y = 0.0;
    std::stringstream ss;

    ss << p;
    REQUIRE(ss.str() == "[0 0]");

    turtlelib::Vector2D v;
    std::stringstream ss2;

    v.x = 1.0;
    v.y = 1.0;
    ss2 << v;
    REQUIRE(ss2.str() == "[1 1]");

}

TEST_CASE("Test operator>>", "[operator>>]")
{
    turtlelib::Point2D p;
    std::stringstream ss;

    ss << "[1 2]";
    ss >> p;
    REQUIRE_THAT(p.x, Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(p.y, Catch::Matchers::WithinAbs(2.0, 1.0e-12));

    ss << "3 4";
    ss >> p;
    REQUIRE_THAT(p.x, Catch::Matchers::WithinAbs(3.0, 1.0e-12));
    REQUIRE_THAT(p.y, Catch::Matchers::WithinAbs(4.0, 1.0e-12));


    turtlelib::Vector2D v;
    std::stringstream ss2;

    ss2 << "[1 2]";
    ss2 >> v;
    REQUIRE_THAT(v.x, Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(v.y, Catch::Matchers::WithinAbs(2.0, 1.0e-12));

    ss2 << "3 4";
    ss2 >> v;
    REQUIRE_THAT(v.x, Catch::Matchers::WithinAbs(3.0, 1.0e-12));
    REQUIRE_THAT(v.y, Catch::Matchers::WithinAbs(4.0, 1.0e-12));
}


TEST_CASE("Test operator-", "[operator-]")
{
    turtlelib::Point2D head;
    turtlelib::Point2D tail;
    turtlelib::Vector2D v;
    std::stringstream ss;

    head.x = 1.0;
    head.y = 1.0;
    tail.x = 2.0;
    tail.y = 2.0;
    v = head - tail;
    REQUIRE_THAT(v.x, Catch::Matchers::WithinAbs(-1.0, 1.0e-12));
    REQUIRE_THAT(v.y, Catch::Matchers::WithinAbs(-1.0, 1.0e-12));

    head.x = 1.0;
    head.y = 1.0;
    tail.x = 1.0;
    tail.y = 1.0;
    v = head - tail;
    REQUIRE_THAT(v.x, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(v.y, Catch::Matchers::WithinAbs(0.0, 1.0e-12));

}

TEST_CASE("Test operator+", "[operator+]")
{
    turtlelib::Point2D tail;
    turtlelib::Vector2D disp;
    turtlelib::Point2D p;
    std::stringstream ss;

    tail.x = 1.0;
    tail.y = 1.0;
    disp.x = 1.0;
    disp.y = 1.0;
    p = tail + disp;
    REQUIRE_THAT(p.x, Catch::Matchers::WithinAbs(2.0, 1.0e-12));
    REQUIRE_THAT(p.y, Catch::Matchers::WithinAbs(2.0, 1.0e-12));

    tail.x = 1.0;
    tail.y = 1.0;
    disp.x = 0.0;
    disp.y = 0.0;
    p = tail + disp;
    REQUIRE_THAT(p.x, Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(p.y, Catch::Matchers::WithinAbs(1.0, 1.0e-12));

}

TEST_CASE("Test vec_operator+", "[vec_operator+]")
{
    turtlelib::Vector2D v1;
    turtlelib::Vector2D v2;

    v1.x = 1.0;
    v1.y = 1.0;
    v2.x = 1.0;
    v2.y = 1.0;

    v1 += v2;

    REQUIRE_THAT(v1.x, Catch::Matchers::WithinAbs(2.0, 1.0e-12));
    REQUIRE_THAT(v1.y, Catch::Matchers::WithinAbs(2.0, 1.0e-12));

}

TEST_CASE("Test vec_operator-", "[vec_operator-]")
{
    turtlelib::Vector2D v1;
    turtlelib::Vector2D v2;

    v1.x = 1.0;
    v1.y = 1.0;
    v2.x = 1.0;
    v2.y = 1.0;

    v1 -= v2;

    REQUIRE_THAT(v1.x, Catch::Matchers::WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(v1.y, Catch::Matchers::WithinAbs(0.0, 1.0e-12));

}

TEST_CASE("Test vec_operator*", "[vec_operator*]")
{
    turtlelib::Vector2D v;
    double s = 2.0;

    v.x = 1.0;
    v.y = 1.0;

    v *= s;

    REQUIRE_THAT(v.x, Catch::Matchers::WithinAbs(2.0, 1.0e-12));
    REQUIRE_THAT(v.y, Catch::Matchers::WithinAbs(2.0, 1.0e-12));

}

TEST_CASE("Test dot", "[dot]")
{
    turtlelib::Vector2D v1;
    turtlelib::Vector2D v2;

    v1.x = 1.0;
    v1.y = 1.0;
    v2.x = 1.0;
    v2.y = 1.0;

    REQUIRE_THAT(turtlelib::dot(v1, v2), Catch::Matchers::WithinAbs(2.0, 1.0e-12));

}

TEST_CASE("Test magnitude", "[magnitude]")
{
    turtlelib::Vector2D v;

    v.x = 1.0;
    v.y = 1.0;

    REQUIRE_THAT(turtlelib::magnitude(v), Catch::Matchers::WithinAbs(sqrt(2.0), 1.0e-12));

}

TEST_CASE("Test angle", "[angle]")
{
    turtlelib::Vector2D v1;
    turtlelib::Vector2D v2;

    v1.x = 1.0;
    v1.y = 0.0;
    v2.x = 0.0;
    v2.y = 1.0;

    REQUIRE_THAT(turtlelib::angle(v1, v2), Catch::Matchers::WithinAbs(turtlelib::PI/2, 1.0e-12));

}



TEST_CASE("Test normalize", "[normalize]")
{
    turtlelib::Vector2D v;
    turtlelib::Vector2D v2;

    v.x = 1.0;
    v.y = 0.0;
    v2 = turtlelib::normalize(v);
    REQUIRE_THAT(v2.x, Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(v2.y, Catch::Matchers::WithinAbs(0.0, 1.0e-12));

}