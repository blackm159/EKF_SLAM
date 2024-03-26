#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include <vector>

TEST_CASE("Test SVG", "[SVG]")
{
    turtlelib::SVG svg;
    turtlelib::Point2D p;
    p.x = 0.0;
    p.y = 0.0;
    svg.Draw(p, "purple");
    turtlelib::Point2D p1;
    turtlelib::Point2D p2;
    p1.x = -2.0;
    p1.y = 1.0;
    p2.x = 2.0;
    p2.y = 4.0;
    svg.Draw(p1, p2, "blue");
    // svg.Draw(p1, "blue");
    // svg.Draw(p2, "green");

    turtlelib::Transform2D T;
    turtlelib::Vector2D v;
    v.x = -1.0;
    v.y = -2.0;
    T = turtlelib::Transform2D(v, 0.0);
    svg.Draw(T, "test");
    // svg.Export("test.svg");

    std::string test_string = "\n<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    test_string += "\n<defs>";
    test_string += "\n\t<marker";
    test_string += "\n\t\tstyle=\"overflow:visible\"";
    test_string += "\n\t\tid=\"Arrow1Sstart\"";
    test_string += "\n\t\trefX=\"0.0\"";
    test_string += "\n\t\trefY=\"0.0\"";
    test_string += "\n\t\torient=\"auto\">";
    test_string += "\n\t\t\t<path";
    test_string += "\n\t\t\t\ttransform=\"scale(0.2) translate(6,0)\"";
    test_string += "\n\t\t\t\tstyle=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"";
    test_string += "\n\t\t\t\td=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"";
    test_string += "/>";
    test_string += "\n\t</marker>";
    test_string += "\n</defs>";
    test_string += "\n\n";
    test_string += "\n<circle cx=\"408.000000\" cy=\"528.000000\" r=\"3\" stroke=\"purple\" fill=\"purple\" stroke-width=\"1\"/>";
    test_string += "\n\n";
    test_string += "\n<line x1=\"216.000000\" y1=\"432.000000\" x2=\"600.000000\" y2=\"144.000000\" stroke=\"blue\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\"/>";
    // test_string += "\n\n";
    test_string += "\n\n\n<g>";
    test_string += "\n<line x1=\"408.000000\" y1=\"720.000000\" x2=\"312.000000\" y2=\"720.000000\" stroke=\"red\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\"/>";
    test_string += "\n<line x1=\"312.000000\" y1=\"624.000000\" x2=\"312.000000\" y2=\"720.000000\" stroke=\"green\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\"/>";
    test_string += "\n<text x=\"312.000000\" y=\"720.250000\">{test}</text>";
    test_string += "\n</g>";
    test_string += "\n</svg>";


    REQUIRE(svg.Export() == test_string);

}