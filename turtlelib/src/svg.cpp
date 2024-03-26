#include "turtlelib/svg.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath> // contains math functions
#include <iostream> // contains iostream objects
#include <fstream>
#include <cstdlib> 
#include <vector>

turtlelib::SVG::SVG()
{
    // turtlelib::Vector2D v;
    // v.x = 8.5/2.0;
    // v.y = 11.0/2.0;
    // T_to_pixel = turtlelib::Transform2D(v);
    // T_to_pixel.matrix[1][1] = -1*96;
    // T_to_pixel.matrix[0][0] = 1*96;


    svg_string = "\n<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    svg_string += "\n<defs>";
    svg_string += "\n\t<marker";
    svg_string += "\n\t\tstyle=\"overflow:visible\"";
    svg_string += "\n\t\tid=\"Arrow1Sstart\"";
    svg_string += "\n\t\trefX=\"0.0\"";
    svg_string += "\n\t\trefY=\"0.0\"";
    svg_string += "\n\t\torient=\"auto\">";
    svg_string += "\n\t\t\t<path";
    svg_string += "\n\t\t\t\ttransform=\"scale(0.2) translate(6,0)\"";
    svg_string += "\n\t\t\t\tstyle=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"";
    svg_string += "\n\t\t\t\td=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"";
    svg_string += "/>";
    svg_string += "\n\t</marker>";
    svg_string += "\n</defs>";

    svg_string += "\n\n";
}

void turtlelib::SVG::Draw(turtlelib::Point2D p, std::string color)
{
    turtlelib::Point2D p_pixel;
    p_pixel.x = 96.0*p.x + 8.5/2.0*96;
    p_pixel.y = -96.0*p.y + 11/2.0*96;
    svg_string += "\n<circle";
    svg_string += " cx=\"" + std::to_string(p_pixel.x) + "\"";
    svg_string += " cy=\"" + std::to_string(p_pixel.y) + "\"";
    svg_string += " r=\"3\"";
    svg_string += " stroke=\"" + color + "\"";
    svg_string += " fill=\"" + color + "\"";
    svg_string += " stroke-width=\"1\"";
    svg_string += "/>";
    svg_string += "\n\n";
}

void turtlelib::SVG::Draw(turtlelib::Point2D head, turtlelib::Point2D tail, std::string color)
{
    turtlelib::Point2D head_pixel;
    head_pixel.x = 96.0*head.x + 8.5/2.0*96;
    head_pixel.y = -96.0*head.y + 11/2.0*96;
    turtlelib::Point2D tail_pixel;
    tail_pixel.x = 96.0*tail.x + 8.5/2.0*96;
    tail_pixel.y = -96.0*tail.y + 11/2.0*96;
    
    svg_string += "\n<line";
    svg_string += " x1=\"" + std::to_string(head_pixel.x) + "\"";
    svg_string += " y1=\"" + std::to_string(head_pixel.y) + "\"";
    svg_string += " x2=\"" + std::to_string(tail_pixel.x) + "\"";
    svg_string += " y2=\"" + std::to_string(tail_pixel.y) + "\"";
    svg_string += " stroke=\"" + color + "\"";
    svg_string += " stroke-width=\"5\"";
    svg_string += " marker-start=\"url(#Arrow1Sstart)\"";
    svg_string += "/>";
    // svg_string += "\n\n";
}

void turtlelib::SVG::Draw(turtlelib::Transform2D T, std::string name)
{
    turtlelib::Point2D x_head;
    turtlelib::Point2D x_tail;
    turtlelib::Point2D y_head;
    turtlelib::Point2D y_tail;

    x_head.x = 1.0;
    x_head.y = 0.0;
    x_tail.x = 0.0;
    x_tail.y = 0.0;
    y_head.x = 0.0;
    y_head.y = 1.0;
    y_tail.x = 0.0;
    y_tail.y = 0.0;

    x_head = T(x_head);
    x_tail = T(x_tail);
    y_head = T(y_head);
    y_tail = T(y_tail);

    turtlelib::Point2D origin;
    // origin.x = x_tail.x;
    // origin.y = x_tail.y;
    // origin = T_to_pixel(x_tail);
    origin.x = x_tail.x*96 + 8.5/2.0*96;
    origin.y = -96.0*x_tail.y + 11/2.0*96;


    svg_string += "\n\n\n<g>";
    Draw(x_head, x_tail, "red");
    Draw(y_head, y_tail, "green");
    svg_string += "\n<text x=\"" + std::to_string(origin.x) + "\" y=\"" + std::to_string(origin.y+0.25) + "\">" + "{" + name + "}" + "</text>";
    svg_string += "\n</g>";
}

std::string turtlelib::SVG::Export()
{
    svg_string += "\n</svg>";
    return svg_string;
}

void turtlelib::SVG::Export(std::string filename)
{
    svg_string += "\n</svg>";
    std::ofstream file;
    file.open(filename);
    file << svg_string;
    file.close();
}