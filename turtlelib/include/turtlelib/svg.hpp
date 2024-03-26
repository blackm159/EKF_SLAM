#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief SVG drawings.


#include<iosfwd> // contains forward definitions for iostream objects

#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"


namespace turtlelib
{

    /// \brief a svg drawing
    class SVG
    {
    private:
        // turtlelib::Transform2D T_to_pixel; // Transform from world to pixel coordinates
        // T_to_pixel = turtlelib::Transform2D(turtlelib::Vector2D(0.0, 0.0));

        std::string svg_string;

    public:
        /// \brief Create an svg drawing object
        SVG();
        // std::string svg_string;

        /// \brief add a point to the drawing
        /// \param p - the point to add
        /// \param color - the color of the point
        void Draw(turtlelib::Point2D p, std::string color);

        /// \brief add a vector to the drawing
        /// \param head - the first point
        /// \param tail - the second point
        /// \param color - the color of the line
        void Draw(turtlelib::Point2D head, turtlelib::Point2D tail, std::string color);

        /// \brief add a transform to the drawing
        /// \param T - the transform to add
        /// \param name - the name/label of the transform
        void Draw(turtlelib::Transform2D T, std::string name);

        /// \brief export SVG as string
        /// \returns the SVG as a string
        std::string Export();

        /// \brief export SVG to file
        /// \param filename - the name of the file to export to
        void Export(std::string filename);

    };


}

#endif
