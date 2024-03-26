#ifndef TURTLELIB_DIFF_DRIVE_HPP_
#define TURTLELIB_DIFF_DRIVE_HPP_

#include "turtlelib/svg.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath> // contains math functions
#include <iostream> // contains iostream objects
#include <fstream>
#include <cstdlib> 
#include <vector>


namespace turtlelib
{

    /// \brief a struct to hold the wheel positions
    /// \details the wheel positions are in radians
    struct WheelPositions
    {
        /// \brief the left wheel position
        double phi_left_;

        /// \brief the right wheel position
        double phi_right_;
    };

    /// \brief a struct to hold the configuration of the robot
    /// \details the configuration is in the form (x, y, theta)
    struct Configuration
    {
        /// \brief the x position
        double x_;
        /// \brief the y position
        double y_;
        /// \brief the theta position
        double theta_;
    };


    /// \brief a differential drive class
    class DiffDrive
    {
    public:
        /// \brief Create an empty differential drive
        DiffDrive();

        /// \brief Create a differential drive with the given parameters
        /// \param wheel_track - the distance between the wheels
        /// \param wheel_radius - the radius of the wheels
        DiffDrive(double wheel_track, double wheel_radius);

        /// \brief update configuration and wheel positions
        /// \param new_wheel_pos - the new wheel positions
        void update_configuration(WheelPositions new_wheel_pos);

        /// \brief compute wheel velocities from a twist
        /// \param twist - the twist to compute the wheel velocities from
        /// \returns the wheel velocities
        WheelPositions twist_to_wheel_velocities(Twist2D twist);

        /// \brief get the current configuration
        /// \returns the current configuration
        Configuration get_configuration() const 
        { 
            return configuration_; 
        }

        /// \brief set the configuration
        /// \param configuration - the new configuration
        void set_configuration(Configuration configuration);

        /// \brief get the twist
        /// \returns the twist
        Twist2D get_twist() const 
        { 
            return twist_; 
        }


    private:
        double wheel_track_;
        double wheel_radius_;  

        WheelPositions wheel_positions_;
        Configuration configuration_;

        Twist2D twist_;


    };
}

#endif // TURTLELIB_DIFF_DRIVE_HPP_