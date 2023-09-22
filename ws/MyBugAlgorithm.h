#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

class my_ConvH_class
{
public:
    std::vector<std::vector<double>> A;
    std::vector<double> b;
    my_ConvH_class() = default;
    inline my_ConvH_class(std::vector<std::vector<double>>& A, std::vector<double>& b)
    {
        this-> A = A;
        this-> b = b;
    }
};

class my_primitives_class
{
public:
    std::vector<my_ConvH_class> my_ConvHs;
    //my_primitives_class(/* args */);
};

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& my_problem) override;

        // Add any other methods here...

        /// @brief Computes distance between @param x and @param y
        double my_dist(Eigen::Vector2d& x, Eigen::Vector2d& y);

        my_ConvH_class my_ConvH_function(amp::Polygon& my_polygon);

        my_primitives_class my_make_primitives(const amp::Problem2D& my_problem);

        Eigen::Vector2d my_move_function(Eigen::Vector2d& q,double& heading, double& step_size, std::string forward_or_left);

        unsigned int my_sense_walls(my_primitives_class &my_primitives, Eigen::Vector2d &q, double &heading, double& step_size);
        /*
        Old stuff
                /// @brief Checks if a line defined by points @param v1 and @param v2 intersects an edge
        /// @brief defined by @param u1 and @param u2. @param v1 is usually the position of the robot and
        /// @brief @param v2 is usually the goal
        /// @return (-1000.0,-1000.0) if no collision or point of collision if collision
        Eigen::Vector2d my_check_interection_with_edge(Eigen::Vector2d& u1, Eigen::Vector2d& u2, Eigen::Vector2d& v1, Eigen::Vector2d& v2);
        
        /// @brief Move to goal in a straight line. If goal is achieved, 
        my_current_obstacle my_move_to_target(const amp::Problem2D& problem, Eigen::Vector2d& q, Eigen::Vector2d& q_target, bool& following_obstacle, my_collision_class& my_collision_data);

        
        */

    private:
        // Add any member variables here...
};


/*
Old stuff
class my_collision_class
{
public:
    amp::Polygon my_polygonH;
    Eigen::Vector2d qH;
    Eigen::Vector2d my_vertex_in_front;
    Eigen::Vector2d my_vertex_at_back;
    Eigen::Vector2d q_min_dist_inrange_q_qH;
    double dist_min_inrange_q_qH;

    ///@brief Constructor of the class
    inline my_collision_class(Eigen::Vector2d& qH)
    {
        this-> qH = qH;
    }
    inline my_collision_class(amp::Polygon& my_polygonH, Eigen::Vector2d& qH, Eigen::Vector2d& my_vertex_in_front, Eigen::Vector2d& my_vertex_at_back, Eigen::Vector2d& q_min_dist_inrange_q_qH, double& dist_min_inrange_q_qH)
    {
        this-> my_polygonH = my_polygonH;
        this-> qH = qH;
        this-> my_vertex_in_front = my_vertex_in_front;
        this-> my_vertex_at_back = my_vertex_at_back;
        this-> q_min_dist_inrange_q_qH = q_min_dist_inrange_q_qH;
        this-> dist_min_inrange_q_qH = dist_min_inrange_q_qH;
    }
};

*/