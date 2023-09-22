#include "MyBugAlgorithm.h"
#include <cmath>

/// @brief Main logic of the algorithm.
/// @return A sequence of points that define the @param path

amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D &my_problem)
{

    amp::Path2D path;
    Eigen::Vector2d q_goal = my_problem.q_goal;
    Eigen::Vector2d q = my_problem.q_init;
    double step_size = 0.01;
    double sensor_range = 0.05;
    Eigen::Vector2d qp;
    Eigen::Vector2d qH;
    Eigen::Vector2d qL;
    Eigen::Vector2d qprev;
    Eigen::Vector2d qanchor;
    Eigen::Vector2d dir_to_goal;
    double dist_to_goal;
    double dist_qH_to_goal;
    double dist_qL_to_goal;
    double dist_to_qH;
    double dist_to_qL;
    double termination_threshold = 0.01;
    double closeness_threshold = 0.1;
    double heading;
    double Delta_heading = 0.1;   // Radians
    bool follow_obstacle = false; // First, start by moving towards goal in a straight line
    bool switch_to_follow_obstacle = false;
    bool returning_to_qL = false;
    unsigned int sensor_output = 4;

    // First obtain primitives
    my_primitives_class my_primitives = my_make_primitives(my_problem);

    /* DEBUGGING (GOOD): check if primitives are well stored
    // std::cout << "Size of my_primitives = " << my_primitives.my_ConvHs.size() <<std::endl;
    std::vector<std::vector<double>> A;
    std::vector<double> b;

    for (size_t o = 0; o < my_primitives.my_ConvHs.size(); o++) // Search over all obstacles
    {
        std::cout << "Primitives for obstacle " << o <<std::endl;
        A = my_primitives.my_ConvHs.at(o).A;
        b = my_primitives.my_ConvHs.at(o).b;
        for (size_t k = 0; k < b.size(); k++)
        {
            for (size_t j = 0; j < 2; j++)
            {
                std::cout << A.at(k).at(j) << " ";
            }
            std::cout << b.at(k) << std::endl;
        }
    }

    */

    unsigned int counter = 0;
    unsigned int counter_for_qH = 0;
    unsigned int counter_timeout = 50000;
    unsigned int counter_for_qH_th = 50;

    path.waypoints.push_back(q);
    do
    {
        if (follow_obstacle == false) // Move towards q_goal in a straight line
        {
            dist_to_goal = my_dist(q, q_goal);
            dir_to_goal = Eigen::Vector2d((q_goal(0) - q(0)) / dist_to_goal, (q_goal(1) - q(1)) / dist_to_goal);
            heading = atan2(q_goal(1) - q(1), q_goal(0) - q(0));

            // Check if no hit
            sensor_output = my_sense_walls(my_primitives, q, heading, sensor_range);
            //std::cout << "sensor_output = " << sensor_output << std::endl;
            if ((sensor_output == 3) || (sensor_output == 4)) // Either left hits or neither hit (no front hit)
            {
                qp = my_move_function(q, heading, step_size, "forward");
                path.waypoints.push_back(qp);
                q = qp;
                //std::cout << "qp = " << qp(0) << "," << qp(1) << " . Counter = " << counter << std::endl;
            }
            else
            {
                switch_to_follow_obstacle = true;
                follow_obstacle = true;
            }
        }
        else // Follow obstacle
        {
            if (switch_to_follow_obstacle) // First time I hit the obstacle
            {
                //std::cout << std::endl;
                std::cout << "Switch to follow obstacle " << std::endl;
                qH = q;
                dist_qH_to_goal = my_dist(q, q_goal);
                dist_qL_to_goal = dist_qH_to_goal;
                switch_to_follow_obstacle = false;
                counter_for_qH = 0;
            }
            else // I keep following the obstacle
            {
                // Check if q is the closest point (so far) to the goal
                dist_to_goal = my_dist(q, q_goal);
                if (dist_to_goal < dist_qL_to_goal)
                {
                    dist_qL_to_goal = dist_to_goal;
                    qL = q;
                }

                sensor_output = my_sense_walls(my_primitives, q, heading, sensor_range);
                //std::cout << "sensor_output = " << sensor_output << std::endl;
                switch (sensor_output)
                {
                case 1:                       // "Forward hits":
                    heading -= 3.141596/2.0; // Turn right
                    break;
                case 2:                       // "Both hit":
                    heading -= Delta_heading;//3.141596/2.0; // Turn 90 deg right
                    break;
                case 3: //"Left hits":
                    qp = my_move_function(q, heading, step_size, "forward"); // Go straight
                    //std::cout << "qp = " << qp(0) << "," << qp(1) << " . Counter = " << counter << std::endl;
                    path.waypoints.push_back(qp);
                    qprev = q;
                    q = qp;
                    break;
                case 4: //"Neither hit": turn left using anchor
                    q = qprev; // Go back one step
                    path.waypoints.push_back(q);
                    qanchor = my_move_function(q, heading, step_size, "left"); // Find the anchor point
                    heading += Delta_heading; // Turn left
                    qp = my_move_function(qanchor, heading, step_size, "right"); // Rotate around the anchor
                    path.waypoints.push_back(qp);
                    //std::cout << "qp = " << qp(0) << "," << qp(1) << " . Counter = " << counter << std::endl;
                    q = qp;
                    break;
                default:
                    break;
                }

                if (returning_to_qL)
                {
                    // Check if we have returned to qL
                    dist_to_qL = my_dist(q, qL);
                    if (dist_to_qL < closeness_threshold)
                    {
                        returning_to_qL = false;
                        follow_obstacle = false;
                        std::cout << std::endl;
                        std::cout << "Switch to move to goal " << std::endl;
                    }
                }
                else if(counter_for_qH == counter_for_qH_th)// Returning to qH
                {
                    // Check if we have returned to qH
                    dist_to_qH = my_dist(q, qH);
                    if (dist_to_qH < closeness_threshold)
                    {
                        returning_to_qL = true; // Return to qL
                    }
                }
            }
        }
        //std::cout << "follow_obstacle = " << follow_obstacle << ". q =" << qp(0) << "," << qp(1) << ". Heading = "<< heading << ". Counter = " << counter << std::endl;
        counter++;
        if(counter_for_qH < counter_for_qH_th)
        {
            counter_for_qH++;
        }
        
    } while ((dist_to_goal > termination_threshold) && (counter < counter_timeout));

    if (counter == counter_timeout) // Print a warning
    {
        std::cout << "WARNING: COUNTER TIMEOUT \n\n";
    }
    else
    {
        // q_goal reached!!
    }

    return path;
}

double MyBugAlgorithm::my_dist(Eigen::Vector2d &x, Eigen::Vector2d &y)
{
    return std::sqrt((x(0) - y(0)) * (x(0) - y(0)) + (x(1) - y(1)) * (x(1) - y(1)));
}

my_ConvH_class MyBugAlgorithm::my_ConvH_function(amp::Polygon &my_polygon)
{
    std::vector<Eigen::Vector2d> my_verticesCCW = my_polygon.verticesCCW();
    Eigen::Vector2d u;
    Eigen::Vector2d v;
    int n_vertices = my_verticesCCW.size();
    // double A[n_vertices](1);
    // double b[n_vertices];
    std::vector<std::vector<double>> A;
    std::vector<double> b;
    double m;
    double n;
    // std::cout << "I'm here (my_ConvH_function)." << std::endl;
    for (size_t i = 0; i < n_vertices - 1; i++) // Search for the closest collision with an edge of my_polygon
    {
        // std::cout << "i = " << i << std::endl;
        u = my_verticesCCW.at(i);
        v = my_verticesCCW.at(i + 1);
        // std::cout << u(0) << "," << u(1) << " " << v(0) << "," << v(1) << std::endl;
        if (u(0) != v(0)) // Not vertical edge
        {
            // std::cout << "Not a vertical edge!" << std::endl;
            m = (u(1) - v(1)) / (u(0) - v(0));
            n = u(1) - m * u(0);
            // std::cout << "m = " << m << std::endl;
            // std::cout << "n = " << n << std::endl;
            if (u(0) > v(0))
            {
                // A[i][0] = -m;
                // A[i][1] = 1.0;
                A.push_back({-m, 1.0});
                b.push_back(-n);
                // std::cout << "Push back done!" << std::endl;
            }
            else if (u(0) < v(0))
            {
                // A[i][0] = m;
                // A[i][1] = -1.0;
                A.push_back({m, -1.0});
                b.push_back(n);
                // std::cout << "Push back done!" << std::endl;
            }
        }
        else if (u(1) > v(1))
        {
            // A[i][0] = -1.0;
            // A[i][1] = 0.0;
            A.push_back({-1.0, 0.0});
            b.push_back(u(0));
        }
        else if (u(1) < v(1))
        {
            // A[i][0] = 1.0;
            //[i][1] = 0.0;
            A.push_back({1.0, 0.0});
            b.push_back(-u(0));
        }
    }
    // std::cout << "I'm here (my_make_primitives, before closing polytope)." << std::endl;
    //   Now close the polytope
    u = my_verticesCCW.back();
    v = my_verticesCCW.front();
    if (u(0) != v(0)) // Not vertical edge
    {
        m = (u(1) - v(1)) / (u(0) - v(0));
        n = u(1) - m * u(0);
        if (u(0) > v(0))
        {
            // A[n_vertices - 1][0] = -m;
            // A[n_vertices - 1][1] = 1.0;
            A.push_back({-m, 1.0});
            b.push_back(-n);
        }
        else if (u(0) < v(0))
        {
            // A[n_vertices - 1][0] = m;
            // A[n_vertices - 1][1] = -1.0;
            A.push_back({m, -1.0});
            b.push_back(n);
        }
    }
    else if (u(1) > v(1))
    {
        // A[n_vertices - 1][0] = -1.0;
        // A[n_vertices - 1][1] = 0.0;
        A.push_back({-1.0, 0.0});
        b.push_back(u(0));
    }
    else if (u(1) < v(1))
    {
        // A[n_vertices - 1][0] = 1.0;
        // A[n_vertices - 1][1] = 0.0;
        A.push_back({1.0, 0.0});
        b.push_back(-u(0));
    }

    // std::cout << "I'm here (my_make_primitives, after closing polytope)." << std::endl;
    return my_ConvH_class(A, b);
}

my_primitives_class MyBugAlgorithm::my_make_primitives(const amp::Problem2D &my_problem)
{
    my_primitives_class my_primitives;
    my_ConvH_class my_ConvH;
    amp::Polygon my_polygon;
    // std::cout << "I'm here (my_make_primitives)." << std::endl;

    unsigned int i = 0;
    for (auto it = my_problem.obstacles.begin(); it != my_problem.obstacles.end(); it++) // Search over all obstacles
    {
        my_polygon = *it;
        my_ConvH = my_ConvH_function(my_polygon);
        my_primitives.my_ConvHs.push_back(my_ConvH);
        /* Debugging: print primitives
        std::vector<Eigen::Vector2d> A;
        std::vector<double> b;
        A = my_ConvH.A;
        b = my_ConvH.b;
        std::cout << "Matrices for obstacle " << i << std::endl;
        for (size_t k = 0; k < b.size(); k++)
        {
            for (size_t j = 0; j < 2; j++)
            {
                std::cout << A.at(k)(j) << " ";
            }
                std::cout << b.at(k) << std::endl;
        }
        std::cout << "Primitives made for obstacle " << i << std::endl;
        */
        i++;
    }

    return my_primitives;
}

Eigen::Vector2d MyBugAlgorithm::my_move_function(Eigen::Vector2d &q, double &heading, double& step_size, std::string forward_or_left)
{
    Eigen::Vector2d qp;
    if (forward_or_left == "forward")
    {
        qp = Eigen::Vector2d(q(0) + step_size*cos(heading), q(1) + step_size*sin(heading));
        return qp;
    }
    else if (forward_or_left == "left")
    {
        qp = Eigen::Vector2d(q(0) - step_size*sin(heading), q(1) + step_size*cos(heading));
        return qp;
    }
    else if (forward_or_left == "right")
    {
        qp = Eigen::Vector2d(q(0) + step_size*sin(heading), q(1) - step_size*cos(heading));
        return qp;
    }
    else
    {
        std::cout << "WARNING: forward or left or right \n\n";
        return Eigen::Vector2d(-1000.0, -1000.0); // Default
    }
}


    unsigned int MyBugAlgorithm::my_sense_walls(my_primitives_class &my_primitives, Eigen::Vector2d &q, double &heading,double& step_size)
{
    // Check wether directions intersect with obstacle or not

    my_ConvH_class my_ConvHull;
    std::vector<std::vector<double>> A;
    std::vector<double> b;
    double f;
    double l;
    bool q_forward_hits;
    bool q_left_hits;
    Eigen::Vector2d q_forward;
    Eigen::Vector2d q_left;

    q_forward = my_move_function(q, heading, step_size, "forward");
    q_left = my_move_function(q, heading, step_size, "left");
    //std::cout << "Measuring: q = " << q(0) << "," << q(1) << "    q_forward = " << q_forward(0) << "," << q_forward(1) << std::endl;

    for (size_t o = 0; o < my_primitives.my_ConvHs.size(); o++) // Search over all obstacles
    {
        /*if ((q_forward(0)>3.0) && (q_forward(0)<3.1) )
        {
            std::cout << "I'm checking collision with obstacle o = " << o << std::endl;
        }
            */
        my_ConvHull = my_primitives.my_ConvHs.at(o);
        A = my_ConvHull.A;
        b = my_ConvHull.b;
        q_forward_hits = true;

        for (size_t i = 0; i < b.size(); i++)
        {
            f = A.at(i).at(0) * q_forward(0) + A.at(i).at(1) * q_forward(1) + b.at(i);
            if (f > 0)
            {
                q_forward_hits = false;
                break;
            }
        }
        /*
        if ((q_forward(0)>3.0) && (q_forward(0)<3.1) )
        {
            std::cout << "Front collision? = " << q_forward_hits << std::endl;
        }
        */
        if (q_forward_hits)
        {
            break;
        }
    }
    //std::cout << "After measuring: q = " << q(0) << "," << q(1) << "    q_forward = " << q_forward(0) << "," << q_forward(1) << std::endl;

    for (size_t o = 0; o < my_primitives.my_ConvHs.size(); o++) // Search over all obstacles
    {
        my_ConvHull = my_primitives.my_ConvHs.at(o);
        A = my_ConvHull.A;
        b = my_ConvHull.b;
        q_left_hits = true;

        for (size_t i = 0; i < b.size(); i++)
        {
            l = A.at(i).at(0) * q_left(0) + A.at(i).at(1) * q_left(1) + b.at(i);
            if (l > 0)
            {
                q_left_hits = false;
                break;
            }
        }
        if (q_left_hits)
        {
            break;
        }
    }

    // Return integer that correspond to the type of sensor output
    if ((q_forward_hits == true) && (q_left_hits == true))
    {
        return 2;
    }
    else if ((q_forward_hits == false) && (q_left_hits == true))
    {
        return 3;
    }
    else if ((q_forward_hits == true) && (q_left_hits == false))
    {
        return 1;
    }
    else if ((q_forward_hits == false) && (q_left_hits == false))
    {
        return 4;
    }

    std::cout << "WARNING: SENSOR FAIL" << std::endl;
    return 5;
}

// Old stuff

/*

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D &my_problem)
{
    amp::Path2D path;
    std::cin >> debug_key;


    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    //amp::Path2D path;
    path.waypoints.push_back(my_problem.q_init);
    path.waypoints.push_back(Eigen::Vector2d(1.0, 5.0));
    path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
    path.waypoints.push_back(my_problem.q_goal);

    return path;
}


amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D &my_problem)
{

    amp::Path2D path;
    Eigen::Vector2d q_goal = my_problem.q_goal;
    Eigen::Vector2d q = my_problem.q_init;
    double step_size = 0.1;
    Eigen::Vector2d qp;
    Eigen::Vector2d qH;
    Eigen::Vector2d qL;
    Eigen::Vector2d dir_to_goal;
    double dist_to_goal;
    double dist_qH_to_goal;
    double dist_qL_to_goal;
    double dist_to_qH;
    double dist_to_qL;
    double termination_threshold = 0.01;
    double closeness_threshold = 0.1;
    double heading;
    double Delta_heading = 0.1;   // Radians
    bool follow_obstacle = false; // First, start by moving towards goal in a straight line
    bool switch_to_follow_obstacle = false;
    bool returning_to_qL = false;
    unsigned int sensor_output = 4;

    // First obtain primitives
    my_primitives_class my_primitives = my_make_primitives(my_problem);

    unsigned int counter = 0;
    unsigned int counter_timeout = 100;

    path.waypoints.push_back(q);
    do
    {
        if (follow_obstacle == false) // Move towards q_goal in a straight line
        {
            dist_to_goal = my_dist(q, q_goal);
            dir_to_goal = Eigen::Vector2d((q_goal(0) - q(0)) / dist_to_goal, (q_goal(1) - q(1)) / dist_to_goal);
            heading = atan2(q_goal(1) - q(1), q_goal(0) - q(0));

            // Check if no hit
            sensor_output = my_sense_walls(my_primitives, q, heading, step_size);
            std::cout << "sensor_output = " << sensor_output << std::endl;
            if ((sensor_output == 3) || (sensor_output == 4)) // Either left hits or neither hit (no front hit)
            {
                qp = my_move_function(q, heading, step_size, "forward");
                path.waypoints.push_back(qp);
                q = qp;
            }
            else
            {
                switch_to_follow_obstacle = true;
                follow_obstacle = true;
            }
        }
        else // Follow obstacle
        {
            if (switch_to_follow_obstacle) // First time I hit the obstacle
            {
                std::cout << std::endl;
                std::cout << "Switch to follow obstacle " << std::endl;
                qH = q;
                dist_qH_to_goal = my_dist(q, q_goal);
                dist_qL_to_goal = dist_qH_to_goal;
                switch_to_follow_obstacle = false;
            }
            else // I keep following the obstacle
            {
                // Check if q is the closest point (so far) to the goal
                dist_to_goal = my_dist(q, q_goal);
                if (dist_to_goal < dist_qL_to_goal)
                {
                    dist_qL_to_goal = dist_to_goal;
                    qL = q;
                }

                sensor_output = my_sense_walls(my_primitives, q, heading, step_size);
                switch (sensor_output)
                {
                case 1:                       // "Forward hits":
                    heading -= Delta_heading; // Turn right
                    break;
                case 2:                       // "Both hit":
                    heading -= Delta_heading; // Turn right
                    break;
                case 3:                                                                                     //"Left hits":
                    qp = my_move_function(q, heading, step_size, "forward"); // Go straight
                    path.waypoints.push_back(qp);
                    q = qp;
                    break;
                case 4:                       //"Neither hit":
                    heading += Delta_heading; // Turn left
                    break;

                default:
                    break;
                }

                if (returning_to_qL)
                {
                    // Check if we have returned to qL
                    dist_to_qL = my_dist(q, qL);
                    if (dist_to_qL < closeness_threshold)
                    {
                        returning_to_qL = false;
                        follow_obstacle = false;
                        std::cout << std::endl;
                        std::cout << "Switch to move to goal " << std::endl;
                    }
                }
                else // Returning to qH
                {
                    // Check if we have returned to qH
                    dist_to_qH = my_dist(q, qH);
                    if (dist_to_qH < closeness_threshold)
                    {
                        returning_to_qL = true; // Return to qL
                    }
                }
            }
        }
        std::cout << "Mode = " << follow_obstacle << ". q =" << qp(0) << "," << qp(1) << ". Heading = "<< heading << ". Counter = " << counter << std::endl;
        counter++;
    } while ((dist_to_goal > termination_threshold) && (counter < counter_timeout));

    if (counter == counter_timeout) // Print a warning
    {
        std::cout << "WARNING: COUNTER TIMEOUT \n\n";
    }
    else
    {
        // q_goal reached!!
    }

    return path;
}


amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D &my_problem)
{

    Eigen::Vector2d q_goal = my_problem.q_goal;
    Eigen::Vector2d q = my_problem.q_init;
    Eigen::Vector2d q_target = q_goal;                                      // First, start by moving to goal in a straight line
    amp::Polygon my_obstacle_im_following = my_problem.obstacles.front();   // Default obstacle
    amp::Polygon my_obstacle_iwas_following = my_problem.obstacles.front(); // Default obstacle
    bool following_obstacle = false;                                      // First, start by moving to goal in a straight line
    my_collision_class my_collision_data;
    my_collision_class my_collision_data(my_polygonH, qH, my_vertex_in_front, my_vertex_at_back, q_min_dist_inrange_q_qH, dist_min_inrange_q_qH);

    unsigned int counter = 0;
    unsigned int max_path_length = 1000;
    do
    {
        my_collision_data = MyBugAlgorithm.my_move_to_target(my_problem, q, q_target, following_obstacle, obstacle_im_following, obstacle_iwas_following, my_collision_data);
        if (my_collision_data.qH == q_target) // I have reached the target. Keep going
        {
            following_obstacle = true;
            q = my_collision_data.qH; // (same as q_target)
            my_collision_data.my_vertex_at_back = my_collision_data.my_vertex_in_front;
            my_collision_data.my_vertex_in_front =
                q_target = obstacle_im_following.my_verticesCCW.at(i);
        }
        else // I've hit an obstacle at my_collision_data.qH
        {
            if (counter == 0) // First hit
            {
                my_obstacle_iwas_following = my_collision_data.my_polygonH;
                my_obstacle_iwas_following = my_collision_data.obstacle_im_following;
                q_target = my_collision_data.my_vertex_in_front;
            }
        }

        counter++;
    } while (qH = !q_goal) && (counter < max_path_length);

    if (counter == max_path_length) // Print a warning
    {
        std::cout << "WARNING: MAX PATH LENGTH REACHED \n\n";
    }
    else
    {
        // q_goal reached!!
    }

    return path;
}

Eigen::Vector2d MyBugAlgorithm.my_check_interection_with_edge(Eigen::Vector2d &u1, Eigen::Vector2d &u2, Eigen::Vector2d &v1, Eigen::Vector2d &v2)
{

    double mu;
    double nu;
    double mv;
    double nv;
    double xc;                      // Horizontal coordinate of collision point
    double yc;                      // Vertical coordinate of collision point
    Eigen::Vector2d q_default = v2; // Default value (goal)

    if (u1[1] != u2[1]) // u is not vertical segment
    {
        mu = (u2[2] - u1[2]) / (u2[1] - u1[1]);
        nu = u1[2] - mu * u1[1];
    }

    if (v1[1] != v2[1]) // v is not vertical segment
    {
        mv = (v2[2] - v1[2]) / (v2[1] - v1[1]);
        nv = v1[2] - mv * v1[1];
    }

    if (u1[1] != u2[1] && v1[1] != v2[1]) // u and v are not vertical segments
    {
        if (mu != mv) // u and v not parallel
        {
            xc = (nu - nv) / (mu - mv);
            if ((xc < max(min(u1[1], u2[1]), min(v1[1], v2[1]))) || (xc > min(max(u1[1], u2[1]), max(v1[1], v2[1])))) // Collision out of range
            {
                return q_default; // Default value
            }
            else // Collision in range!
            {
                yc = mu * xc + nu;
                return Eigen::Vector2d(xc, yc);
            }
        }
        else // Parallel segments: no collision
        {
            return q_default; // Default value
        }
    }
    else if ((u1[1] == u2[1]) && (v1[1] != v2[1])) // u is a vertical segment. v is not
    {
        xc = u1[1];
        yc = mv * xc + nv;
        if (yc > min(u1[2], u2[2]) && yc < max(u1[2], u2[2])) // Solution in range
        {
            return Eigen::Vector2d(xc, yc);
        }
        else
        {
            return q_default; // Default value
        }
    }
    else if ((v1[1] == v2[1]) && (u1[1] != u2[1])) // v is a vertical segment. u is not
    {
        xc = v1[1];
        yc = mu * xc + nu;
        if (yc > min(v1[2], v2[2]) && yc < max(v1[2], v2[2])) // Solution in range
        {
            return Eigen::Vector2d(xc, yc);
        }
        else // Solution out of range
        {
            return q_default; // Default value
        }
    }
    else // Both are vertical segments
    {
        if (u1[1] == v1[1])
        {
            if ((v1[2] >= min(u1[2], u2[2])) && (v1[2] <= max(u1[2], u2[2]))) // v1 is between u1 and u2
            {
                xc = v1[1];
                yc = v1[2];
                return Eigen::Vector2d(xc, yc);
            }
            else if ((v1[2] <= min(u1[2], u2[2])) && (v2[2] >= min(u1[2], u2[2]))) // v1 is below u1 and u2, and approaches u
            {
                xc = v1[1];
                yc = min(u1[2], u2[2]);
                return Eigen::Vector2d(xc, yc);
            }
            else if ((v1[2] >= max(u1[2], u2[2])) && (v2[2] <= max(u1[2], u2[2]))) // v1 is below u1 and u2, and approaches u
            {
                xc = v1[1];
                yc = max(u1[2], u2[2]);
                return Eigen::Vector2d(xc, yc);
            }
            else
            {
                return q_default; // Default value
            }
        }
        else
        {
            return q_default; // Default value
        }
    }

    return q_default; // Default value;
}

my_collision_class MyBugAlgorithm.my_move_to_target(const amp::Problem2D &my_problem, Eigen::Vector2d &q, Eigen::Vector2d &q_target, bool &following_obstacle, amp::Polygon &obstacle_im_following, amp::Polygon &obstacle_iwas_following, my_collision_class &my_collision_data)
{
    std::vector<Obstacle2D> my_obstacles = my_problem.obstacles;
    amp::Polygon my_polygonH;
    amp::Polygon my_polygonH_dummy;
    std::vector<Eigen::Vector2d> my_verticesCCW;
    Eigen::Vector2d q_goal = my_problem.q_goal;
    Eigen::Vector2d qH = q_target;
    std::vector<Eigen::Vector2d> qH_dummy;
    Eigen::Vector2d q_default = Eigen::Vector2d(-1000.0, -1000.0);
    Eigen::Vector2d u1
        Eigen::Vector2d u2;
    double dist_to_qH = 1000.0;
    double dist_to_qH_dummy;
    Eigen::Vector2d q_min_dist_inrange_q_qH = Eigen::Vector2d(-1000.0, -1000.0);
    double dist_min_inrange_q_qH = -1000.0;

    for (iter_obs; iter_obs < my_obstacles.back(); iter_obs++) // Search over all obstacles
    {
        my_polygonH_dummy = *iter_obs;
        if ((following_obstacle == false) || ((my_obstacle_im_following != my_polygonH_dummy) && (my_obstacle_iwas_following != my_polygonH_dummy)))
        {
            my_verticesCCW = my_polygon.verticesCCW();
            for (size_t i = 0; i < my_verticesCCW.size() - 1; i++) // Search for the closest collision with an edge of my_polygon
            {
                u1 = my_verticesCCW.at(i);
                u2 = my_verticesCCW.at(i + 1);
                qH_dummy = MyBugAlgorithm.my_check_interection_with_edge(u1, u2, q, q_target);
                if (qH_dummy != q_default)
                {
                    dist_to_qH_dummy = MyBugAlgorithm.my_dist(q, qH_dummy);
                    if (dist_to_qH > dist_to_qH_dummy)
                    {
                        dist_to_qH = dist_to_qH_dummy;
                        qH = qH_dummy;
                        my_polygonH = my_polygonH_dummy;
                        ind_vertex_in_front = i + 1;
                        my_vertex_at_back = u1;
                    }
                }
            }
        }
    }

    if (qH == q_target)
    {
        return my_collision_data(q_target); // We can go straight to the target
    }
    else // We have hit an obstacle
    {
        /// Compute the min distance to goal in the segment that goes from @param q to @param q_H:
        double m_1;
        double m_2;
        double n_1;
        double n_2;
        if (qH[1] != q[1])
            &&(qH[2] != q[2]) // The line that goes through q and qH is neither vertical nor horizontal
            {
                m_1 = (qH[2] - q[2]) / (qH[1] - q[1]);
                n_1 = q[2] - m_1 * q[1];
                m_2 = -1 / m_1;
                n_2 = -m_2 * q_goal[1] + q_goal[2];
                q_min_dist_inrange_q_qH[1] = (n_2 - n_1) / (m_1 - m_2);
                q_min_dist_inrange_q_qH[2] = m_1 * q_min_dist_inrange_q_qH[1] + n_1;
            }
        else if (qH[1] != q[1])
            &&(qH[2] == q[2]) // The line that goes through q and qH IS horizontal line
            {
                q_min_dist_inrange_q_qH[1] = q_goal[1];
                q_min_dist_inrange_q_qH[2] = q[2];
            }
        else if (qH[1] == q[1])
            &&(qH[2] != q[2]) // The line that goes through q and qH IS vertical line
            {
                q_min_dist_inrange_q_qH[1] = q[1];
                q_min_dist_inrange_q_qH[2] = q_goal[2];
            }
        else if (qH[1] == q[1])
            &&(qH[2] == q[2]) // q and qH are the same point
            {
                q_min_dist_inrange_q_qH[1] = q[1];
                q_min_dist_inrange_q_qH[2] = q[2];
            }

        // Check if in range
        if ((q_min_dist_inrange_q_qH[1] >= min(qH[1], q_goal[1])) && (q_min_dist_inrange_q_qH[1] <= max(qH[1], q_goal[1]))) // Point of minimum distance to goal is in range
        {
            // Compute dist_min_inrange_q_qH
            dist_min_inrange_q_qH = MyBugAlgorithm.my_dist(q_min_dist_inrange_q_qH, q_goal)
        }

        return my_collision_data(my_polygonH, qH, my_vertex_in_front, my_vertex_at_back, q_min_dist_inrange_q_qH, dist_min_inrange_q_qH);
    }
}

*/