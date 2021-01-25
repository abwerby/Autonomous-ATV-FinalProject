/**
 * @file main.cpp
 *
 * @author abdelrhman werby
 * Contact: abdelrhmanwerby@gmail.com
 *
 */

#include <iostream>
#include <cmath>
#include <vector>
#include <dlib/optimization.h>
#include <dlib/global_optimization.h>
#include "vehicle_model.h"
#include <chrono>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/String.h"

#include <sstream>


using namespace dlib;
using namespace std;
using namespace std::chrono;


typedef matrix<double, 0, 1> column_vector;

double objective(const column_vector& m);

/* number of horizen point */
#define N 20

/* MPC cost function weights */
#define cte_W 2000
#define eth_W 2000
#define v_W 5000
#define st_rate_W  100
#define acc_rate_W 100
#define st_W  10
#define acc_W  10

/* refernce velocity */
#define v_ref 1

/* sample time */
#define dt 0.1

/* Base length */
#define Base_length 0.74

/* coff of refernce polynomial */
double coff[] = {  0.00926813, -0.1468977,  0.71420536, -0.02785882 };

state last_state_g;
state current_state_g;
double th_des = 0;

double velocity2RPM(double v)
{
    return (v * 60 * 5) / (pi * 0.3);
}

double RPM2velocity(double n)
{
    return (n * pi * 0.3) / (60 * 5);
}

void Callback_V(const std_msgs::UInt16::ConstPtr& msg)
{
    last_state_g.v = RPM2velocity(msg->data);
  	last_state_g.x = last_state_g.x + last_state_g.v * cos(last_state_g.th) * dt;
	last_state_g.y = last_state_g.y + last_state_g.v * sin(last_state_g.th) * dt;

	th_des = atan(coff[2] + 2 * coff[1] * last_state_g.x + 3 * coff[0] * pow(last_state_g.x, 2));

	last_state_g.cte = polyval(coff, 4, last_state_g.x) - last_state_g.y;
	last_state_g.eth = last_state_g.th - th_des;
}

void Callback_yaw(const std_msgs::Float32::ConstPtr& msg)
{
    last_state_g.th = msg->data / RAD2DEG;
}

void Callback_planner(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "MPC_node");
    ros::NodeHandle n;
    ros::Publisher pub_1 = n.advertise<std_msgs::Int32>("MPC_SteerAngle", 2);
    ros::Publisher pub_2 = n.advertise<std_msgs::UInt16>("MPC_velocity", 2);
    ros::Subscriber sub_1 = n.subscribe("motor_speed", 2, Callback_V);
    ros::Subscriber sub_2 = n.subscribe("imu_raw", 2, Callback_yaw);
    ros::Subscriber sub_3 = n.subscribe("planner_data", 2, Callback_planner);
    ros::Rate loop_rate(1/dt);

    inputs act;

    /* init x solution */
    column_vector x = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    /* Upper bounds */
    column_vector ub = { 0.43, 0.43, 0.43, 0.43, 0.43, 0.43, 0.43, 0.43, 0.43, 0.43,
                         0.43, 0.43, 0.43, 0.43, 0.43, 0.43,0.43, 0.43, 0.43, 0.43,
                        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

    /* lower bounds */
    column_vector lb = { -0.43, -0.43, -0.43, -0.43, -0.43, -0.43, -0.43, -0.43, -0.43, -0.43,
                         -0.43, -0.43, -0.43, -0.43, -0.43, -0.43, -0.43, -0.43, -0.43, -0.43,
                        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };

    std_msgs::UInt16 vel;
    std_msgs::Int32 st;

    /* ROS loop */
    while (ros::ok())
    {
        /* save start time of optamization */
        auto start = high_resolution_clock::now();

        /* MPC local constrained optamization, the final solutions will be in "column_vector x" */
        find_min_box_constrained(bfgs_search_strategy(),
                                 objective_delta_stop_strategy(1e-6),
                                 objective, derivative(objective), x, lb, ub);

        /* save end time of optamization */
        auto stop = high_resolution_clock::now();

        /* time of optamization */
        auto duration = duration_cast<milliseconds>(stop - start);

        /* Applay output command inputs from optamization to the noisy model of vehicle */
        vel.data = x(N);
        st.data = x(0);
        pub_1.publish(st);
        pub_2.publish(vel);

        last_state_g.print_state();
        cout << "Time = " << duration.count() << " MS" << endl;
        cout << "Cost = " << objective(x) << endl;


        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}


/* the cost function */
double objective(const column_vector& m)
{
    inputs u;
    state last_state = last_state_g;
    state current_state = current_state_g;
    double Error = 0;
    for (int i = 0; i < N; i++)
    {
        u.steerangle = m(i);
        u.accelartion = m(i + N);
        current_state = update_state(u, last_state, coff, dt, Base_length);
        if (i == 0)
        {
            Error += cte_W * pow(current_state.cte, 2) + eth_W * pow(current_state.eth, 2) + v_W * pow((v_ref - current_state.v), 2)
                + st_W * pow(u.steerangle, 2) + acc_W * pow(u.accelartion, 2);
        }
        else
        {
            Error += cte_W * pow(current_state.cte, 2) + eth_W * pow(current_state.eth, 2) + v_W * pow((v_ref - current_state.v), 2)
                + st_rate_W * pow(u.steerangle - m(i - 1), 2) + acc_rate_W * pow(u.accelartion - m(i + N - i), 2)
                + st_W * pow(u.steerangle, 2) + acc_W * pow(u.accelartion, 2);
        }
        
        last_state = current_state;
    }
    return Error;
}
