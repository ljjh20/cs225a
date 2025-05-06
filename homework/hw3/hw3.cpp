// some standard library includes
#include <math.h>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

// sai main libraries includes
#include "SaiModel.h"

// sai utilities from sai-common
#include "timer/LoopTimer.h"
#include "redis/RedisClient.h"

// redis keys
#include "redis_keys.h"

// for handling ctrl+c and interruptions properly
#include <signal.h>
bool runloop = true;
void sighandler(int) { runloop = false; }
    // handle break for file writing
    std::ofstream jointFile;
    void onSigInt(int) {
    std::cerr << "Caught SIGINT, closing file and exiting.\n";
    if (jointFile.is_open()) jointFile.close();
    std::exit(0);
    }

// namespaces for compactness of code
using namespace std;
using namespace Eigen;

// config file names and object names
const string robot_file = "${CS225A_URDF_FOLDER}/panda/panda_arm.urdf";

Eigen::VectorXd computeMidRangeTorque(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& q_min,
    const Eigen::VectorXd& q_max,
    double k_mid)
{
    Eigen::VectorXd q_center = 0.5 * (q_max + q_min);
    return -2.0 * k_mid * (q - q_center);
}

int main(int argc, char** argv) {
    SaiModel::URDF_FOLDERS["CS225A_URDF_FOLDER"] = string(CS225A_URDF_FOLDER);

    // check for command line arguments
    if (argc != 2) {
        cout << "Incorrect number of command line arguments" << endl;
        cout << "Expected usage: ./{HW_NUMBER} {QUESTION_NUMBER}" << endl;
        return 1;
    }
    // convert char to int, check for correct controller number input
    string arg = argv[1];
    int controller_number;
    try {
        size_t pos;
        controller_number = stoi(arg, &pos);
        if (pos < arg.size()) {
            cerr << "Trailing characters after number: " << arg << '\n';
            return 1;
        }
        else if (controller_number < 1 || controller_number > 4) {
            cout << "Incorrect controller number" << endl;
            return 1;
        }
    } catch (invalid_argument const &ex) {
        cerr << "Invalid number: " << arg << '\n';
        return 1;
    } catch (out_of_range const &ex) {
        cerr << "Number out of range: " << arg << '\n';
        return 1;
    }

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load robots
    auto robot = new SaiModel::SaiModel(robot_file);

    // prepare controller
	int dof = robot->dof();
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);
	VectorXd control_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	Jv = robot->Jv(link_name, pos_in_link);
	Lambda = robot->taskInertiaMatrix(Jv);
	J_bar = robot->dynConsistentInverseJacobian(Jv);
	N = robot->nullspaceMatrix(Jv);

    // flag for enabling gravity compensation
    bool gravity_comp_enabled = false;

    // start redis client
    auto redis_client = SaiCommon::RedisClient();
    redis_client.connect();

    // setup send and receive groups
    VectorXd robot_q = redis_client.getEigen(JOINT_ANGLES_KEY);
    VectorXd robot_dq = redis_client.getEigen(JOINT_VELOCITIES_KEY);
    redis_client.addToReceiveGroup(JOINT_ANGLES_KEY, robot_q);
    redis_client.addToReceiveGroup(JOINT_VELOCITIES_KEY, robot_dq);

    redis_client.addToSendGroup(JOINT_TORQUES_COMMANDED_KEY, control_torques);
    redis_client.addToSendGroup(GRAVITY_COMP_ENABLED_KEY, gravity_comp_enabled);

    redis_client.receiveAllFromGroup();
    redis_client.sendAllFromGroup();

    // update robot model from simulation configuration
    robot->setQ(robot_q);
    robot->setDq(robot_dq);
    robot->updateModel();

    // record initial configuration
    VectorXd initial_q = robot->q();

    jointFile.open("../../homework/hw3/q1.csv");
    if(!jointFile){
        std::cerr << "ERROR: could not open joint_positions.csv for writing\n";
        return 1;
    }
    jointFile << "t,x,y,z,xd,yd,zd,xdot,ydot,zdot,norm\n"; // ,q0,q1,q2,q3,q4,q5,q6\n"; // header
    // jointFile << "t,t1,t2,t3,t4,t5,t6,t7\n"; // ,q0,q1,q2,q3,q4,q5,q6\n"; // header
    std::size_t counter = 0;

    // create a loop timer
    const double control_freq = 1000;
    SaiCommon::LoopTimer timer(control_freq);

    while (runloop) {
        // wait for next scheduled loop
        timer.waitForNextLoop();
        double time = timer.elapsedTime();

        // read robot state from redis
        redis_client.receiveAllFromGroup();
        robot->setQ(robot_q);
        robot->setDq(robot_dq);
        robot->updateModel();

        // **********************
        // WRITE YOUR CODE AFTER
        // **********************

        // ---------------------------  question 1 ---------------------------------------
        if(controller_number == 1) {

            Jv = robot->Jv(link_name, pos_in_link);
	        Lambda = robot->taskInertiaMatrix(Jv);
	        J_bar = robot->dynConsistentInverseJacobian(Jv);
	        N = robot->nullspaceMatrix(Jv);

            VectorXd x = robot->positionInWorld(link_name, pos_in_link);
            VectorXd xdot = robot->linearVelocity(link_name, pos_in_link);
            MatrixXd Kvj =  14*MatrixXd::Identity(7, 7);
            MatrixXd Kpj =  50*MatrixXd::Identity(7, 7);

            double Kp = 100;
            double Kv = 20;

            VectorXd x_desired(3);
            x_desired << 0.3 + 0.1*sin(M_PI*time), 0.1 + 0.1*cos(M_PI*time), 0.5; 

            VectorXd q_desired(7);
            q_desired << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            // desired velocity
            VectorXd xdot_desired(3);
            xdot_desired <<  0.1*M_PI*cos(M_PI*time),
                            -0.1*M_PI*sin(M_PI*time),
                            0.0;

            // desired acceleration
            VectorXd xddot_desired(3);
            xddot_desired << -0.1*M_PI*M_PI*sin(M_PI*time),
                            -0.1*M_PI*M_PI*cos(M_PI*time),
                            0.0;

            // VectorXd p = J_bar.transpose() * robot->jointGravityVector();
            VectorXd F = Lambda * (xddot_desired - Kp * (x - x_desired) - Kv * (xdot - xdot_desired)); //+ p;
            Eigen::Matrix<double,7,1> tau_derivative_null = -1* N.transpose() * ((Kvj * robot_dq) + (Kpj * (robot_q - q_desired)));
            control_torques = (Jv.transpose() * F) + tau_derivative_null + robot->jointGravityVector();

            if (++counter % 10 == 0) {
                jointFile
                    << time << ','
                    << x(0) << ','
                    << x(1) << ','
                    << x(2) << ','
                    << x_desired(0) << ','
                    << x_desired(1) << ','
                    << x_desired(2) << '\n';

                    // << robot_q(0) << ','
                    // << robot_q(1) << ','
                    // << robot_q(2) << ','
                    // << robot_q(3) << ','
                    // << robot_q(4) << ','
                    // << robot_q(5) << ','
                    // << robot_q(6) << '\n';
            }
        }

        // ---------------------------  question 2 ---------------------------------------
        else if(controller_number == 2) {
            Jv = robot->Jv(link_name, pos_in_link);
	        Lambda = robot->taskInertiaMatrix(Jv);
	        J_bar = robot->dynConsistentInverseJacobian(Jv);
	        N = robot->nullspaceMatrix(Jv);
            

            VectorXd x = robot->positionInWorld(link_name, pos_in_link);
            VectorXd xdot = robot->linearVelocity(link_name, pos_in_link);
            MatrixXd Kvj =  14*MatrixXd::Identity(7, 7);
            MatrixXd Kpj =  50*MatrixXd::Identity(7, 7);
            // MatrixXd Kmid =  50*MatrixXd::Identity(7, 7);
            // MatrixXd Kdamp =  50*MatrixXd::Identity(7, 7);

            double Kp = 100;
            double Kv = 20;
            double Kmid = 25;
            double Kdamp = 14;

            VectorXd x_desired(3);
            // x_desired << -0.1, 0.15, 0.2; 
            x_desired << -0.65, 0.45, 0.7; 

            VectorXd q_desired(7);
            VectorXd q_lower(7);
            VectorXd q_upper(7);
            q_desired << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            q_lower << -165.0, -100.0, -165.0, -170.0, -165.0, 0.0, -165.0;
            q_upper << 165.0, 100.0, 165.0, -30.0, 165.0, 210.0, 165.0;

            q_lower << q_lower * M_PI / 180.0;
            q_upper << q_lower * M_PI / 180.0;

            VectorXd Tmid = computeMidRangeTorque(robot_q, q_lower, q_upper, Kmid);
            VectorXd tau_mid_null = N.transpose() * Tmid;

            VectorXd Tdamp = -Kdamp * robot_dq;
            Eigen::VectorXd tau_damp_null = N.transpose() * Tdamp;

            VectorXd F = Lambda * (- Kp * (x - x_desired) - Kv * (xdot));
            control_torques = (Jv.transpose() * F) + Tmid + tau_damp_null + robot->jointGravityVector();
            
            if (++counter % 10 == 0) {
                jointFile
                    << time << ','
                    << x(0) << ','
                    << x(1) << ','
                    << x(2) << ','
                    << x_desired(0) << ','
                    << x_desired(1) << ','
                    << x_desired(2) << ','
                    << robot_q(3) << ','
                    << robot_q(5) << '\n';
            }


            // if (++counter % 10 == 0) {
            //     jointFile
            //         << time << ','
            //         << null_torques(0) << ','
            //         << null_torques(1) << ','
            //         << null_torques(2) << ','
            //         << null_torques(3) << ','
            //         << null_torques(4) << ','
            //         << null_torques(5) << ','
            //         << null_torques(6) << '\n';
            // }
        }

        // ---------------------------  question 3 ---------------------------------------
        else if(controller_number == 3) {
            Eigen::MatrixXd J0 = robot->J(link_name, pos_in_link);
            Jv = robot->Jv(link_name, pos_in_link);
	        Eigen::Matrix<double,6,6> Lambda0 = robot->taskInertiaMatrix(J0);
	        J_bar = robot->dynConsistentInverseJacobian(Jv);
	        N = robot->nullspaceMatrix(Jv);
            
            Eigen::Matrix3d R = robot->rotationInWorld(link_name);
            Eigen::Matrix3d Rd; 
            Rd <<  std::cos(M_PI/3), 0, std::sin(M_PI/3), 0, 1, 0, -std::sin(M_PI/3), 0, std::cos(M_PI/3);
            VectorXd xd(3);
            xd << 0.6, 0.3, 0.5;

            Eigen::Vector3d delta_phi = Eigen::Vector3d::Zero();
            for(int i=0; i<3; ++i) {
                delta_phi += R.col(i).cross(Rd.col(i));
            }
            delta_phi *= -0.5;

            Eigen::Vector3d x    = robot->positionInWorld(link_name, pos_in_link);
            Eigen::Vector3d xdot = robot->linearVelocity (link_name, pos_in_link);
            Eigen::Vector3d omega= robot->angularVelocity(link_name);
            
            MatrixXd kvj =  14*MatrixXd::Identity(7, 7);
            MatrixXd kpj =  50*MatrixXd::Identity(7, 7);
            double kp = 100;
            double kv = 20;

            Eigen::Matrix<double,6,1> controls;
            controls.head<3>() =  kp * (xd - x) - kv * xdot;     // position
            controls.tail<3>() =  kp * (-delta_phi) - kv * omega;    // orientation

            Eigen::Matrix<double,6,1> F = Lambda0 * controls; // 3 controls for pos, 3 for orientation
            Eigen::VectorXd tau_task_space = J0.transpose() * F; 
            Eigen::VectorXd tau_null_space = N.transpose() * (- kvj * robot_dq);
            Eigen::VectorXd tau_gravity = robot->jointGravityVector();

            control_torques = tau_task_space + tau_null_space + tau_gravity;

            
            if (++counter % 10 == 0) {
                jointFile
                    << time << ','
                    << x(0) << ','
                    << x(1) << ','
                    << x(2) << ','
                    << xd(0) << ','
                    << xd(1) << ','
                    << xd(2) << ','
                    << delta_phi(0) << ','
                    << delta_phi(1) << ','
                    << delta_phi(2) << '\n';
            }

        }

        // ---------------------------  question 4 ---------------------------------------
        else if(controller_number == 4) {
            Jv = robot->Jv(link_name, pos_in_link);
	        Lambda = robot->taskInertiaMatrix(Jv);
	        J_bar = robot->dynConsistentInverseJacobian(Jv);
	        N = robot->nullspaceMatrix(Jv);
            
            VectorXd xd(3);
            xd << 0.6, 0.3, 0.4;
            Eigen::Vector3d x = robot->positionInWorld(link_name, pos_in_link);
            Eigen::Vector3d xdot = robot->linearVelocity (link_name, pos_in_link);
            VectorXd q_desired(7);
            q_desired << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            MatrixXd kvj = 14*MatrixXd::Identity(7, 7);
            MatrixXd kpj = 50*MatrixXd::Identity(7, 7);
            double kp = 200;
            double kv = 2*std::sqrt(kp); // critically damp
            // double Vmax = 0.1;
            
            // Eigen::Vector3d xdot_d = (kp/kv) * (xd - x);
            // double nu;
            // double norm_xd = xdot_d.norm();
            // double s = Vmax / norm_xd;
            // if (std::abs(s) <= 1.0) {
            //     nu = s;
            // } else if (s > 0) {
            //     nu =  1.0;
            // } else {
            //     nu = -1.0;
            // }

            double normx = xdot.norm();
          

            // Eigen::Vector3d F = Lambda * (- kv * (xdot - (nu * xdot_d)) );
            Eigen::Vector3d F = Lambda * ( kp * (xd - x) - kv * xdot);
            
            Eigen::Matrix<double,7,1> tau_derivative_null = -1* N.transpose() * (robot->M()) * (kvj * robot_dq + kpj * (robot_q - q_desired));

            // VectorXd F = Lambda * (kp * (xd - x) - kv * (xdot));
            control_torques = Jv.transpose() * F + tau_derivative_null + robot->jointGravityVector();

            if (++counter % 10 == 0) {
                jointFile
                    << time << ','
                    << x(0) << ','
                    << x(1) << ','
                    << x(2) << ','
                    << xd(0) << ','
                    << xd(1) << ','
                    << xd(2) << ','
                    << xdot(0) << ','
                    << xdot(1) << ','
                    << xdot(2) << ','
                    << normx << '\n';
            }
            
        }

        // **********************
        // WRITE YOUR CODE BEFORE
        // **********************

        // send to redis
        redis_client.sendAllFromGroup();
    }
    jointFile.close();

    control_torques.setZero();
    gravity_comp_enabled = true;
    redis_client.sendAllFromGroup();

    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();

    return 0;
}
