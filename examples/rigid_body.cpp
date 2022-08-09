#include <RigidBody.h>
#include <Controllers.h>
#include <chrono>
#include <iomanip>
#include <iostream>
int main(int argc, char const* argv[])
{
    char* p;
    long max = 10000;

    if (argc > 1) {
        max = strtol(argv[1], &p, 10);
    }

    // satellite object of type RigidBody
    std::shared_ptr<RigidBody> sat = std::make_shared<RigidBody>();
    sat->set_inertia(2.0, 1.0, 0.5);

    // initial state
    Eigen::Quaterniond q(cos(1.0), 0, sin(1.0), 0);
    Eigen::Quaterniond qd(1.0, 0, 0, 0);

    Eigen::Matrix<double, 3, 1> w(10.0, -0.5, 0.2);

    RigidBody::state_type X0 = { q.w(), q.x(), q.y(), q.z(), w.x(), w.y(), w.z() };

    sat->set_state(X0);

    // time
    double t = 0;

    auto t1 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < max; i++) {
        // get current state of Rigid Body (observations)
        RigidBody::state_type X;
        sat->get_state(X);

        q = Eigen::Quaterniond(X[0], X[1], X[2], X[3]);
        w = Eigen::Vector3d(X[4], X[5], X[6]);

        // calculate error in reference state and current state
        auto qe = get_quaternion_error(q, qd);

        // calcualte control torques with required to bring rigid body in steady state
        auto u = Controller::quaternion_feedback(qe, w, 1.0, 0.1);

        // apply controlled torque action and take time step of forward dynamics
        sat->step(u, t, t + 0.001, 0.001);

        // print state
        if (i % 10 == 0) {
            std::cout << *sat << "\n";
            /*
            std::cout << std::setw(8) << std::setprecision(3) << std::fixed
                      << qe.w() << " " << qe.vec().transpose()
                      << " w :" << w.transpose()
                      << " u: " << u.transpose() << "\n";
            */
        }

        t += 0.001;
    }
    auto t2 = std::chrono::high_resolution_clock::now();

    // benchmarking
    std::cout << " "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
              << " ms \n";

    return 0;
}
