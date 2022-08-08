#include <VSCMG.h>
#include <chrono>
#include <iostream>

int main(int argc, char const* argv[])
{
    char* p;
    long max = 10000;

    if (argc > 1) {
        max = strtol(argv[1], &p, 10);
    }

    // satellite object of type VSCMG
    std::shared_ptr<VSCMG> sat = std::make_shared<VSCMG>();

    // initial state
    Eigen::Quaterniond q(cos(0.5), 0, sin(0.5), 0);
    Eigen::Matrix<double, 3, 1> w(0.01, 0.01, -0.01);

    // Reaction wheel angular velocities
    Eigen::Matrix<double, 4, 1> Omega(100, 100, 100, 100);

    // gimbal angles
    Eigen::Matrix<double, 4, 1> delta(0, 0, 0, 0);

    sat->set_state(q, w, delta, Omega);
    sat->set_gimbal_angle(delta);
    sat->set_wheel_velocity(Omega);

    // time
    double t = 0;

    auto t1 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < max; i++) {

        // integrate a step
        sat->step(t, t + 0.001, 0.001);

        // verbose
        if (i % 100 == 0)
            std::cout << *sat << "\n";

        t += 0.001;
    }
    auto t2 = std::chrono::high_resolution_clock::now();

    // benchmarking
    std::cout << " "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
              << " ms \n";

    return 0;
}
