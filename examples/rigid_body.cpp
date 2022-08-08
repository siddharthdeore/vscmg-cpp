#include <RigidBody.h>
#include <chrono>
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
    Eigen::Quaterniond q(cos(0.5), 0, sin(0.5), 0);
    Eigen::Matrix<double, 3, 1> w(0.01, 0.01, -0.01);
    RigidBody::state_type X = { q.w(), q.x(), q.y(), q.z(), w.x(), w.y(), w.z() };

    sat->set_state(X);

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
