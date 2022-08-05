#include <VSCMG.h>
#include <iostream>

std::ostream& operator<<(std::ostream& os, const Eigen::Quaterniond q)
{
    os << q.w() << ", " << q.z() << ", " << q.y() << ", " << q.z();
    return os;
}

int main(int argc, char const* argv[])
{
    Eigen::Quaterniond q(1, 0, 0, 0);
    Eigen::Matrix<double, 3, 1> w(1, 0, 0);

    q = get_quaternion_kinematics(q, w);

    std::cout << q << std::endl;
    std::cout << get_quaternion_error(q, q) << std::endl;

    return 0;
}
