#include "apriltag_ros/ceres_bundle_solver.hpp"

using namespace apriltag_ros;
int main(int argc, char **argv) {

    CeresBundleSolver s;
    s.solve(0.1);
}
