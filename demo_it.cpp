#include <iostream>
#include <Eigen/Dense>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include "ofunctions/InverseTaskFunctor.h"

int main(int argc, char *argv[]) {
    Eigen::Vector2d posA(0, 5);
    Eigen::Vector2d posB(3, 1);
    Eigen::Vector2d posC(1, 2);


    Eigen::Vector2d posD(8, 15);
    Eigen::Vector2d posE(12, 7);
    Eigen::Vector2d posF(11, 12);

    double dAB_from_D = (posB - posD).norm() - (posA - posD).norm();
    double dBC_from_D = (posC - posD).norm() - (posB - posD).norm();
    double dAC_from_D = (posC - posD).norm() - (posA - posD).norm();

    double dAB_from_E = (posB - posE).norm() - (posA - posE).norm();
    double dBC_from_E = (posC - posE).norm() - (posB - posE).norm();
    double dAC_from_E = (posC - posE).norm() - (posA - posE).norm();

    double dAB_from_F = (posB - posF).norm() - (posA - posF).norm();
    double dBC_from_F = (posC - posF).norm() - (posB - posF).norm();
    double dAC_from_F = (posC - posF).norm() - (posA - posF).norm();

    InverseTaskFunctor functor(
            posD.x(), posD.y(),
            posE.x(), posE.y(),
            posF.x(), posF.y(),
            dAB_from_D,
            dBC_from_D,
            dAC_from_D,
            dAB_from_E,
            dBC_from_E,
            dAC_from_E,
            dAB_from_F,
            dBC_from_F,
            dAC_from_F
    );

    Eigen::VectorXd x(6);
    x(0) = functor.startPos.x();
    x(1) = functor.startPos.y();
    x(2) = functor.startPos.x();
    x(3) = functor.startPos.y();
    x(4) = functor.startPos.x();
    x(5) = functor.startPos.y();

    std::cout << "x: " << x.transpose() << std::endl;

    Eigen::NumericalDiff<InverseTaskFunctor> numDiff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<InverseTaskFunctor>, double> lm(numDiff);
    lm.parameters.maxfev = 2000;
    lm.parameters.xtol = 1.0e-10;
    std::cout << lm.parameters.maxfev << std::endl;

    int ret = lm.minimize(x);
    std::cout << lm.iter << std::endl;
    std::cout << ret << std::endl;

    std::cout << "x that minimizes the function: " << x.transpose() << std::endl;

    return 0;
}