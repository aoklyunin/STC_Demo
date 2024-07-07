#include <iostream>
#include <Eigen/Dense>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include "ofunctions/ForwardTaskFunctor.h"


int main(int argc, char *argv[]) {

    Eigen::Vector2d source(21, 17);

    Eigen::Vector2d posA(0, 5);
    Eigen::Vector2d posB(3, 1);
    Eigen::Vector2d posC(1, 2);

    double dAB = (posB - source).norm() - (posA - source).norm();
    double dBC = (posC - source).norm() - (posB - source).norm();
    double dAC = (posC - source).norm() - (posA - source).norm();

    ForwardTaskFunctor functor(
            posA.x(), posA.y(),
            posB.x(), posB.y(),
            posC.x(), posC.y(),
            dAB, dBC, dAC
    );

    Eigen::VectorXd x(2);
    x(0) = functor.startPos.x();
    x(1) = functor.startPos.y();
    std::cout << "x: " << x.transpose() << std::endl;


    Eigen::NumericalDiff<ForwardTaskFunctor> numDiff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<ForwardTaskFunctor>, double> lm(numDiff);
    lm.parameters.maxfev = 2000;
    lm.parameters.xtol = 1.0e-10;
    std::cout << lm.parameters.maxfev << std::endl;

    int ret = lm.minimize(x);
    std::cout << lm.iter << std::endl;
    std::cout << ret << std::endl;

    std::cout << "x that minimizes the function: " << x.transpose() << std::endl;

    return 0;
}