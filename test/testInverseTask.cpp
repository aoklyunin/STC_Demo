#include <Eigen/Core>
#include <iostream>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#include "../ofunctions/InverseTaskFunctor.h"

void makeTest(
        Eigen::Vector2d posA, Eigen::Vector2d posB, Eigen::Vector2d posC,
        Eigen::Vector2d posD, Eigen::Vector2d posE, Eigen::Vector2d posF
) {

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


    assert(std::abs(posA.x() - x(0)) < 1.0e-4);
    assert(std::abs(posA.y() - x(1)) < 1.0e-4);

    assert(std::abs(posB.x() - x(2)) < 1.0e-4);
    assert(std::abs(posB.y() - x(3)) < 1.0e-4);

    assert(std::abs(posC.x() - x(4)) < 1.0e-4);
    assert(std::abs(posC.y() - x(5)) < 1.0e-4);

}

void test1() {
    std::cout << "it.test1" << std::endl;

    Eigen::Vector2d posA(0, 5);
    Eigen::Vector2d posB(3, 1);
    Eigen::Vector2d posC(1, 2);


    Eigen::Vector2d posD(8, 15);
    Eigen::Vector2d posE(12, 7);
    Eigen::Vector2d posF(11, 12);

    makeTest(posA, posB, posC, posD, posE, posF);
}


void test2() {
    std::cout << "it.test2" << std::endl;

    Eigen::Vector2d posA(5, 5);
    Eigen::Vector2d posB(3, 1);
    Eigen::Vector2d posC(1, 2);


    Eigen::Vector2d posD(8, 15);
    Eigen::Vector2d posE(12, 7);
    Eigen::Vector2d posF(17, 12);

    makeTest(posA, posB, posC, posD, posE, posF);
}


void test3() {
    std::cout << "it.test3" << std::endl;

    Eigen::Vector2d posA(5, 5);
    Eigen::Vector2d posB(10, 1);
    Eigen::Vector2d posC(3, 2);

    Eigen::Vector2d posD(28, 15);
    Eigen::Vector2d posE(12, 7);
    Eigen::Vector2d posF(17, 12);

    makeTest(posA, posB, posC, posD, posE, posF);
}

void test4() {
    std::cout << "it.test4" << std::endl;

    Eigen::Vector2d posA(5, 9);
    Eigen::Vector2d posB(10, 14);
    Eigen::Vector2d posC(3, 2);

    Eigen::Vector2d posD(28, 25);
    Eigen::Vector2d posE(22, 27);
    Eigen::Vector2d posF(27, 22);

    makeTest(posA, posB, posC, posD, posE, posF);
}


int main() {
    test1();
    test2();
    test3();
    test4();
}