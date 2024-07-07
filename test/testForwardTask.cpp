#include <Eigen/Core>
#include <iostream>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#include "../ofunctions/ForwardTaskFunctor.h"

void makeTest(const Eigen::Vector2d& source,
              Eigen::Vector2d posA,
              Eigen::Vector2d posB,
              Eigen::Vector2d posC) {

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

    Eigen::NumericalDiff<ForwardTaskFunctor> numDiff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<ForwardTaskFunctor>, double> lm(numDiff);
    lm.parameters.maxfev = 2000;
    lm.parameters.xtol = 1.0e-10;

    int ret = lm.minimize(x);
    std::cout << ret << std::endl;

    assert((source - x).norm() < 1.0e-9);

}

void test1() {
    std::cout<<"ft.test1"<<std::endl;

    Eigen::Vector2d source(21, 17);

    Eigen::Vector2d posA(0, 5);
    Eigen::Vector2d posB(3, 1);
    Eigen::Vector2d posC(1, 2);

    makeTest(source, posA, posB, posC);
}

void test2() {
    std::cout<<"ft.test2"<<std::endl;
    Eigen::Vector2d source(21, 17);

    Eigen::Vector2d posA(-2, 5);
    Eigen::Vector2d posB(7, 1);
    Eigen::Vector2d posC(3, 2);

    makeTest(source, posA, posB, posC);
}


void test3() {
    std::cout<<"ft.test3"<<std::endl;
    Eigen::Vector2d source(14, 25);

    Eigen::Vector2d posA(0, 5);
    Eigen::Vector2d posB(3, 1);
    Eigen::Vector2d posC(1, -2);

    makeTest(source, posA, posB, posC);
}



void test4() {
    std::cout<<"ft.test4"<<std::endl;
    Eigen::Vector2d source(14, 25);

    Eigen::Vector2d posA(9, 11);
    Eigen::Vector2d posB(13, 9);
    Eigen::Vector2d posC(10, 8);

    makeTest(source, posA, posB, posC);
}

int main() {
    test1();
    test2();
    test3();
    test4();
}