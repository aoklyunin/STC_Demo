#pragma once


#include "Functor.h"

struct ForwardTaskFunctor : Functor<double> {
    ForwardTaskFunctor(double posAx, double posAy, double posBx, double posBy,
                       double posCx, double posCy,
                       double dAB, double dBC, double dAC) :
            Functor<double>(2, 3),
            posD(posAx, posAy),
            posB(posBx, posBy),
            posC(posCx, posCy),
            dAB(dAB),
            dBC(dBC),
            dAC(dAC),
            startPos((posAx + posBx + posCx) / 3, (posAy + posBy + posCy) / 3) {}

    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {
        // Implement y = 10*(x0+3)^2 + (x1-5)^2
        fvec(0) = calculateDistance(x(0), x(1), posB, posD) - dAB;
        fvec(1) = calculateDistance(x(0), x(1), posC, posB) - dBC;
        fvec(2) = calculateDistance(x(0), x(1), posC, posD) - dAC;

        return 0;
    }


    double calculateDistance(double x, double y,
                             Eigen::Vector2d pos1, Eigen::Vector2d pos2) const {
        return std::sqrt(
                (x - pos1(0)) * (x - pos1(0)) +
                (y - pos1(1)) * (y - pos1(1))
        ) - std::sqrt(
                (x - pos2(0)) * (x - pos2(0)) +
                (y - pos2(1)) * (y - pos2(1))
        );
    }

    Eigen::Vector2d posD;
    Eigen::Vector2d posB;
    Eigen::Vector2d posC;
    double dAB;
    double dBC;
    double dAC;

    Eigen::Vector2d startPos;
};
