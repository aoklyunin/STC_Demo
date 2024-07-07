#pragma once

#include "Functor.h"

/**
 * Функтор обратной задачаи
 */
struct InverseTaskFunctor : Functor<double> {
    InverseTaskFunctor(double posDx, double posDy, double posEx, double posEy,
                       double posFx, double posFy,
                       double dAB_from_D, double dBC_from_D, double dAC_from_D,
                       double dAB_from_E, double dBC_from_E, double dAC_from_E,
                       double dAB_from_F, double dBC_from_F, double dAC_from_F
    ) :
            Functor<double>(6, 9),
            posD(posDx, posDy),
            posE(posEx, posEy),
            posF(posFx, posFy),
            dAB_from_D(dAB_from_D),
            dBC_from_D(dBC_from_D),
            dAC_from_D(dAC_from_D),
            dAB_from_E(dAB_from_E),
            dBC_from_E(dBC_from_E),
            dAC_from_E(dAC_from_E),
            dAB_from_F(dAB_from_F),
            dBC_from_F(dBC_from_F),
            dAC_from_F(dAC_from_F),
            startPos((posDx + posEx + posFx) / 3, (posDy + posEy + posFy) / 3) {}

    /**
     * Вычисление значния функций в точке
     * @param x вектор аргументов
     * @param fvec вектор значений
     * @return
     */
    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {
        fvec(0) = getCriterionForTwo(
                posD.x(), posD.y(), x(2), x(3), x(0), x(1)
        ) - dAB_from_D;
        fvec(1) = getCriterionForTwo(
                posD.x(), posD.y(), x(4), x(5), x(2), x(3)
        ) - dBC_from_D;
        fvec(2) = getCriterionForTwo(
                posD.x(), posD.y(), x(4), x(5), x(0), x(1)
        ) - dAC_from_D;


        fvec(3) = getCriterionForTwo(
                posE.x(), posE.y(), x(2), x(3), x(0), x(1)
        ) - dAB_from_E;
        fvec(4) = getCriterionForTwo(
                posE.x(), posE.y(), x(4), x(5), x(2), x(3)
        ) - dBC_from_E;
        fvec(5) = getCriterionForTwo(
                posE.x(), posE.y(), x(4), x(5), x(0), x(1)
        ) - dAC_from_E;

        fvec(6) = getCriterionForTwo(
                posF.x(), posF.y(), x(2), x(3), x(0), x(1)
        ) - dAB_from_F;
        fvec(7) = getCriterionForTwo(
                posF.x(), posF.y(), x(4), x(5), x(2), x(3)
        ) - dBC_from_F;
        fvec(8) = getCriterionForTwo(
                posF.x(), posF.y(), x(4), x(5), x(0), x(1)
        ) - dAC_from_F;

        return 0;
    }

    /**
     * Получить аналитическую разность хода
     * @param x координата X передатчика
     * @param y координата Y передатчика
     * @param pos1x координата X первого приёмника
     * @param pos1y координата Y первого приёмника
     * @param pos2x координата X второго приёмника
     * @param pos2y координата Y второго приёмника
     * @return
     */
    static double getCriterionForTwo(
            double x, double y, double pos1x, double pos1y, double pos2x, double pos2y
    ) {
        return std::sqrt(
                (x - pos1x) * (x - pos1x) + (y - pos1y) * (y - pos1y)
        ) - std::sqrt(
                (x - pos2x) * (x - pos2x) + (y - pos2y) * (y - pos2y));
    }

    Eigen::Vector2d posD;
    Eigen::Vector2d posE;
    Eigen::Vector2d posF;

    double dAB_from_D;
    double dBC_from_D;
    double dAC_from_D;
    double dAB_from_E;
    double dBC_from_E;
    double dAC_from_E;
    double dAB_from_F;
    double dBC_from_F;
    double dAC_from_F;

    Eigen::Vector2d startPos;
};

