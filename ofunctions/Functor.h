#pragma once

#include <Eigen/Dense>

/**
 * Базовый функтор оптимизации
 * @tparam _Scalar - тип данных для скалаяра
 * @tparam NX тип данных для входных значений
 * @tparam NY тип данных для выходных значений
 */
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor {
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;
    /**
     * кол-во входов
     */
    int m_inputs;
    /**
     * кол-во выходов
     */
    int m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}

    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const { return m_inputs; }

    int values() const { return m_values; }

};
