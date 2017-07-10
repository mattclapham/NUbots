
/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2016 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_SUPPORT_EIGEN_H
#define UTILITY_SUPPORT_EIGEN_H

#include <Eigen/Core>
#include <random>
#include <vector>

namespace utility {
namespace support {


    template <typename Comp>
    inline Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1> find(const Comp& comp) {
        Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1> indices(comp.count());

        for (Eigen::Index index = 0, count = 0; index < comp.size(); index++) {
            if (comp(index)) {
                indices(count++) = index;
            }
        }

        return indices;
    }

    template <typename Scalar, int RowsIn, int ColsIn>
    inline Eigen::Matrix<Scalar, Eigen::Dynamic, 1> index(
        const Eigen::Matrix<Scalar, RowsIn, ColsIn>& A,
        const Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1>& indices) {
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> values{indices.size()};

        for (Eigen::Index index = 0; index < indices.size(); index++) {
            values(index) = A(indices[index]);
        }

        return values;
    }

    template <typename Scalar, int RowsIn, int ColsIn>
    inline Eigen::Matrix<Scalar, RowsIn, Eigen::Dynamic> indexCols(
        const Eigen::Matrix<Scalar, RowsIn, ColsIn>& A,
        const Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1>& indices) {
        Eigen::Matrix<Scalar, RowsIn, Eigen::Dynamic> values{RowsIn, indices.size()};

        for (Eigen::Index index = 0; index < indices.size(); index++) {
            values.template middleCols<1>(index) = A.template middleCols<1>(indices[index]);
        }

        return values;
    }

    template <typename Scalar, int RowsIn, int ColsIn>
    inline Eigen::Matrix<Scalar, Eigen::Dynamic, ColsIn> indexRows(
        const Eigen::Matrix<Scalar, RowsIn, ColsIn>& A,
        const Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1>& indices) {
        Eigen::Matrix<Scalar, Eigen::Dynamic, ColsIn> values{indices.size(), ColsIn};

        for (Eigen::Index index = 0; index < indices.size(); index++) {
            values.template middleRows<1>(index) = A.template middleRows<1>(indices[index]);
        }

        return values;
    }

    template <typename Scalar, int N>
    inline Eigen::Matrix<Scalar, N, 1> sort(const Eigen::Matrix<Scalar, N, 1>& in) {
        Eigen::Matrix<Scalar, N, 1> out{in};

        std::sort(out.data(), out.data() + N);
        return out;
    }

    template <typename Scalar, int N>
    inline Eigen::Matrix<Eigen::Index, N, 1> sort_index(const Eigen::Matrix<Scalar, N, 1>& in) {
        Eigen::Matrix<Eigen::Index, N, 1> out = Eigen::Matrix<Eigen::Index, N, 1>::LinSpaced(N, 0, N - 1);

        // sort indexes based on comparing values in v
        std::sort(out.data(), out.data() + N, [&in](const Eigen::Index& i1, const Eigen::Index& i2) {
            return in[i1] < in[i2];
        });

        return out;
    }

    template <typename T,
              std::enable_if_t<((T::RowsAtCompileTime != Eigen::Dynamic)
                                && (T::ColsAtCompileTime != Eigen::Dynamic))>* = nullptr>
    inline T randn(typename T::Scalar mean   = static_cast<typename T::Scalar>(0.0),
                   typename T::Scalar stddev = static_cast<typename T::Scalar>(1.0)) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<typename T::Scalar> d(mean, stddev);

        T out;
        return out.unaryExpr([&](typename T::Scalar) -> typename T::Scalar { return d(gen); });
    }

    template <typename T,
              std::enable_if_t<((T::RowsAtCompileTime == Eigen::Dynamic) && (T::ColsAtCompileTime == 1))>* = nullptr>
    inline T randn(size_t rows,
                   typename T::Scalar mean   = static_cast<typename T::Scalar>(0.0),
                   typename T::Scalar stddev = static_cast<typename T::Scalar>(1.0)) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<typename T::Scalar> d(mean, stddev);

        T out(rows);
        return out.unaryExpr([&](typename T::Scalar) -> typename T::Scalar { return d(gen); });
    }

    template <typename T,
              std::enable_if_t<((T::RowsAtCompileTime == 1) && (T::ColsAtCompileTime == Eigen::Dynamic))>* = nullptr>
    inline T randn(size_t cols,
                   typename T::Scalar mean   = static_cast<typename T::Scalar>(0.0),
                   typename T::Scalar stddev = static_cast<typename T::Scalar>(1.0)) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<typename T::Scalar> d(mean, stddev);

        T out(cols);
        return out.unaryExpr([&](typename T::Scalar) -> typename T::Scalar { return d(gen); });
    }

    template <typename T,
              std::enable_if_t<((T::RowsAtCompileTime == Eigen::Dynamic)
                                && (T::ColsAtCompileTime == Eigen::Dynamic))>* = nullptr>
    inline T randn(size_t rows,
                   size_t cols,
                   typename T::Scalar mean   = static_cast<typename T::Scalar>(0.0),
                   typename T::Scalar stddev = static_cast<typename T::Scalar>(1.0)) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<typename T::Scalar> d(mean, stddev);

        T out(rows, cols);
        return out.unaryExpr([&](typename T::Scalar) -> typename T::Scalar { return d(gen); });
    }

    template <typename T,
              std::enable_if_t<((T::RowsAtCompileTime != Eigen::Dynamic)
                                && (T::ColsAtCompileTime != Eigen::Dynamic))>* = nullptr>
    inline T randu(typename T::Scalar a = static_cast<typename T::Scalar>(0.0),
                   typename T::Scalar b = static_cast<typename T::Scalar>(1.0)) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<typename T::Scalar> d(a, b);

        T out;
        return out.unaryExpr([&](typename T::Scalar) -> typename T::Scalar { return d(gen); });
    }

    template <typename T,
              std::enable_if_t<((T::RowsAtCompileTime == Eigen::Dynamic) && (T::ColsAtCompileTime == 1))>* = nullptr>
    inline T randu(size_t rows,
                   typename T::Scalar a = static_cast<typename T::Scalar>(0.0),
                   typename T::Scalar b = static_cast<typename T::Scalar>(1.0)) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<typename T::Scalar> d(a, b);

        T out(rows);
        return out.unaryExpr([&](typename T::Scalar) -> typename T::Scalar { return d(gen); });
    }

    template <typename T,
              std::enable_if_t<((T::RowsAtCompileTime == 1) && (T::ColsAtCompileTime == Eigen::Dynamic))>* = nullptr>
    inline T randu(size_t cols,
                   typename T::Scalar a = static_cast<typename T::Scalar>(0.0),
                   typename T::Scalar b = static_cast<typename T::Scalar>(1.0)) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<typename T::Scalar> d(a, b);

        T out(cols);
        return out.unaryExpr([&](typename T::Scalar) -> typename T::Scalar { return d(gen); });
    }

    template <typename T,
              std::enable_if_t<((T::RowsAtCompileTime == Eigen::Dynamic)
                                && (T::ColsAtCompileTime == Eigen::Dynamic))>* = nullptr>
    inline T randu(size_t rows,
                   size_t cols,
                   typename T::Scalar a = static_cast<typename T::Scalar>(0.0),
                   typename T::Scalar b = static_cast<typename T::Scalar>(1.0)) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<typename T::Scalar> d(a, b);

        T out(rows, cols);
        return out.unaryExpr([&](typename T::Scalar) -> typename T::Scalar { return d(gen); });
    }

    // https://gist.github.com/javidcf/25066cf85e71105d57b6
    template <typename T>
    T pseudoinverse(const T& mat, typename T::Scalar tolerance = static_cast<typename T::Scalar>(1e-6)) {
        auto svd                   = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
        const auto& singularValues = svd.singularValues();
        T singularValuesInv        = T::Zero(mat.cols(), mat.rows());

        for (ssize_t i = 0; i < singularValues.size(); ++i) {
            if (singularValues(i) > tolerance) {
                singularValuesInv(i, i) = static_cast<typename T::Scalar>(1) / singularValues(i);
            }
            else {
                singularValuesInv(i, i) = static_cast<typename T::Scalar>(0);
            }
        }

        return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
    }

    template <typename T>
    void SVD(Eigen::Matrix<typename T::Scalar, Eigen::Dynamic, Eigen::Dynamic>& U,
             Eigen::Matrix<typename T::Scalar, Eigen::Dynamic, 1>& s,
             Eigen::Matrix<typename T::Scalar, Eigen::Dynamic, Eigen::Dynamic>& V,
             const T& A) {
        auto svd = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
        U        = svd.matrixU();
        s        = svd.singularValues();
        V        = svd.matrixV();
    }

    template <typename T1, typename T2>
    Eigen::MatrixXd kronecker(const T1& m1, const T2& m2) {
        Eigen::MatrixXd m3(m1.rows() * m2.rows(), m1.cols() * m2.cols());

        for (ssize_t i = 0; i < m1.rows(); i++) {
            for (ssize_t j = 0; j < m1.cols(); j++) {
                m3.block(i * m2.rows(), j * m2.cols(), m2.rows(), m2.cols()) = m1(i, j) * m2;
            }
        }

        return m3;
    }
}
}


#endif
