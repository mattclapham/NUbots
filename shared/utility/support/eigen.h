
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
    inline T randn() {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<typename T::Scalar> d(static_cast<typename T::Scalar>(0.0),
                                                       static_cast<typename T::Scalar>(1.0));

        return T().unaryExpr([&](typename T::Scalar elem) -> typename T::Scalar { return d(gen); });
    }

    template <typename T,
              std::enable_if_t<((T::RowsAtCompileTime == Eigen::Dynamic) && (T::ColsAtCompileTime == 1))>* = nullptr>
    inline T randn(size_t cols) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<typename T::Scalar> d(static_cast<typename T::Scalar>(0.0),
                                                       static_cast<typename T::Scalar>(1.0));

        return T(cols).unaryExpr([&](typename T::Scalar elem) -> typename T::Scalar { return d(gen); });
    }

    template <typename T,
              std::enable_if_t<((T::RowsAtCompileTime == 1) && (T::ColsAtCompileTime == Eigen::Dynamic))>* = nullptr>
    inline T randn(size_t rows) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<typename T::Scalar> d(static_cast<typename T::Scalar>(0.0),
                                                       static_cast<typename T::Scalar>(1.0));

        return T(rows).unaryExpr([&](typename T::Scalar elem) -> typename T::Scalar { return d(gen); });
    }

    template <typename T,
              std::enable_if_t<((T::RowsAtCompileTime == Eigen::Dynamic)
                                && (T::ColsAtCompileTime == Eigen::Dynamic))>* = nullptr>
    inline T randn(size_t rows, size_t cols) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<typename T::Scalar> d(static_cast<typename T::Scalar>(0.0),
                                                       static_cast<typename T::Scalar>(1.0));

        return T(rows, cols).unaryExpr([&](typename T::Scalar elem) -> typename T::Scalar { return d(gen); });
    }
}
}

#endif
