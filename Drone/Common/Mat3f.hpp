/*
 * Mat3f.hpp
 *
 *  Created on: Aug 25, 2026
 *      Author: KAVINDU
 */

#ifndef COMMON_MAT3F_HPP_
#define COMMON_MAT3F_HPP_

#include "Vector3f.hpp"

struct Mat3f {
    float m[3][3];

    static Mat3f identity() {
        Mat3f r{};
        r.m[0][0] = r.m[1][1] = r.m[2][2] = 1.0f;
        r.m[0][1] = r.m[0][2] = r.m[1][0] = r.m[1][2] = r.m[2][0] = r.m[2][1] = 0.0f;
        return r;
    }

    Vector3f mul(const Vector3f &v) const {
        return Vector3f(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
    }

    // this * other
    Mat3f mul(const Mat3f &o) const {
        Mat3f r{};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                float s = 0.0f;
                for (int k = 0; k < 3; k++) s += m[i][k] * o.m[k][j];
                r.m[i][j] = s;
            }
        return r;
    }

    Mat3f scaled(float s) const {
        Mat3f r{};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) r.m[i][j] = m[i][j] * s;
        return r;
    }

    Mat3f transposed() const {
        Mat3f r{};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) r.m[i][j] = m[j][i];
        return r;
    }
};

#endif /* COMMON_MAT3F_HPP_ */
