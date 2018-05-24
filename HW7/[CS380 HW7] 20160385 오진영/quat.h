#ifndef QUAT_H
#define QUAT_H

#include <iostream>
#include <cassert>
#include <cmath>

#include "cvec.h"
#include "matrix4.h"

// Forward declarations used in the definition of Quat;
class Quat;
double dot(const Quat& q, const Quat& p);
double norm2(const Quat& q);
Quat inv(const Quat& q);
Quat normalize(const Quat& q);
Matrix4 quatToMatrix(const Quat& q);

class Quat {
  Cvec4 q_;  // layout is: q_[0]==w, q_[1]==x, q_[2]==y, q_[3]==z

public:
  double operator [] (const int i) const {
    return q_[i];
  }

  double& operator [] (const int i) {
    return q_[i];
  }

  double operator () (const int i) const {
    return q_[i];
  }

  double& operator () (const int i) {
    return q_[i];
  }

  Quat() : q_(1,0,0,0) {}
  Quat(const double w, const Cvec3& v) : q_(w, v[0], v[1], v[2]) {}
  Quat(const double w, const double x, const double y, const double z) : q_(w, x,y,z) {}

  Quat& operator += (const Quat& a) {
    q_ += a.q_;
    return *this;
  }

  Quat& operator -= (const Quat& a) {
    q_ -= a.q_;
    return *this;
  }

  Quat& operator *= (const double a) {
    q_ *= a;
    return *this;
  }

  Quat& operator /= (const double a) {
    q_ /= a;
    return *this;
  }

  Quat operator + (const Quat& a) const {
    return Quat(*this) += a;
  }

  Quat operator - (const Quat& a) const {
    return Quat(*this) -= a;
  }

  Quat operator * (const double a) const {
    return Quat(*this) *= a;
  }

  Quat operator / (const double a) const {
    return Quat(*this) /= a;
  }

  Quat operator * (const Quat& a) const {
    const Cvec3 u(q_[1], q_[2], q_[3]), v(a.q_[1], a.q_[2], a.q_[3]);
    return Quat(q_[0]*a.q_[0] - dot(u, v), (v*q_[0] + u*a.q_[0]) + cross(u, v));
  }

  Cvec4 operator * (const Cvec4& a) const {
    const Quat r = *this * (Quat(0, a[0], a[1], a[2]) * inv(*this));
    return Cvec4(r[1], r[2], r[3], a[3]);
  }

  static Quat makeXRotation(const double ang) {
    Quat r;
    const double h = 0.5 * ang * CS175_PI/180;
    r.q_[1] = std::sin(h);
    r.q_[0] = std::cos(h);
    return r;
  }

  static Quat makeYRotation(const double ang) {
    Quat r;
    const double h = 0.5 * ang * CS175_PI/180;
    r.q_[2] = std::sin(h);
    r.q_[0] = std::cos(h);
    return r;
  }

  static Quat makeZRotation(const double ang) {
    Quat r;
    const double h = 0.5 * ang * CS175_PI/180;
    r.q_[3] = std::sin(h);
    r.q_[0] = std::cos(h);
    return r;
  }
};

inline double dot(const Quat& q, const Quat& p) {
  double s = 0.0;
  for (int i = 0; i < 4; ++i) {
    s += q(i) * p(i);
  }
  return s;
}

inline double norm2(const Quat& q) {
  return dot(q, q);
}

inline Quat inv(const Quat& q) {
  const double n = norm2(q);
  assert(n > CS175_EPS2);
  return Quat(q(0), -q(1), -q(2), -q(3)) * (1.0/n);
}

inline Quat normalize(const Quat& q) {
  return q / std::sqrt(norm2(q));
}

inline Matrix4 quatToMatrix(const Quat& q) {
  Matrix4 r;
  const double n = norm2(q);
  if (n < CS175_EPS2)
    return Matrix4(0);

  const double two_over_n = 2/n;
  r(0, 0) -= (q(2)*q(2) + q(3)*q(3)) * two_over_n;
  r(0, 1) += (q(1)*q(2) - q(0)*q(3)) * two_over_n;
  r(0, 2) += (q(1)*q(3) + q(2)*q(0)) * two_over_n;
  r(1, 0) += (q(1)*q(2) + q(0)*q(3)) * two_over_n;
  r(1, 1) -= (q(1)*q(1) + q(3)*q(3)) * two_over_n;
  r(1, 2) += (q(2)*q(3) - q(1)*q(0)) * two_over_n;
  r(2, 0) += (q(1)*q(3) - q(2)*q(0)) * two_over_n;
  r(2, 1) += (q(2)*q(3) + q(1)*q(0)) * two_over_n;
  r(2, 2) -= (q(1)*q(1) + q(2)*q(2)) * two_over_n;

  assert(isAffine(r));
  return r;
}

inline Quat cn(const Quat& q){
  if(q[0] < 0) return Quat(-1 * q(0), q(1), q(2), q(3));
  return q;
}

inline Quat power(const Quat& q1, const float alpha){
  Quat q;
  if(q1[0] < 0) q = Quat(-1 * q1(0), q1(1), q1(2), q1(3));
  else q = q1;

  assert(q[0] > 0);
  double sinTheta = sqrt(q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  double theta = atan2(sinTheta, q[0]);

  Quat answer =  Quat(cos(alpha*theta), sin(alpha * theta) * q[1], sin(alpha * theta) * q[2], sin(alpha * theta) * q[3]);
  if (sinTheta != 0) {
      answer /=sinTheta;
      answer[0] *= sinTheta;
  }
  return answer;

  //assert(answer[0] > 0);
  //return normalize(answer);
}

inline Quat slerp(const Quat& startQuat, const Quat& endQuat, const float alpha){
  //Quat temp = cn(endQuat * inv(startQuat));

  Quat answer = power(endQuat * inv(startQuat),alpha) * startQuat;

  return answer;
}

inline Quat catmullRom(const Quat& prev, const Quat& start, const Quat& end, const Quat& next, const float alpha){
  Quat d = power(cn(end * inv(prev)), 1/6) * start;   //  ((end - prev) / 6) + start;
  Quat e = power(cn(next * inv(start)), 1/6) * next; //((next - start) / -6) + end;

  Quat p01 = slerp(start, d, alpha);
  Quat p12 = slerp(d, e, alpha);
  Quat p23 = slerp(e, end, alpha);

  Quat p012 = slerp(p01, p12, alpha);
  Quat p123 = slerp(p12, p23, alpha);

  return slerp(p012, p123, alpha);
}

#endif
