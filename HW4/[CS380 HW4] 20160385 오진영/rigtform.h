#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>

#include "matrix4.h"
#include "quat.h"


class RigTForm {
  Cvec3 t_; // translation component
  Quat r_;  // rotation component represented as a quaternion

public:
  RigTForm() : t_(0) {
    assert(norm2(Quat(1,0,0,0) - r_) < CS175_EPS2);
  }

  RigTForm(const Cvec3& t, const Quat& r) {

    // TODO
    this->t_ = t;
    this->r_ = r;
  }

  explicit RigTForm(const Cvec3& t) {

    // TODO
    this->t_ = t;
    this->r_ = Quat();
  }

  explicit RigTForm(const Quat& r) {

    // TODO
    this->t_ = Cvec3();
    this->r_ = r;
  }

  Cvec3 getTranslation() const {
    return t_;
  }

  Quat getRotation() const {
    return r_;
  }

  RigTForm& setTranslation(const Cvec3& t) {
    t_ = t;
    return *this;
  }

  RigTForm& setRotation(const Quat& r) {
    r_ = r;
    return *this;
  }

  Cvec4 operator * (const Cvec4& a) {
    // TODO

    return (this->r_) * a + Cvec4(this->t_,0);
  }

  RigTForm operator * (const RigTForm& a) const {

    Cvec3 t = Cvec3(this -> t_);
    t += Cvec3(this->r_ * Cvec4(a.getTranslation(),0));

    Quat r = this->r_ * a.getRotation();

    return RigTForm(t,r);


    // TODO
  }
};

inline RigTForm inv(const RigTForm& tform) {
  // TODO

  Quat newR = inv(tform.getRotation());
  Cvec4 newT4 = newR * Cvec4(tform.getTranslation()[0], tform.getTranslation()[1], tform.getTranslation()[2], 0) * -1.0;
  Cvec3 newT = Cvec3(newT4);
  return RigTForm(newT, newR);
}

inline RigTForm transFact(const RigTForm& tform) {
  return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm& tform) {
  return RigTForm(tform.getRotation());
}

static Matrix4 makeTranslation_rig(const Cvec3& t) {
  Matrix4 r;
  for (int i = 0; i < 3; ++i) {
    r(i,3) = t[i];
  }
  return r;
}


inline Matrix4 rigTFormToMatrix(const RigTForm& tform) {
  // TODO
  Matrix4 T = makeTranslation_rig(tform.getTranslation());
  Matrix4 R = quatToMatrix(tform.getRotation());
  return T * R;
}




#endif
