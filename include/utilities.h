#pragma once
#ifndef _FC_BRIDGE_ULTILITIES_H_
#define _FC_BRIDGE_ULTILITIES_H_

#include <Eigen/Geometry>

#define PI 3.1415926

// remember to allocate 2rd arrays!
// remember to delete pose_traj!
void MotionPlanning(const float *pose0, const float *pose_set, const int Nsteps,
    float **pose_traj) {
  Eigen::Quaternionf q0(pose0[3], pose0[4], pose0[5],pose0[6]);
  Eigen::Quaternionf qset(pose_set[3], pose_set[4], pose_set[5],pose_set[6]);
  Eigen::Quaternionf q;

  for (int i = 0; i < Nsteps; ++i) {
    pose_traj[i] = new float[7];
    q = q0.slerp(float(i+1)/float(Nsteps), qset);
    pose_traj[i][0] = (pose0[0]*float(Nsteps-i-1) + pose_set[0]*float(i+1))/float(Nsteps);
    pose_traj[i][1] = (pose0[1]*float(Nsteps-i-1) + pose_set[1]*float(i+1))/float(Nsteps);
    pose_traj[i][2] = (pose0[2]*float(Nsteps-i-1) + pose_set[2]*float(i+1))/float(Nsteps);
    pose_traj[i][3] = q.w();
    pose_traj[i][4] = q.x();
    pose_traj[i][5] = q.y();
    pose_traj[i][6] = q.z();
  }
}

Eigen::Quaternionf QuatMTimes(const Eigen::Quaternionf &q1,
    const Eigen::Quaternionf &q2)  {
  float s1 = q1.w();
  Eigen::Vector3f v1(q1.x(), q1.y(), q1.z());

  float s2 = q2.w();
  Eigen::Vector3f v2(q2.x(), q2.y(), q2.z());

  float cr_v1 = v1(1)*v2(2) - v1(2)*v2(1);
  float cr_v2 = v1(2)*v2(0) - v1(0)*v2(2);
  float cr_v3 = v1(0)*v2(1) - v1(1)*v2(0);

  Eigen::Quaternionf qp;
  qp.w() = s1*s2 - v2.dot(v1);
  qp.x() = v2(0)*s1 + s2*v1(0) + cr_v1;
  qp.y() = v2(1)*s1 + s2*v1(1) + cr_v2;
  qp.z() = v2(2)*s1 + s2*v1(2) + cr_v3;

  return qp;
}


float angBTquat(Eigen::Quaternionf q1, Eigen::Quaternionf q2) {
  q1.normalize();
  q2.normalize();

  Eigen::Quaternionf q_ = QuatMTimes(q1.inverse(), q2);

  float ang = 2.0f*acos(q_.w()); // acos: [0, pi]

  if (ang > PI){
    ang = ang - 2.0f*PI;
  }
  return fabs(ang);
}

Eigen::Matrix3f quat2m(const Eigen::Quaternionf &q) {
  float q11 = q.x()*q.x();
  float q22 = q.y()*q.y();
  float q33 = q.z()*q.z();
  float q01 = q.w()*q.x();
  float q02 = q.w()*q.y();
  float q03 = q.w()*q.z();
  float q12 = q.x()*q.y();
  float q13 = q.x()*q.z();
  float q23 = q.y()*q.z();

  Eigen::Matrix3f m;
  m << 1.0f - 2.0f*q22 - 2.0f*q33, 2.0f*(q12 - q03),      2.0f*(q13 + q02),
      2.0f*(q12 + q03),     1.0f - 2.0f*q11 - 2.0f*q33,  2.0f*(q23 - q01),
      2.0f*(q13 - q02),     2.0f*(q23 + q01),      1.0f - 2.0f*q11 - 2.0f*q22;
  return m;
}


// wedge operation.
//   cross(v, u) = v_wedge * u
//
// input:
//   v: 3 x 1
// output:
// v_wedge: 3 x 3
Eigen::Matrix3f wedge(const Eigen::Vector3f v) {
  Eigen::Matrix3f v_wedge;
  v_wedge << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
  return v_wedge;
}

#endif // _FC_BRIDGE_ULTILITIES_H_
