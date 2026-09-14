#pragma once
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace go2_exploration {
struct SelfBox {
  std::string name;
  Eigen::Vector3d center, size;
  Eigen::Matrix3d rotation=Eigen::Matrix3d::Identity();
  double padding=0;
  void validate() const {
    if (name.empty() || !center.allFinite() || !size.allFinite() ||
        !rotation.allFinite() || !std::isfinite(padding) || padding<0 || padding>.03 ||
        (size.array()<=0).any() || std::abs(rotation.determinant()-1)>1e-6 ||
        !(rotation.transpose()*rotation).isApprox(Eigen::Matrix3d::Identity(),1e-6))
      throw std::invalid_argument("invalid self-filter box: "+name);
  }
  Eigen::Vector3d local(const Eigen::Vector3d& point) const {
    return rotation.transpose()*(point-center);
  }
  bool contains(const Eigen::Vector3d& point) const {
    return point.allFinite() && (local(point).array().abs() <= size.array()*.5+padding).all();
  }
  // Check the measured ray segment, never its extension behind the endpoint.
  // Returns behind a mounted part cannot clear that part's shadow as free.
  bool intersectsRay(const Eigen::Vector3d& origin,const Eigen::Vector3d& point) const {
    if (!origin.allFinite() || !point.allFinite()) return false;
    const Eigen::Vector3d o=local(origin),d=rotation.transpose()*(point-origin);
    const Eigen::Vector3d half=size*.5+Eigen::Vector3d::Constant(padding);
    double lo=0,hi=1;
    for (int i=0;i<3;++i) {
      if (std::abs(d[i])<1e-12) {if (std::abs(o[i])>half[i]) return false;continue;}
      double a=(-half[i]-o[i])/d[i],b=(half[i]-o[i])/d[i];
      if (a>b) std::swap(a,b);
      lo=std::max(lo,a);hi=std::min(hi,b);
      if (lo>hi) return false;
    }
    return hi>=0 && lo<=1;
  }
};
class SelfFilterGeometry {
 public:
  Eigen::Vector3d origin=Eigen::Vector3d::Zero();
  Eigen::Matrix3d lidar_rotation=Eigen::Matrix3d::Identity();
  std::vector<SelfBox> boxes;
  void validate() const {
    if (!origin.allFinite() || !lidar_rotation.allFinite()) throw std::invalid_argument("invalid lidar mount");
    for (const auto& b:boxes) {
      b.validate();
      if (b.contains(origin)) throw std::invalid_argument("self-filter box includes lidar origin: "+b.name);
    }
  }
  Eigen::Vector3d toBase(const Eigen::Vector3d& p) const {return lidar_rotation*p+origin;}
  bool reject(const Eigen::Vector3d& base_point) const {
    for (const auto& box:boxes) if (box.intersectsRay(origin,base_point)) return true;
    return false;
  }
};
inline Eigen::Matrix3d rpyRotation(double roll,double pitch,double yaw) {
  return (Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
          Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*
          Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
}
}
