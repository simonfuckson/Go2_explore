#pragma once
#include "self_filter_geometry.h"
#include <boost/make_shared.hpp>
#include <cstring>
#include <functional>
#include <set>
#include <ros/ros.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

namespace go2_exploration {
class CameraSelfFilter {
  ros::NodeHandle nh_, params_{"~self_filter"};
  SelfFilterGeometry geometry_;
  bool enabled_=false;
  double maximum_fraction_=.10;
  double ground_resolution_=.15,ground_min_x_=-5.,ground_min_y_=-4.,map_resolution_=.05;
  std::string calibration_,error_;
  ros::Publisher status_,markers_,removed_,ground_,clearing_;
  ros::Subscriber ground_in_,clearing_in_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_{tf_};
  std::function<void(const std::string&)> fail_;
  size_t input_count_=0,rejected_count_=0;
  Eigen::Vector3d vector(const XmlRpc::XmlRpcValue& item,const std::string& key) {
    if (!item.hasMember(key) || item[key].getType()!=XmlRpc::XmlRpcValue::TypeArray || item[key].size()!=3)
      throw std::invalid_argument("self-filter requires 3 values: "+key);
    Eigen::Vector3d v;
    for(int i=0;i<3;++i) v[i]=number(item[key][i]);
    if (!v.allFinite()) throw std::invalid_argument("nonfinite self-filter vector: "+key);
    return v;
  }
  double number(const XmlRpc::XmlRpcValue& v) {
    if(v.getType()==XmlRpc::XmlRpcValue::TypeDouble)return static_cast<double>(v);
    if(v.getType()==XmlRpc::XmlRpcValue::TypeInt)return static_cast<int>(v);
    throw std::invalid_argument("self-filter value must be numeric");
  }
  void load() {
    params_.param("enabled",enabled_,false);
    params_.param<std::string>("calibration_id",calibration_,"");
    params_.param("max_removed_fraction",maximum_fraction_,.10);
    if (!std::isfinite(maximum_fraction_) || maximum_fraction_<=0 || maximum_fraction_>.25)
      throw std::invalid_argument("self-filter max_removed_fraction must be in (0, 0.25]");
    std::vector<double> reference;
    if(!params_.getParam("reference_lidar_mount",reference) || reference.size()!=6)
      throw std::invalid_argument("missing self-filter reference_lidar_mount");
    const std::vector<std::string> keys={"x","y","z","roll_deg","pitch_deg","yaw_deg"};
    std::vector<double> mount(6);
    for(size_t i=0;i<6;++i) {
      if (!nh_.getParam("/mid360_mount/base_link_to_lidar_link/"+keys[i],mount[i]) ||
          !std::isfinite(mount[i]) || !std::isfinite(reference[i]) || std::abs(mount[i]-reference[i])>1e-6)
        throw std::invalid_argument("self-filter calibration does not match existing lidar mount: "+keys[i]);
    }
    geometry_.origin={mount[0],mount[1],mount[2]};
    geometry_.lidar_rotation=rpyRotation(mount[3]*M_PI/180,mount[4]*M_PI/180,mount[5]*M_PI/180);
    XmlRpc::XmlRpcValue boxes;
    if(!params_.getParam("boxes",boxes) || boxes.getType()!=XmlRpc::XmlRpcValue::TypeArray)
      throw std::invalid_argument("self-filter boxes must be an array");
    std::set<std::string> names;
    for(int i=0;i<boxes.size();++i) {
      const auto& item=boxes[i];SelfBox b;
      if(item.getType()!=XmlRpc::XmlRpcValue::TypeStruct || !item.hasMember("name"))
        throw std::invalid_argument("self-filter box name required");
      b.name=static_cast<std::string>(item["name"]);
      if (!names.insert(b.name).second) throw std::invalid_argument("duplicate self-filter box name");
      b.center=vector(item,"center");b.size=vector(item,"size");
      auto rpy=vector(item,"rpy_deg")*M_PI/180;b.rotation=rpyRotation(rpy[0],rpy[1],rpy[2]);
      b.padding=item.hasMember("padding")?number(item["padding"]):0;
      geometry_.boxes.push_back(b);
    }
    geometry_.validate();
    nh_.param("/go2_terrain_guard/grid/resolution",ground_resolution_,.15);
    nh_.param("/go2_terrain_guard/grid/min_x",ground_min_x_,-5.);
    nh_.param("/go2_terrain_guard/grid/min_y",ground_min_y_,-4.);
    nh_.param("/move_base/local_costmap/resolution",map_resolution_,.05);
    if (!(ground_resolution_>0 && map_resolution_>0) || !std::isfinite(ground_resolution_) ||
        !std::isfinite(map_resolution_) || ground_resolution_/map_resolution_>100)
      throw std::invalid_argument("invalid self-filter support sampling resolution");
    if(enabled_ && (calibration_.empty() || geometry_.boxes.empty()))
      throw std::invalid_argument("enabled self-filter requires measured boxes and a calibration_id");
  }
  void fault(const std::string& reason) {
    if (error_.empty()) error_=reason;
    fail_("camera_self_filter:"+reason);
  }
  void publishBoxes() {
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker clear;clear.action=visualization_msgs::Marker::DELETEALL;array.markers.push_back(clear);
    for(size_t i=0;i<geometry_.boxes.size();++i) {
      const auto& b=geometry_.boxes[i];visualization_msgs::Marker m;
      m.header.frame_id="base_link";m.ns="camera_self_filter";m.id=i;
      m.type=visualization_msgs::Marker::CUBE;m.action=visualization_msgs::Marker::ADD;m.frame_locked=true;
      m.pose.position.x=b.center.x();m.pose.position.y=b.center.y();m.pose.position.z=b.center.z();
      const Eigen::Quaterniond q(b.rotation);
      m.pose.orientation.x=q.x();m.pose.orientation.y=q.y();m.pose.orientation.z=q.z();m.pose.orientation.w=q.w();
      m.scale.x=b.size.x()+2*b.padding;m.scale.y=b.size.y()+2*b.padding;m.scale.z=b.size.z()+2*b.padding;
      m.color.r=enabled_?0.1:1.;m.color.g=1.;m.color.b=0.1;m.color.a=.25;array.markers.push_back(m);
    }
    markers_.publish(array);
  }
  void cloud(const sensor_msgs::PointCloud2::ConstPtr& msg,ros::Publisher* output) {
    if (!error_.empty()) return;
    if (!enabled_ || geometry_.boxes.empty()) {output->publish(msg);return;}
    try {
      Eigen::Affine3d transform=Eigen::Affine3d::Identity();
      if (msg->header.frame_id!="base_link") {
        const auto t=tf_.lookupTransform("base_link",msg->header.frame_id,msg->header.stamp,ros::Duration(.03)).transform;
        transform.translation()=Eigen::Vector3d(t.translation.x,t.translation.y,t.translation.z);
        transform.linear()=Eigen::Quaterniond(t.rotation.w,t.rotation.x,t.rotation.y,t.rotation.z).toRotationMatrix();
      }
      size_t offsets[3];const char* names[]={"x","y","z"};
      for(int i=0;i<3;++i) {
        bool found=false;
        for(const auto& f:msg->fields) if(f.name==names[i] && f.datatype==sensor_msgs::PointField::FLOAT32 && f.count==1) {offsets[i]=f.offset;found=true;}
        if(!found || offsets[i]+4>msg->point_step)throw std::invalid_argument("unsupported support cloud layout");
      }
      if(!msg->point_step || msg->is_bigendian || msg->row_step<msg->width*msg->point_step || msg->data.size()<size_t(msg->row_step)*msg->height)
        throw std::invalid_argument("invalid support cloud layout");
      sensor_msgs::PointCloud2 filtered=*msg;filtered.data.clear();filtered.data.reserve(msg->data.size());
      for(size_t row=0;row<msg->height;++row)for(size_t col=0;col<msg->width;++col) {
        const auto* data=msg->data.data()+row*msg->row_step+col*msg->point_step;float xyz[3];
        for(int i=0;i<3;++i)std::memcpy(xyz+i,data+offsets[i],4);
        const Eigen::Vector3d p=transform*Eigen::Vector3d(xyz[0],xyz[1],xyz[2]);
        bool blocked=geometry_.reject(p);
        if (!blocked && output==&ground_ && p.allFinite()) {
          // Coverage memory expands a confirmed terrain cell into these exact
          // support samples. Reject the cell if any would enter the shadow,
          // preventing later upsampling from filling a masked region again.
          const double gx=std::floor((xyz[0]-ground_min_x_)/ground_resolution_)*ground_resolution_+ground_min_x_;
          const double gy=std::floor((xyz[1]-ground_min_y_)/ground_resolution_)*ground_resolution_+ground_min_y_;
          const double step=map_resolution_*.5;
          for(double sx=step*.5;sx<ground_resolution_ && !blocked;sx+=step)
            for(double sy=step*.5;sy<ground_resolution_ && !blocked;sy+=step)
              blocked=geometry_.reject(transform*Eigen::Vector3d(gx+sx,gy+sy,xyz[2]));
        }
        if(p.allFinite() && !blocked) filtered.data.insert(filtered.data.end(),data,data+msg->point_step);
      }
      filtered.height=1;filtered.width=filtered.data.size()/msg->point_step;filtered.row_step=filtered.width*filtered.point_step;
      output->publish(filtered);
    } catch(const std::exception& e) {fault(std::string("support_transform_or_layout:")+e.what());}
  }
 public:
  explicit CameraSelfFilter(std::function<void(const std::string&)> fail):fail_(std::move(fail)) {
    load();
    status_=nh_.advertise<std_msgs::String>("/exploration/self_filter/status",1,true);
    markers_=nh_.advertise<visualization_msgs::MarkerArray>("/exploration/self_filter/boxes",1,true);
    removed_=nh_.advertise<sensor_msgs::PointCloud2>("/exploration/self_filter/candidate_points",1);
    ground_=nh_.advertise<sensor_msgs::PointCloud2>("/exploration/self_filter/ground_points",1);
    clearing_=nh_.advertise<sensor_msgs::PointCloud2>("/exploration/self_filter/clearing_points",1);
    ground_in_=nh_.subscribe<sensor_msgs::PointCloud2>("/terrain/ground_points",1,[this](const sensor_msgs::PointCloud2::ConstPtr& m){cloud(m,&ground_);});
    clearing_in_=nh_.subscribe<sensor_msgs::PointCloud2>("/exploration/clearing_points",1,[this](const sensor_msgs::PointCloud2::ConstPtr& m){cloud(m,&clearing_);});
    publishBoxes();publishStatus();
  }
  void publishStatus() {
    std_msgs::String msg;
    msg.data=!error_.empty()?"FAULT: "+error_:geometry_.boxes.empty()?"disabled: waiting for measured camera/bracket geometry":
      std::string(enabled_?"active: ":"preview only: ")+calibration_+"; masked_candidates="+std::to_string(rejected_count_)+"/"+std::to_string(input_count_);
    status_.publish(msg);
  }
  livox_ros_driver2::CustomMsg::ConstPtr filter(const livox_ros_driver2::CustomMsg::ConstPtr& msg) {
    if (!error_.empty())return {};
    if (geometry_.boxes.empty())return msg;
    auto out=boost::make_shared<livox_ros_driver2::CustomMsg>(*msg);out->points.clear();out->points.reserve(msg->points.size());
    std::vector<Eigen::Vector3d> rejected;
    for(const auto& p:msg->points) {
      auto base=geometry_.toBase({p.x,p.y,p.z});
      if (geometry_.reject(base))rejected.push_back(base);else out->points.push_back(p);
    }
    input_count_=msg->points.size();rejected_count_=rejected.size();
    sensor_msgs::PointCloud2 cloud;cloud.header=msg->header;cloud.header.frame_id="base_link";
    sensor_msgs::PointCloud2Modifier modifier(cloud);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(rejected.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
    for(const auto& p:rejected) {*x=p.x();*y=p.y();*z=p.z();++x;++y;++z;}
    removed_.publish(cloud);
    if (!enabled_)return msg;
    if (out->points.size()<1000 || rejected.size()>msg->points.size()*maximum_fraction_) {
      fault("mask_exceeds_calibrated_fraction_or_too_few_points");return {};
    }
    out->point_num=out->points.size();return out;
  }
};
}
