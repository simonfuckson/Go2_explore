//
// Created by bruce on 2022/3/29.
//

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>

#include "pclomp/ndt_omp.h"

using namespace std;

typedef pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> NDT;
typedef pcl::PointCloud<pcl::PointXYZI> Cloud;


class Config
{
public:
    string mapFrame = "map";
    // GO2 导航系统要求 NDT 定位发布：
    //
    // map -> odom
    //
    // 不使用上游示例工程的内部世界帧作为公共导航接口。
    string odomFrame = "odom";
    string baseFrame = "base_link";
    string cloudTopic = "/cloud_registered_base";
    string odomTopic = "/odom_nav";
    string localizationTopic = "/localization";
    string healthTopic = "/localization/ndt_ok";

    // map -> odom TF 向未来预发布的时间。
    // 用于避免 TEB / move_base 控制周期与 localization TF
    // 发布周期之间几十毫秒的相位差导致 future extrapolation。
    double tfPostdateSec = 0.10;

    struct
    {
        bool debug = false;
        int numThreads = 4;
        int maximumIterations = 20;
        float voxelLeafSize = 0.1;
        float resolution = 1.0;
        double transformationEpsilon = 0.01;
        double stepSize = 0.1;
        double threshShift = 0.25;
        double threshRot = M_PI / 36;
        double minScanRange = 1.0;
        double maxScanRange = 100;
        double maxFitnessScore = 1.0;
        double maxUpdateInterval = 1.0;
        double healthTimeout = 3.0;
        double maxCorrectionShift = 0.80;
        double maxCorrectionRotation = M_PI / 9;
    } ndt;

    explicit Config(ros::NodeHandle &nh) : _nh(nh)
    {
        // 必须读取 odom_frame。
        // 如果 launch 中没有设置，则默认使用 "odom"。
        _nh.param<string>("map_frame", mapFrame, string("map"));
        _nh.param<string>("odom_frame", odomFrame, string("odom"));
        _nh.param<string>("base_frame", baseFrame, string("base_link"));
        _nh.param<string>("cloud_topic", cloudTopic, string("/cloud_registered_base"));
        _nh.param<string>("odom_topic", odomTopic, string("/odom_nav"));
        _nh.param<string>("localization_topic", localizationTopic, string("/localization"));
        _nh.param<string>("health_topic", healthTopic, string("/localization/ndt_ok"));

        // map -> odom TF 向未来预发布时间。
        _nh.param("tf_postdate_sec", tfPostdateSec, 0.10);

        _nh.getParam("ndt/debug", ndt.debug);
        _nh.getParam("ndt/num_threads", ndt.numThreads);
        _nh.getParam("ndt/maximum_iterations", ndt.maximumIterations);
        _nh.getParam("ndt/voxel_leaf_size", ndt.voxelLeafSize);
        _nh.getParam("ndt/transformation_epsilon", ndt.transformationEpsilon);
        _nh.getParam("ndt/step_size", ndt.stepSize);
        _nh.getParam("ndt/resolution", ndt.resolution);
        _nh.getParam("ndt/thresh_shift", ndt.threshShift);
        _nh.getParam("ndt/thresh_rot", ndt.threshRot);
        _nh.getParam("ndt/min_scan_range", ndt.minScanRange);
        _nh.getParam("ndt/max_scan_range", ndt.maxScanRange);
        _nh.param("ndt/max_fitness_score", ndt.maxFitnessScore, 1.0);
        _nh.param("ndt/max_update_interval", ndt.maxUpdateInterval, 1.0);
        _nh.param("ndt/health_timeout", ndt.healthTimeout, 3.0);
        _nh.param("ndt/max_correction_shift", ndt.maxCorrectionShift, 0.80);
        _nh.param("ndt/max_correction_rotation", ndt.maxCorrectionRotation, M_PI / 9);

        ROS_INFO("fast_lio_localization config:");
        ROS_INFO("  odom_frame      = %s", odomFrame.c_str());
        ROS_INFO("  cloud_topic     = %s", cloudTopic.c_str());
        ROS_INFO("  odom_topic      = %s", odomTopic.c_str());
        ROS_INFO("  tf_postdate_sec = %.3f", tfPostdateSec);
    }

private:
    ros::NodeHandle &_nh;
};


class Localizer
{
public:
    explicit Localizer(ros::NodeHandle &nh) :
            _nh(nh),
            _cfg(nh),
            _mapPtr(new Cloud),
            _mapFilteredPtr(new Cloud)
    {
        _localizationPub = _nh.advertise<geometry_msgs::PoseStamped>(
                _cfg.localizationTopic, 10, true);
        _healthPub = _nh.advertise<std_msgs::Bool>(
                _cfg.healthTopic, 10, true);
        _scorePub = _nh.advertise<std_msgs::Float64>(
                "/localization/ndt_score", 10, true);
        _iterationsPub = _nh.advertise<std_msgs::Int32>(
                "/localization/ndt_iterations", 10, true);
        _translationJumpPub = _nh.advertise<std_msgs::Float64>(
                "/localization/translation_jump", 10, true);
        _rotationJumpPub = _nh.advertise<std_msgs::Float64>(
                "/localization/rotation_jump", 10, true);

        _mapSub = _nh.subscribe(
                "/map_cloud",
                10,
                &Localizer::mapCallback,
                this
        );

        _initPoseSub = _nh.subscribe(
                "/initialpose",
                10,
                &Localizer::initPoseWithNDTCallback,
                this
        );

        _pcSubPtr =
                new message_filters::Subscriber<sensor_msgs::PointCloud2>(
                        nh,
                        _cfg.cloudTopic,
                        1
                );

        _odomSubPtr =
                new message_filters::Subscriber<nav_msgs::Odometry>(
                        nh,
                        _cfg.odomTopic,
                        1
                );

        _syncPtr =
                new message_filters::Synchronizer<syncPolicy>(
                        syncPolicy(10),
                        *_pcSubPtr,
                        *_odomSubPtr
                );

        _syncPtr->registerCallback(
                boost::bind(
                        &Localizer::syncCallback,
                        this,
                        _1,
                        _2
                )
        );

        _voxelGridFilter.setLeafSize(
                _cfg.ndt.voxelLeafSize,
                _cfg.ndt.voxelLeafSize,
                _cfg.ndt.voxelLeafSize
        );

        _ndt.setNumThreads(_cfg.ndt.numThreads);
        _ndt.setTransformationEpsilon(_cfg.ndt.transformationEpsilon);
        _ndt.setStepSize(_cfg.ndt.stepSize);
        _ndt.setResolution(_cfg.ndt.resolution);
        _ndt.setMaximumIterations(_cfg.ndt.maximumIterations);

        _odomMap.setIdentity();
        _lastMatchOdom.setIdentity();
        _tfTimer = _nh.createTimer(
                ros::Duration(0.10),
                &Localizer::tfTimerCallback,
                this
        );
        publishHealth(false);
        ROS_WARN("Publishing provisional identity map->odom until an initial NDT pose is accepted");
    }

private:
    ros::NodeHandle &_nh;

    ros::Subscriber _mapSub;
    ros::Subscriber _initPoseSub;
    ros::Publisher _localizationPub;
    ros::Publisher _healthPub;
    ros::Publisher _scorePub;
    ros::Publisher _iterationsPub;
    ros::Publisher _translationJumpPub;
    ros::Publisher _rotationJumpPub;

    tf2_ros::TransformBroadcaster _br;
    ros::Timer _tfTimer;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *_pcSubPtr;
    message_filters::Subscriber<nav_msgs::Odometry> *_odomSubPtr;

    typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::PointCloud2,
            nav_msgs::Odometry
    > syncPolicy;

    message_filters::Synchronizer<syncPolicy> *_syncPtr;

    NDT _ndt;
    pcl::VoxelGrid<pcl::PointXYZI> _voxelGridFilter;

    Config _cfg;

    Cloud::Ptr _mapPtr;
    Cloud::Ptr _mapFilteredPtr;

    tf::Pose _baseOdom;
    tf::Pose _odomMap;
    tf::Pose _lastMatchOdom;

    sensor_msgs::PointCloud2::ConstPtr _pcPtr = nullptr;
    ros::Time _latestStamp;
    ros::WallTime _lastSuccessfulMatch;
    bool _mapReady = false;
    bool _initialized = false;
    bool _lastMatchGood = false;
    bool _haveLastMatchOdom = false;


    void tfTimerCallback(const ros::TimerEvent &)
    {
        // RViz needs the map frame before the operator can publish
        // /initialpose in that frame.  Before NDT initialization _odomMap is
        // identity and localization health remains false, so this transform
        // is only a visualization/bootstrap transform and cannot enable
        // motion.  After initialization the same broadcaster publishes the
        // accepted NDT map->odom transform, avoiding duplicate TF authorities.
        publishTF();
    }


    void mapCallback(
            const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        ROS_INFO("Get map");

        pcl::fromROSMsg<pcl::PointXYZI>(
                *msg,
                *_mapPtr
        );

        _ndt.setInputTarget(_mapPtr);
        _mapReady = !_mapPtr->empty();
        ROS_INFO("NDT map contains %zu points", _mapPtr->size());
    }


    void initPoseCallback(
            const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        auto &q = msg->pose.pose.orientation;
        auto &p = msg->pose.pose.position;

        tf::Pose baseMap(
                tf::Quaternion(
                        q.x,
                        q.y,
                        q.z,
                        q.w
                ),
                tf::Vector3(
                        p.x,
                        p.y,
                        p.z
                )
        );

        _odomMap =
                baseMap *
                _baseOdom.inverse();

        ROS_INFO("Initial pose set");
    }


    void initPoseWithNDTCallback(
            const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        if (msg->header.frame_id != _cfg.mapFrame)
        {
            ROS_ERROR("Initial pose must use frame '%s', got '%s'",
                      _cfg.mapFrame.c_str(), msg->header.frame_id.c_str());
            return;
        }

        if (!_mapReady)
        {
            ROS_WARN("Initial pose ignored: map is not ready");
            return;
        }

        if (_pcPtr == nullptr)
        {
            ROS_WARN("No point cloud");
            return;
        }

        ROS_INFO("Initial pose set");

        auto &q = msg->pose.pose.orientation;
        auto &p = msg->pose.pose.position;

        tf::Pose baseMap(
                tf::Quaternion(
                        q.x,
                        q.y,
                        q.z,
                        q.w
                ),
                tf::Vector3(
                        p.x,
                        p.y,
                        p.z
                )
        );

        if (match(_pcPtr, baseMap))
        {
            _initialized = true;
            publishLocalization();
            publishTF();
            publishHealth(true);
            ROS_INFO("Initial NDT pose accepted");
        }
        else
        {
            _initialized = false;
            publishHealth(false);
            ROS_ERROR("Initial NDT pose rejected; adjust /initialpose and retry");
        }
    }


    void syncCallback(
            const sensor_msgs::PointCloud2::ConstPtr &pcMsg,
            const nav_msgs::Odometry::ConstPtr &odomMsg)
    {
        _pcPtr = pcMsg;
        _latestStamp = pcMsg->header.stamp;

        tf::poseMsgToTF(
                odomMsg->pose.pose,
                _baseOdom
        );

        if (!_initialized)
        {
            publishHealth(false);
            return;
        }

        auto T =
                _lastMatchOdom.inverseTimes(
                        _baseOdom
                );

        const double shift =
                hypot(
                        T.getOrigin().x(),
                        T.getOrigin().y()
                );

        const double rotation =
                std::fabs(
                        tf::getYaw(
                                T.getRotation()
                        )
                );

        const double matchAge = _lastSuccessfulMatch.isZero()
                ? std::numeric_limits<double>::infinity()
                : (ros::WallTime::now() - _lastSuccessfulMatch).toSec();

        if (!_haveLastMatchOdom ||
            shift > _cfg.ndt.threshShift ||
            rotation > _cfg.ndt.threshRot ||
            matchAge > _cfg.ndt.maxUpdateInterval)
        {
            match(
                    pcMsg,
                    _odomMap * _baseOdom
            );
        }

        publishLocalization();
        publishTF();

        const double healthAge = _lastSuccessfulMatch.isZero()
                ? std::numeric_limits<double>::infinity()
                : (ros::WallTime::now() - _lastSuccessfulMatch).toSec();
        const bool healthOk = healthAge <= _cfg.ndt.healthTimeout;
        if (!_lastMatchGood && healthOk)
        {
            ROS_WARN_THROTTLE(
                    1.0,
                    "NDT match rejected; retaining the last accepted localization for %.2f s more",
                    std::max(0.0, _cfg.ndt.healthTimeout - healthAge)
            );
        }
        publishHealth(healthOk);
    }


    /**
     * Matching the point cloud with map to calculate `_odomMap`.
     *
     * @param pcPtr  The point cloud for matching.
     * @param baseMap The guess matrix.
     */
    bool match(
            const sensor_msgs::PointCloud2::ConstPtr &pcPtr,
            const tf::Transform &baseMap)
    {
        if (!_mapReady)
        {
            ROS_WARN_THROTTLE(1.0, "NDT match skipped: map is not ready");
            return false;
        }

        static chrono::steady_clock::time_point t0;
        static chrono::steady_clock::time_point t1;

        Cloud::Ptr tmpCloudPtr(
                new Cloud
        );

        pcl::fromROSMsg(
                *pcPtr,
                *tmpCloudPtr
        );

        Cloud::Ptr filteredCloudPtr(
                new Cloud
        );

        _voxelGridFilter.setInputCloud(
                tmpCloudPtr
        );

        _voxelGridFilter.filter(
                *filteredCloudPtr
        );

        Cloud::Ptr scanCloudPtr(
                new Cloud
        );

        for (const auto &p : *filteredCloudPtr)
        {
            const auto r =
                    hypot(
                            p.x,
                            p.y
                    );

            if (r > _cfg.ndt.minScanRange &&
                r < _cfg.ndt.maxScanRange)
            {
                scanCloudPtr->push_back(p);
            }
        }

        if (scanCloudPtr->size() < 100)
        {
            ROS_WARN_THROTTLE(1.0, "NDT match rejected: only %zu usable points",
                              scanCloudPtr->size());
            _lastMatchGood = false;
            return false;
        }

        _ndt.setInputSource(
                scanCloudPtr
        );

        Eigen::Affine3d baseMapMat;

        tf::poseTFToEigen(
                baseMap,
                baseMapMat
        );

        Cloud::Ptr outputCloudPtr(
                new Cloud
        );

        if (_cfg.ndt.debug)
        {
            t0 = chrono::steady_clock::now();
        }

        _ndt.align(
                *outputCloudPtr,
                baseMapMat.matrix().cast<float>()
        );

        if (_cfg.ndt.debug)
        {
            t1 = chrono::steady_clock::now();
        }

        auto tNDT =
                _ndt.getFinalTransformation();

        const double fitness = _ndt.getFitnessScore();
        const bool converged = _ndt.hasConverged();

        std_msgs::Float64 score;
        score.data = fitness;
        _scorePub.publish(score);
        std_msgs::Int32 iterations;
        iterations.data = _ndt.getFinalNumIteration();
        _iterationsPub.publish(iterations);

        if (!converged || !std::isfinite(fitness) ||
            fitness > _cfg.ndt.maxFitnessScore || !tNDT.allFinite())
        {
            _lastMatchGood = false;
            ROS_WARN("NDT rejected: converged=%s fitness=%.6f limit=%.6f",
                     converged ? "true" : "false", fitness,
                     _cfg.ndt.maxFitnessScore);
            return false;
        }

        tf::Transform baseMapNDT;

        tf::poseEigenToTF(
                Eigen::Affine3d(
                        tNDT.cast<double>()
                ),
                baseMapNDT
        );

        // 计算：
        //
        // T_map_odom =
        // T_map_base *
        // inverse(T_odom_base)
        //
        // 当前 GO2 系统中，
        // 这个 correction 最终作为：
        //
        // map -> odom
        //
        // 发布。
        const tf::Transform previousOdomMap = _odomMap;
        const tf::Transform candidateOdomMap =
                baseMapNDT * _baseOdom.inverse();

        const tf::Transform correctionJump =
                previousOdomMap.inverseTimes(candidateOdomMap);
        const double correctionShift = correctionJump.getOrigin().length();
        const double correctionRotation =
                std::fabs(tf::getYaw(correctionJump.getRotation()));

        if (_initialized &&
            (correctionShift > _cfg.ndt.maxCorrectionShift ||
             correctionRotation > _cfg.ndt.maxCorrectionRotation))
        {
            _lastMatchGood = false;
            ROS_WARN("NDT correction rejected: shift=%.3f m rotation=%.2f deg",
                     correctionShift, correctionRotation * 180.0 / M_PI);
            return false;
        }

        _odomMap = candidateOdomMap;
        std_msgs::Float64 translation_jump;
        translation_jump.data = correctionShift;
        _translationJumpPub.publish(translation_jump);
        std_msgs::Float64 rotation_jump;
        rotation_jump.data = correctionRotation;
        _rotationJumpPub.publish(rotation_jump);

        _lastMatchOdom = _baseOdom;
        _haveLastMatchOdom = true;
        _lastSuccessfulMatch = ros::WallTime::now();
        _lastMatchGood = true;

        if (_cfg.ndt.debug)
        {
            ROS_INFO(
                    "NDT: %ldms",
                    chrono::duration_cast<chrono::milliseconds>(
                            t1 - t0
                    ).count()
            );
        }

        ROS_INFO("NDT accepted: fitness=%.6f iterations=%d",
                 fitness, _ndt.getFinalNumIteration());
        return true;
    }


    void publishHealth(bool ok)
    {
        std_msgs::Bool msg;
        msg.data = ok;
        _healthPub.publish(msg);
    }


    void publishLocalization()
    {
        if (!_initialized)
        {
            return;
        }

        const tf::Transform baseMap = _odomMap * _baseOdom;
        geometry_msgs::PoseStamped localization;
        localization.header.stamp = _latestStamp.isZero()
                ? ros::Time::now() : _latestStamp;
        localization.header.frame_id = _cfg.mapFrame;
        tf::poseTFToMsg(baseMap, localization.pose);
        _localizationPub.publish(localization);
    }


    void publishTF()
    {
        geometry_msgs::TransformStamped tfMsg;

        // 关键修改：
        //
        // map -> odom 向未来预发布 0.25 秒，
        // 避免 TEB 查询当前时刻 TF 时，
        // 最新 map -> odom 尚落后几十毫秒而出现：
        //
        // Lookup would require extrapolation into the future
        //
        tfMsg.header.stamp =
                ros::Time::now() +
                ros::Duration(
                        _cfg.tfPostdateSec
                );

        tfMsg.header.frame_id = _cfg.mapFrame;

        // 必须是 odom。
        tfMsg.child_frame_id =
                _cfg.odomFrame;

        tfMsg.transform.translation.x =
                _odomMap.getOrigin().x();

        tfMsg.transform.translation.y =
                _odomMap.getOrigin().y();

        tfMsg.transform.translation.z =
                _odomMap.getOrigin().z();

        tfMsg.transform.rotation.x =
                _odomMap.getRotation().x();

        tfMsg.transform.rotation.y =
                _odomMap.getRotation().y();

        tfMsg.transform.rotation.z =
                _odomMap.getRotation().z();

        tfMsg.transform.rotation.w =
                _odomMap.getRotation().w();

        _br.sendTransform(
                tfMsg
        );
    }
};


int main(
        int argc,
        char **argv)
{
    ros::init(
            argc,
            argv,
            "fast_lio_localization"
    );

    ros::NodeHandle nh("~");

    Localizer localizer(
            nh
    );

    ros::spin();

    return 0;
}
