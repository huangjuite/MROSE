#include "RadarInertialOdom.h"

RadarIO::RadarIO(NodeHandle *nh)
{
    // radar init
    radar_pc.reset(new pcl::PointCloud<pcl::PointXYZIS>);
    inlier_pc.reset(new pcl::PointCloud<pcl::PointXYZIS>);
    pre_pc.reset(new pcl::PointCloud<pcl::PointXYZIS>);
    ndt_omp.setResolution(1.5);
    ndt_omp.setNumThreads(8);
    ndt_omp.setNeighborhoodSearchMethod(pclomp::DIRECT7);

    // imu init
    tf_imu2radar = Rot3::Quaternion(-0.006, 1.000, 0.005, 0.016);
    Rot3 prior_rotation = Rot3::Quaternion(1, 0, 0, 0); // ::quaternion(w,x,y,z);
    Point3 prior_point(0, 0, 0);
    Pose3 prior_pose(prior_rotation, prior_point);
    Vector3 prior_velocity(0, 0, 0);
    imuBias::ConstantBias prior_imu_bias; // assume zero initial bias
    // preintegrated = std::make_shared<PreintegratedImuMeasurements>(imu_param, prior_imu_bias);
    preintegrated = std::make_shared<PreintegratedCombinedMeasurements>(imu_param, prior_imu_bias);
    assert(preintegrated);

    // pose init
    initial_values.insert(X(0), prior_pose);
    initial_values.insert(V(0), prior_velocity);
    initial_values.insert(B(0), prior_imu_bias);
    prev_state = NavState(prior_pose, prior_velocity);
    prop_state = prev_state;
    prev_bias = prior_imu_bias;

    // pose noise models
    pose_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
    velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);    // m/s
    bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);
    altitude_noise_model = noiseModel::Diagonal::Sigmas(Vector1(1));

    // radar noise model
    radar_doppler_noise = noiseModel::Robust::Create(
        noiseModel::mEstimator::Cauchy::Create(0.7),
        noiseModel::Diagonal::Sigmas(Vector1(0.1)));
    radar_velocity_noise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.2));
    radar_pose_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.02, 0.02, 0.02, 0.8, 0.8, 0.8).finished()); // rad,rad,rad,m, m, m

    // Add all prior factors (pose, velocity, bias) to the graph.
    graph = new NonlinearFactorGraph();
    graph->addPrior(X(0), prior_pose, pose_noise_model);
    graph->addPrior(V(0), prior_velocity, velocity_noise_model);
    graph->addPrior(B(0), prior_imu_bias, bias_noise_model);

    // optimizer init
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.0;
    parameters.relinearizeSkip = 1;
    isam2 = new ISAM2(parameters);

    double lag = 10.0;
    smootherISAM2 = new IncrementalFixedLagSmoother(lag, parameters);

    // ros init
    pub_v = nh->advertise<visualization_msgs::Marker>("velocity", 1);
    pub_odom = nh->advertise<nav_msgs::Odometry>("dopplerIO", 1);
    sub_radar = nh->subscribe(radar_topic, 1, &RadarIO::cb_radar, this);
    sub_imu = nh->subscribe(imu_topic, 200, &RadarIO::cb_imu, this);
}

void RadarIO::cb_radar(const sensor_msgs::PointCloud2 pc_msg)
{
    if (imu_count <= 10)
    {
        return;
    }
    imu_count = 0;
    pcl::fromROSMsg(pc_msg, *radar_pc);

    // IMU pose-graph ---------------------------------------------------------------------------
    correction_count++;

    // imu integration node
    auto preint_imu_combined =
        dynamic_cast<const PreintegratedCombinedMeasurements &>(*preintegrated);
    CombinedImuFactor imu_factor(X(correction_count - 1), V(correction_count - 1), X(correction_count),
                                 V(correction_count), B(correction_count - 1), B(correction_count),
                                 preint_imu_combined);
    graph->add(imu_factor);

    prop_state = preintegrated->predict(prev_state, prev_bias);
    initial_values.insert(X(correction_count), prop_state.pose());
    initial_values.insert(V(correction_count), prop_state.v());
    initial_values.insert(B(correction_count), prev_bias);

    // Altitude constraint
    graph->add(AltitudeFactor(X(correction_count), 0, altitude_noise_model));

    // ------------------------------------------------------------------------------------
    // local radar velocity
    Matrix radar_velocity_covariance;
    Vector3 measured_v = BodyFrameVelocityEstimation(
        radar_pc, radar_doppler_noise, radar_velocity_covariance, inlier_pc, 0.075);

    // 1. velocity measurement
    // measured_v = prop_state.pose().rotation() * measured_v;
    // graph->add(VelocityMeasurementFactor(V(correction_count), measured_v, radar_velocity_noise));

    // 2. velocity pose measurement
    graph->add(VelocityPoseMeasurementFactor(V(correction_count), X(correction_count), measured_v, radar_velocity_noise));

    // scan match---------------------------------------------------------------------------

    if (!pre_pc->empty())
    {
        auto t1 = ros::WallTime::now();
        ndt_omp.setInputSource(inlier_pc);
        ndt_omp.setInputTarget(pre_pc);
        pcl::PointCloud<pcl::PointXYZIS> result_pc;
        ndt_omp.align(result_pc);
        Eigen::Matrix4f tf = ndt_omp.getFinalTransformation();
        auto t2 = ros::WallTime::now();
        cout << "time : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;

        graph->add(BetweenFactor<Pose3>(X(correction_count - 1), X(correction_count),
                                        Pose3(tf.cast<double>()), radar_pose_noise));
    }
    *pre_pc = *inlier_pc;

    // optimize ----------------------------------------------------------------------------
    Values result;

    // 1. Fix-lag smoother
    smootherISAM2->update(*graph, initial_values);
    result = smootherISAM2->calculateEstimate();
    graph->resize(0);
    initial_values.clear();

    // 2. isam2
    // isam2->update(*graph, initial_values);
    // result = isam2->calculateEstimate();
    // graph->resize(0);
    // initial_values.clear();

    // 3. LM full graph
    // LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
    // result = optimizer.optimize();?
    // printf("graph size: %ld", graph->size());

    //-----------------------------------------------------------------------------
    // Overwrite the beginning of the preintegration for the next step.
    prev_state = NavState(result.at<Pose3>(X(correction_count)),
                          result.at<Vector3>(V(correction_count)));
    prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));
    preintegrated->resetIntegrationAndSetBias(prev_bias);

    // reset & visualization -----------------------------------------------------------------------------
    radar_pc->points.clear();
    nav_msgs::Odometry odom = toOdometry(prev_state.pose().matrix());
    odom.header.stamp = pc_msg.header.stamp;
    odom.header.seq = correction_count;
    odom.header.frame_id = "map";
    pub_odom.publish(odom);

    pub_v.publish(velocityMarker(prev_state.velocity(),
                                 prev_state.pose().translation(), "map", "velocity"));
}

void RadarIO::cb_imu(const sensor_msgs::Imu imu_msg)
{

    // imu preintegration
    double imu_t = imu_msg.header.stamp.toSec();
    double dt;
    if (imu_t - pre_imu_t > 100)
    {
        pre_imu_t = imu_t;
        printf("init\n");
        return;
    }
    dt = imu_t - pre_imu_t;
    Vector6 imu;
    imu(0) = imu_msg.linear_acceleration.x;
    imu(1) = imu_msg.linear_acceleration.y;
    imu(2) = imu_msg.linear_acceleration.z;
    imu(3) = imu_msg.angular_velocity.x;
    imu(4) = imu_msg.angular_velocity.y;
    imu(5) = imu_msg.angular_velocity.z;
    // printf("%.6f\n", dt);

    auto acc = tf_imu2radar * imu.head<3>();
    auto omega = tf_imu2radar * imu.tail<3>();
    preintegrated->integrateMeasurement(acc, omega, dt);
    pre_imu_t = imu_t;
    imu_count++;

    // broadcast odometry transformation
    prop_state = preintegrated->predict(prev_state, prev_bias);
    tf::Transform transform;
    auto trans = prop_state.pose().translation();
    auto orien = prop_state.pose().rotation().toQuaternion();
    transform.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));
    tf::Quaternion q(orien.x(), orien.y(), orien.z(), orien.w());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, imu_msg.header.stamp, "map", "base"));
}

RadarIO::~RadarIO()
{
}

int main(int argc, char *argv[])
{
    init(argc, argv, "collector");
    NodeHandle nh;
    RadarIO rio(&nh);
    spin();
    return 0;
}