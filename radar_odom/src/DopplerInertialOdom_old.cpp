#include "DopplerInertialOdom.h"

DopplerIO::DopplerIO(NodeHandle *nh)
{
    param::get("~doppler_resolution", doppler_resolution);
    param::get("~lag", lag);

    // radar init
    radar_pc.reset(new pcl::PointCloud<pcl::PointXYZIS>);

    // priors
    Pose3 prior_pose(Rot3::Quaternion(1, 0, 0, 0), Point3(0, 0, 0));
    Vector3 prior_velocity(0, 0, 0);
    imuBias::ConstantBias prior_imu_bias; // assume zero initial bias
    preintegrated = std::make_shared<PreintegratedCombinedMeasurements>(imu_param, prior_imu_bias);
    assert(preintegrated);

    // pose init
    initial_values.insert(X(0), prior_pose);
    initial_values.insert(V(0), prior_velocity);
    initial_values.insert(B(0), prior_imu_bias);
    prev_state = NavState(prior_pose, prior_velocity);
    prop_state = prev_state;
    prev_bias = prior_imu_bias;

    // pose prior
    pose_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);       // m/s
    bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

    // Add all prior
    graph = std::make_unique<NonlinearFactorGraph>();
    graph->addPrior(X(0), prior_pose, pose_noise_model);
    graph->addPrior(V(0), prior_velocity, velocity_noise_model);
    graph->addPrior(B(0), prior_imu_bias, bias_noise_model);

    // radar noise model

    // velocity_estimator = new RadarVelocityEstimator(
    //     noiseModel::Robust::Create(
    //         noiseModel::mEstimator::Cauchy::Create(0.1),
    //         noiseModel::Diagonal::Sigmas(Vector1(doppler_resolution))));

    // velocity_estimator = std::make_unique<RadarVelocityEstimatorCeres>();

    velocity_estimator = std::make_unique<RadarVelocityEstimatorCeresHCS>();

    // velocity_estimator = new RadarVelocityEstimatorGNC(
    //         noiseModel::Diagonal::Sigmas(Vector1(doppler_resolution)));

    // velocity_estimator = new RadarVelocityEstimatorCelestial(
    //     noiseModel::Robust::Create(
    //         noiseModel::mEstimator::Cauchy::Create(0.1),
    //         noiseModel::Diagonal::Sigmas(Vector1(doppler_resolution))),
    //     noiseModel::Diagonal::Sigmas(Vector2(2 * M_PI / 256.0, 2 * M_PI / 256.0)));

    // velocity_estimator = new RadarVelocityEstimatorHCS(
    //     noiseModel::Robust::Create(
    //         noiseModel::mEstimator::Cauchy::Create(0.1),
    //         noiseModel::Diagonal::Sigmas(Vector1(doppler_resolution))));

    velocity_estimator->set_inlier_thresh(doppler_resolution);
    // velocity_estimator->set_prior_noise_model(noiseModel::Diagonal::Sigmas(Vector(0.2, 0.2, 0.2)));

    // optimizer init
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.0;
    parameters.relinearizeSkip = 1;
    smootherISAM2 = std::make_unique<IncrementalFixedLagSmoother>(lag, parameters);

    // imu tf init
    while (nh->ok())
    {
        try
        {
            tf::StampedTransform tf_ir;
            listener.lookupTransform("epson", "radar", Time(0), tf_ir);
            tf::Quaternion q = tf_ir.getRotation();
            tf_imu2radar = Rot3::Quaternion(q.w(), q.x(), q.y(), q.z());
            break;
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s", "Fail to catch tf imu to radar. Next attempt in 0.5s.");
            Duration(0.5).sleep();
        }
    }

    // ros init
    pub_v = nh->advertise<visualization_msgs::Marker>("velocity", 1);
    pub_twist = nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("radar/velocity", 1);
    pub_bias = nh->advertise<geometry_msgs::Twist>("radar/bias", 1);
    pub_error = nh->advertise<geometry_msgs::Twist>("radar/error", 1);
    pub_odom = nh->advertise<nav_msgs::Odometry>("dopplerIO", 1);
    pub_radar = nh->advertise<sensor_msgs::PointCloud2>("radar_inlier", 1);
    sub_radar = nh->subscribe(radar_topic, 1, &DopplerIO::cb_radar, this);
    sub_imu = nh->subscribe(imu_topic, 200, &DopplerIO::cb_imu, this);

    // tmp
    // err_file.open("tmp.txt");
}

void DopplerIO::cb_radar(const sensor_msgs::PointCloud2 pc_msg)
{
    if (imu_count <= 5)
    {
        return;
    }
    imu_count = 0;
    step++;

    pcl::fromROSMsg(pc_msg, *radar_pc);
    double current_t = pc_msg.header.stamp.toSec();

    // combined IMU factor ---------------------------------------------------------------------------
    auto preint_imu_combined = dynamic_cast<const PreintegratedCombinedMeasurements &>(*preintegrated);
    CombinedImuFactor imu_factor(
        X(step - 1), V(step - 1), X(step),
        V(step), B(step - 1), B(step), preint_imu_combined);
    graph->add(imu_factor);

    prop_state = preintegrated->predict(prev_state, prev_bias);
    initial_values.insert(X(step), prop_state.pose());
    initial_values.insert(V(step), prop_state.v());
    initial_values.insert(B(step), prev_bias);
    key_timestamps[X(step)] = current_t;
    key_timestamps[V(step)] = current_t;
    key_timestamps[B(step)] = current_t;

    // ------------------------------------------------------------------------------------

    velocity_estimator->set_initial_velocity(prop_state.bodyVelocity());
    Vector3 measured_v = velocity_estimator->optimize(radar_pc);
    pcl::PointCloud<pcl::PointXYZIS>::Ptr radar_inlier =
        velocity_estimator->get_inlier(radar_pc, measured_v);

    // second inlier estimate
    // velocity_estimator->set_initial_velocity(measured_v);
    // measured_v = velocity_estimator->optimize(radar_inlier);

    Matrix radar_velocity_cov = velocity_estimator->get_covariance(
        radar_pc, measured_v);

    // velocity pose measurement
    // auto robust_rv_noise = noiseModel::Gaussian::Covariance(radar_velocity_cov);
    auto robust_rv_noise = noiseModel::Robust::Create(
        noiseModel::mEstimator::Huber::Create(1.345),
        noiseModel::Gaussian::Covariance(radar_velocity_cov));

    graph->add(VelocityPoseMeasurementFactor(
        V(step), X(step), measured_v, robust_rv_noise));

    // optimize ----------------------------------------------------------------------------
    smootherISAM2->update(*graph, initial_values, key_timestamps);
    Values result = smootherISAM2->calculateEstimate();
    Matrix x_cov = smootherISAM2->marginalCovariance(X(step));

    graph->resize(0);
    initial_values.clear();
    key_timestamps.clear();

    //-----------------------------------------------------------------------------
    // Overwrite the beginning of the preintegration for the next step.
    prev_state = NavState(result.at<Pose3>(X(step)),
                          result.at<Vector3>(V(step)));
    prev_bias = result.at<imuBias::ConstantBias>(B(step));
    preintegrated->resetIntegrationAndSetBias(prev_bias);

    // reset & publish -----------------------------------------------------------------------------
    radar_pc->points.clear();

    nav_msgs::Odometry odom = toOdometry(prev_state.pose().matrix(), x_cov);
    odom.header.stamp = pc_msg.header.stamp;
    odom.header.seq = step;
    odom.header.frame_id = "map";
    pub_odom.publish(odom);

    sensor_msgs::PointCloud2 inlier_msg;
    pcl::toROSMsg(*radar_inlier, inlier_msg);
    inlier_msg.header = pc_msg.header;
    inlier_msg.header.seq = step;
    geometry_msgs::TwistWithCovarianceStamped radar_v =
        toROSTwist(measured_v, radar_velocity_cov, pc_msg.header);
    radar_v.header.seq = step;

    geometry_msgs::Twist v_err;
    gtsam::Vector3 verr = prev_state.pose().rotation().unrotate(prev_state.velocity()) - measured_v;
    v_err.linear.x = verr(0);
    v_err.linear.y = verr(1);
    v_err.linear.z = verr(2);
    pub_error.publish(v_err);

    pub_radar.publish(inlier_msg);
    pub_twist.publish(radar_v);
    pub_bias.publish(toROStwist(prev_bias));
    pub_v.publish(velocityMarker(prev_state.velocity(),
                                 prev_state.pose().translation(), "map", "velocity"));
}

void DopplerIO::cb_imu(const sensor_msgs::Imu imu_msg)
{
    
    // imu preintegration
    double imu_t = imu_msg.header.stamp.toSec();
    double dt = imu_t - pre_imu_t;
    if (dt > 100)
    {
        pre_imu_t = imu_t;
        key_timestamps[X(0)] = imu_t;
        key_timestamps[V(0)] = imu_t;
        key_timestamps[B(0)] = imu_t;
        ROS_INFO("doppler inertial odometry init %.4f", imu_t);
        return;
    }

    Vector6 imu = (Vector6() << imu_msg.linear_acceleration.x,
                   imu_msg.linear_acceleration.y,
                   imu_msg.linear_acceleration.z,
                   imu_msg.angular_velocity.x,
                   imu_msg.angular_velocity.y,
                   imu_msg.angular_velocity.z)
                      .finished();

    auto acc = tf_imu2radar * imu.head<3>();
    auto omega = tf_imu2radar * imu.tail<3>();
    preintegrated->integrateMeasurement(acc, omega, dt);

    pre_imu_t = imu_t;
    imu_count++;

    // broadcast odometry transformation
    // prop_state = preintegrated->predict(prev_state, prev_bias);
    // tf::Transform transform;
    // auto trans = prop_state.pose().translation();
    // auto orien = prop_state.pose().rotation().toQuaternion();
    // transform.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));
    // transform.setRotation(tf::Quaternion(orien.x(), orien.y(), orien.z(), orien.w()));
    // br.sendTransform(tf::StampedTransform(transform, imu_msg.header.stamp, "map", "base"));
}

DopplerIO::~DopplerIO()
{
}

int main(int argc, char *argv[])
{
    init(argc, argv, "doppler_inertial_odom");
    NodeHandle nh;
    DopplerIO dio(&nh);
    spin();

    // tmp
    // dio.err_file.close();

    return 0;
}