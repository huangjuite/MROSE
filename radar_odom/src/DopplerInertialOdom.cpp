#include "DopplerInertialOdom.h"

DopplerIO::DopplerIO(ros::NodeHandle *nh)
{
    params = rplrio::get_params(*nh);
    // radar noise model

    // velocity_estimator = new RadarVelocityEstimator(
    //     noiseModel::Robust::Create(
    //         noiseModel::mEstimator::Cauchy::Create(0.1),
    //         noiseModel::Diagonal::Sigmas(Vector1(doppler_resolution))));

    // velocity_estimator = std::make_unique<RadarVelocityEstimatorCeres>();

    velocity_estimator = std::make_unique<RadarVelocityEstimatorCeresHCS>(params);

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

    velocity_estimator->set_inlier_thresh(params.doppler_std);

    // optimizer init
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.0;
    parameters.relinearizeSkip = 1;
    smootherISAM2 = std::make_unique<gtsam::IncrementalFixedLagSmoother>(params.lag, parameters);

    // imu init
    initializer = std::make_unique<StaticInitializer>(params.init_window, params.init_thres);

    // ros init
    pub_v = nh->advertise<visualization_msgs::Marker>("velocity", 1);
    pub_twist = nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("radar/velocity", 1);
    pub_bias = nh->advertise<geometry_msgs::Twist>("radar/bias", 1);
    pub_error = nh->advertise<geometry_msgs::Twist>("radar/error", 1);
    pub_odom = nh->advertise<nav_msgs::Odometry>("dopplerIO", 1);
    pub_radar = nh->advertise<sensor_msgs::PointCloud2>("radar_inlier", 1);
    sub_radar = nh->subscribe(params.radar_topic, 1, &DopplerIO::cb_radar, this);
    sub_imu = nh->subscribe(params.imu_topic, 1000, &DopplerIO::cb_imu, this);
}

void DopplerIO::graph_init(const Eigen::Quaterniond &rotation, const Eigen::Vector3d &bias_acc,
                           const Eigen::Vector3d &bias_gyro, const double &stamp)
{

    // priors
    gtsam::Pose3 prior_pose(gtsam::Rot3::Quaternion(rotation.w(), rotation.x(), rotation.y(), rotation.z()), gtsam::Point3(0, 0, 0));
    gtsam::Vector3 prior_velocity(0, 0, 0);
    gtsam::imuBias::ConstantBias prior_imu_bias(bias_acc, bias_gyro);
    // imuBias::ConstantBias prior_imu_bias;
    preintegrated = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(rplrio::imuParams(params), prior_imu_bias);
    assert(preintegrated);

    // pose init
    initial_values.insert(X(0), prior_pose);
    initial_values.insert(V(0), prior_velocity);
    initial_values.insert(B(0), prior_imu_bias);
    key_timestamps[X(0)] = stamp;
    key_timestamps[V(0)] = stamp;
    key_timestamps[B(0)] = stamp;

    prev_state = gtsam::NavState(prior_pose, prior_velocity);
    prop_state = prev_state;
    prev_bias = prior_imu_bias;

    // prior noise model
    pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 1e-2);      // m/s
    bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

    // Add all prior
    graph = std::make_unique<gtsam::NonlinearFactorGraph>();
    graph->addPrior(X(0), prior_pose, pose_noise_model);
    graph->addPrior(V(0), prior_velocity, velocity_noise_model);
    graph->addPrior(B(0), prior_imu_bias, bias_noise_model);
}

void DopplerIO::update()
{
    // remove out of date radar measurements
    while (!radar_que.empty() && radar_que.front()->header.stamp.toSec() < imu_que.front()->stamp)
    {
        radar_que.pop_front();
    }
    if (radar_que.empty())
    {
        return;
    }

    auto radar_msg = radar_que.front();
    double radar_stamp = radar_msg->header.stamp.toSec();
    radar_que.pop_front();

    // integrate imu measurements
    while (!imu_que.empty() && radar_stamp > imu_que.front()->stamp)
    {
        auto imu_data = imu_que.front();
        double dt = imu_data->stamp - pre_imu_t;
        preintegrated->integrateMeasurement(imu_data->acc, imu_data->gyro, dt);
        pre_imu_t = imu_data->stamp;

        imu_que.pop_front();
    }

    step++;
    optimize(radar_msg);

    // recurisvely update until latest radar data is processed
    update();
}

void DopplerIO::optimize(const sensor_msgs::PointCloud2::ConstPtr radar_msg)
{
    pcl::PointCloud<pcl::PointXYZIS>::Ptr radar_pc;
    radar_pc.reset(new pcl::PointCloud<pcl::PointXYZIS>);

    pcl::fromROSMsg(*radar_msg, *radar_pc);
    double current_t = radar_msg->header.stamp.toSec();

    // combined IMU factor ---------------------------------------------------------------------------
    auto preint_imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements &>(*preintegrated);
    gtsam::CombinedImuFactor imu_factor(
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
    gtsam::Vector3 measured_v = velocity_estimator->optimize(radar_pc);
    pcl::PointCloud<pcl::PointXYZIS>::Ptr radar_inlier =
        velocity_estimator->get_inlier(radar_pc, measured_v);

    // second inlier estimate
    // velocity_estimator->set_initial_velocity(measured_v);
    // measured_v = velocity_estimator->optimize(radar_inlier);

    gtsam::Matrix radar_velocity_cov = velocity_estimator->get_covariance(
        radar_pc, measured_v);

    // velocity pose measurement
    measured_v = params.R_r_b * measured_v;
    radar_velocity_cov = params.R_r_b * radar_velocity_cov * params.R_r_b.transpose();

    // auto robust_rv_noise = noiseModel::Gaussian::Covariance(radar_velocity_cov);
    auto robust_rv_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345),
        gtsam::noiseModel::Gaussian::Covariance(radar_velocity_cov));

    graph->add(VelocityPoseMeasurementFactor(
        V(step), X(step), measured_v, robust_rv_noise));

    // optimize ----------------------------------------------------------------------------
    smootherISAM2->update(*graph, initial_values, key_timestamps);
    gtsam::Values result = smootherISAM2->calculateEstimate();
    gtsam::Matrix x_cov = smootherISAM2->marginalCovariance(X(step));

    graph->resize(0);
    initial_values.clear();
    key_timestamps.clear();

    //-----------------------------------------------------------------------------
    // Overwrite the beginning of the preintegration for the next step.
    prev_state = gtsam::NavState(result.at<gtsam::Pose3>(X(step)),
                          result.at<gtsam::Vector3>(V(step)));
    prev_bias = result.at<gtsam::imuBias::ConstantBias>(B(step));
    preintegrated->resetIntegrationAndSetBias(prev_bias);

    // reset & publish -----------------------------------------------------------------------------
    radar_pc->clear();

    nav_msgs::Odometry odom = toOdometry(prev_state.pose().matrix(), x_cov);
    odom.header.stamp = radar_msg->header.stamp;
    odom.header.seq = step;
    odom.header.frame_id = "map";
    pub_odom.publish(odom);

    sensor_msgs::PointCloud2 inlier_msg;
    pcl::toROSMsg(*radar_inlier, inlier_msg);
    inlier_msg.header.stamp = radar_msg->header.stamp;
    inlier_msg.header.frame_id = "base";
    inlier_msg.header.seq = step;
    geometry_msgs::TwistWithCovarianceStamped radar_v =
        toROSTwist(measured_v, radar_velocity_cov, radar_msg->header);
    radar_v.header.seq = step;
    radar_v.header.frame_id = "base";

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

    // broadcast odometry transformation
    // tf::Transform transform;
    // auto trans = prev_state.pose().translation();
    // auto orien = prev_state.pose().rotation().toQuaternion();
    // transform.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));
    // transform.setRotation(tf::Quaternion(orien.x(), orien.y(), orien.z(), orien.w()));
    // br.sendTransform(tf::StampedTransform(transform, radar_msg->header.stamp, "map", "base"));
}

void DopplerIO::cb_imu(const sensor_msgs::Imu::ConstPtr &imu_msg)
{

    double imu_t = imu_msg->header.stamp.toSec();

    gtsam::Vector3 acc = params.R_i_b * (gtsam::Vector3() << imu_msg->linear_acceleration.x,
                                         imu_msg->linear_acceleration.y,
                                         imu_msg->linear_acceleration.z)
                                            .finished();
    gtsam::Vector3 omega = params.R_i_b * (gtsam::Vector3() << imu_msg->angular_velocity.x,
                                           imu_msg->angular_velocity.y,
                                           imu_msg->angular_velocity.z)
                                              .finished();
    if (!initialized)
    {
        bool init_success = initializer->init_update(
            std::make_shared<rio::IMU>(acc, omega, imu_t));
        if (init_success)
        {
            initialized = true;
            Eigen::Vector3d bias_a = initializer->get_bias_acc();
            Eigen::Vector3d bias_g = initializer->get_bias_gyro();
            Eigen::Quaterniond init_r = initializer->get_rotation();

            // Eigen::Vector3d bias_a = Eigen::Vector3d::Zero();
            // Eigen::Vector3d bias_g = Eigen::Vector3d::Zero();
            // Eigen::Quaterniond init_r(1, 0, 0, 0);

            pre_imu_t = initializer->get_first_stamp();
            graph_init(init_r, bias_a, bias_g, pre_imu_t);
            imu_que = initializer->get_imu_queue();
            ROS_INFO("static init success");
            ROS_INFO("bias acc: %.4f, %.4f, %.4f", bias_a[0], bias_a[1], bias_a[2]);
            ROS_INFO("bias gyro: %.4f, %.4f, %.4f", bias_g[0], bias_g[1], bias_g[2]);

            update();
        }

        return;
    }
    imu_que.emplace_back(std::make_shared<rio::IMU>(acc, omega, imu_t));
}

void DopplerIO::cb_radar(const sensor_msgs::PointCloud2::ConstPtr &pc_msg)
{
    radar_que.emplace_back(pc_msg);

    if (!initialized)
    {
        return;
    }

    update();

    return;
}

DopplerIO::~DopplerIO()
{
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "doppler_inertial_odom");
    ros::NodeHandle nh;
    DopplerIO dio(&nh);
    ros::spin();

    return 0;
}