#include "ExtrinsicEstimation.h"

DopplerIOex::DopplerIOex(NodeHandle *nh)
{
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
    cache_results.insert(X(0), prior_pose);
    cache_results.insert(V(0), prior_velocity);
    cache_results.insert(B(0), prior_imu_bias);

    prev_state = NavState(prior_pose, prior_velocity);
    prop_state = prev_state;
    prev_bias = prior_imu_bias;

    // pose noise models
    pose_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);       // m/s
    bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

    // radar noise model
    velocity_estimator = new RadarVelocityEstimator(
        noiseModel::Robust::Create(
            noiseModel::mEstimator::Cauchy::Create(0.1),
            noiseModel::Diagonal::Sigmas(Vector1(0.05512325))));
    velocity_estimator->set_inlier_thresh(0.05512325 / 2);
    // velocity_estimator->set_prior_noise_model(noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.2)));
    // radar_velocity_noise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.2));

    // Add all prior factors (pose, velocity, bias) to the graph.
    graph = new NonlinearFactorGraph();
    graph->addPrior(X(0), prior_pose, pose_noise_model);
    graph->addPrior(V(0), prior_velocity, velocity_noise_model);
    graph->addPrior(B(0), prior_imu_bias, bias_noise_model);

    offline_graph.addPrior(X(0), prior_pose, pose_noise_model);
    offline_graph.addPrior(V(0), prior_velocity, velocity_noise_model);
    offline_graph.addPrior(B(0), prior_imu_bias, bias_noise_model);

    // optimizer init
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.0;
    parameters.relinearizeSkip = 1;
    double lag = 2.0;
    smootherISAM2 = new IncrementalFixedLagSmoother(lag, parameters);

    // imu tf init
    while (nh->ok())
    {
        try
        {
            tf::StampedTransform tf_ri;
            listener.lookupTransform("radar", "epson", Time(0), tf_ri);
            tf::Quaternion q = tf_ri.getRotation();
            tf_radar2imu = Rot3::Quaternion(q.w(), q.x(), q.y(), q.z());
            // tf_radar2imu = Rot3::Quaternion(0.0294, 0.9902, -0.0178, 0.1351);
            // tf_radar2imu = Rot3::Quaternion(0.0421, 0.9887, 0.0091, 0.1438);
            // tf_radar2imu = Rot3::Quaternion(0.0414, 0.9966, -0.0235, 0.0670);
            // tf_radar2imu = Rot3::Quaternion( 0.0790, 0.9934, -0.0418, 0.0716);
            ROS_INFO("rotation radar to imu (w:%.4f, x:%.4f, y:%.4f, z:%.4f)",
                     q.w(), q.x(), q.y(), q.z());
            break;
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s", "Fail to catch tf imu to radar. Next attempt in 0.5s.");
            Duration(0.5).sleep();
        }
    }

    // ros init
    calibrate_srv = nh->advertiseService("start_calibrate", &DopplerIOex::cb_calibrate, this);
    pub_path = nh->advertise<nav_msgs::Path>("optimized_odom", 1);
    pub_v = nh->advertise<visualization_msgs::Marker>("velocity", 1);
    pub_odom = nh->advertise<nav_msgs::Odometry>("dopplerIO", 1);
    pub_radar = nh->advertise<sensor_msgs::PointCloud2>("radar_inlier", 1);
    sub_radar = nh->subscribe(radar_topic, 1, &DopplerIOex::cb_radar, this);
    sub_imu = nh->subscribe(imu_topic, 200, &DopplerIOex::cb_imu, this);
}

void DopplerIOex::cb_radar(const sensor_msgs::PointCloud2 pc_msg)
{
    if (imu_count <= 10)
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
    offline_graph.add(imu_factor);

    prop_state = preintegrated->predict(prev_state, prev_bias);
    initial_values.insert(X(step), prop_state.pose());
    initial_values.insert(V(step), prop_state.v());
    initial_values.insert(B(step), prev_bias);
    key_timestamps[X(step)] = current_t;
    key_timestamps[V(step)] = current_t;
    key_timestamps[B(step)] = current_t;

    // ------------------------------------------------------------------------------------
    // local radar velocity

    velocity_estimator->set_initial_velocity(prop_state.bodyVelocity());
    Vector3 measured_v = velocity_estimator->optimize(radar_pc);
    pcl::PointCloud<pcl::PointXYZIS>::Ptr radar_inlier =
        velocity_estimator->get_inlier(radar_pc, measured_v);
    Matrix radar_velocity_cov = velocity_estimator->get_covariance(
        radar_inlier, measured_v);

    // auto rv_noise = radar_velocity_noise;
    auto rv_noise = noiseModel::Gaussian::Covariance(radar_velocity_cov);

    // velocity pose measurement
    Vector3 mv_imu = tf_radar2imu * measured_v;
    graph->add(VelocityPoseMeasurementFactor(V(step), X(step), mv_imu, rv_noise));

    radar_velocity_cache[step] = measured_v;
    cov_cache[step] = radar_velocity_cov;

    // optimize ----------------------------------------------------------------------------

    smootherISAM2->update(*graph, initial_values, key_time_stamps);
    result = smootherISAM2->calculateEstimate();
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

    cache_results.insert(X(step), result.at<Pose3>(X(step)));
    cache_results.insert(V(step), result.at<Vector3>(V(step)));
    cache_results.insert(B(step), result.at<imuBias::ConstantBias>(B(step)));

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
    pub_radar.publish(inlier_msg);

    pub_v.publish(velocityMarker(prev_state.velocity(),
                                 prev_state.pose().translation(), "map", "velocity"));
}

void DopplerIOex::cb_imu(const sensor_msgs::Imu imu_msg)
{

    // imu preintegration
    double imu_t = imu_msg.header.stamp.toSec();
    double dt;
    if (imu_t - pre_imu_t > 100)
    {
        pre_imu_t = imu_t;
        key_timestamps[X(0)] = imu_t;
        key_timestamps[V(0)] = imu_t;
        key_timestamps[B(0)] = imu_t;
        ROS_INFO("doppler inertial odometry init");
        return;
    }
    dt = imu_t - pre_imu_t;
    Vector6 imu = (Vector6() << imu_msg.linear_acceleration.x,
                   imu_msg.linear_acceleration.y,
                   imu_msg.linear_acceleration.z,
                   imu_msg.angular_velocity.x,
                   imu_msg.angular_velocity.y,
                   imu_msg.angular_velocity.z)
                      .finished();

    auto acc = imu.head<3>();
    auto omega = imu.tail<3>();
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
    // br.sendTransform(tf::StampedTransform(transform, imu_msg.header.stamp, "imu", "map"));
}

bool DopplerIOex::cb_calibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &rep)
{

    // extrinsic optimization
    altitude_noise_model = noiseModel::Diagonal::Sigmas(Vector1(0.05));
    auto extrinsic_noise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
    offline_graph.addPrior(E(0), tf_radar2imu, extrinsic_noise);
    cache_results.insert(E(0), tf_radar2imu);

    Values calibrate_result;

    for (uint i = 1; i <= step; i++)
    {

        auto rv_noise = noiseModel::Gaussian::Covariance(cov_cache[i]);
        // auto rv_noise = noiseModel::Diagonal::Sigmas(Vector3(0.05, 0.05, 0.05));
        offline_graph.add(VelocityPoseExtrinsicFactor(
            V(i), X(i), E(0), radar_velocity_cache[i], rv_noise));
        offline_graph.add(AltitudeFactor(X(i), 0, altitude_noise_model));
    }

    calibrate_result = LevenbergMarquardtOptimizer(offline_graph, cache_results).optimize();

    Rot3 extrinsic = calibrate_result.at<Rot3>(E(0));
    tf_radar2imu.print("before");
    extrinsic.print("after");
    auto q = extrinsic.toQuaternion();
    auto e = extrinsic.ypr();
    printf("tf radar to imu\n");
    printf("x,y,z,w: %.4f, %.4f, %.4f, %.4f \n", q.x(), q.y(), q.z(), q.w());
    printf("y,p,r: %.4f, %.4f, %.4f \n", e[0], e[1], e[2]);

    extrinsic = extrinsic.inverse();
    q = extrinsic.toQuaternion();
    e = extrinsic.ypr();
    printf("\ntf imu to radar\n");
    printf("x,y,z,w: %.4f, %.4f, %.4f, %.4f \n", q.x(), q.y(), q.z(), q.w());
    printf("y,p,r: %.4f, %.4f, %.4f \n", e[0], e[1], e[2]);

    nav_msgs::Path path;
    path.header.frame_id = "map";
    for (uint i = 0; i < step; i++)
    {
        path.poses.push_back(toROSPoseStamped(calibrate_result.at<Pose3>(X(i)), path.header));
    }
    pub_path.publish(path);

    rep.success = true;
    return true;
}

DopplerIOex::~DopplerIOex()
{
}

int main(int argc, char *argv[])
{
    init(argc, argv, "doppler_inertial_odom");
    NodeHandle nh;
    DopplerIOex dio(&nh);
    spin();

    return 0;
}