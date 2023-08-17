#include "DopplerInertialOdomMulti.h"

DopplerIOMulti::DopplerIOMulti(NodeHandle *nh)
{
    // radar init
    radar_pc.reset(new pcl::PointCloud<pcl::PointXYZSD>);

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

    // pose noise models
    pose_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);       // m/s
    bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);
    // altitude_noise_model = noiseModel::Diagonal::Sigmas(Vector1(1));
    // rotation_noise_model = noiseModel::Diagonal::Sigmas(Vector2(M_PI / 180.0, M_PI / 180.0));

    // radar noise model
    velocity_estimator = new RadarVelocityEstimator(
        noiseModel::Robust::Create(
            noiseModel::mEstimator::Cauchy::Create(0.7),
            noiseModel::Diagonal::Sigmas(Vector1(0.147))));
    velocity_estimator->set_inlier_thresh(0.1);
    velocity_estimator->set_prior_noise_model(noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.2)));
    radar_velocity_noise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

    // Add all prior factors (pose, velocity, bias) to the graph.
    graph = new NonlinearFactorGraph();
    graph->addPrior(X(0), prior_pose, pose_noise_model);
    graph->addPrior(V(0), prior_velocity, velocity_noise_model);
    graph->addPrior(B(0), prior_imu_bias, bias_noise_model);

    // optimizer init
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.0;
    parameters.relinearizeSkip = 1;
    double lag = 10.0;
    smootherISAM2 = new IncrementalFixedLagSmoother(lag, parameters);

    // imu tf init
    while (nh->ok())
    {
        try
        {

            listener.lookupTransform("base_link", "radar_center", Time(0), tf_c);
            listener.lookupTransform("base_link", "radar_left", Time(0), tf_l);
            listener.lookupTransform("base_link", "radar_right", Time(0), tf_r);
            break;
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s", "Fail to catch tf radar. Next attempt in 0.5s.");
            Duration(0.5).sleep();
        }
    }

    e_c = toMatrix(tf_c);
    e_l = toMatrix(tf_l);
    e_r = toMatrix(tf_r);

    // ros init
    pub_v = nh->advertise<visualization_msgs::Marker>("velocity", 1);
    pub_odom = nh->advertise<nav_msgs::Odometry>("DopplerIO", 1);
    pub_radar = nh->advertise<sensor_msgs::PointCloud2>("radar_inlier", 1);

    sub_radar_center.subscribe(*nh, "/sensor_platform/radar_center/scan", 1);
    sub_radar_left.subscribe(*nh, "/sensor_platform/radar_left/scan", 1);
    sub_radar_right.subscribe(*nh, "/sensor_platform/radar_right/scan", 1);
    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), sub_radar_center, sub_radar_left, sub_radar_right);
    sync->registerCallback(boost::bind(&DopplerIOMulti::cb_radar, this, _1, _2, _3));

    sub_imu = nh->subscribe(imu_topic, 200, &DopplerIOMulti::cb_imu, this);

    // tmp
    // err_file.open("tmp.txt");
}

void DopplerIOMulti::cb_radar(const sensor_msgs::PointCloud2ConstPtr pc_msg_center,
                              const sensor_msgs::PointCloud2ConstPtr pc_msg_left,
                              const sensor_msgs::PointCloud2ConstPtr pc_msg_right)
{
    if (imu_count <= 10)
    {
        return;
    }
    imu_count = 0;
    step++;
    double current_t = pc_msg_center->header.stamp.toSec();

    pcl::PointCloud<pcl::PointXYZSNV> radar_center;
    pcl::PointCloud<pcl::PointXYZSNV> radar_left;
    pcl::PointCloud<pcl::PointXYZSNV> radar_right;

    pcl::fromROSMsg(*pc_msg_center, radar_center);
    pcl::fromROSMsg(*pc_msg_left, radar_left);
    pcl::fromROSMsg(*pc_msg_right, radar_right);

    pcl::transformPointCloud(radar_center, radar_center, e_c);
    pcl::transformPointCloud(radar_left, radar_left, e_l);
    pcl::transformPointCloud(radar_right, radar_right, e_r);

    *radar_pc = from_xrio_type(radar_center) + from_xrio_type(radar_left) + from_xrio_type(radar_right);

    // cout << pc_msg_center->header.stamp << endl
    //      << pc_msg_left->header.stamp << endl
    //      << pc_msg_right->header.stamp << endl;
    // sensor_msgs::PointCloud2 pc_msg;
    // pcl::toROSMsg(*radar_pc, pc_msg);
    // pc_msg.header = pc_msg_center->header;
    // pc_msg.header.frame_id = "base_link";
    // pub_radar.publish(pc_msg);
    // return;

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

    // Altitude constraint
    // graph->add(AltitudeFactor(X(step), 0, altitude_noise_model));
    // graph->add(RotationFactor(X(step), 0, 0, rotation_noise_model));

    // ------------------------------------------------------------------------------------
    // local radar velocity
    velocity_estimator->set_initial_velocity(prop_state.bodyVelocity());
    Vector3 measured_v = velocity_estimator->optimize(radar_pc);
    pcl::PointCloud<pcl::PointXYZSD>::Ptr radar_inlier =
        velocity_estimator->get_inlier(radar_pc, measured_v);
    Matrix radar_velocity_cov = velocity_estimator->get_covariance(
        radar_inlier, measured_v);
    // cout << "======================" << endl;
    // cout << radar_velocity_cov << endl;

    auto rv_noise = radar_velocity_noise;
    // auto rv_noise = noiseModel::Gaussian::Covariance(radar_velocity_cov);
    // auto sigma = radar_velocity_cov.diagonal().array().sqrt();
    // auto rv_noise = noiseModel::Diagonal::Sigmas(Vector3(sigma));

    // velocity pose measurement
    graph->add(VelocityPoseMeasurementFactor(V(step), X(step), measured_v, rv_noise));

    // optimize ----------------------------------------------------------------------------
    Values result;
    smootherISAM2->update(*graph, initial_values);
    result = smootherISAM2->calculateEstimate();
    Matrix x_cov = smootherISAM2->marginalCovariance(X(step));
    // Matrix v_cov = smootherISAM2->marginalCovariance(V(step));

    // cout << "================" << endl;
    // cout << v_cov << endl;

    graph->resize(0);
    initial_values.clear();
    key_timestamps.clear();

    //-----------------------------------------------------------------------------
    // Overwrite the beginning of the preintegration for the next step.
    prev_state = NavState(result.at<Pose3>(X(step)),
                          result.at<Vector3>(V(step)));
    prev_bias = result.at<imuBias::ConstantBias>(B(step));
    preintegrated->resetIntegrationAndSetBias(prev_bias);

    // auto err = prev_state.bodyVelocity() - measured_v;
    // err_file << err[0] << ", " << err[1] << ", " << err[2] << endl;

    // reset & publish -----------------------------------------------------------------------------
    radar_pc->points.clear();

    nav_msgs::Odometry odom = toOdometry(prev_state.pose().matrix(), x_cov);
    odom.header.stamp = pc_msg_center->header.stamp;
    odom.header.seq = step;
    odom.header.frame_id = "map";
    pub_odom.publish(odom);

    sensor_msgs::PointCloud2 inlier_msg;
    pcl::toROSMsg(*radar_inlier, inlier_msg);
    inlier_msg.header = pc_msg_center->header;
    inlier_msg.header.frame_id = "base_link";
    inlier_msg.header.seq = step;
    pub_radar.publish(inlier_msg);

    pub_v.publish(velocityMarker(prev_state.velocity(),
                                 prev_state.pose().translation(), "map", "velocity"));
}

void DopplerIOMulti::cb_imu(const sensor_msgs::Imu imu_msg)
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
    prop_state = preintegrated->predict(prev_state, prev_bias);
    tf::Transform transform;
    auto trans = prop_state.pose().translation();
    auto orien = prop_state.pose().rotation().toQuaternion();
    transform.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));
    transform.setRotation(tf::Quaternion(orien.x(), orien.y(), orien.z(), orien.w()));
    br.sendTransform(tf::StampedTransform(transform, imu_msg.header.stamp, "map", "base_link"));
}

DopplerIOMulti::~DopplerIOMulti()
{
}

int main(int argc, char *argv[])
{
    init(argc, argv, "doppler_inertial_odom");
    NodeHandle nh;
    DopplerIOMulti dio(&nh);
    spin();

    // tmp
    // dio.err_file.close();

    return 0;
}