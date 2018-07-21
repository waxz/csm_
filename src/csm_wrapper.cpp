//
// Created by waxz on 18-3-25.
//

#include <csm_wrapper/csm_wrapper.h>
#include <csm/csm.h>
#include <icp/icp.h>
// CSM is used in the following way:
// The scans are always in the laser frame
// The reference scan (mapscan) has a pose of [0, 0, 0]
// The new scan (currLDPScan) has a pose equal to the movement
// of the laser in the laser frame since the last scan
// The computed correction is then propagated using the tf machinery




Csm_Wrapper::Csm_Wrapper(ros::NodeHandle nh, ros::NodeHandle nh_private) :
        nh_(nh), nh_private_(nh_private) {
//    ROS_INFO("starting Csm_Wrapper!!");

    init_params();



    // Initialize input
    input_.laser[0] = 0.0;
    input_.laser[1] = 0.0;
    input_.laser[2] = 0.0;

    // Initialize output_ vectors as Null for error-checking
    output_.cov_x_m = 0;
    output_.dx_dy1_m = 0;
    output_.dx_dy2_m = 0;
}


void Csm_Wrapper::init_params() {
//    ROS_INFO("init Csm_Wrapper params");
    // **** csm
    // Maximum angular displacement between scans
    if (!nh_private_.getParam("max_angular_correction_deg", input_.max_angular_correction_deg))
        input_.max_angular_correction_deg = 45.0;

    // Maximum translation between scans (m)
    if (!nh_private_.getParam("max_linear_correction", input_.max_linear_correction))
        input_.max_linear_correction = 0.50;

    // Maximum ICP cycle iterations
    if (!nh_private_.getParam("max_iterations", input_.max_iterations))
        input_.max_iterations = 10;

    // A threshold for stopping (m)
    if (!nh_private_.getParam("epsilon_xy", input_.epsilon_xy))
        input_.epsilon_xy = 0.000001;

    // A threshold for stopping (rad)
    if (!nh_private_.getParam("epsilon_theta", input_.epsilon_theta))
        input_.epsilon_theta = 0.000001;

    // Maximum distance for a correspondence to be valid
    if (!nh_private_.getParam("max_correspondence_dist", input_.max_correspondence_dist))
        input_.max_correspondence_dist = 0.3;

    // Noise in the scan (m)
    if (!nh_private_.getParam("sigma", input_.sigma))
        input_.sigma = 0.010;

    // Use smart tricks for finding correspondences.
    if (!nh_private_.getParam("use_corr_tricks", input_.use_corr_tricks))
        input_.use_corr_tricks = 1;

    // Restart: Restart if error is over threshold
    if (!nh_private_.getParam("restart", input_.restart))
        input_.restart = 0;

    // Restart: Threshold for restarting
    if (!nh_private_.getParam("restart_threshold_mean_error", input_.restart_threshold_mean_error))
        input_.restart_threshold_mean_error = 0.01;

    // Restart: displacement for restarting. (m)
    if (!nh_private_.getParam("restart_dt", input_.restart_dt))
        input_.restart_dt = 1.0;

    // Restart: displacement for restarting. (rad)
    if (!nh_private_.getParam("restart_dtheta", input_.restart_dtheta))
        input_.restart_dtheta = 0.1;

    // Max distance for staying in the same clustering
    if (!nh_private_.getParam("clustering_threshold", input_.clustering_threshold))
        input_.clustering_threshold = 0.25;

    // Number of neighbour rays used to estimate the orientation
    if (!nh_private_.getParam("orientation_neighbourhood", input_.orientation_neighbourhood))
        input_.orientation_neighbourhood = 20;

    // If 0, it's vanilla ICP
    if (!nh_private_.getParam("use_point_to_line_distance", input_.use_point_to_line_distance))
        input_.use_point_to_line_distance = 1;

    // Discard correspondences based on the angles
    if (!nh_private_.getParam("do_alpha_test", input_.do_alpha_test))
        input_.do_alpha_test = 0;

    // Discard correspondences based on the angles - threshold angle, in degrees
    if (!nh_private_.getParam("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
        input_.do_alpha_test_thresholdDeg = 20.0;

    // Percentage of correspondences to consider: if 0.9,
    // always discard the top 10% of correspondences with more error
    if (!nh_private_.getParam("outliers_maxPerc", input_.outliers_maxPerc))
        input_.outliers_maxPerc = 0.90;

    // Parameters describing a simple adaptive algorithm for discarding.
    //  1) Order the errors.
    //  2) Choose the percentile according to outliers_adaptive_order.
    //     (if it is 0.7, get the 70% percentile)
    //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
    //     with the value of the error at the chosen percentile.
    //  4) Discard correspondences over the threshold.
    //  This is useful to be conservative; yet remove the biggest errors.
    if (!nh_private_.getParam("outliers_adaptive_order", input_.outliers_adaptive_order))
        input_.outliers_adaptive_order = 0.7;

    if (!nh_private_.getParam("outliers_adaptive_mult", input_.outliers_adaptive_mult))
        input_.outliers_adaptive_mult = 2.0;

    // If you already have a guess of the solution, you can compute the polar angle
    // of the points of one scan in the new position. If the polar angle is not a monotone
    // function of the readings index, it means that the surface is not visible in the
    // next position. If it is not visible, then we don't use it for matching.
    if (!nh_private_.getParam("do_visibility_test", input_.do_visibility_test))
        input_.do_visibility_test = 0;

    // no two points in laser_sens can have the same corr.
    if (!nh_private_.getParam("outliers_remove_doubles", input_.outliers_remove_doubles))
        input_.outliers_remove_doubles = 1;

    // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
    if (!nh_private_.getParam("do_compute_covariance", input_.do_compute_covariance))
        input_.do_compute_covariance = 0;

    // Checks that find_correspondences_tricks gives the right answer
    if (!nh_private_.getParam("debug_verify_tricks", input_.debug_verify_tricks))
        input_.debug_verify_tricks = 0;

    // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
    // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
    if (!nh_private_.getParam("use_ml_weights", input_.use_ml_weights))
        input_.use_ml_weights = 0;

    // If 1, the field 'readings_sigma' in the second scan is used to weight the
    // correspondence by 1/sigma^2
    if (!nh_private_.getParam("use_sigma_weights", input_.use_sigma_weights))
        input_.use_sigma_weights = 0;
    // enable debug log output
    if (!nh_private_.getParam("sm_debug_write_flag", sm_debug_write_flag))
        sm_debug_write_flag = 1;

    // fix amcl pose angle error too much bug
    if (!nh_private_.getParam("offset_degree", offset_degree_))
        offset_degree_ = 20;
    if (!nh_private_.getParam("offset_cnt", offset_cnt_))
        offset_cnt_ = 10;


    sm_debug_write(sm_debug_write_flag);

//    ROS_INFO("csm param max_angular_correction_deg:%f", input_.max_angular_correction_deg);


}

void Csm_Wrapper::scan_to_ldp(const sm::LaserScan &scan, LDP &ldp, double offset = 0.0) {
    int n = scan.ranges.size();
    ldp = ld_alloc_new(n);
    for (int i = 0; i < n; i++) {
        double r = scan.ranges[i];
        bool valid = (r < scan.range_max - 0.01) && (r > scan.range_min + 0.1);

        ldp->valid[i] = (valid) ? 1 : 0;
        ldp->readings[i] = (valid) ? r : -1;
        ldp->theta[i] = scan.angle_min + i * scan.angle_increment + offset;
        ldp->cluster[i] = -1;
        ldp->readings_sigma[i] = 1.0;
    }

    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[n - 1];

    // for each new scan data
    // clear
    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;


}

double Csm_Wrapper::csm_fit(vector<double> &res) {

    res.clear();



    // **** reset ldp pose
    // odometry, estimate, true_pose
    map_ldp_->estimate[0] = 0.0;
    map_ldp_->estimate[1] = 0.0;
    map_ldp_->estimate[2] = 0.0;


    // **** fill data in input
    input_.laser_ref = map_ldp_;
    input_.laser_sens = scan_ldp_;

    // **** pose init guess



    // *** compute relative translation and rotation between two scan
    // get final pose?

    input_.first_guess[0] = 0.0;
    input_.first_guess[1] = 0.0;
    input_.first_guess[2] = 0.0;

    // **** clear output data
    // Initialize output_ vectors as Null for error-checking
    output_.cov_x_m = 0;
    output_.dx_dy1_m = 0;
    output_.dx_dy2_m = 0;

    // If they are non-Null, free covariance gsl matrices to avoid leaking memory
    if (output_.cov_x_m) {
        gsl_matrix_free(output_.cov_x_m);
        output_.cov_x_m = 0;
    }
    if (output_.dx_dy1_m) {
        gsl_matrix_free(output_.dx_dy1_m);
        output_.dx_dy1_m = 0;
    }
    if (output_.dx_dy2_m) {
        gsl_matrix_free(output_.dx_dy2_m);
        output_.dx_dy2_m = 0;
    }

    double match_error = 100.0;
    corr_valid_cnt_ = 0;




    // **** feed to csm
    sm_icp(&input_, &output_);
    if (output_.valid) {


//        ROS_ERROR("get relative translation [x,y,yaw]: [%f,%f,%f] \n", output_.x[0], output_.x[1], output_.x[2]);
        double res_array[] = {output_.x[0], output_.x[1], output_.x[2]};

        res = vector<double>(res_array, res_array + 3);
        // compute match error
        map_ldp_->points_w->phi;
        // point phi to valarray
        valarray<double> map_val(1.0, size_t(map_ldp_->nrays)), scan_val(1.0, size_t(map_ldp_->nrays));


        for (int i = 0; i < map_ldp_->nrays; i++) {
            if (!std::isnan(map_ldp_->readings[i]) && !std::isnan(scan_ldp_->points_w[i].rho) &&
                scan_ldp_->corr[i].valid == 1) {
                map_val[i] = map_ldp_->readings[i];
                scan_val[i] = scan_ldp_->points_w[i].rho;
                corr_valid_cnt_++;
            }
        }
        match_error = (abs(map_val - scan_val)).sum() / double(map_ldp_->nrays);


    } else {
        printf("failure");
    }



    // **** free memory
    ld_free(map_ldp_);
    ld_free(scan_ldp_);


    return match_error;

}


double Csm_Wrapper::get_base_pose(const sm::LaserScan &map_scan, const sm::LaserScan &sensor_scan, gm::Pose &base_pose,
                                  const tf::Transform &base_laser_tf, int &corr_valid_cnt) {


    tf::Pose base_pose_tf;
    tf::poseMsgToTF(base_pose, base_pose_tf);

    // **** prepare data

    input_.min_reading = sensor_scan.range_min;
    input_.max_reading = sensor_scan.range_max;



    // try different angle offset
    // anverage csmfit time 0.06s
    // try 10 angle offset, cost 0.6s
    double offset_range = offset_degree_ * Pi / 180.0;
    vector<double> res;
//    double offset = 0;
//    int cnt = 0;
//    for (cnt = 1; cnt< offset_cnt_;cnt++){
//        offset=((cnt %2 ) == 0 ?1:-1)*(cnt/2)*2*offset_range/offset_cnt_ ;
//        ROS_INFO("csm check offset %f",offset);
//
//        scan_to_ldp(map_scan, map_ldp_, offset);
//        scan_to_ldp(sensor_scan, scan_ldp_);
//
//        res = csm_fit();
//        if (res.empty())
//            continue;
//        else
//            break;
//
//    }
    scan_to_ldp(map_scan, map_ldp_);
    scan_to_ldp(sensor_scan, scan_ldp_);

    double match_error = csm_fit(res);
    corr_valid_cnt = corr_valid_cnt_;

    if (res.empty()) {
//        ROS_ERROR("******************\ncsm failure \n******************\n");
        return match_error;
    }


    tf::Transform laser_pose_chage;
    set_tf(res[0], res[1], res[2], laser_pose_chage);


    // laser pose


    tf::poseTFToMsg(base_pose_tf * base_laser_tf * laser_pose_chage * (base_laser_tf.inverse()), base_pose);


    return match_error;


}


// find match point between two laser scan

int Csm_Wrapper::find_match_point(const sm::LaserScan &map_scan, const sm::LaserScan &sensor_scan) {

    input_.min_reading = sensor_scan.range_min;
    input_.max_reading = sensor_scan.range_max;


    scan_to_ldp(map_scan, map_ldp_);
    scan_to_ldp(sensor_scan, scan_ldp_);

    // **** reset ldp pose
    // odometry, estimate, true_pose
    map_ldp_->estimate[0] = 0.0;
    map_ldp_->estimate[1] = 0.0;
    map_ldp_->estimate[2] = 0.0;


    // **** fill data in input
    input_.laser_ref = map_ldp_;
    input_.laser_sens = scan_ldp_;

    // **** pose init guess



    // *** compute relative translation and rotation between two scan
    // get final pose?

    input_.first_guess[0] = 0.0;
    input_.first_guess[1] = 0.0;
    input_.first_guess[2] = 0.0;

    LDP laser_ref = input_.laser_ref;
    LDP laser_sens = input_.laser_sens;


    // start csm
    ld_create_jump_tables(laser_ref);

    // reading,theta to x,y
    ld_compute_cartesian(laser_ref);
    ld_compute_cartesian(laser_sens);

    gsl_vector *x_new = gsl_vector_alloc(3);
    gsl_vector *x_old_ = vector_from_array(3, input_.first_guess);

    const double *q0 = x_old_->data();

    double x_old[3];
    copy_d(q0, 3, x_old);

    ld_compute_world_coords(laser_sens, x_old);

    find_correspondences_tricks(&input_);
    kill_outliers_double(&input_);
    double error = 0;
    /* Trim correspondences */
    kill_outliers_trim(&input_, &error);

    int num_corr_after = ld_num_valid_correspondences(laser_sens);

    return num_corr_after;
}


void Csm_Wrapper::set_tf(double x, double y, double yaw, tf::Transform &t) {

    t.setOrigin(tf::Vector3(x, y, 0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    t.setRotation(q);

}