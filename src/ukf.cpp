#include<iostream>;
#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    is_initialized_ = false;
    cout << "Object constructor is called" << endl;


    // State dimension
    n_x_ = 5;
    cout << "n_x_ set to " << n_x_ << endl;

    // Augmented state dimension
    n_aug_ = 7;

    // initial state vector
    x_ = VectorXd(5);
    x_.fill(0);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);
    
    P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;
        

    P_.fill(0.001);
    


    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 30;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30;

    /**
     * DO NOT MODIFY measurement noise values below.
     * These are provided by the sensor manufacturer.
     */

     // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;

    /**
     * End DO NOT MODIFY section for measurement noise values
     */

     /**
      * TODO: Complete the initialization. See ukf.h for other member properties.
      * Hint: one or more values initialized above might be wildly off...
      */





      // Sigma point spreading parameter
    lambda_ = 3 - n_aug_; // lambda = 3 - n_aug_

    // Weights of sigma points
    //Eigen::VectorXd 
    weights_ = VectorXd(2 * n_aug_ + 1);
    weights_.fill(0.5 / (lambda_ + n_aug_));
    weights_(0) = lambda_ / (lambda_ + n_aug_);

    /*
    double weight_0 = lambda / (lambda + n_aug);
    weights(0) = weight_0;
    for (int i = 1; i < 2 * n_aug + 1; ++i) {  // 2n+1 weights
        double weight = 0.5 / (n_aug + lambda);
        weights(i) = weight;
    }
    */
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    Xsig_pred_.fill(0);

    time_us_ = 0.0;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
     * TODO: Complete this function! Make sure you switch between lidar and radar
     * measurements.
     */
    
    if (!is_initialized_)
    {
        if (meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            VectorXd z_ = VectorXd(2);
            z_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
            x_ << z_[0], z_[1], 2.2, 0.5, 0.3;
        }
        else
        {
            VectorXd z_ = VectorXd(3);
            z_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
            x_ << z_[0] * cos(z_(1)), z_[0] * sin(z_(1)), 2.2, 0.5, 0.3;
        }

        is_initialized_ = true;
        time_us_ = meas_package.timestamp_;
        cout << "Initialized" << endl;
        return;

    }







    double dt = (double)((meas_package.timestamp_ - time_us_) * 1e-6);
    time_us_ = meas_package.timestamp_;
    
    
    cout << "calling prediction" << endl;
    Prediction(dt);
    std::cout << "Prediction Successful" << std::endl;



    if (meas_package.sensor_type_ == meas_package.LASER)
    {
        VectorXd z_out = VectorXd(2);
        MatrixXd S_out = MatrixXd(2, 2);

        // predicted sigma points matrix in measurement space
        Eigen::MatrixXd Zsig;
        PredictLidarMeasurement(&z_out, &S_out, &Zsig);

        UpdateLidar(meas_package, Zsig, z_out, S_out);

    }
    else if (meas_package.sensor_type_ == meas_package.RADAR)
    {
        VectorXd z_out = VectorXd(3);
        MatrixXd S_out = MatrixXd(3, 3);

        // predicted sigma points matrix in measurement space
        Eigen::MatrixXd Zsig;
        PredictRadarMeasurement(&z_out, &S_out, &Zsig);

        UpdateRadar(meas_package, Zsig, z_out, S_out);


    }
}



void UKF::Prediction(double delta_t) {
    /**
     * TODO: Complete this function! Estimate the object's location.
     * Modify the state vector, x_. Predict sigma points, the state,
     * and the state covariance matrix.
     */
    
     //Generate Augmented sigma points
    cout << "Generating Augmented sigma Points..." << endl;
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    AugmentedSigmaPoints(&Xsig_aug);
    cout << "Augmented sigma points created." << endl;

    //Predict sigma points
    MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_x_ + 1);
    cout << "Predicting sigma points" << endl;
    SigmaPointPrediction(&Xsig_pred, Xsig_aug, delta_t);
    cout << "sigma points predicted" << endl;

    //Predict Mean and Covariance
    VectorXd xp = VectorXd(n_x_);
    MatrixXd Pp = MatrixXd(n_x_, n_x_);
    cout << "Predicting Mean and Variance" << endl;
    PredictMeanAndCovariance(&xp, &Pp, Xsig_pred);
    x_ = xp;
    P_ = Pp;
    cout << "Mean and Variance predicted" << endl;

    Xsig_pred_ = Xsig_pred;


}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
     * TODO: Complete this function! Use lidar data to update the belief
     * about the object's position. Modify the state vector, x_, and
     * covariance, P_.
     * You can also calculate the lidar NIS, if desired.
     */
}

void UKF::UpdateRadar(MeasurementPackage meas_package, MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S)
{
    /**
    * TODO: Complete this function! Use radar data to update the belief
    * about the object's position. Modify the state vector, x_, and
    * covariance, P_.
    * You can also calculate the radar NIS, if desired.
    */

    // set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 3;

    // create example vector for mean predicted measurement
    //VectorXd z_pred = VectorXd(n_z);

    // create example matrix for predicted measurement covariance
    //MatrixXd S = MatrixXd(n_z, n_z);

    // create example vector for incoming radar measurement
    VectorXd z = meas_package.raw_measurements_;

    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);


    // calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
      // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // angle normalization
        while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization
        while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

    // update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    // write result
    //*x_out = x;
    //*P_out = P;

}





void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out)
{

    // create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);

    // create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

    // create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);


    // create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;

    // create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5, 5) = P_;
    P_aug(5, 5) = std_a_ * std_a_;
    P_aug(6, 6) = std_yawdd_ * std_yawdd_;

    // create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    // create augmented sigma points
    Xsig_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug_; ++i) {
        Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }


    // print result
    //std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

    // write result
    *Xsig_out = Xsig_aug;
}


void UKF::SigmaPointPrediction(MatrixXd* Xsig_out, MatrixXd& Xsig_aug, double delta_t)
{

    // create matrix with predicted sigma points as columns
    MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);


    // predict sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        // extract values for better readability
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        // predicted state values
        double px_p, py_p;

        // avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
            py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
        }
        else {
            px_p = p_x + v * delta_t * cos(yaw);
            py_p = p_y + v * delta_t * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;

        // add noise
        px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_a * delta_t;

        yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
        yawd_p = yawd_p + nu_yawdd * delta_t;

        // write predicted sigma point into right column
        Xsig_pred(0, i) = px_p;
        Xsig_pred(1, i) = py_p;
        Xsig_pred(2, i) = v_p;
        Xsig_pred(3, i) = yaw_p;
        Xsig_pred(4, i) = yawd_p;
    }


    // print result
    //std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

    // write result
    *Xsig_out = Xsig_pred;


}


void UKF::PredictMeanAndCovariance(VectorXd* X_out, MatrixXd* P_out, MatrixXd& Xsig_pred)
{
    // create vector for predicted state
    VectorXd x = VectorXd(n_x_);

    // create covariance matrix for prediction
    MatrixXd P = MatrixXd(n_x_, n_x_);

    // predicted state mean
    x.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
        x = x + weights_(i) * Xsig_pred.col(i);
    }

    // predicted state covariance matrix
    P.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
      // state difference
        VectorXd x_diff = Xsig_pred.col(i) - x;
        // angle normalization
        while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

        P = P + weights_(i) * x_diff * x_diff.transpose();
    }



    // print result
    //std::cout << "Predicted state" << std::endl;
    //std::cout << x << std::endl;
    //std::cout << "Predicted covariance matrix" << std::endl;
    //std::cout << P << std::endl;

    // write result
    *X_out = x;
    *P_out = P;
}


void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out)
{

    // set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 3;


    // create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    // mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0);

    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    
    // transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
      // extract values for better readability
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);

        double v1 = cos(yaw) * v;
        double v2 = sin(yaw) * v;

        // measurement model
        Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                       // r
        Zsig(1, i) = atan2(p_y, p_x);                                // phi
        Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);   // r_dot
    }

    // mean predicted measurement
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    // innovation covariance matrix S
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
      // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // angle normalization
        while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_radr_ * std_radr_, 0, 0,
        0, std_radphi_* std_radphi_, 0,
        0, 0, std_radrd_* std_radrd_;
    S = S + R;

    // write result
    *z_out = z_pred;
    *S_out = S;
    *Zsig_out = Zsig;
}




void UKF::PredictLidarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out)
{

    // set measurement dimension, radar can measure p_x, and p_y
    int n_z = 2;


    // create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    // mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0);

    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);

    // transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
      // extract values for better readability
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        
        // measurement model
        Zsig(0, i) = p_x;                       // x
        Zsig(1, i) = p_y;                       // y
    }

    // mean predicted measurement
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    // innovation covariance matrix S
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
      // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_laspx_ * std_laspx_, 0,
        0, std_laspy_* std_laspy_;
        
    S = S + R;

    // write result
    *z_out = z_pred;
    *S_out = S;
    *Zsig_out = Zsig;
}



void UKF::UpdateLidar(MeasurementPackage meas_package, MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S)
{
    /**
    * TODO: Complete this function! Use radar data to update the belief
    * about the object's position. Modify the state vector, x_, and
    * covariance, P_.
    * You can also calculate the radar NIS, if desired.
    */

    // set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 2;

    // create example vector for mean predicted measurement
    //VectorXd z_pred = VectorXd(n_z);

    // create example matrix for predicted measurement covariance
    //MatrixXd S = MatrixXd(n_z, n_z);

    // create example vector for incoming radar measurement
    VectorXd z = meas_package.raw_measurements_;

    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);


    // calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
      // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    
    // update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    // write result
    //*x_out = x;
    //*P_out = P;

}

