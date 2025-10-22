#include "KF.h"

arma::mat q2R(double w, double x, double y, double z) {
    arma::mat R(3,3);
    R(0,0) = 1 - 2*y*y - 2*z*z;
    R(0,1) = 2*x*y - 2*z*w;
    R(0,2) = 2*x*z + 2*y*w;
    R(1,0) = 2*x*y + 2*z*w;
    R(1,1) = 1 - 2*x*x - 2*z*z;
    R(1,2) = 2*y*z - 2*x*w;
    R(2,0) = 2*x*z - 2*y*w;
    R(2,1) = 2*y*z + 2*x*w;
    R(2,2) = 1 - 2*x*x - 2*y*y;
    return R;
}

KF::KF(const ros::NodeHandle & nh):
    _nh(nh),

    _yPos(3),
    _yQ(4),
    _yVel(3),
    _yAcc(3),
    _yOut(6),

    _yPosR(3,3),
    _yVelR(3,3),
    _yAccR(3,3),
    _yOutR(6,6),
    _yQR(4,4),

    _preState(6),
    _estState(6),

    _z(6),

    _A(6,6),
    _B(6,3),
    _C(6,6),
    _Q(6,6),
    
    _K(6,3),

    _preP(6,6),
    _P(6,6),

    _mixMsgSub(_nh.subscribe("/leader/information", 10, &KF::mixMsgCallback, this)),
    _filterPosPub(_nh.advertise<kalman_filter::Vector3Stamped>("/leader/filter_pos", 10)),
    _filterVelPub(_nh.advertise<kalman_filter::Vector3Stamped>("/leader/filter_vel", 10))
    {
        _timeLast = 0;
        _timeNow = 0;
        _timeGap = 0;

        ROS_INFO("Kalman Filter initialized");
        arma::mat I3(3,3, arma::fill::eye);
        arma::mat I6(6,6, arma::fill::eye);

        // porcess noise covariance
        arma::vec Qvec = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
        _Q = diagmat(Qvec);

        // measurement noise covariance
        arma::vec posRVec = {0.1, 0.1, 0.1};
        arma::vec velRVec = {0.1, 0.1, 0.1};
        _yPosR = diagmat(posRVec);
        _yVelR = diagmat(velRVec);

        _P.eye(6,6);
        _preP.eye(6,6);

        _C.eye(6,6);

    } 

// double generateGaussianNoise(double mean = 0.0, double stddev = 1.0) {
//     // 生成标准正态分布 N(0,1)
//     double noise = arma::randn();

//     // 调整均值和标准差
//     noise = mean + stddev * noise;

//     return noise;
// }
// arma::vec generateGaussianNoise6x1(arma::mat stddev = arma::mat(6,6, arma::fill::eye),arma::vec mean = arma::vec(6, arma::fill::ones))  {
//     arma::vec noise(6);
//     for (int i = 1; i <= 6; i++){
//         noise(i) = generateGaussianNoise(mean(i), stddev(i,i));
//     }
    
//     return noise;
// }

void KF::mixMsgCallback(const offboard_control::PosVelAcc::ConstPtr& msg){    
    // pos and vel of System output is used to update the state. C 6x6, y 6x1
    // Assume Acc is not disrupt
    // Using small angle assumption, so Acc dont need to be converted by quaternion
    _timeNow = msg->timestamp;
    _timeGap = _timeNow - _timeLast;
    _timeLast = _timeNow;

    _yPos(0) = msg->x_pos;
    _yPos(1) = msg->y_pos;
    _yPos(2) = msg->z_pos;

    _yVel(0) = msg->x_vel;
    _yVel(1) = msg->y_vel;
    _yVel(2) = msg->z_vel;

    arma::mat R = q2R(msg->w_ori, msg->x_ori, msg->y_ori, msg->z_ori);
    _yAcc(0) = msg->x_acc;
    _yAcc(1) = msg->y_acc;
    _yAcc(2) = msg->z_acc;
    _yAcc = R * _yAcc;
    
    _yOut = join_cols(_yPos, _yVel);
    // _yOut += generateGaussianNoise6x1(_yOutR, _yOut);

    _A.eye(6,6);
    arma::mat I3(3,3, arma::fill::eye);
    _A.submat(0,3,2,5) = _timeGap * I3;

    _B.eye(6,3);
    _B.submat(3,0,5,2) = _timeGap * I3;
}
void KF::predict(){
    _preState = _A * _estState + _B * _yAcc;
    _preP = _A * _P * _A.t() + _Q;
}

void KF::update(){
    arma::mat I6(6,6, arma::fill::eye);

    _z = _yOut - _C * _preState;
    _K = _preP * _C.t() * (_C * _preP * _C.t() + _yOutR).i();
    _estState = _preState + _K * _z;
    _P = (I6 - _K * _C) * _preP;
}

void KF::run(){
    predict();
    update();

    // ROS_INFO("Kalman Filter run");
    kalman_filter::Vector3Stamped posMsg;
    kalman_filter::Vector3Stamped velMsg;
    posMsg.time = _timeNow;
    velMsg.time = _timeNow;

    posMsg.x = _estState(0);
    posMsg.y = _estState(1);
    posMsg.z = _estState(2);
    velMsg.x = _estState(3);
    velMsg.y = _estState(4);
    velMsg.z = _estState(5);

    _filterPosPub.publish(posMsg);
    _filterVelPub.publish(velMsg);
}