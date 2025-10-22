#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <udp_pkg/PositionVelocityAccel.h>
#include <armadillo>
#include <kalman_filter/Vector3Stamped.h>
class KF {
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _mixMsgSub;
        ros::Publisher _filterPosPub;
        ros::Publisher _filterVelPub;


        double _timeNow;
        double _timeLast;
        double _timeGap;

        arma::vec _yPos;
        arma::vec _yQ;
        arma::vec _yVel;
        arma::vec _yAcc;
        arma::vec _yOut;

        arma::mat _yPosR;
        arma::mat _yVelR;
        arma::mat _yAccR;
        arma::mat _yQR;
        arma::mat _yOutR;
        arma::vec _posRVec = {0.1, 0.1, 0.1};
        arma::vec _velRVec = {0.1, 0.1, 0.1};

        arma::vec _preState;
        arma::vec _estState;
        
        arma::vec _z; //innovation

        arma::mat _A;
        arma::mat _B;
        arma::mat _C;
        arma::mat _Q;

        arma::mat _K;

        arma::mat _preP;
        arma::mat _P;
    public:
        KF(const ros::NodeHandle & nh);

        void mixMsgCallback(const udp_pkg::PositionVelocityAccel::ConstPtr& msg);

        // double generateGaussianNoise(double mean , double stddev );
        // arma::vec generateGaussianNoise6x1(arma::mat stddev ,arma::vec mean );

        void predict();
        void update();
        void run();

};