#include<ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include<armadillo>
#include<sensor_msgs/Imu.h>
#include<message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include<tf/tf.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Float64.h>
#include "udp_pkg/PositionVelocityAccel.h"
#include <kalman_filter/Vector3Stamped.h>

// // ros::Time pos_ros_last_time = ros::Time::now();
// ros::Time vel_ros_last_time = ros::Time::now();

// // double pos_last_time = pos_ros_last_time.toSec();
// double vel_last_time = vel_ros_last_time.toSec();

// // double pos_total_time = 0;
// double vel_total_time = 0;

// // double pos_time_gap = 0;
// double vel_time_gap = 0;

// void pose_call_back()
// {
//     ros::Time pos_ros_now_time = ros::Time::now();
//     double pos_now_time = pos_ros_now_time.toSec();
//     pos_time_gap = pos_now_time - pos_last_time;


// }

// 定义数据类型以及函数
using namespace std;

struct DoubleArray
{
	double* array;
	int length;
};

struct StringArray
{
	string* array;
	int length;
};

struct DataArray
{
	int length;
	double* time;
	double* timegap;
	double* pos;
	double* vel;
	double* acc;
};

struct TimeArray
{
	double* time;
	double* timegap;
};

using namespace arma;

struct KalmanData
{
	mat x;
	mat P;
};

//struct KalmanMissData
//{
//	mat x;
//	mat P;
//	mat mid_term;
//};


struct BernulliData
{
	mat mean;
	mat var;
	//double mean;
	//double var;
};

//ȫ�ֱ�������

const double sensor_mean[2] = { 0,0 };	//������ֵ
const double sensor_stddev[2] = {0.8,0.5 }; //������׼��
//mat R(2,2);


const double process_mean[2] = { 0,0 };	//����������ֵ
const double process_stddev[2] = { 1,0.8 }; //����������׼��
//mat Q = zeros(2, 2);

const double gravity_coefficent = 9.8;

//mat mid_term(2,2);
//KalmanData  data_miss_predict = { zeros(2,1) , zeros(2,2)};

double cbTime=0;

//��������
double bernoulli(double prob)
{
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	default_random_engine e(seed);
	bernoulli_distribution u(prob);
	//e.seed(time(0));

	//for (int i = 0; i < 10; i++)
	//{
	//	cout << u(e) << endl;
	//}

	return u(e);
}


//  DoubleArray static readdoublejson(string& filepath, string datatype) {

// 	Json::Reader reader;
// 	Json::Value root;
// 	int root_size = 0;
// 	const int array_size = 10000;
// 	double pos[array_size];
// 	DoubleArray POS;
// 	std::ifstream jsonfile(filepath, std::ios::binary);

// 	if (reader.parse(jsonfile, root))
// 	{
// 		root_size = root.size();
// 		POS.array = new double[root_size];
// 		for (int i = 0; i < root_size; i++)
// 		{
// 			POS.array[i] = root[i][datatype].asDouble();
// 			//cout << pos[i] << endl;

// 		}
// 	}

// 	//POS.array = pos;
// 	POS.length = root_size;
// 	//cout << root_size << endl;
// 	// 
// 	//for (int i = 0; i < root_size; i++)
// 	//{
// 	//	cout << POS.array[i] << endl;

// 	//}

// 	return POS;
// }


 double calculate_acc(double pre_acc)
 {

		double acc = pre_acc - gravity_coefficent;

	 return acc;
 }


//  StringArray static readstringjson(string& filepath,string datatype) {

// 	Json::Reader reader;
// 	Json::Value root;
// 	int root_size = 0;
// 	const int array_size = 10000;
// 	string str[array_size];
// 	StringArray STR;
// 	std::ifstream jsonfile(filepath, std::ios::binary);

// 	if (reader.parse(jsonfile, root))
// 	{
// 		root_size = root.size();
// 		STR.array = new string[array_size];
// 		for (int i = 0; i < root_size; i++)
// 		{
// 			STR.array[i] = root[i][datatype].asString();
// 			//cout << str[i] << endl;

// 		}
// 	}

// 	//STR.array = str;
// 	STR.length = root_size;

// 	//for (int i = 0; i < root_size; i++)
// 	//{
// 	//	cout << STR.array[i] << endl;

// 	//}
// 	//cout << root_size << endl;


// 	return STR;
// }

 TimeArray str2double(StringArray time_array)
 {
	 string* array = time_array.array;
	 int len = time_array.length;
	 TimeArray datatime;
	 datatime.time = new double[len];
	 datatime.timegap = new double[len - 1];

	 
	 for (int i = 0; i < len; i++)
	 {
		 array[i] = array[i].substr(array[i].size() - 9, array[i].size());
		 datatime.time[i] = stod(array[i]);

	 };
	 

	 for (int i = 0; i < (len-1); i++)
	 {
		 datatime.timegap[i] = datatime.time[i+1] - datatime.time[i];

	 };



	 return datatime;


 }

 double* generateSensorGaussianNoise()
 {
	 static std::default_random_engine generator1; // �����������
	 static std::default_random_engine generator2; // �����������

	 static std::normal_distribution<double> distribution1(sensor_mean[0], sensor_stddev[0]); // ��˹�ֲ�

	 static std::normal_distribution<double> distribution2(sensor_mean[1], sensor_stddev[1]); // ��˹�ֲ�

	 double* y_noise;

	 y_noise = new double[2];

	 y_noise[0] = distribution1(generator1);
	 y_noise[1] = distribution2(generator2);



	 return y_noise; // ����һ����˹�ֲ��������
 }


 using namespace arma;
DataArray SensorData(DataArray data)
 {
	DataArray y = data;
	double* p_noise;
	double* v_noise;

	p_noise = new double[data.length];
	v_noise = new double[data.length];

	y.pos = new double[data.length];
	y.vel = new double[data.length];

	double amount_pos = 0;
	double mean_pos = 0;
	double var_pos = 0;

	double amount_vel = 0;
	double mean_vel = 0;
	double var_vel = 0;

	//for (int i = 0; i < data.length; i++)
	//{
	//	amount_pos += data.pos[i];
	//	amount_vel += data.vel[i];
	//}




	for (int i = 0; i < data.length; i++)
	{
		p_noise[i] = generateSensorGaussianNoise()[0];
		v_noise[i] = generateSensorGaussianNoise()[1];

		y.pos[i] = data.pos[i] + p_noise[i];
		y.vel[i] = data.vel[i] + v_noise[i];

		//cout << "generateSensorGaussianNoise()[0]:: \t" << generateSensorGaussianNoise()[0] << endl;
		//cout << "y.pos:: \t" << y.pos[i] << endl;

		//cout << "generateSensorGaussianNoise()[1]:: \t" << generateSensorGaussianNoise()[1] << endl;
		//cout << "y.vel:: \t" << y.vel[i] << endl;

		amount_pos += p_noise[i];
		amount_vel += v_noise[i];

	}

	mean_pos = amount_pos / data.length;
	mean_vel = amount_vel / data.length;


	for (int i = 0; i < data.length; i++)
	{
		var_pos += pow((p_noise[i] - mean_pos), 2);
		var_vel += pow((v_noise[i] - mean_vel), 2);

	}

	var_pos = var_pos / (data.length - 1);
	var_vel = var_vel / (data.length - 1);

	cout << "Pos_Noise Mean: \n" << mean_pos << endl;
	cout << "Given Pos Noise Mean: \n" << sensor_mean[0] << endl;

	cout << "Pos_Noise Var: \n" << var_pos << endl;
	cout << "Given Pos Noise Var: \n" << sensor_stddev[0] * sensor_stddev[0] << endl;

	cout << "Vel_Noise Mean: \n" << mean_vel << endl;
	cout << "Given Vel Noise Mean: \n" << sensor_mean[1] << endl;

	cout << "Vel_Noise Var: \n" << var_vel << endl;
	cout << "Given Vel Noise Var: \n" << sensor_stddev[1] * sensor_stddev[1] << endl;


	return y;
 }

DataArray MissSensorData(DataArray data , BernulliData miss_prob)
{
	DataArray y = data;
	y.pos = new double[data.length];
	y.vel = new double[data.length];

	int count_pos = 0;
	int count_pos_1 = 0;

	int count_vel = 0;
	int count_vel_1 = 0;
	for (int i = 0; i < data.length; i++)
	{

		//unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		//default_random_engine e_pos(seed);
		//default_random_engine e_vel(seed);

		//bernoulli_distribution u_pos(miss_prob.mean(0,0));
		//bernoulli_distribution u_vel(miss_prob.mean(1, 1));

		//
		//double alpha_pos = u_pos(e_pos);
		//double alpha_vel = u_vel(e_vel);
		
		double alpha_pos = bernoulli(miss_prob.mean(0,0));
		double alpha_vel = bernoulli(miss_prob.mean(1, 1));

		if (alpha_pos == 1)
		{
			count_pos_1++;
		}

		if (alpha_vel == 1)
		{
			count_vel_1++;
		}

		//cout << alpha_pos << endl;
		//cout << alpha_vel << endl;
		//cin.get();
		y.pos[i] = data.pos[i] * alpha_pos;
		//cout << "miss_y.pos: \t" << y.pos[i] << endl;
		y.vel[i] = data.vel[i] * alpha_vel;


		count_pos++;

		count_vel++;
	}

	//cout << count_pos_1;
	cout << "pos_mean: \n" << miss_prob.mean(0,0) << endl;

	cout<< "count_pos_prob: \n" << double(count_pos_1) / double(count_pos) << endl;

	cout << "vel_mean: \n" << miss_prob.mean(1,1) << endl;

	cout << "count_vel_prob: \n" << double(count_vel_1) / double(count_vel) << endl;



	return y;

}

DataArray FDISensorData(DataArray data)
{
	DataArray y = data;
	y.pos = new double[data.length];
	y.vel = new double[data.length];


	for (int i = 0; i < data.length; i++)
	{
		double alpha_pos = bernoulli(0.5);
		double alpha_vel = bernoulli(0.6);

		double pos_attak = -10+ 20 * randu();
		double vel_attak = -10 + 20 * randu();

		y.pos[i] = data.pos[i] + alpha_pos* pos_attak;
			
		y.vel[i] = data.vel[i] + alpha_vel*	vel_attak;

	}

	return y;

}



mat createMatA(int n, double T)
{
	mat A = eye(n,n);
	for (int i = 0; i < n - 1; i++)
	{
		A(i, i + 1) = T;
	}

	//cout << "A: \n" << A << "\n";
	return A;
}

mat createMatB(int m,int n, double T)
{
	mat B = zeros(m,n);
	
	B(m - 1, 0) = T;

	//cout << "B: \n" << B << "\n";

	return B;
}

mat createMatC(int n)
{
	mat C = eye(n, n);

	//cout << "C: \n" << C << endl;

	return C;
}

mat createMaty(double pos, double vel)
{
	mat y(2,1);
	y(0, 0) = pos;
	y(1, 0) = vel;


	//cout << "C: \n" << C << endl;

	return y;
}

mat MidTerm_Expectance(mat A , mat B , double a, mat Expectance_x) 
{
	
	mat temp;

	temp = A * Expectance_x + B * a;

	return temp;


}

KalmanData KalmanMissRobustPredict(KalmanData data, double a, mat A, mat B, mat Q, double T)
{
	KalmanData data_predict;

	data_predict.x = A * data.x + B * a;

	data_predict.P = A * data.P * A.t() + T * Q * T;

	std::cout << "data.P is " << data.P << std::endl;
	std::cout << "data.x is " << data.x << std::endl;
	std::cout << "A is " << A << std::endl;
	std::cout << "B is " << A << std::endl;
	std::cout << "Q is " << A << std::endl;

	return data_predict;

}

KalmanData KalmanFDIRobustPredict(KalmanData data, double a, mat A, mat B, mat Q, double T)
{
	KalmanData data_predict;

	data_predict.x = A * data.x + B * a;

	data_predict.P = A * data.P * A.t() + T * Q * T;

	return data_predict;

}

double Psi(double x , double c)
{
	if (abs(x) <= c)
	{
		return x;
	}

	else
	{
		return sign(x) * c;
	}
}

mat omega_function( mat y , mat x , mat beta , int dimension , double c	)
{
	mat Omega = zeros(dimension,dimension);

	for (int i = 0; i < dimension; i++)
	{
		double y_temp = y(i, 0);
		mat x_temp = (x.row(i) * beta);
		double x_temp_double = x_temp(0, 0);

		if ( y_temp == x_temp_double )
		{
			Omega(i, i) = 1;
		}

		else
		{
			Omega(i, i) = Psi(y_temp - x_temp_double,c) /( y_temp  - x_temp_double);
		}
	}

	return Omega;
}

KalmanData KalmanMissRobustFusion(KalmanData data ,KalmanData data_predict, mat y ,mat C, mat R,double c)
{
	mat Zero = zeros(2, 2);
	mat I = eye(2, 2);

	mat temp1 = join_rows(data_predict.P, Zero);
	std::cout << "data_predict.P is :" << data_predict.P << std::endl;

	mat temp2 = join_rows(Zero, R);

	mat S_square = join_cols(temp1,temp2);
	double lambda = 1e-6;
	mat eye_lambda = eye(4,4);
	S_square += lambda * eye_lambda; 

	std::cout << "S_square is :" << S_square << std::endl;
	// cout << "KalmanMissRobustFusion: \n" << S_square << endl;
	// cin.get();

	mat S = chol(S_square).t();

	//cout << "S:\n" << S << endl;

	mat X_temp = join_cols( I ,  C );
	//cout << "X_temp:\n" << X_temp << endl;

	mat X = inv(S) * X_temp; 
	//cout << "X:\n" << X << endl;

	mat Y_temp = join_cols(data_predict.x, y);
	//cout << "Y_temp:\n" << Y_temp << endl;

	mat Y = inv(S) * Y_temp;
	//cout << "Y:\n" << Y << endl;

	mat Omega = omega_function(Y, X, data.x, 4,c);
	//cout << "Omega:\n" << Omega << endl;

	KalmanData data_next;

	data_next.x = inv(X.t() * Omega * X) * ( X.t() * Omega * Y);
	data_next.P = inv(X.t() * Omega * X);
	return data_next;
}

KalmanData KalmanFDIRobustFusion(KalmanData data, KalmanData data_predict, mat y, mat C, mat R, double c)
{
	mat Zero = zeros(2, 2);
	mat I = eye(2, 2);

	mat temp1 = join_rows(data_predict.P, Zero);
	mat temp2 = join_rows(Zero, R);

	mat S_square = join_cols(temp1, temp2);
	double lambda = 1e-6;
	mat eye_lambda = eye(4,4);
	S_square += lambda * eye_lambda; //预防不正定

	// cout << "KalmanFDIRobustFusion: \n"<< S_square << endl;

	mat S = chol(S_square).t();

	//cout << "S:\n" << S << endl;

	mat X_temp = join_cols(I, C);
	//cout << "X_temp:\n" << X_temp << endl;

	mat X = inv(S) * X_temp;
	//cout << "X:\n" << X << endl;

	mat Y_temp = join_cols(data_predict.x, y);
	//cout << "Y_temp:\n" << Y_temp << endl;

	mat Y = inv(S) * Y_temp;
	//cout << "Y:\n" << Y << endl;

	mat Omega = omega_function(Y, X, data.x, 4, c);
	//cout << "Omega:\n" << Omega << endl;

	KalmanData data_next;

	data_next.x = inv(X.t() * Omega * X) * (X.t() * Omega * Y);
	data_next.P = inv(X.t()* Omega * X);
	return data_next;
}

KalmanData KalmanPredict(KalmanData data, double a, mat A, mat B, mat Q, double T)
{
	KalmanData data_predict;
	data_predict.x = A * data.x + B * a;

	//cout << A;

	//cout << B * a;

	data_predict.P = A * data.P * A.t() + T * Q * T;

	//cout << "KalmanPredict Finished!" << endl;


	return data_predict;
	//return data;
}

KalmanData KalmanFusion(KalmanData data_predict, mat y, mat C, mat R)
{
	KalmanData data_next;
	int dimension = data_predict.P.n_rows;
	mat KGain(dimension, dimension);

	mat S = C * data_predict.P * C.t() + R;
	//double regularization = 1e-6; // ������
	//S.diag() += regularization;

	//KGain = data_predict.P * C.t() * inv(C * data_predict.P * C.t() + R);
	KGain = data_predict.P * C.t() * inv(S);


	mat I = eye(dimension, dimension);
	//cout << data_predict.P;

	data_next.x = data_predict.x + KGain * (y - C * data_predict.x);
	data_next.P = (I - KGain * C) * data_predict.P * (I - KGain * C).t() + KGain * R * KGain.t();

	//cout << "KalmanFusion Finished!" << endl;

	return data_next;
	//return data_predict;
}

KalmanData KalmanMissPredict(KalmanData data, double a, mat A, mat B, mat Q, double T )
{
	//std::cout << "KalmanMissPredict Begin!" << endl;

	KalmanData data_predict;

	data_predict.x = A * data.x + B * a;

	//cout << A;

	//cout << B * a;

	data_predict.P = A * data.P * A.t() + T * Q * T;


	//std::cout << "KalmanMissPredict Finished!" << endl;


	return data_predict;
}

KalmanData KalmanMissFusion(mat midterm,KalmanData data_predict, mat y, mat C, mat R , BernulliData bernulli)
{

	//std::cout << "KalmanMissFusion Begin!" << endl;

	KalmanData data_next;
	int dimension = data_predict.P.n_rows;
	mat KGain(dimension, dimension);

	//cout << "C: \n" << C << endl;
	//cout << "�м�� \n" << bernulli.mean * C * data_predict.P * C.t() * bernulli.mean << endl;
	//cout << bernulli.mean << endl << bernulli.var << endl;
	//cin.get();
	mat S = bernulli.mean * C * data_predict.P * C.t() * bernulli.mean + bernulli.var % (C * midterm * C.t()) + R;
	//mat S = bernulli.mean * C * data_predict.P * C.t() * bernulli.mean + bernulli.var % C * midterm * C.t() + R;

	//cout << "S: \t" << S << endl;

	//double regularization = 1e-6; // ������
	//S.diag() += regularization;


	//cout << "S:\n" << S << endl;
	//cout << "inv(S):\n" << inv(S) << endl;
	//cin.get();
	//KGain = data_predict.P * C.t() * inv(C * data_predict.P * C.t() + R);
	KGain = data_predict.P * C.t() * bernulli.mean * inv(S);
	//cout << bernulli.mean << endl;

	mat I = eye(dimension, dimension);
	//cout << I << endl;
	//cin.get();

	//cout << data_predict.P;

	//cout << KGain <<endl;
	data_next.P = (I - KGain * bernulli.mean * C) * data_predict.P * (I - KGain * bernulli.mean * C).t() +  KGain * (bernulli.var %  (C * midterm * C.t())) * KGain.t()  + KGain * R * KGain.t();
	//cout << "Fusion_P \n" << data_next.P << endl;
	//data_next.P = (I - KGain  * C) * data_predict.P * (I - KGain  * C).t()  + KGain * R * KGain.t();

	data_next.x = data_predict.x + KGain * (y - bernulli.mean *  C * data_predict.x);

	//cout << bernulli.mean << endl;

	//data_next.x = data_predict.x + KGain * (y -  C * data_predict.x);

	//cout << "KalmanFusion Finished!" << endl;



	//std::cout << "KalmanMissFusion Finished!" << endl;


	return data_next;
}


mat MidTerm(mat midterm ,mat data, double a , mat A , mat B, mat Q, double T)
{
	//std::cout << "MidTerm Begin!" << endl;

	//mat midterm_next(2,2);
	//cout << midterm << endl;
	//cout << A * midterm * A.t() << endl;

	mat midterm_next = A * midterm * A.t() + A * data * (B * a).t() + B * a * (data).t() * A.t() + B * a * (B * a).t()  + T * Q * T;

	//cout << B; 
	//cout << A << endl << A.t() << endl;
	//cin.get();

	//cout << "midterm_calculate: \n" << midterm_next << endl;
	//cout << "A" << A << endl << "B \n " << B << endl;

	//std::cout << "MidTerm Finished!" << endl;


	return midterm_next;

}

KalmanData KalmanFilter(KalmanData data ,mat y,double a , mat A, mat B, mat C , mat Q , mat R , double T)
{
	KalmanData data_predict = KalmanPredict( data,  a,  A, B,  Q,  T );
	std::cout << "A" << A << std::endl;
	std::cout << "B" << B << std::endl;
	std::cout << "C" << C << std::endl;
	std::cout << "Q" << Q << std::endl;
	std::cout << "R" << R << std::endl;
	std::cout << "T" << T << std::endl;

	std::cout << "data_predict" << data_predict.x << std::endl;
	KalmanData data_next = KalmanFusion( data_predict,  y,  C,  R);
	
	//cout << "KalmanFilter Finished!" << endl;

	return data_next;

	//return data;
}


KalmanData KalmanMissFilter(KalmanData data, mat y, double a, mat A, mat B, mat C, mat Q, mat R, double T,BernulliData bernulli , mat* mid_term , KalmanData* data_miss_predict , mat* Expectance_x)
{
	//std::cout << "KalmanMissFilter Begin!" << endl;
	//cout << "data_miss_predict_first: \n" << data_miss_predict.x << endl;


	*mid_term = MidTerm(*mid_term, *Expectance_x, a, A, B, Q, T);
	//cout << "Expectance_x:\n" << *Expectance_x << endl;

	*Expectance_x = MidTerm_Expectance(A, B, a, *Expectance_x);
	//cout << "Expectance_x:\n" << *Expectance_x << endl;
	//cout << "mid_term_before: \n" << mid_term << endl;
	//cout << "mid_term_after: \n" << mid_term << endl;


	//cout << "data_miss_predict_before: \n" << data_miss_predict.x << endl;
	*data_miss_predict = KalmanMissPredict(data, a, A, B, Q, T);
	//cout << "data_miss_predict:\n" << data_miss_predict->x << endl;

	//cout << "data_miss_predict_after: \n" << data_miss_predict.x << endl;

	//cin.get();
	//cout << "mid_term: \n" << mid_term << endl;


	KalmanData data_next = KalmanMissFusion(*mid_term,*data_miss_predict, y, C, R, bernulli);

	//std::cout << "KalmanMissFilter Finished!" << endl;


	return data_next;

	//return data;
}



KalmanData KalmanMissRobustFilter(KalmanData data, mat y, double a, mat A, mat B, mat C, mat Q, mat R, double T, KalmanData* data_robust_predict,double c)
{

	*data_robust_predict = KalmanMissRobustPredict(data, a, A, B, Q, T);


	
	KalmanData data_next = KalmanMissRobustFusion(data, *data_robust_predict,y,C,R,c);

	return data_next;

}

KalmanData KalmanFDIRobustFilter(KalmanData data, mat y, double a, mat A, mat B, mat C, mat Q, mat R, double T, KalmanData* data_robust_predict, double c)
{

	*data_robust_predict = KalmanFDIRobustPredict(data, a, A, B, Q, T);

	KalmanData data_next = KalmanFDIRobustFusion(data, *data_robust_predict, y, C, R, c);

	return data_next;

}


// 定义全局变量以及设置回调函数保存
float px_now;
float py_now;
float pz_now;

float vx_now;
float vy_now;
float vz_now;

float ax_now;
float ay_now;
float az_now;

geometry_msgs::PoseStamped temp_now;

bool spin_flag = false; //用于标记数据同步


// void pos_call_back(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//     geometry_msgs::PoseStamped pos_now = *msg;
//     px_now = pos_now.pose.position.x;
//     py_now = pos_now.pose.position.y;
//     pz_now = pos_now.pose.position.z;
//     // ROS_INFO("pos_call_back");
//     ROS_INFO("px_now is %f",px_now);

// }
// void vel_call_back(const geometry_msgs::TwistStamped::ConstPtr& msg)
// {
//     // ros::Time vel_ros_now_time = ros::Time::now();
//     // double vel_now_time = vel_ros_now_time.toSec();

//     geometry_msgs::TwistStamped vel_now = *msg;
//     vx_now = vel_now.twist.linear.x;
//     vy_now = vel_now.twist.linear.y;
//     vz_now = vel_now.twist.linear.z;

//     ROS_INFO("vx_now is %f",vx_now);
// }

// void acc_call_back(const sensor_msgs::Imu::ConstPtr& msg)
// {
//     sensor_msgs::Imu acc_now = *msg;

//     ax_now = acc_now.linear_acceleration.x;
//     ay_now = acc_now.linear_acceleration.y;
//     az_now = acc_now.linear_acceleration.z;

//    ROS_INFO("ax_now is %f",ax_now); 
// }

// 同步订阅回调函数
// void callback(const geometry_msgs::PoseStamped::ConstPtr& pos_msg,const geometry_msgs::TwistStamped::ConstPtr& vel_msg,const sensor_msgs::Imu::ConstPtr& acc_msg)
// {

// 	spin_flag = true;

// 	geometry_msgs::PoseStamped pos_now = *pos_msg;
//     geometry_msgs::TwistStamped vel_now = *vel_msg;
//     sensor_msgs::Imu acc_body = *acc_msg;


// 	temp_now.header = pos_now.header;
// 	temp_now.pose  = pos_now.pose;

// 	px_now = pos_now.pose.position.x;
//     py_now = pos_now.pose.position.y;
//     pz_now = pos_now.pose.position.z;

//     vx_now = vel_now.twist.linear.x;
//     vy_now = vel_now.twist.linear.y;
//     vz_now = vel_now.twist.linear.z;

// 	tf::Quaternion q1 = {acc_body.orientation.x, acc_body.orientation.y, acc_body.orientation.z, acc_body.orientation.w};
// 	tf::Matrix3x3 r1;
// 	r1.setRotation(q1);

// 	// cout << "R is " << acc_body.orientation.z << endl;


// 	// ROS_INFO("acc_body.linear_acceleration.x is %f",acc_body.linear_acceleration.x);
// 	// ROS_INFO("acc_body.linear_acceleration.y is %f",acc_body.linear_acceleration.y);
// 	// ROS_INFO("acc_body.linear_acceleration.z is %f",acc_body.linear_acceleration.z);
// 	// cout << "R is " << r1[0][0] << endl;

// 	ax_now = (r1[0][0]*acc_body.linear_acceleration.x+r1[0][1]*acc_body.linear_acceleration.y+r1[0][2]*acc_body.linear_acceleration.z);
// 	ay_now = (r1[1][0]*acc_body.linear_acceleration.x+r1[1][1]*acc_body.linear_acceleration.y+r1[1][2]*acc_body.linear_acceleration.z);
// 	az_now = (r1[2][0]*acc_body.linear_acceleration.x+r1[2][1]*acc_body.linear_acceleration.y+r1[2][2]*acc_body.linear_acceleration.z);

// 	// ax_now = acc_now.linear_acceleration.x;
//     // ay_now = acc_now.linear_acceleration.y;
//     // az_now = acc_now.linear_acceleration.z;

//     // ROS_INFO("sub loop px_now is %f",px_now);
// 	// ROS_INFO("sub loop vx_now is %f",vx_now);
//     // ROS_INFO("sub loop ax_now is %f",ax_now);
//     // ROS_INFO("sub loop az_now is %f",az_now);

// }

// mixmsg订阅回调函数
void callback(const udp_pkg::PositionVelocityAccel::ConstPtr &mix_msg)
{

	spin_flag = true;

	geometry_msgs::PoseStamped pos_now;
	geometry_msgs::TwistStamped vel_now;
	sensor_msgs::Imu acc_body;

	pos_now.pose.position.x = mix_msg->x_pos;
	pos_now.pose.position.y = mix_msg->y_pos;
	pos_now.pose.position.z = mix_msg->z_pos;

	vel_now.twist.linear.x = mix_msg->x_vel;
	vel_now.twist.linear.y = mix_msg->y_vel;
	vel_now.twist.linear.z = mix_msg->z_vel;

	acc_body.linear_acceleration.x = mix_msg->x_acc;
	acc_body.linear_acceleration.y = mix_msg->y_acc;
	acc_body.linear_acceleration.z = mix_msg->z_acc;

	acc_body.orientation.x = 1;
	acc_body.orientation.y = 0;
	acc_body.orientation.z = 0;
	acc_body.orientation.w = 0;
	// geometry_msgs::PoseStamped pos_now = *pos_msg;
	// geometry_msgs::TwistStamped vel_now = *vel_msg;
	// sensor_msgs::Imu acc_body = *acc_msg;

	temp_now.header.stamp = ros::Time::now();
	temp_now.pose = pos_now.pose;
	cbTime = mix_msg->stamp;

	px_now = pos_now.pose.position.x;
	py_now = pos_now.pose.position.y;
	pz_now = pos_now.pose.position.z;

	vx_now = vel_now.twist.linear.x;
	vy_now = vel_now.twist.linear.y;
	vz_now = vel_now.twist.linear.z;

	tf::Quaternion q1 = {acc_body.orientation.x, acc_body.orientation.y, acc_body.orientation.z, acc_body.orientation.w};
	std::cout << "now Quaternion is " << acc_body.orientation << std::endl;
	tf::Matrix3x3 r1;
	r1.setRotation(q1);

	// cout << "R is " << acc_body.orientation.z << endl;

	// ROS_INFO("acc_body.linear_acceleration.x is %f",acc_body.linear_acceleration.x);
	// ROS_INFO("acc_body.linear_acceleration.y is %f",acc_body.linear_acceleration.y);
	// ROS_INFO("acc_body.linear_acceleration.z is %f",acc_body.linear_acceleration.z);
	// cout << "R is " << r1[0][0] << endl;

	ax_now = (r1[0][0] * acc_body.linear_acceleration.x + r1[0][1] * acc_body.linear_acceleration.y + r1[0][2] * acc_body.linear_acceleration.z);
	ay_now = (r1[1][0] * acc_body.linear_acceleration.x + r1[1][1] * acc_body.linear_acceleration.y + r1[1][2] * acc_body.linear_acceleration.z);
	az_now = (r1[2][0] * acc_body.linear_acceleration.x + r1[2][1] * acc_body.linear_acceleration.y + r1[2][2] * acc_body.linear_acceleration.z);

	// ax_now = acc_now.linear_acceleration.x;
	// ay_now = acc_now.linear_acceleration.y;
	// az_now = acc_now.linear_acceleration.z;

	// ROS_INFO("sub loop px_now is %f",px_now);
	// ROS_INFO("sub loop vx_now is %f",vx_now);
	// ROS_INFO("sub loop ax_now is %f",ax_now);
	// ROS_INFO("sub loop az_now is %f",az_now);
}

int main(int argc , char **argv)
{
    ros::init(argc,argv,"state_est");
    ros::NodeHandle n;

    // ros::Time pos_ros_last_time = ros::Time::now();
    ros::Time vel_ros_last_time = ros::Time::now();
    ros::Time vel_ros_now_time = ros::Time::now();

    // double pos_last_time = pos_ros_last_time.toSec();
    double vel_last_time = vel_ros_last_time.toSec();
    double vel_now_time = vel_ros_now_time.toSec();

    // double pos_total_time = 0;
    double vel_total_time = 0;

    // double pos_time_gap = 0;
    double vel_time_gap = 0;

    float pos_x_hat_now;
    float pos_y_hat_now;
    float pos_z_hat_now;

    float pos_x_hat_next;
    float pos_y_hat_next;
    float pos_z_hat_next;
   
   
    // arma::mat A = { {1,2},{2,6} };
	// arma::mat B = { {3,4},{4,3} };
    // arma::mat C = A * B;
    // std::cout << A << std::endl;
    // std::cout << B << std::endl;
    // std::cout << C << std::endl;
    // std::cin.get();

    // // ros::Subscriber pose = n.subscriber("/mavros/local_position/pose",pose_call_back,1);
	// // 时间同步订阅
	// message_filters::Subscriber<geometry_msgs::PoseStamped> pos_sub(n,"/mavros/vision_pose/pose",1);  //这个订阅话题改为受攻击的传感器话题  应该是/mavros/vision_pose/pose
	// message_filters::Subscriber<geometry_msgs::TwistStamped> vel_sub(n,"/mavros/local_position/velocity_local",1);
	// message_filters::Subscriber<sensor_msgs::Imu> acc_sub(n,"/mavros/imu/data",1);

	// typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,geometry_msgs::TwistStamped,sensor_msgs::Imu> MySyncPolicy;
	// message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), pos_sub, vel_sub, acc_sub);
	// // message_filters::TimeSynchronizer<geometry_msgs::PoseStamped,geometry_msgs::TwistStamped,sensor_msgs::Imu>sync(pos_sub,vel_sub,acc_sub,10);
	// sync.registerCallback(boost::bind(&callback,_1,_2,_3));

	// pos,vel,acc新msg订阅
	ros::Subscriber mixposition_sub = n.subscribe("/leader/information",1,callback);

	// 发布滤波数据 暂不需要
	// ros::Publisher pos_kalman_pub = n.advertise<double>("/result/pos_kalman",1);

	// 创建发布者
	// ros::Publisher filter_data_pub = n.advertise<geometry_msgs::PoseStamped>("/leader/rkf/pos",1);
	ros::Publisher filter_data_pub = n.advertise<kalman_filter::Vector3Stamped>("/leader/rkf/pos", 1);
	// // x轴
	// 	// 传感器数据
	// ros::Publisher pos_x_miss_sensor_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_x_miss_sensor",1);
	// ros::Publisher pos_x_fdi_sensor_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_x_fdi_sensor",1);

	// 	// 真值
	// ros::Publisher pos_x_true_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_x_true",1);

	// 	// 测量丢失
	// ros::Publisher pos_x_miss_kalman_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_x_miss_kalman",1);
	// ros::Publisher pos_x_miss_robust_kalman_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_x_miss_robust_kalman",1);

	// 	// FDI
	// ros::Publisher pos_x_fdi_kalman_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_x_fdi_kalman",1);
	// ros::Publisher pos_x_fdi_robust_kalman_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_x_fdi_robust_kalman",1);

	// // y轴
	// 	// 传感器数据
	// ros::Publisher pos_y_miss_sensor_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_y_miss_sensor",1);
	// ros::Publisher pos_y_fdi_sensor_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_y_fdi_sensor",1);

	// 	// 真值
	// ros::Publisher pos_y_true_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_y_true",1);

	// 	// 测量丢失
	// ros::Publisher pos_y_miss_kalman_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_y_miss_kalman",1);
	// ros::Publisher pos_y_miss_robust_kalman_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_y_miss_robust_kalman",1);

	// 	// FDI
	// ros::Publisher pos_y_fdi_kalman_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_y_fdi_kalman",1);
	// ros::Publisher pos_y_fdi_robust_kalman_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_y_fdi_robust_kalman",1);

	// // z轴
	// 	// 传感器数据
	// ros::Publisher pos_z_miss_sensor_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_z_miss_sensor",1);
	// ros::Publisher pos_z_fdi_sensor_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_z_fdi_sensor",1);

	// 	// 真值
	// ros::Publisher pos_z_true_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_z_true",1);

	// 	// 测量丢失
	// ros::Publisher pos_z_miss_kalman_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_z_miss_kalman",1);
	// ros::Publisher pos_z_miss_robust_kalman_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_z_miss_robust_kalman",1);

	// 	// FDI
	// ros::Publisher pos_z_fdi_kalman_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_z_fdi_kalman",1);
	// ros::Publisher pos_z_fdi_robust_kalman_pub = n.advertise<std_msgs::Float64>("/state_estimation/pos_z_fdi_robust_kalman",1);


	// 时间不同步订阅
    // ros::Subscriber pos = n.subscribe("/mavros/local_position/pose",1,pos_call_back);
    // ros::Subscriber vel = n.subscribe("/mavros/local_position/velocity_local",1,vel_call_back);
    // ros::Subscriber acc = n.subscribe("/mavros/imu/data_raw",1,acc_call_back);
    ros::Rate loop_rate(20);

    // 预定义数组大小
    int array_size = 100000;

    // 定义数据类型
	// x 轴数据
    DoubleArray pos_x_array;
	DoubleArray vel_x_array;
	DoubleArray pre_acc_x_array;
	DoubleArray acc_x_array;
	DataArray data_x;

    pos_x_array.array = new double[array_size];
    vel_x_array.array = new double[array_size];
    pre_acc_x_array.array = new double[array_size];
    acc_x_array.array = new double[array_size];

	data_x.time = new double[array_size];
	data_x.pos = new double[array_size];
	data_x.vel = new double[array_size];
	data_x.acc = new double[array_size];
	data_x.timegap = new double[array_size];

	// y轴数据
	DoubleArray pos_y_array;
	DoubleArray vel_y_array;
	DoubleArray pre_acc_y_array;
	DoubleArray acc_y_array;
	DataArray data_y;

    pos_y_array.array = new double[array_size];
    vel_y_array.array = new double[array_size];
    pre_acc_y_array.array = new double[array_size];
    acc_y_array.array = new double[array_size];

	data_y.time = new double[array_size];
	data_y.pos = new double[array_size];
	data_y.vel = new double[array_size];
	data_y.acc = new double[array_size];
	data_y.timegap = new double[array_size];

	// z轴数据
	DoubleArray pos_z_array;
	DoubleArray vel_z_array;
	DoubleArray pre_acc_z_array;
	DoubleArray acc_z_array;
	DataArray data_z;

    pos_z_array.array = new double[array_size];
    vel_z_array.array = new double[array_size];
    pre_acc_z_array.array = new double[array_size];
    acc_z_array.array = new double[array_size];

	data_z.time = new double[array_size];
	data_z.pos = new double[array_size];
	data_z.vel = new double[array_size];
	data_z.acc = new double[array_size];
	data_z.timegap = new double[array_size];

	// 时间数据
	TimeArray datatime;

    datatime.time = new double[array_size];
    datatime.timegap = new double[array_size];



	// data.time = datatime.time;
	// data.pos = pos_x_array.array;
	// data.vel = vel_x_array.array;
	// data.acc = acc_x_array.array;
	// data.timegap = datatime.timegap;

	// 创建状态估计数据
	// x轴数据
	mat x_hat = zeros(2, 1);
	x_hat(0, 0) = data_x.pos[0];
	x_hat(1, 0) = data_x.vel[0];
	mat P_x = zeros(2, 2);
	mat R_x = zeros(2, 2);
	mat Q_x = zeros(2, 2);
	R_x(0, 0) = sensor_stddev[0] * sensor_stddev[0];  	//传感器噪声协方差矩阵
	R_x(1, 1) = sensor_stddev[1] * sensor_stddev[1];

	Q_x(0, 0) = process_stddev[0] * process_stddev[0];	//过程噪声协方差矩阵
	Q_x(1, 1) = process_stddev[1] * process_stddev[1];

	KalmanData kalmandata_x;
	kalmandata_x.x = x_hat;
	kalmandata_x.P = P_x;
	
	KalmanData kalmanmissdata_x;
	kalmanmissdata_x.x = x_hat;
	kalmanmissdata_x.P = P_x;

	KalmanData kalmanrobustdata_x;
	kalmanrobustdata_x.x = x_hat;
	kalmanrobustdata_x.P = P_x;
	std::cout << "Initial xhat:" << x_hat << std::endl;
	std::cout << "Initial Px:" << P_x << std::endl;

	KalmanData kalmanfusiondata_x;
	kalmanfusiondata_x.x = x_hat;
	kalmanfusiondata_x.P = P_x;

	KalmanData kalmanfdidata_x;
	kalmanfdidata_x.x = x_hat;
	kalmanfdidata_x.P = P_x;

	KalmanData norm_kalman_fdidata_x;
	norm_kalman_fdidata_x.x = x_hat;
	norm_kalman_fdidata_x.P = P_x;

	mat mid_term_x = zeros(2,2);


	KalmanData  data_x_miss_predict = { x_hat , P_x};
	KalmanData	data_x_robust_predict = { x_hat , P_x };
	KalmanData	data_x_fdi_predict = { x_hat , P_x };

	mat Expectance_x = x_hat;

	// y轴数据
	mat y_hat = zeros(2, 1);
	y_hat(0, 0) = data_y.pos[0];
	y_hat(1, 0) = data_y.vel[0];
	mat P_y = zeros(2, 2);
	mat R_y = zeros(2, 2);
	mat Q_y = zeros(2, 2);
	R_y(0, 0) = sensor_stddev[0] * sensor_stddev[0];
	R_y(1, 1) = sensor_stddev[1] * sensor_stddev[1];

	Q_y(0, 0) = process_stddev[0] * process_stddev[0];
	Q_y(1, 1) = process_stddev[1] * process_stddev[1];

	KalmanData kalmandata_y;
	kalmandata_y.x = y_hat;
	kalmandata_y.P = P_y;
	
	KalmanData kalmanmissdata_y;
	kalmanmissdata_y.x = y_hat;
	kalmanmissdata_y.P = P_y;

	KalmanData kalmanrobustdata_y;
	kalmanrobustdata_y.x = y_hat;
	kalmanrobustdata_y.P = P_y;

	KalmanData kalmanfusiondata_y;
	kalmanfusiondata_y.x = y_hat;
	kalmanfusiondata_y.P = P_y;

	KalmanData kalmanfdidata_y;
	kalmanfdidata_y.x = y_hat;
	kalmanfdidata_y.P = P_y;

	KalmanData norm_kalman_fdidata_y;
	norm_kalman_fdidata_y.x = y_hat;
	norm_kalman_fdidata_y.P = P_y;

	mat mid_term_y = zeros(2,2);


	KalmanData  data_y_miss_predict = { y_hat , P_y};
	KalmanData	data_y_robust_predict = { y_hat , P_y };
	KalmanData	data_y_fdi_predict = { y_hat , P_y };

	mat Expectance_y = y_hat;
	
	// z轴数据
	mat z_hat = zeros(2, 1);
	z_hat(0, 0) = data_z.pos[0];
	z_hat(1, 0) = data_z.vel[0];
	mat P_z = zeros(2, 2);
	mat R_z = zeros(2, 2);
	mat Q_z = zeros(2, 2);
	R_z(0, 0) = sensor_stddev[0] * sensor_stddev[0];
	R_z(1, 1) = sensor_stddev[1] * sensor_stddev[1];

	Q_z(0, 0) = process_stddev[0] * process_stddev[0];
	Q_z(1, 1) = process_stddev[1] * process_stddev[1];

	KalmanData kalmandata_z;
	kalmandata_z.x = z_hat;
	kalmandata_z.P = P_z;
	
	KalmanData kalmanmissdata_z;
	kalmanmissdata_z.x = z_hat;
	kalmanmissdata_z.P = P_z;

	KalmanData kalmanrobustdata_z;
	kalmanrobustdata_z.x = z_hat;
	kalmanrobustdata_z.P = P_z;

	KalmanData kalmanfusiondata_z;
	kalmanfusiondata_z.x = z_hat;
	kalmanfusiondata_z.P = P_z;

	KalmanData kalmanfdidata_z;
	kalmanfdidata_z.x = z_hat;
	kalmanfdidata_z.P = P_z;

	KalmanData norm_kalman_fdidata_z;
	norm_kalman_fdidata_z.x = z_hat;
	norm_kalman_fdidata_z.P = P_z;

	mat mid_term_z = zeros(2,2);


	KalmanData  data_z_miss_predict = { z_hat , P_z};
	KalmanData	data_z_robust_predict = { z_hat , P_z };
	KalmanData	data_z_fdi_predict = { z_hat , P_z };

	mat Expectance_z = z_hat;

	// 定义测量丢失概率
	BernulliData miss_prob;
	miss_prob.mean = mat{ {0.6,0},{0,0.8} };

	miss_prob.var = miss_prob.mean * ( eye(2,2) - miss_prob.mean );

    int i = 0;
    int length = i+1;

    while(ros::ok())
    {
		


        // 更新位置、速度、加速度、时间、标志位
		ros::spinOnce();

		if ( i == 0) //初始化
		{
			// 创建状态估计数据
			// x轴数据
			mat x_hat = zeros(2, 1);
			x_hat(0, 0) = px_now;
			x_hat(1, 0) = vx_now;
			mat P_x = zeros(2, 2);
			mat R_x = zeros(2, 2);
			mat Q_x = zeros(2, 2);
			R_x(0, 0) = sensor_stddev[0] * sensor_stddev[0];
			R_x(1, 1) = sensor_stddev[1] * sensor_stddev[1];

			Q_x(0, 0) = process_stddev[0] * process_stddev[0];
			Q_x(1, 1) = process_stddev[1] * process_stddev[1];

			KalmanData kalmandata_x;
			kalmandata_x.x = x_hat;
			kalmandata_x.P = P_x;
			
			KalmanData kalmanmissdata_x;
			kalmanmissdata_x.x = x_hat;
			kalmanmissdata_x.P = P_x;

			KalmanData kalmanrobustdata_x;
			kalmanrobustdata_x.x = x_hat;
			kalmanrobustdata_x.P = P_x;

			KalmanData kalmanfusiondata_x;
			kalmanfusiondata_x.x = x_hat;
			kalmanfusiondata_x.P = P_x;

			KalmanData kalmanfdidata_x;
			kalmanfdidata_x.x = x_hat;
			kalmanfdidata_x.P = P_x;

			KalmanData norm_kalman_fdidata_x;
			norm_kalman_fdidata_x.x = x_hat;
			norm_kalman_fdidata_x.P = P_x;

			mat mid_term_x = zeros(2,2);


			KalmanData  data_x_miss_predict = { x_hat , P_x};
			KalmanData	data_x_robust_predict = { x_hat , P_x };
			KalmanData	data_x_fdi_predict = { x_hat , P_x };

			mat Expectance_x = x_hat;

			// y轴数据
			mat y_hat = zeros(2, 1);
			y_hat(0, 0) = py_now;
			y_hat(1, 0) = vy_now;
			mat P_y = zeros(2, 2);
			mat R_y = zeros(2, 2);
			mat Q_y = zeros(2, 2);
			R_y(0, 0) = sensor_stddev[0] * sensor_stddev[0];
			R_y(1, 1) = sensor_stddev[1] * sensor_stddev[1];

			Q_y(0, 0) = process_stddev[0] * process_stddev[0];
			Q_y(1, 1) = process_stddev[1] * process_stddev[1];

			KalmanData kalmandata_y;
			kalmandata_y.x = y_hat;
			kalmandata_y.P = P_y;
			
			KalmanData kalmanmissdata_y;
			kalmanmissdata_y.x = y_hat;
			kalmanmissdata_y.P = P_y;

			KalmanData kalmanrobustdata_y;
			kalmanrobustdata_y.x = y_hat;
			kalmanrobustdata_y.P = P_y;

			KalmanData kalmanfusiondata_y;
			kalmanfusiondata_y.x = y_hat;
			kalmanfusiondata_y.P = P_y;

			KalmanData kalmanfdidata_y;
			kalmanfdidata_y.x = y_hat;
			kalmanfdidata_y.P = P_y;

			KalmanData norm_kalman_fdidata_y;
			norm_kalman_fdidata_y.x = y_hat;
			norm_kalman_fdidata_y.P = P_y;

			mat mid_term_y = zeros(2,2);


			KalmanData  data_y_miss_predict = { y_hat , P_y};
			KalmanData	data_y_robust_predict = { y_hat , P_y };
			KalmanData	data_y_fdi_predict = { y_hat , P_y };

			mat Expectance_y = y_hat;
			
			// z轴数据
			mat z_hat = zeros(2, 1);
			z_hat(0, 0) = pz_now;
			z_hat(1, 0) = vz_now;
			mat P_z = zeros(2, 2);
			mat R_z = zeros(2, 2);
			mat Q_z = zeros(2, 2);
			R_z(0, 0) = sensor_stddev[0] * sensor_stddev[0];
			R_z(1, 1) = sensor_stddev[1] * sensor_stddev[1];

			Q_z(0, 0) = process_stddev[0] * process_stddev[0];
			Q_z(1, 1) = process_stddev[1] * process_stddev[1];

			KalmanData kalmandata_z;
			kalmandata_z.x = z_hat;
			kalmandata_z.P = P_z;
			
			KalmanData kalmanmissdata_z;
			kalmanmissdata_z.x = z_hat;
			kalmanmissdata_z.P = P_z;

			KalmanData kalmanrobustdata_z;
			kalmanrobustdata_z.x = z_hat;
			kalmanrobustdata_z.P = P_z;

			KalmanData kalmanfusiondata_z;
			kalmanfusiondata_z.x = z_hat;
			kalmanfusiondata_z.P = P_z;

			KalmanData kalmanfdidata_z;
			kalmanfdidata_z.x = z_hat;
			kalmanfdidata_z.P = P_z;

			KalmanData norm_kalman_fdidata_z;
			norm_kalman_fdidata_z.x = z_hat;
			norm_kalman_fdidata_z.P = P_z;

			mat mid_term_z = zeros(2,2);


			KalmanData  data_z_miss_predict = { z_hat , P_z};
			KalmanData	data_z_robust_predict = { z_hat , P_z };
			KalmanData	data_z_fdi_predict = { z_hat , P_z };

			mat Expectance_z = z_hat;
		}

		// 只有同步的时候，更新
		if (spin_flag == true)
		{
			vel_ros_now_time = ros::Time::now();
			vel_now_time = vel_ros_now_time.toSec();
			vel_time_gap = vel_now_time - vel_last_time;
			vel_total_time += vel_time_gap;
			length = i + 1;

			// std::cout << "time_gap::"  << vel_time_gap << std::endl;

			std::cout << "Security Esitmation is Upadating!!" << std::endl;

			// 时间更新
			datatime.time[i] = vel_total_time;
			datatime.timegap[i] = vel_time_gap;

			//// 数据更新
			// x轴更新
			pos_x_array.array[i] = px_now;
			vel_x_array.array[i] = vx_now;
			pre_acc_x_array.array[i] = ax_now;
			acc_x_array.array[i] = pre_acc_x_array.array[i]; //x轴不需要考虑重力加速度
			pos_x_array.length = length;
			vel_x_array.length = length;
			pre_acc_x_array.length = length;
			acc_x_array.length = length;

			data_x.length = length;
			data_x.time[i] = datatime.time[i];
			data_x.pos[i] = pos_x_array.array[i];
			data_x.vel[i] = vel_x_array.array[i];
			data_x.acc[i] = acc_x_array.array[i];
			data_x.timegap[i] = datatime.timegap[i];

			// y轴更新
			pos_y_array.array[i] = py_now;
			vel_y_array.array[i] = vy_now;
			pre_acc_y_array.array[i] = ay_now;
			acc_y_array.array[i] = pre_acc_y_array.array[i]; //y轴不需要考虑重力加速度
			pos_y_array.length = length;
			vel_y_array.length = length;
			pre_acc_y_array.length = length;
			acc_y_array.length = length;

			data_y.length = length;
			data_y.time[i] = datatime.time[i];
			data_y.pos[i] = pos_y_array.array[i];
			data_y.vel[i] = vel_y_array.array[i];
			data_y.acc[i] = acc_y_array.array[i];
			data_y.timegap[i] = datatime.timegap[i];

			
			// z轴更新
			pos_z_array.array[i] = pz_now;
			vel_z_array.array[i] = vz_now;
			pre_acc_z_array.array[i] = az_now;
			acc_z_array.array[i] = calculate_acc(pre_acc_z_array.array[i]); //z轴考虑重力加速度
			pos_z_array.length = length;
			vel_z_array.length = length;
			pre_acc_z_array.length = length;
			acc_z_array.length = length;

			data_z.length = length;
			data_z.time[i] = datatime.time[i];
			data_z.pos[i] = pos_z_array.array[i];
			data_z.vel[i] = vel_z_array.array[i];
			data_z.acc[i] = acc_z_array.array[i];
			data_z.timegap[i] = datatime.timegap[i];
			
			// std::cout << "data_z.time:" << datatime.time[i] << std::endl;
			// std::cout << "data_z.timegap:" << data_z.timegap[i] << std::endl;



			// std::cout << "datatime.timegap" << datatime.timegap[i] << std::endl;



			////创建传感器数据
			// x轴数据
			DataArray sensordata_x = data_x;
			// DataArray sensordata_x = SensorData(data_x);
			// DataArray miss_sensordata_x = MissSensorData(sensordata_x,miss_prob);
			// DataArray fdi_sensordata_x = FDISensorData(sensordata_x);

			// y轴数据
			DataArray sensordata_y = data_y;
			// DataArray sensordata_y = SensorData(data_y);
			// DataArray miss_sensordata_y = MissSensorData(sensordata_y,miss_prob);
			// DataArray fdi_sensordata_y = FDISensorData(sensordata_y);

			// z轴数据
			DataArray sensordata_z = data_z;
			// DataArray sensordata_z = SensorData(data_z);
			// DataArray miss_sensordata_z = MissSensorData(sensordata_z,miss_prob);
			// DataArray fdi_sensordata_z = FDISensorData(sensordata_z);



			//// 创建A、B、C矩阵
			// x轴
			mat A_x = createMatA(2,data_x.timegap[i]);
			mat B_x = createMatB(2,1, data_x.timegap[i]);
			mat C_x = createMatC(2);        

			// y轴
			mat A_y = createMatA(2,data_y.timegap[i]);
			mat B_y = createMatB(2,1, data_y.timegap[i]);
			mat C_y = createMatC(2);   

			// z轴
			mat A_z = createMatA(2,data_z.timegap[i]);
			mat B_z = createMatB(2,1, data_z.timegap[i]);
			mat C_z = createMatC(2);   


			// 创建输出矩阵
			// x轴
			mat output_x = createMaty(sensordata_x.pos[i], sensordata_x.vel[i]);
			// mat miss_output_x = createMaty(miss_sensordata_x.pos[i], miss_sensordata_x.vel[i]);
			// mat fdi_output_x = createMaty(fdi_sensordata_x.pos[i], fdi_sensordata_x.vel[i]);

			//y轴
			mat output_y = createMaty(sensordata_y.pos[i], sensordata_y.vel[i]);
			// mat miss_output_y = createMaty(miss_sensordata_y.pos[i], miss_sensordata_y.vel[i]);
			// mat fdi_output_y = createMaty(fdi_sensordata_y.pos[i], fdi_sensordata_y.vel[i]);

			// z轴 
			mat output_z = createMaty(sensordata_z.pos[i], sensordata_z.vel[i]);
			// mat miss_output_z = createMaty(miss_sensordata_z.pos[i], miss_sensordata_z.vel[i]);
			// mat fdi_output_z = createMaty(fdi_sensordata_z.pos[i], fdi_sensordata_z.vel[i]);

			// 滤波函数
			// x轴
			// std::cout << "kalmandata_x" << kalmandata_x.x << std::endl;
			kalmanrobustdata_x = KalmanMissRobustFilter(kalmanrobustdata_x, output_x, data_x.acc[i], A_x, B_x, C_x, Q_x, R_x, data_x.timegap[i], &data_x_robust_predict,0.5); //0.25适用于调节鲁棒卡尔曼滤波的参数
			// kalmandata_x = KalmanFilter(kalmandata_x, miss_output_x,data_x.acc[i], A_x, B_x, C_x, Q_x, R_x, data_x.timegap[i]);
			// kalmanmissdata_x = KalmanMissFilter(kalmanmissdata_x, miss_output_x, data_x.acc[i], A_x, B_x, C_x, Q_x, R_x, data_x.timegap[i],miss_prob ,  &mid_term_x , &data_x_miss_predict , &Expectance_x);
			// kalmanrobustdata_x = KalmanMissRobustFilter(kalmanrobustdata_x, miss_output_x, data_x.acc[i], A_x, B_x, C_x, Q_x, R_x, data_x.timegap[i], &data_x_robust_predict,0.25);
			// kalmanfusiondata_x.x = (0.3*kalmanmissdata_x.x + 0.7*kalmanrobustdata_x.x);

			// norm_kalman_fdidata_x = KalmanFilter(norm_kalman_fdidata_x, fdi_output_x, data_x.acc[i], A_x, B_x, C_x, Q_x, R_x, data_x.timegap[i]);
			// kalmanfdidata_x = KalmanFDIRobustFilter(kalmanfdidata_x, fdi_output_x, data_x.acc[i], A_x, B_x, C_x, Q_x, R_x, data_x.timegap[i], &data_x_fdi_predict,0.1);

			// y轴
			kalmanrobustdata_y = KalmanMissRobustFilter(kalmanrobustdata_y, output_y, data_y.acc[i], A_y, B_y, C_y, Q_y, R_y, data_y.timegap[i], &data_y_robust_predict,0.5); //0.25适用于调节鲁棒卡尔曼滤波的参数

			// kalmandata_y = KalmanFilter(kalmandata_y, miss_output_y,data_y.acc[i], A_y, B_y, C_y, Q_y, R_y, data_y.timegap[i]);
			// kalmanmissdata_y = KalmanMissFilter(kalmanmissdata_y, miss_output_y, data_y.acc[i], A_y, B_y, C_y, Q_y, R_y, data_y.timegap[i],miss_prob ,  &mid_term_y , &data_y_miss_predict , &Expectance_y);
			// kalmanrobustdata_y = KalmanMissRobustFilter(kalmanrobustdata_y, miss_output_y, data_y.acc[i], A_y, B_y, C_y, Q_y, R_y, data_y.timegap[i], &data_y_robust_predict,0.25);
			// kalmanfusiondata_y.x = (0.3*kalmanmissdata_y.x + 0.7*kalmanrobustdata_y.x);

			// norm_kalman_fdidata_y = KalmanFilter(norm_kalman_fdidata_y, fdi_output_y, data_y.acc[i], A_y, B_y, C_y, Q_y, R_y, data_y.timegap[i]);
			// kalmanfdidata_y = KalmanFDIRobustFilter(kalmanfdidata_y, fdi_output_y, data_y.acc[i], A_y, B_y, C_y, Q_y, R_y, data_y.timegap[i], &data_y_fdi_predict,0.1);

			// z轴
			kalmanrobustdata_z = KalmanMissRobustFilter(kalmanrobustdata_z, output_z, data_z.acc[i], A_z, B_z, C_z, Q_z, R_z, data_z.timegap[i], &data_z_robust_predict,0.5); //0.25适用于调节鲁棒卡尔曼滤波的参数

			// kalmandata_z = KalmanFilter(kalmandata_z, miss_output_z,data_z.acc[i], A_z, B_z, C_z, Q_z, R_z, data_z.timegap[i]);
			// kalmanmissdata_z = KalmanMissFilter(kalmanmissdata_z, miss_output_z, data_z.acc[i], A_z, B_z, C_z, Q_z, R_z, data_z.timegap[i],miss_prob ,  &mid_term_z , &data_z_miss_predict , &Expectance_z);
			// kalmanrobustdata_z = KalmanMissRobustFilter(kalmanrobustdata_z, miss_output_z, data_z.acc[i], A_z, B_z, C_z, Q_z, R_z, data_z.timegap[i], &data_z_robust_predict,0.25);
			// kalmanfusiondata_z.x = (0.3*kalmanmissdata_z.x + 0.7*kalmanrobustdata_z.x);

			// norm_kalman_fdidata_z = KalmanFilter(norm_kalman_fdidata_z, fdi_output_z, data_z.acc[i], A_z, B_z, C_z, Q_z, R_z, data_z.timegap[i]);
			// kalmanfdidata_z = KalmanFDIRobustFilter(kalmanfdidata_z, fdi_output_z, data_z.acc[i], A_z, B_z, C_z, Q_z, R_z, data_z.timegap[i], &data_z_fdi_predict,0.1);

			// 定义发布信息
			geometry_msgs::PoseStamped filter_data;
			filter_data.header = temp_now.header;
			filter_data.pose.position.x = kalmanrobustdata_x.x(0);
			filter_data.pose.position.y = kalmanrobustdata_y.x(0);
			filter_data.pose.position.z = kalmanrobustdata_z.x(0);
			filter_data.pose.orientation = temp_now.pose.orientation;

			kalman_filter::Vector3Stamped rkf_data;
			rkf_data.time = cbTime;
			rkf_data.x = kalmanrobustdata_x.x(0);
			rkf_data.y = kalmanrobustdata_y.x(0);
			rkf_data.z = kalmanrobustdata_z.x(0);
			filter_data_pub.publish(rkf_data);

			std::cout <<  filter_data << std::endl;

			// //x轴
			// std_msgs::Float64 pos_x_true;
			// std_msgs::Float64 pos_x_miss_sensor;
			// std_msgs::Float64 pos_x_fdi_sensor;
			// std_msgs::Float64 pos_x_miss_kalman;
			// std_msgs::Float64 pos_x_miss_robust_kalman;
			// std_msgs::Float64 pos_x_fdi_kalman;
			// std_msgs::Float64 pos_x_fdi_robust_kalman;

			// pos_x_true.data = data_x.pos[i];

			// pos_x_miss_sensor.data = miss_sensordata_x.pos[i];
			// pos_x_fdi_sensor.data = fdi_sensordata_x.pos[i];

			// pos_x_miss_kalman.data = kalmandata_x.x(0,0);
			// pos_x_miss_robust_kalman.data =  kalmanfusiondata_x.x(0,0);

			// pos_x_fdi_kalman.data = norm_kalman_fdidata_x.x(0,0);
			// pos_x_fdi_robust_kalman.data = kalmanfdidata_x.x(0,0);

			// // y轴
			// std_msgs::Float64 pos_y_true;
			// std_msgs::Float64 pos_y_miss_sensor;
			// std_msgs::Float64 pos_y_fdi_sensor;
			// std_msgs::Float64 pos_y_miss_kalman;
			// std_msgs::Float64 pos_y_miss_robust_kalman;
			// std_msgs::Float64 pos_y_fdi_kalman;
			// std_msgs::Float64 pos_y_fdi_robust_kalman;

			// pos_y_true.data = data_y.pos[i];

			// pos_y_miss_sensor.data = miss_sensordata_y.pos[i];
			// pos_y_fdi_sensor.data = fdi_sensordata_y.pos[i];

			// pos_y_miss_kalman.data = kalmandata_y.x(0,0);
			// pos_y_miss_robust_kalman.data =  kalmanfusiondata_y.x(0,0);

			// pos_y_fdi_kalman.data = norm_kalman_fdidata_y.x(0,0);
			// pos_y_fdi_robust_kalman.data = kalmanfdidata_y.x(0,0);

			// // z轴
			// std_msgs::Float64 pos_z_true;
			// std_msgs::Float64 pos_z_miss_sensor;
			// std_msgs::Float64 pos_z_fdi_sensor;
			// std_msgs::Float64 pos_z_miss_kalman;
			// std_msgs::Float64 pos_z_miss_robust_kalman;
			// std_msgs::Float64 pos_z_fdi_kalman;
			// std_msgs::Float64 pos_z_fdi_robust_kalman;

			// pos_z_true.data = data_z.pos[i];

			// pos_z_miss_sensor.data = miss_sensordata_z.pos[i];
			// pos_z_fdi_sensor.data = fdi_sensordata_z.pos[i];

			// pos_z_miss_kalman.data = kalmandata_z.x(0,0);
			// pos_z_miss_robust_kalman.data =  kalmanfusiondata_z.x(0,0);

			// pos_z_fdi_kalman.data = norm_kalman_fdidata_z.x(0,0);
			// pos_z_fdi_robust_kalman.data = kalmanfdidata_z.x(0,0);


			// 发布信息
			// // x轴
			// pos_x_true_pub.publish(pos_x_true);

			// pos_x_miss_sensor_pub.publish(pos_x_miss_sensor);
			// pos_x_fdi_sensor_pub.publish(pos_x_fdi_sensor);

			// pos_x_miss_kalman_pub.publish(pos_x_miss_kalman);
			// pos_x_miss_robust_kalman_pub.publish(pos_x_miss_robust_kalman);

			// pos_x_fdi_kalman_pub.publish(pos_x_fdi_kalman);
			// pos_x_fdi_robust_kalman_pub.publish(pos_x_fdi_robust_kalman);

			// // y轴
			// pos_y_true_pub.publish(pos_y_true);

			// pos_y_miss_sensor_pub.publish(pos_y_miss_sensor);
			// pos_y_fdi_sensor_pub.publish(pos_y_fdi_sensor);

			// pos_y_miss_kalman_pub.publish(pos_y_miss_kalman);
			// pos_y_miss_robust_kalman_pub.publish(pos_y_miss_robust_kalman);

			// pos_y_fdi_kalman_pub.publish(pos_y_fdi_kalman);
			// pos_y_fdi_robust_kalman_pub.publish(pos_y_fdi_robust_kalman);
			
			// // z轴
			// pos_z_true_pub.publish(pos_z_true);

			// pos_z_miss_sensor_pub.publish(pos_z_miss_sensor);
			// pos_z_fdi_sensor_pub.publish(pos_z_fdi_sensor);

			// pos_z_miss_kalman_pub.publish(pos_z_miss_kalman);
			// pos_z_miss_robust_kalman_pub.publish(pos_z_miss_robust_kalman);

			// pos_z_fdi_kalman_pub.publish(pos_z_fdi_kalman);
			// pos_z_fdi_robust_kalman_pub.publish(pos_z_fdi_robust_kalman);


			// ROS_INFO("true pos_x is %f",data_x.pos[i]);
			// ROS_INFO("kalmandata pos_x is %f",kalmanmissdata_x.x(0,0));
			// ROS_INFO("kalmissdata pos_x is %f",kalmanrobustdata_x.x(0,0));
			// ROS_INFO("norm_kalman_fdidata pos_x is %f",norm_kalman_fdidata_x.x(0,0));
			// ROS_INFO("kalmanfdidata pos_x is %f",kalmanfdidata_x.x(0,0));

			// ROS_INFO("main loop px_now is %f",data_x.pos[i]);
			// ROS_INFO("main loop vx_now is %f",data_x.vel[i]);
			// ROS_INFO("main loop ax_now is %f",data_x.acc[i]);
			// ROS_INFO("main loop az_now is %f",data.acc[i]);

			// ROS_INFO("vel_time_gap is %f \n",vel_time_gap);
			// ROS_INFO("vx_now is %f \n",vx_now);
			// ROS_INFO("vx_last is %f \n",vx_now);


			// 循环的终端
			vel_ros_last_time = ros::Time::now();
			vel_last_time = vel_ros_last_time.toSec();  
			i++;
		}

		spin_flag = false ;
        loop_rate.sleep();
    }



}
