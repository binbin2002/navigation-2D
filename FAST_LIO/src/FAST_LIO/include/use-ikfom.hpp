#ifndef USE_IKFOM_H
#define USE_IKFOM_H

#include <IKFoM_toolkit/esekfom/esekfom.hpp>

typedef MTK::vect<3, double> vect3;//三维向量
typedef MTK::SO3<double> SO3;//三维旋转矩阵
typedef MTK::S2<double, 98090, 10000, 1> S2; //S2流形
typedef MTK::vect<1, double> vect1;//一维向量
typedef MTK::vect<2, double> vect2;//二维向量

MTK_BUILD_MANIFOLD(state_ikfom,//状态
((vect3, pos))//位置
((SO3, rot))//姿态
((SO3, offset_R_L_I))//激光雷达到IMU的旋转矩阵
((vect3, offset_T_L_I))//激光雷达到IMU的平移向量
((vect3, vel))//速度
((vect3, bg))//陀螺仪偏置
((vect3, ba))//加速度计偏置
((S2, grav))//重力加速度
);

MTK_BUILD_MANIFOLD(input_ikfom,//输入
((vect3, acc))//加速度
((vect3, gyro))//陀螺仪
);

MTK_BUILD_MANIFOLD(process_noise_ikfom,//过程噪声
((vect3, ng))//加速度噪声
((vect3, na))//加速度噪声
((vect3, nbg))//陀螺仪噪声
((vect3, nba))//陀螺仪噪声
);

MTK::get_cov<process_noise_ikfom>::type process_noise_cov()//过程噪声协方差
{
	MTK::get_cov<process_noise_ikfom>::type cov = MTK::get_cov<process_noise_ikfom>::type::Zero();
	MTK::setDiagonal<process_noise_ikfom, vect3, 0>(cov, &process_noise_ikfom::ng, 0.0001);// 0.03
	MTK::setDiagonal<process_noise_ikfom, vect3, 3>(cov, &process_noise_ikfom::na, 0.0001); // *dt 0.01 0.01 * dt * dt 0.05
	MTK::setDiagonal<process_noise_ikfom, vect3, 6>(cov, &process_noise_ikfom::nbg, 0.00001); // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
	MTK::setDiagonal<process_noise_ikfom, vect3, 9>(cov, &process_noise_ikfom::nba, 0.00001);   //0.001 0.05 0.0001/out 0.01
	return cov;
}

//double L_offset_to_I[3] = {0.04165, 0.02326, -0.0284}; // Avia 
//vect3 Lidar_offset_to_IMU(L_offset_to_I, 3);
Eigen::Matrix<double, 24, 1> get_f(state_ikfom &s, const input_ikfom &in)//系统动力学方程
{   // 24*1
	Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);//陀螺仪误差
	vect3 a_inertial = s.rot * (in.acc-s.ba); //加速度误差转移到世界坐标系
	for(int i = 0; i < 3; i++ ){
		res(i) = s.vel[i];
		res(i + 3) =  omega[i]; 
		res(i + 12) = a_inertial[i] + s.grav[i]; 
	}
	return res;
}

Eigen::Matrix<double, 24, 23> df_dx(state_ikfom &s, const input_ikfom &in)//状态雅可比矩阵
{
	Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
	cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
	vect3 acc_;
	in.acc.boxminus(acc_, s.ba);
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);
	cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix()*MTK::hat(acc_);//速度对旋转的偏导
	cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();//速度对加速度偏置的偏导
	Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
	Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
	s.S2_Mx(grav_matrix, vec, 21);
	cov.template block<3, 2>(12, 21) =  grav_matrix; //重力雅可比矩阵
	cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity(); //速度雅可比矩阵
	return cov;
}


Eigen::Matrix<double, 24, 12> df_dw(state_ikfom &s, const input_ikfom &in)//噪声雅可比矩阵
{
	Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
	cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();
	cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
	cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
	cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
	return cov;
}

vect3 SO3ToEuler(const SO3 &orient) 
{
	Eigen::Matrix<double, 3, 1> _ang;
	Eigen::Vector4d q_data = orient.coeffs().transpose();
	//scalar w=orient.coeffs[3], x=orient.coeffs[0], y=orient.coeffs[1], z=orient.coeffs[2];
	double sqw = q_data[3]*q_data[3];
	double sqx = q_data[0]*q_data[0];
	double sqy = q_data[1]*q_data[1];
	double sqz = q_data[2]*q_data[2];
	double unit = sqx + sqy + sqz + sqw; // if normalized is one, otherwise is correction factor
	double test = q_data[3]*q_data[1] - q_data[2]*q_data[0];

	if (test > 0.49999*unit) { // singularity at north pole
	
		_ang << 2 * std::atan2(q_data[0], q_data[3]), M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
	if (test < -0.49999*unit) { // singularity at south pole
		_ang << -2 * std::atan2(q_data[0], q_data[3]), -M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
		
	_ang <<
			std::atan2(2*q_data[0]*q_data[3]+2*q_data[1]*q_data[2] , -sqx - sqy + sqz + sqw),
			std::asin (2*test/unit),
			std::atan2(2*q_data[2]*q_data[3]+2*q_data[1]*q_data[0] , sqx - sqy - sqz + sqw);
	double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
	vect3 euler_ang(temp, 3);
		// euler_ang[0] = roll, euler_ang[1] = pitch, euler_ang[2] = yaw
	return euler_ang;
}

#endif