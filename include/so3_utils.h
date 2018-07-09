#ifndef SO3_UTILS
#define SO3_UTILS

#include <Eigen/Dense>

class so3
{
public:
	static Eigen::Quaternion<double> R2q( Eigen::Matrix<double,3,3> R )
	{
		double R11 = R(0,0);
		double R12 = R(0,1);
		double R13 = R(0,2);
		double R21 = R(1,0);
		double R22 = R(1,1);
		double R23 = R(1,2);
		double R31 = R(2,0);
		double R32 = R(2,1);
		double R33 = R(2,2);
		
		Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Ones();
		T(1,1) = -1.0;
		T(1,2) = -1.0;
		T(2,0) = -1.0;
		T(2,2) = -1.0;
		T(3,0) = -1.0;
		T(3,1) = -1.0;
		
		Eigen::Matrix<double,4,1> diag;
		diag(0,0) = R11;
		diag(1,0) = R22;
		diag(2,0) = R33;
		diag(3,0) = 1.0;
		
		Eigen::Matrix<double,4,1> sq = 0.25*(T*diag);
		double q0 = sqrt(sq(0,0));
		double q1 = sqrt(sq(1,0));
		double q2 = sqrt(sq(2,0));
		double q3 = sqrt(sq(3,0));
		
		if( (q0 >= q1) && (q0 >= q2) && (q0 >= q3) )
		{
		    q1 = copysign(q1, R32-R23);
		    q2 = copysign(q2, R13-R31);
		    q3 = copysign(q3, R21-R12);
		}
		else if( (q1 >= q0) && (q1 >= q2) && (q1 >= q3) )
		{
		    q0 = copysign(q0, R32-R23);
		    q2 = copysign(q2, R21+R12);
		    q3 = copysign(q3, R13+R31);
		}
		else if( (q2 >= q0) && (q2 >= q1) && (q2 >= q3) )
		{
		    q0 = copysign(q0, R13-R31);
		    q1 = copysign(q1, R21+R12);
		    q3 = copysign(q3, R32+R23);
		}
		else if( (q3 >= q0) && (q3 >= q1) && (q3 >= q2) )
		{
		    q0 = copysign(q0, R21-R12);
		    q1 = copysign(q1, R31+R13);
		    q2 = copysign(q2, R32+R23);
		}
		
		Eigen::Quaternion<double> q;
		q.w() = q0;
		q.x() = q1;
		q.y() = q2;
		q.z() = q3;
		
		return q;
	}
	static Eigen::Matrix<double,3,1> R2rpy( Eigen::Matrix<double,3,3> R )
	{
		double r = atan2(R(2,1), R(2,2));
		double p = atan2(-R(2,0), sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
		double y = atan2(R(1,0), R(0,0));
		
		Eigen::Matrix<double,3,1> euler;
		euler(0,0) = r;
		euler(1,0) = p;
		euler(2,0) = y;
		
		return euler;
	}
	static Eigen::Matrix<double,3,3> rpy2R( double phi, double theta, double psi )
	{
		Eigen::Matrix<double,3,3> R;
		R(0,0) = cos(psi)*cos(theta);
		R(0,1) = cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi);
		R(0,2) = sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
		R(1,0) = cos(theta)*sin(psi);
		R(1,1) = cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta);
		R(1,2) = cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);
		R(2,0) = -sin(theta);
		R(2,1) = cos(theta)*sin(phi);
		R(2,2) = cos(phi)*cos(theta);
		return R;
	}
	static Eigen::Matrix<double,3,3> q2R( double w, double x, double y, double z )
	{
	    double q0 = w;
	    double q1 = x;
	    double q2 = y;
	    double q3 = z;
	    Eigen::Matrix<double,3,3> R;
	    R(0,0) = pow(q0,2)+pow(q1,2)-pow(q2,2)-pow(q3,2);
	    R(0,1) = 2*(q1*q2-q0*q3);
	    R(0,2) = 2*(q1*q3+q0*q2);
	    R(1,0) = 2*(q1*q2+q0*q3);
	    R(1,1) = pow(q0,2)-pow(q1,2)+pow(q2,2)-pow(q3,2);
	    R(1,2) = 2*(q2*q3-q0*q1);
	    R(2,0) = 2*(q1*q3-q0*q2);
	    R(2,1) = 2*(q2*q3+q0*q1);
	    R(2,2) = pow(q0,2)-pow(q1,2)-pow(q2,2)+pow(q3,2);
	    return R;
	}

};

#endif
