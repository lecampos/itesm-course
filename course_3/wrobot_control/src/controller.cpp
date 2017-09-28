#include "wrobot_control/controller.hpp"


namespace controller
{

CController::CController(double *_K)
{
	this->K_gains[0] = _K[0];
	this->K_gains[1] = _K[1];
	this->K_gains[2] = _K[2];

	this->x_pos = 0.0;
	this->y_pos = 0.0;
}

CController::~CController(void)
{

}

void CController::new_reference(double x_ref, double y_ref)
{
	this->x_ref = x_ref;
	this->y_ref = y_ref;
}

void CController::new_worldPos(double x_pos, double y_pos)
{
	this->x_pos_past = this->x_pos;
	this->y_pos_past = this->y_pos;

	this->x_pos = x_pos;
	this->y_pos = y_pos;
}

void CController::getNewVelocity(double &vw_left, double &vw_right)
{
	double phi_des;
	double phi;
	double error;
	double ang_vel;

	double tang_vel;
	double v_x;
	double v_y;

	double L = 0.3;
	double R = 0.05; //metros!

	phi_des = atan2( (this->y_ref - this->y_pos) , (this->x_ref - this->x_pos) );
	phi = atan2(this->y_pos, this->x_pos);
	error = phi_des - phi;

	ang_vel = this->K_gains[0]*error; // *P*ID

	v_x = (this->x_pos - this->x_pos_past) * 30;
	v_y = (this->y_pos - this->y_pos_past) * 30;

	tang_vel = sqrt(v_x*v_x + v_y*v_y);

	vw_left = (2*tang_vel + ang_vel * L)/(2*R);
	vw_right = (2*tang_vel - ang_vel * L)/(2*R);

}

} /* controller */
