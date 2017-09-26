#include "wrobot_control/controller.hpp"


namespace controller
{

CController::CController(double *_K)
{
	this->K_gains[0] = _K[0];
	this->K_gains[1] = _K[1];
	this->K_gains[2] = _K[2];
}

CController::~CController(void)
{

}

void CController::new_reference(double x_ref, double y_ref)
{
	
}

void CController::new_worldPos(double x_pos, double y_pos)
{
	
}

void CController::getNewVelocity(double &vw_left, double &vw_right)
{
	
}

} /* controller */
