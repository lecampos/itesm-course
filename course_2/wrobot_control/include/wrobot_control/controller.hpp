#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

/* add any C++ library need it */

#include <iostream>

namespace controller
{
class CController
{
  protected:
	/*Any variable or private members here */

	/// \brief control gains
	double K_gains[3];
  public:
	/// \brief The constructor
    /// \param[in] _K An array of gains
  	CController(double *_K);

  	/// \brief The destructor
    virtual ~CController(void);

    /// \brief A method for refresh a new reference
    /// \param[in] x_ref the x coordinate reference
    /// \param[in] y_ref the y coordinate reference 
    void new_reference(double x_ref, double y_ref);

    /// \brief A method for refresh the current robot position
    /// \param[in] x_pos the x coordinate robot position 
    /// \param[in] y_pos the y coordinate robot position 
    void new_worldPos(double x_pos, double y_pos);

    /// \brief A method for obtaining the new angular velocity
    /// \param[out] vw_left the angular velocity of the left wheel
    /// \param[out] vw_right the angular velocity of the right wheel
    void getNewVelocity( double &vw_left, double &vw_right);
};

} /* controller */

#endif // CONTROLLER_HPP_
