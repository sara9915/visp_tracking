#ifndef __QUATERNIONVECTOR_H__
#define __QUATERNIONVECTOR_H__

/*!
  \file vpQuaternionVector.h
  \brief Class that consider the case of a quaternion and basic
   operations on it.
*/

#include <visp/vpConfig.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpColVector.h>


/*!
  \class vpQuaternionVector
  \ingroup RotTransformation
  
  \brief Defines a quaternion and its basic operations.
  This class allows to compute a quaternion from a rotation matrix
  using either vpQuaternionVector(const vpRotationMatrix &) contructor
  or buildFrom() method.
  It also defines common operations on a quaternion such as:
	- multiplication (scalar and quaternion)
	- addition
	- substraction.
  */
class VISP_EXPORT vpQuaternionVector : public vpColVector
{
private:        
  static const double minimum;
  double r[4];
public:

  vpQuaternionVector() ;    
  vpQuaternionVector(const double x, const double y, const double z,const double w) ;    
  vpQuaternionVector(const vpQuaternionVector &q);
  vpQuaternionVector(const vpRotationMatrix &R);

  void buildFrom(const vpRotationMatrix& R);

  void set(const double x, const double y, const double z,const double w) ;
  unsigned int size(){return 4;}
  //! returns x-component of the quaternion
  inline double x() const {return r[0];}
  //! returns y-component of the quaternion
  inline double y() const {return r[1];}
  //! returns z-component of the quaternion
  inline double z() const {return r[2];}
  //! returns w-component of the quaternion
  inline double w() const {return r[3];}

  vpQuaternionVector operator+( vpQuaternionVector &q)  ;
  vpQuaternionVector operator-( vpQuaternionVector &q)  ;
  vpQuaternionVector operator-()  ;
  vpQuaternionVector operator*(const double l) ;
  vpQuaternionVector operator* ( vpQuaternionVector &rq) ;
  vpQuaternionVector &operator=( vpQuaternionVector &q);
  
    
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */