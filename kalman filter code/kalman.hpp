#ifndef __OPENCV_TRACKING_HPP__
#define __OPENCV_TRACKING_HPP__

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

namespace cv
{

//! @addtogroup video_track
//! @{

enum { OPTFLOW_USE_INITIAL_FLOW     = 4,
       OPTFLOW_LK_GET_MIN_EIGENVALS = 8,
       OPTFLOW_FARNEBACK_GAUSSIAN   = 256
     };

/** @brief Kalman filter class.
The class implements a standard Kalman filter <http://en.wikipedia.org/wiki/Kalman_filter>,
@cite Welch95 . However, you can modify transitionMatrix, controlMatrix, and measurementMatrix to get
an extended Kalman filter functionality. See the OpenCV sample kalman.cpp.
@note
-   An example using the standard Kalman filter can be found at
    opencv_source_code/samples/cpp/kalman.cpp
 */
class CV_EXPORTS_W KalmanFilter
{
public:
    /** @brief The constructors.
    @note In C API when CvKalman\* kalmanFilter structure is not needed anymore, it should be released
    with cvReleaseKalman(&kalmanFilter)
     */
    CV_WRAP KalmanFilter();
    /** @overload
    @param dynamParams Dimensionality of the state.
    @param measureParams Dimensionality of the measurement.
    @param controlParams Dimensionality of the control vector.
    @param type Type of the created matrices that should be CV_32F or CV_64F.
    */
    CV_WRAP KalmanFilter( int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F );

    /** @brief Re-initializes Kalman filter. The previous content is destroyed.
    @param dynamParams Dimensionality of the state.
    @param measureParams Dimensionality of the measurement.
    @param controlParams Dimensionality of the control vector.
    @param type Type of the created matrices that should be CV_32F or CV_64F.
     */
    void init( int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F );

    /** @brief Computes a predicted state.
    @param control The optional input control
     */
    CV_WRAP const Mat& predict( const Mat& control = Mat() );

    /** @brief Updates the predicted state from the measurement.
    @param measurement The measured system parameters
     */
    CV_WRAP const Mat& correct( const Mat& measurement );

    CV_PROP_RW Mat statePre;           //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
    CV_PROP_RW Mat statePost;          //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
    CV_PROP_RW Mat transitionMatrix;   //!< state transition matrix (A)
    CV_PROP_RW Mat controlMatrix;      //!< control matrix (B) (not used if there is no control)
    CV_PROP_RW Mat measurementMatrix;  //!< measurement matrix (H)
    CV_PROP_RW Mat processNoiseCov;    //!< process noise covariance matrix (Q)
    CV_PROP_RW Mat measurementNoiseCov;//!< measurement noise covariance matrix (R)
    CV_PROP_RW Mat errorCovPre;        //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
    CV_PROP_RW Mat gain;               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
    CV_PROP_RW Mat errorCovPost;       //!< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)

    // temporary matrices
    Mat temp1;
    Mat temp2;
    Mat temp3;
    Mat temp4;
    Mat temp5;
};
} // cv

#endif