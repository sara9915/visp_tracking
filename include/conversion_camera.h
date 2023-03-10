/*!
  \file camera.cpp
  \brief conversions between ROS and ViSP structures representing camera parameters
*/
#ifndef _VISP_BRIDGE_CAMERA_H_
#define _VISP_BRIDGE_CAMERA_H_
#include <visp3/core/vpCameraParameters.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

namespace visp_bridge
{

    vpCameraParameters toVispCameraParameters(const sensor_msgs::CameraInfo &cam_info)
    {
        vpCameraParameters cam;
        // Check that the camera is calibrated, as specified in the
        // sensor_msgs/CameraInfo message documentation.
        if (cam_info.K.size() != 3 * 3 || cam_info.K[0] == 0.)
            throw std::runtime_error("uncalibrated camera");

        // Check matrix size.
        if (cam_info.P.size() != 3 * 4)
            throw std::runtime_error("camera calibration P matrix has an incorrect size");

        if (cam_info.distortion_model.empty())
        {
            const double &px = cam_info.K[0 * 3 + 0];
            const double &py = cam_info.K[1 * 3 + 1];
            const double &u0 = cam_info.K[0 * 3 + 2];
            const double &v0 = cam_info.K[1 * 3 + 2];
            cam.initPersProjWithoutDistortion(px, py, u0, v0);
            return cam;
        }

        if (cam_info.distortion_model == sensor_msgs::distortion_models::PLUMB_BOB)
        {
            const double &px = cam_info.P[0 * 4 + 0];
            const double &py = cam_info.P[1 * 4 + 1];
            const double &u0 = cam_info.P[0 * 4 + 2];
            const double &v0 = cam_info.P[1 * 4 + 2];
            cam.initPersProjWithoutDistortion(px, py, u0, v0);
            return cam;
        }

        throw std::runtime_error("unsupported distortion model");

        // return vpCameraParameters(cam_info.P[0 * 4 + 0],cam_info.P[1 * 4 + 1],cam_info.P[0 * 4 + 2],cam_info.P[1 * 4 + 2],-cam_info.D[0],cam_info.D[0]);
    }

    sensor_msgs::CameraInfo toSensorMsgsCameraInfo(vpCameraParameters &cam_info, unsigned int cam_image_width, unsigned int cam_image_height)
    {
        sensor_msgs::CameraInfo ret;

        std::vector<double> D(5);
        D[0] = cam_info.get_kdu();
        D[1] = D[2] = D[3] = D[4] = 0.;
        ret.D = D;
        ret.P.assign(0.);
        ret.K.assign(0.);
        ret.R.assign(0.);

        ret.R[0] = 1.;
        ret.R[1 * 3 + 1] = 1.;
        ret.R[2 * 3 + 2] = 1.;

        ret.P[0 * 4 + 0] = cam_info.get_px();
        ret.P[1 * 4 + 1] = cam_info.get_py();
        ret.P[0 * 4 + 2] = cam_info.get_u0();
        ret.P[1 * 4 + 2] = cam_info.get_v0();
        ret.P[2 * 4 + 2] = 1;

        ret.K[0 * 3 + 0] = cam_info.get_px();
        ret.K[1 * 3 + 1] = cam_info.get_py();
        ret.K[0 * 3 + 2] = cam_info.get_u0();
        ret.K[1 * 3 + 2] = cam_info.get_v0();
        ret.K[2 * 3 + 2] = 1;

        ret.distortion_model = "plumb_bob";
        ret.binning_x = 0;
        ret.binning_y = 0;
        ret.width = cam_image_width;
        ret.height = cam_image_height;

        return ret;
    }
}
#endif /* CAMERA_H_ */