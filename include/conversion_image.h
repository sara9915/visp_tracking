/*!
 \file image.cpp
 \brief Implements conversions between ViSP and ROS image types
 */
#ifndef __VISP_BRIDGE_IMAGE_H__
#define __VISP_BRIDGE_IMAGE_H__

#include "sensor_msgs/Image.h"
#include "visp/vpImage.h"
#include "visp/vpRGBa.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/format.hpp>
#include <stdexcept>

namespace visp_bridge
{

    sensor_msgs::Image toSensorMsgsImage(const vpImage<unsigned char> &src)
    {
        sensor_msgs::Image dst;
        dst.width = src.getWidth();
        dst.height = src.getHeight();
        dst.encoding = sensor_msgs::image_encodings::MONO8;
        dst.step = src.getWidth();
        dst.data.resize(dst.height * dst.step);
        memcpy(&dst.data[0], src.bitmap, dst.height * dst.step * sizeof(unsigned char));

        return dst;
    }

    vpImage<unsigned char> toVispImage(const sensor_msgs::Image &src)
    {
        using sensor_msgs::image_encodings::BGR8;
        using sensor_msgs::image_encodings::BGRA8;
        using sensor_msgs::image_encodings::MONO8;
        using sensor_msgs::image_encodings::RGB8;
        using sensor_msgs::image_encodings::RGBA8;

        vpImage<unsigned char> dst(src.height, src.width);

        if (src.encoding == MONO8)
            memcpy(dst.bitmap, &(src.data[0]), dst.getHeight() * src.step * sizeof(unsigned char));
        else if (src.encoding == RGB8 || src.encoding == RGBA8 || src.encoding == BGR8 || src.encoding == BGRA8)
        {
            unsigned nc = sensor_msgs::image_encodings::numChannels(src.encoding);
            unsigned cEnd = (src.encoding == RGBA8 || src.encoding == BGRA8) ? nc - 1 : nc;

            for (unsigned i = 0; i < dst.getWidth(); ++i)
            {
                for (unsigned j = 0; j < dst.getHeight(); ++j)
                {
                    int acc = 0;
                    for (unsigned c = 0; c < cEnd; ++c)
                        acc += src.data[j * src.step + i * nc + c];
                    dst[j][i] = acc / nc;
                }
            }
        }
        return dst;
    }

    vpImage<vpRGBa> toVispImageRGBa(const sensor_msgs::Image &src)
    {
        using sensor_msgs::image_encodings::BGR8;
        using sensor_msgs::image_encodings::BGRA8;
        using sensor_msgs::image_encodings::MONO8;
        using sensor_msgs::image_encodings::RGB8;
        using sensor_msgs::image_encodings::RGBA8;

        vpImage<vpRGBa> dst(src.height, src.width);

        if (src.encoding == MONO8)
            for (unsigned i = 0; i < dst.getWidth(); ++i)
            {
                for (unsigned j = 0; j < dst.getHeight(); ++j)
                {

                    dst[j][i] = vpRGBa(src.data[j * src.step + i], src.data[j * src.step + i], src.data[j * src.step + i]);
                }
            }
        else
        {
            unsigned nc = sensor_msgs::image_encodings::numChannels(src.encoding);

            for (unsigned i = 0; i < dst.getWidth(); ++i)
            {
                for (unsigned j = 0; j < dst.getHeight(); ++j)
                {
                    dst[j][i] = vpRGBa(src.data[j * src.step + i * nc + 0], src.data[j * src.step + i * nc + 1], src.data[j * src.step + i * nc + 2]);
                }
            }
        }
        return dst;
    }

    sensor_msgs::Image toSensorMsgsImage(const vpImage<vpRGBa> &src)
    {
        sensor_msgs::Image dst;
        dst.width = src.getWidth();
        dst.height = src.getHeight();
        dst.encoding = sensor_msgs::image_encodings::RGB8;
        unsigned nc = sensor_msgs::image_encodings::numChannels(dst.encoding);
        dst.step = src.getWidth() * nc;

        dst.data.resize(dst.height * dst.step);
        for (unsigned i = 0; i < src.getWidth(); ++i)
        {
            for (unsigned j = 0; j < src.getHeight(); ++j)
            {
                dst.data[j * dst.step + i * nc + 0] = src.bitmap[j * src.getWidth() + i].R;
                dst.data[j * dst.step + i * nc + 1] = src.bitmap[j * src.getWidth() + i].G;
                dst.data[j * dst.step + i * nc + 2] = src.bitmap[j * src.getWidth() + i].B;
                // dst.data[j * dst.step + i * nc + 3] = src.bitmap[j * dst.step + i].A;
            }
        }
        return dst;
    }
}
#endif /* IMAGE_H_ */