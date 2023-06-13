//***************************************************************************
// Copyright 2007-2022 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Universidade do Porto. For licensing   *
// terms, conditions, and further information contact lsts@fe.up.pt.        *
//                                                                          *
// European Union Public Licence - EUPL v.1.1 Usage                         *
// Alternatively, this file may be used under the terms of the EUPL,        *
// Version 1.1 only (the "Licence"), appearing in the file LICENCE.md       *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Pedro Gonçalves                                                  *
// Author: Bernardo Gabriel                                                 *
// Author: Nuno Nascimento                                                  *
//***************************************************************************

// ISO C++ 98 headers.
#include <queue>
#include <cstring>
#include <string>
#include <iostream>
#include <cassert>
#include <stdexcept>
#include <cstdio>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local header
#include "CaptureImage.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

double getYValue(double pixels)
{
  // Coefficients obtained from previous calculation
  double coeffs[3] = {2.21916367e-04, -5.32765161e-07, 5.24004043e-10};

  double y_value = 0.0;

  for (int i = 0; i < 3; i++)
  {
    y_value += coeffs[i] * std::pow(pixels, i);
  }

  return y_value;
}

namespace MiniASV
{
  namespace BuoyDetection
  {
    using DUNE_NAMESPACES;

    //! %Task arguments.
    struct Arguments
    {
      //! Stream url of pionner device
      std::string url;
      //! Display Imshow
      std::string imshow;
      //! Detection method
      std::string method;
      //! Maximum fps
      int max_fps;
      //! Hue interval
      std::vector<int> hue_interval;
      //! Saturation interval
      std::vector<int> saturation_interval;
      //! Value interval
      std::vector<int> value_interval;
    };

    //! Task.
    struct Task : public DUNE::Tasks::Task
    {
      //! Configuration parameters
      Arguments m_args;
      //! Capture video frames of pioneer
      CaptureImage *m_cap;
      //! Flag to control state of task
      bool m_task_ready;

      Task(const std::string &name, Tasks::Context &ctx) : Tasks::Task(name, ctx),
                                                           m_task_ready(false)
      {
        paramActive(Tasks::Parameter::SCOPE_MANEUVER,
                    Tasks::Parameter::VISIBILITY_USER);

        param("Stream URL", m_args.url)
            .visibility(Tasks::Parameter::VISIBILITY_USER)
            .defaultValue("rtsp://192.168.1.101:8554/test")
            .description("Url of video stream");

        param("Imshow Display", m_args.imshow)
            .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER)
            .defaultValue("None")
            .description("Display image output, only available in xorg systems");

        param("Detection Method", m_args.method)
            .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER)
            .defaultValue("None")
            .description("Detection Method");

        param("Maximum Fps", m_args.max_fps)
            .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER)
            .defaultValue("8")
            .description("Maximum Fps");

        param("Hue Interval", m_args.hue_interval)
            .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER)
            .defaultValue("10")
            .size(2)
            .description("Hue Interval");

        param("Saturation Interval", m_args.saturation_interval)
            .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER)
            .defaultValue("40")
            .size(2)
            .description("Saturation Interval");

        param("Value Interval", m_args.value_interval)
            .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER)
            .defaultValue("40")
            .size(2)
            .description("Value Interval");
      }

      void
      onUpdateParameters(void)
      {
      }

      void
      onResourceInitialization(void)
      {
        m_cap = new CaptureImage(this, m_args.url, m_args.imshow);
        m_cap->start();
        m_task_ready = true;
      }

      void
      onResourceRelease(void)
      {
        if (m_task_ready)
        {
          m_cap->stopAndJoin();
          delete m_cap;
        }
      }

      void
      onRequestActivation(void)
      {
        inf("received activation request");
        activate();
      }

      void
      onRequestDeactivation(void)
      {
        inf("received deactivation request");
        deactivate();
      }

      void
      onActivation(void)
      {
        inf("on Activation");
      }

      void
      onDeactivation(void)
      {
      }

      void
      onMain(void)
      {
        double coord_x = 0;
        double coord_y = 0;
        double buoy_x = 0;
        double buoy_y = 0;

        // Create Kalman filter
        cv::KalmanFilter kf(4, 2, 0);
        cv::Mat state(4, 1, CV_32F); // [x, y, Vx, Vy]
        cv::Mat measurement = cv::Mat::zeros(2, 1, CV_32F);
        cv::Mat prediction;

        // Initialize Kalman filter
        kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
        cv::setIdentity(kf.measurementMatrix);
        cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-4));
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));

        // Define HSV color range, maybe tenho que melhorar para diminuir ruido
        cv::Scalar lowerHSV(0, 100, 0);
        cv::Scalar upperHSV(10, 255, 255);

        cv::Mat m_frame;

        while (!stopping())
        {
          waitForMessages(0.001);

          if (m_cap->isCapturing())
          {
            m_frame = m_cap->getFrame();
            if (!m_frame.empty())
            {
              cv::Mat frameHSV, mask;
              cv::cvtColor(m_frame, frameHSV, cv::COLOR_BGR2HSV);
              cv::inRange(frameHSV, lowerHSV, upperHSV, mask);

              // Perform object detection using contour analysis or any other method
              std::vector<std::vector<cv::Point>> contours;
              cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

              // Find the contour with the largest area
              double maxArea = 0;
              int maxAreaIdx = -1;
              for (int i = 0; i < contours.size(); i++)
              {
                double area = cv::contourArea(contours[i]);
                if (area > maxArea)
                {
                  maxArea = area;
                  maxAreaIdx = i;
                }
              }

              if (maxAreaIdx != -1)
              {
                // Calculate the centroid of the largest contour
                cv::Moments mu = cv::moments(contours[maxAreaIdx]);
                cv::Point centroid(mu.m10 / mu.m00, mu.m01 / mu.m00);

                // Update the measurement matrix with the centroid coordinates
                measurement.at<float>(0) = centroid.x;
                measurement.at<float>(1) = centroid.y;

                // Kalman filter prediction step
                prediction = kf.predict();

                // Kalman filter update step
                cv::Mat estimated = kf.correct(measurement);

                // Draw the predicted state (red) and estimated state (green)
                cv::Point predicted(prediction.at<float>(0), prediction.at<float>(1));
                cv::Point estimatedPos(estimated.at<float>(0), estimated.at<float>(1));
                cv::circle(m_frame, predicted, 5, cv::Scalar(0, 0, 255), -1);
                cv::circle(m_frame, estimatedPos, 5, cv::Scalar(0, 255, 0), -1);

                // Find the leftmost and rightmost points on the contour
                cv::Point leftmost = contours[maxAreaIdx][0];
                cv::Point rightmost = contours[maxAreaIdx][0];
                for (int i = 1; i < contours[maxAreaIdx].size(); i++)
                {
                  if (contours[maxAreaIdx][i].x < leftmost.x)
                    leftmost = contours[maxAreaIdx][i];
                  if (contours[maxAreaIdx][i].x > rightmost.x)
                    rightmost = contours[maxAreaIdx][i];
                }

                // Calculate the pixel distance between leftmost and rightmost points
                int pixelDistance = abs(rightmost.x - leftmost.x);

                printf("Width of the buoy %d \n", pixelDistance);

                double y_value = getYValue(pixelDistance);
                std::cout << "The corresponding value on the Y-axis for " << pixelDistance << " pixels is: " << y_value << std::endl;

                double new_value = 10.0 / (pixelDistance * y_value);
                std::cout << new_value << " cm" << std::endl;

                int ref_x = m_frame.cols / 2;
                int x_desloc;

                x_desloc = centroid.x - ref_x;

                double dist_value = getYValue(x_desloc);
                double real_x_dist = 10.0 / (x_desloc * dist_value);

                buoy_x = coord_x + real_x_dist;
                buoy_y = coord_y + new_value;
                std::cout << "Coordenates of buoy are (" << buoy_x << "," << buoy_y << ")" << std::endl;
              }

              cv::imshow("ola", m_frame);
            }
          }
        }

        cv::destroyAllWindows();
      }
    };
  }
}

DUNE_TASK