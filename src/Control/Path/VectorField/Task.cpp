//***************************************************************************
// Copyright 2007-2023 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Eduardo Marques                                                  *
//***************************************************************************

//***************************************************************************
// Reference:
//  "Vector Field Path Following for Miniature Air Vehicles",
//  Nelson, Barber, McLain and Beard,
//  Proc. American Control Conference, 2006 (ACC'06)
//***************************************************************************

// ISO C++ 98 headers.
#include <iomanip>
#include <cmath>

// DUNE headers.
#include <DUNE/DUNE.hpp>

using DUNE_NAMESPACES;

namespace Control
{
  namespace Path
  {
    namespace VectorField
    {
      struct Arguments
      {
        double corridor;
        double entry_angle;
        bool ext_control;
        double ext_gain;
        double ext_trgain;
      };

      struct Task : public DUNE::Control::PathController
      {
        //! Controller gain.
        double m_gain;
        //! Outgoing desired heading message.
        IMC::DesiredHeading m_heading;
        //! Task arguments.
        Arguments m_args;

        Task(const std::string &name, Tasks::Context &ctx) : DUNE::Control::PathController(name, ctx)
        {
          param("Corridor -- Width", m_args.corridor)
              .minimumValue("1.0")
              .maximumValue("50.0")
              .defaultValue("5.0")
              .units(Units::Meter)
              .description("Width of corridor for attack entry angle");

          param("Corridor -- Entry Angle", m_args.entry_angle)
              .minimumValue("2")
              .maximumValue("45")
              .defaultValue("15")
              .units(Units::Degree)
              .description("Attack angle when lateral track error equals corridor width");

          param("Extended Control -- Enabled", m_args.ext_control)
              .defaultValue("false")
              .description("Enable extended (refined) corridor control");

          param("Extended Control -- Controller Gain", m_args.ext_gain)
              .defaultValue("1.0")
              .description("Gain for extended control");

          param("Extended Control -- Turn Rate Gain", m_args.ext_trgain)
              .defaultValue("1.0")
              .description("Turn rate gain for extended control");
        }

        void
        onUpdateParameters(void)
        {
          PathController::onUpdateParameters();

          if (paramChanged(m_args.entry_angle))
            m_args.entry_angle = Angles::radians(m_args.entry_angle);

          m_gain = std::tan(m_args.entry_angle) / m_args.corridor;
        }

        void
        onEntityReservation(void)
        {
          PathController::onEntityReservation();
        }

        void
        onPathActivation(void)
        {
          // Activate heading cotroller.
          enableControlLoops(IMC::CL_YAW);
        }

        double
        x_pos_to_latitude(double x)
        {
          return (x - 0) * (41.18388408 - 41.18365927) / (25 - 0) + 41.18365927;
        }

        double
        y_pos_to_longitude(double y)
        {
          return (y - 0) * (-8.7080671 - -8.70836583) / (25 - 0) - 8.70836583;
        }

        //! In meters
        double
        latitude_to_x_pos(double lat)
        {
          return (lat - 41.18365927) * (25 - 0) / (41.18388408 - 41.18365927) + 0;
        }

        //! In meters
        double
        longitude_to_y_pos(double lon)
        {
          return (lon + 8.70836583) * (25 - 0) / (-8.7080671 + 8.70836583) + 0;
        }

        //! Execute a path control step
        //! From base class PathController
        //! X and Y are inverted. X is the vertical axis and Y is the horizontal axis
        void
        step(const IMC::EstimatedState &state, const TrackingState &ts)
        {
          // Note:
          // cross-track position (lateral error) = ts.track_pos.y
          // and along-track position = ts.track_pos.x
          double kcorr = ts.track_pos.y / m_args.corridor;
          double akcorr = std::fabs(kcorr);

          bool laranja = true;

          double ref = 0; // Radians of vector

          // for testing
          double obs_x = 12.5;
          double obs_y = 3;

          double in_radius = 3.5, out_radius = 5;
          double x_pos, y_pos;

          // calculate x and y based on track and current state
          DUNE::Coordinates::toWGS84(state, x_pos, y_pos); // Returns coordinates in radians
          x_pos = DUNE::Math::Angles::degrees(x_pos);      // radians to degrees
          y_pos = DUNE::Math::Angles::degrees(y_pos);      // radians to degrees

          x_pos = latitude_to_x_pos(x_pos);  // degrees to pool
          y_pos = longitude_to_y_pos(y_pos); // degrees to pool

          double in_x = abs(obs_x - x_pos);
          double in_y = abs(obs_y - y_pos);
          double in_angle = atan2(y_pos - obs_y, x_pos - obs_x);

          double in_abs = sqrt(pow(in_x, 2) + pow(in_y, 2));

          inf("Pos(X, Y): %.2f, %.2f -> ABS: %.2f", x_pos, y_pos, in_abs);

          double x_final = x_pos + ts.range * cos(ts.los_angle);
          double y_final = y_pos + ts.range * sin(ts.los_angle);

          double theta = std::atan2(y_final - obs_y, x_final - obs_x); // Angle between obstacle and final position

          // angle used to know if is past buoy
          double leaving_angle = in_angle - theta;

          double aux_x, aux_y;
          double centerX, centerY;

          aux_x = x_pos - obs_x;
          aux_y = y_pos - obs_y;

          // Doesn't have space to go because of the wall
          if ((obs_y - out_radius) <= 1 || (out_radius + obs_y) >= 24)
          {
            out_radius = out_radius - 1 - abs(obs_y - out_radius);
            in_radius = 1.1;
          }
          else if ((obs_x - out_radius) <= 1 || (out_radius + obs_x) >= 24)
          {
            out_radius = out_radius - 1 - abs(obs_x - out_radius);
            in_radius = 1.1;
          }

          if (out_radius < in_radius)
          {
            out_radius = 2.5;
            inf("AHHHH MERDEI, ERRO ENTRE PAREDE E OBSTACULO");
          }

          if (ts.track_pos.x > ts.track_length)
          {
            // Past the track goal: this should never happen but ...
            ref = getBearing(state, ts.end);
          }
          else if (akcorr > 1 || !m_args.ext_control)
          {
            // Outside corridor
            ref = ts.track_bearing - std::atan(m_gain * ts.track_pos.y);
          }
          else if (akcorr > 0.05)
          {
            // Inside corridor
            ref += ts.track_bearing - std::pow(kcorr, m_args.ext_gain) * m_args.entry_angle * (1 + (m_gain * ts.speed * std::sin(ts.course - ts.track_bearing)) / (m_args.ext_trgain * ts.track_pos.y));
          }
          else
          {
            // Over track (avoid singularities)
            ref += ts.track_bearing;
          }

          // Inside out radius, change bearing to tan of radius
          if (leaving_angle <= DUNE::Math::Angles::radians(-90) || leaving_angle >= DUNE::Math::Angles::radians(90)) // entrou aqui quando devia
          {
            if (in_abs < out_radius)
            {
              ref = std::atan2(aux_y, aux_x);
              inf("out radius initial: %.2f", Angles::degrees(Angles::normalizeRadian(ref)));
              // if (laranja)
              // {
              //   ref -= DUNE::Math::Angles::radians(90) - DUNE::Math::Angles::radians(10);
              // }
              // else
              // {
              //   ref += DUNE::Math::Angles::radians(90) + DUNE::Math::Angles::radians(10);
              // }

              // Works with this <3
              if (leaving_angle > DUNE::Math::Angles::radians(90))
              {
                ref -= DUNE::Math::Angles::radians(90) - DUNE::Math::Angles::radians(10); // Extra value to make ti a little more agressive
              }
              else
              {
                ref += DUNE::Math::Angles::radians(90) + DUNE::Math::Angles::radians(10); // Extra value to make ti a little more agressive
              }
              inf("out radius final: %.2f", Angles::degrees(Angles::normalizeRadian(ref)));
            }

            // Inside inner circle, go all the way backwards
            if (in_abs < in_radius)
            {
              ref = std::atan2(aux_y, aux_x);
              inf("Inside Radius overwrite: %.3f", Angles::degrees(Angles::normalizeRadian(ref)));
            }
          }

          // Stay away from the pool wall
          // TODO: verify all walls
          if (x_pos <= 1 || x_pos >= 24 || y_pos <= 1 || y_pos >= 24)
          {
            aux_x = 12.5 - x_pos;
            aux_y = 12.5 - y_pos;

            ref = std::atan2(aux_y, aux_x); // Always points to the center of the pool
            inf("Bro too close to the WALL");
          }

          inf("Loop ref: %.3f", Angles::degrees(Angles::normalizeRadian(ref)));

          if (ts.cc)
            ref += state.psi - ts.course; // course control rather than yaw control

          spew("lte=%0.1f cadj=%0.1f attack=%0.1f", std::fabs(ts.track_pos.y),
               Angles::degrees(Angles::normalizeRadian(std::fabs(state.psi - std::atan2(state.vy, state.vx)))),
               Angles::degrees(Angles::normalizeRadian(std::fabs(ts.track_bearing - ref))));

          // Dispatch heading reference
          m_heading.value = Angles::normalizeRadian(ref);
          dispatch(m_heading);
        }

        //! Execute a loiter control step
        //! From base class PathController
        void
        loiter(const IMC::EstimatedState &state, const TrackingState &ts)
        {
          double ref = DUNE::Math::c_half_pi + std::atan(2 * m_gain * (ts.range - ts.loiter.radius));

          if (!ts.loiter.clockwise)
            ref = -ref;

          ref += DUNE::Math::c_pi + ts.los_angle;

          if (ts.cc)
            ref += state.psi - ts.course; // course control

          // Dispatch heading reference
          m_heading.value = Angles::normalizeRadian(ref);
          dispatch(m_heading);
        }
      };
    }
  }
}

DUNE_TASK
