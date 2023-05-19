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
        x_pos_to_longitude(double x)
        {
          return (x - 0) * (-8.7080671 - -8.70836583) / (20 - 0) - 8.70836583;
        }

        double
        y_pos_to_latitude(double y)
        {
          return (y - 0) * (41.18388408 - 41.18365927) / (20 - 0) + 41.18365927;
        }

        double
        longitude_to_x_pos(double lon)
        {
          return (lon + 8.70836583) * (20 - 0) / (-8.7080671 + 8.70836583) + 0;
        }

        double
        latitude_to_y_pos(double lat)
        {
          return (lat - 41.18365927) * (20 - 0) / (41.18388408 - 41.18365927) + 0;
        }

        //! Execute a path control step
        //! From base class PathController
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

          double obs_x = x_pos_to_longitude(3), obs_y = y_pos_to_latitude(10); // FIXME: not fault free!

          obs_x = DUNE::Math::Angles::radians(obs_x);
          obs_y = DUNE::Math::Angles::radians(obs_y);

          // calculate x and y based on track and current state

          // for testing
          obs_x = 3.5;
          obs_y = 9;

          double in_radius = 1.5, out_radius = 4;
          double x_pos, y_pos;

          DUNE::Coordinates::toWGS84(state, y_pos, x_pos); // Returns coordinates in radians
          x_pos = DUNE::Math::Angles::degrees(x_pos);      // radians to degrees
          y_pos = DUNE::Math::Angles::degrees(y_pos);      // radians to degrees
          x_pos = longitude_to_x_pos(x_pos);               // degrees to pool
          y_pos = latitude_to_y_pos(y_pos);                // degrees to pool

          double in_x = abs(obs_x - x_pos);
          double in_y = abs(obs_y - y_pos);

          double in_abs = sqrt(pow(in_x, 2) + pow(in_y, 2));

          inf("Pos(X, Y): %.2f, %.2f -> ABS: %.2f", x_pos, y_pos, in_abs);

          double aux_x, aux_y;
          double centerX, centerY;
          double theta;

          aux_x = obs_x - x_pos;
          aux_y = obs_y - y_pos;

          // Inside out radius, change bearing to tan of radius
          // if (in_abs < out_radius)
          // {
          //   if (aux_y > 0)
          //   {
          //     ref += std::atan2(aux_y, aux_x);

          //     if (aux_x > 0)
          //     {
          //       ref += DUNE::Math::Angles::degrees(90);
          //     }
          //     else
          //     {
          //       ref -= DUNE::Math::Angles::degrees(90);
          //     }
          //   }
          //   inf("Out radius vector angle: %.3f", Angles::degrees(Angles::normalizeRadian(ref)));
          // }

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
            inf("All going good");
            // Over track (avoid singularities)
            ref += ts.track_bearing;
          }

          // // Inside inner circle, go all the way backwards
          // if (in_abs < in_radius)
          // {
          //   ref = std::atan2(-aux_y, -aux_x);
          //   inf("Inside Radius overwrite: %.3f", Angles::degrees(Angles::normalizeRadian(ref)));
          // }

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
