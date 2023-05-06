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
// Author: BGabriel                                                         *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <DUNE/Coordinates/WGS84.hpp>
#include <DUNE/Coordinates/General.hpp>
#include <string.h>
// #include "Controller.hpp"

namespace MiniASV
{
  namespace KeyboardInputs
  {
    //! Insert short task description here.
    //!
    //! Insert explanation on task behaviour here.
    //! @author BGabriel
    using DUNE_NAMESPACES;

    struct Task : public DUNE::Tasks::Task
    {

      IMC::EstimatedState m_st;
      //! Timer
      Counter<double> m_wdog;
      //! IMC msg
      IMC::PWM m_pwmR;
      IMC::PWM m_pwmL;
      //! Read timestamp.
      double m_tstamp;
      //! Count for attempts
      int m_count_attempts;
      //! Flag to control reset of board
      bool m_is_first_reset;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string &name, Tasks::Context &ctx) : DUNE::Tasks::Task(name, ctx)
      {
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        m_pwmR.setDestination(42);
        m_pwmL.setDestination(42);

        m_pwmR.id = 1;
        m_pwmL.id = 2;
        m_pwmR.period = 20000;
        m_pwmL.period = 20000;
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }

      void
      dispatchDataMotor()
      {
        m_tstamp = Clock::getSinceEpoch();
        m_pwmR.setTimeStamp(m_tstamp);
        dispatch(m_pwmR, DF_KEEP_TIME);

        m_tstamp = Clock::getSinceEpoch();
        m_pwmL.setTimeStamp(m_tstamp);
        dispatch(m_pwmL, DF_KEEP_TIME);
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(1.0);

          std::string input;

          std::cin >> input;

          switch (input[0])
          {
          //! For motor commands
          case 'm':
          {
            int value = std::stoi(input.substr(1, 3));
            m_pwmR.duty_cycle = m_pwmL.duty_cycle = value;

            dispatchDataMotor();
            break;
          }
          //! For GoTo Commands. FIrst is x, second is y
          case 'l':
            int comma = input.find(',');
            std::cout << "Comma is in " << comma << std::endl;
            float x = std::stof(input.substr(1, comma));
            float y = std::stof(input.substr(comma + 1, input.size()));

            std::cout << "(x, y) " << x << ", " << y << std::endl;

            IMC::Goto maneuver;

            if ((x > 0) && (x <= 20) && (y > 0) & (y <= 20))
            {
              maneuver.lon = (x - 0) * (-8.7080671 - -8.70836583) / (20 - 0) - 8.70836583;
              maneuver.lat = (y - 0) * (41.18388408 - 41.18365927) / (20 - 0) + 41.18365927;
              std::cout << std::fixed;
              std::cout << std::setprecision(30);
              std::cout << "First(X,Y) = " << maneuver.lat << maneuver.lon << std::endl;
              // maneuver.lon = (x - 0) * (-8.7080671 - -8.70836583) / (20 - 0);
              // maneuver.lat = (y - 0) * (41.18388408 - 41.18365927) / (20 - 0);
            }
            // WGS84::displace()
            // Coordinates::toWGS84(m_st, maneuver.lat, maneuver.lon);

            maneuver.lon = DUNE::Math::Angles::radians(maneuver.lon);
            maneuver.lat = DUNE::Math::Angles::radians(maneuver.lat);

            std::cout << std::fixed;
            std::cout << std::setprecision(30);
            std::cout << "Last(X,Y) = " << maneuver.lat << maneuver.lon << std::endl;
            // maneuver.lat = 0.718790586427262900848234039586;
            // maneuver.lat = -0.151989547887982384688498882497;

            maneuver.z = 0;
            dispatch(maneuver);

            break;
          }
        }
      }
    };
  }
}

DUNE_TASK