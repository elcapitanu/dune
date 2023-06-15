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
// Author: Bernardo Gabriel                                                 *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace MiniASV
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Bernardo Gabriel
  namespace Tag
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Serial port device.
      std::string uart_dev;
      //! Serial port baud rate.
      unsigned uart_baud;
      //! Input timeout.
      double input_timeout;
      //! Number of attempts before error
      int number_attempts;
      //! Distance between anchors
      int dist_anchors;
    };

    struct Task : public DUNE::Tasks::Task
    {
      //! Serial port handle
      SerialPort *m_uart;
      //! I/O Multiplexer
      Poll m_poll;
      //! Task arguments
      Arguments m_args;
      //! Timer
      Counter<double> m_wdog;
      //! IMC msg
      IMC::DeviceState m_position;
      //! GPS
      IMC::GpsFix m_gps;
      //! Read timestamp.
      double m_tstamp;
      //! Distances
      float m_distance1 = 0;
      float m_distance2 = 0;
      //! Position
      float m_x;
      float i = 0;
      float m_y;

      char bfr[128];

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string &name, Tasks::Context &ctx) : DUNE::Tasks::Task(name, ctx)
      {
        param("Serial Port - Device", m_args.uart_dev)
            .defaultValue("")
            .description("Serial port device");

        param("Serial Port - Baud Rate", m_args.uart_baud)
            .defaultValue("")
            .description("Serial port baud rate");

        param("Input Timeout", m_args.input_timeout)
            .defaultValue("3.0")
            .minimumValue("2.0")
            .maximumValue("4.0")
            .units(Units::Second)
            .description("Amount of seconds to wait for data before reporting an error");

        param("Distance Between Anchors", m_args.dist_anchors)
            .defaultValue("4.0")
            .minimumValue("1.0")
            .maximumValue("20.0")
            .units(Units::Meter)
            .description("Distance between UWB Anchors");
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

        setEntityState(IMC::EntityState::ESTA_BOOT, Status::CODE_INIT);
        try
        {
          m_uart = new SerialPort(m_args.uart_dev, m_args.uart_baud);
          m_uart->setCanonicalInput(true); // waits for terminator caharacter
          m_uart->flush();
          m_poll.add(*m_uart);
        }
        catch (const std::runtime_error &e)
        {
          throw RestartNeeded(e.what(), 10);
        }
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        m_uart->flush();
        Delay::wait(1.0f);
        m_wdog.setTop(m_args.input_timeout);
        m_wdog.reset();

        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        /* if (m_uart != nullptr)
          {
            m_poll.remove(*m_uart);
            Memory::clear(m_uart);
          } */
      }

      double
      x_pos_to_latitude(double x)
      {
        return (x - 0) * (41.18388408 - 41.18365927) / (m_args.dist_anchors - 0) + 41.18365927;
      }

      double
      y_pos_to_longitude(double y)
      {
        return (y - 0) * (-8.7080671 - -8.70836583) / (m_args.dist_anchors - 0) - 8.70836583;
      }

      void dispatchData()
      {
        // m_gps.type = IMC::GpsFix::GFT_MANUAL_INPUT;

        m_tstamp = Clock::getSinceEpoch();
        m_gps.setTimeStamp(m_tstamp);
        m_gps.lat = x_pos_to_latitude(m_x);
        m_gps.lon = y_pos_to_longitude(m_y);
        m_gps.lat = Angles::radians(m_gps.lat);
        m_gps.lon = Angles::radians(m_gps.lon);

        // m_gps.lat = Angles::radians(41.18529547);
        // m_gps.lon = Angles::radians(-8.7080671);

        m_gps.validity = IMC::GpsFix::GFV_VALID_POS;
        dispatch(m_gps, DF_KEEP_TIME);
        // inf("TAG:(x, y) || (lat, lon) = %.2f, %.2f || %.2f, %.2f", m_x, m_y, m_gps.lat, m_gps.lon);
      }

      bool
      haveNewData()
      {
        std::size_t rv = m_uart->readString(bfr, sizeof(bfr));

        if (rv == 0)
        {
          err(DTR("I/O error"));
          return false;
        }

        bfr[strlen(bfr) - 3] = '\0';

        char *param = std::strtok(bfr, ",");

        if (!strcmp(param, "$DIST"))
        {
          param = std::strtok(NULL, ",");
          m_distance1 = atof(param);
          param = std::strtok(NULL, ",");
          m_distance2 = atof(param);

          while (!(m_distance1 + m_distance2 > m_args.dist_anchors && m_args.dist_anchors + m_distance1 > m_distance2 && m_args.dist_anchors + m_distance2 > m_distance1))
          {
            m_distance1 += 0.01;
            m_distance2 += 0.01;
          }

          // inf("d1: %.3f, d2: %.3f", m_distance1, m_distance2);

          // anchors pos: (0,0) and (m_args.dist_anchors,0)
          m_x = (pow(m_distance1, 2) + pow(m_args.dist_anchors, 2) - pow(m_distance2, 2)) / (2 * m_args.dist_anchors);

          m_y = sqrt(pow(m_distance1, 2) - pow(m_x, 2));
        }

        bfr[0] = '\0';

        m_uart->flush();

        return true;
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(0.1);

          if (haveNewData())
            dispatchData();
        }
      }
    };
  }
}

DUNE_TASK
