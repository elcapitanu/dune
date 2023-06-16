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
// Author: Filipe Peixoto                                                   *
//***************************************************************************

#include <fcntl.h>

#include <DUNE/DUNE.hpp>

namespace MiniASV
{
  namespace App
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      std::string plan;
    };

    struct Task : public DUNE::Tasks::Task
    {
      // Socket handle.
      TCPSocket *m_socket;

      Arguments m_args;

      double m_lat = 41.87254; // yes
      double m_lon = -8.26382; // yes

      double m_connection_timeout = 1;
      double m_temp_cpu = 45, m_temp_batt = 45; // neither
      double m_yaw = 23;                        // yes, meybe not the one I want though
      double m_batt_percentage = 110;           // nein
      int m_asv_state = 0;                      // yes m8
      int m_current_task = 0;                   // TODO: when to restart this one
      int count = 0;
      // Buffer for incoming messages
      uint8_t m_buffer[512];

      Task(const std::string &name, Tasks::Context &ctx) : DUNE::Tasks::Task(name, ctx),
                                                           m_socket(NULL)
      {

        bind<IMC::GpsFix>(this);
        bind<IMC::EulerAngles>(this);
        bind<IMC::VehicleState>(this);
        bind<IMC::Temperature>(this);

        param("Main Execution Plan", m_args.plan)
            .defaultValue("plano_teste_1")
            .description("Serial port device used to communicate with the sensor");
      }

      ~Task(void)
      {
        onResourceRelease();
      }

      void
      onResourceAcquisition(void)
      {
        try
        {
          m_socket = new TCPSocket;
          m_socket->bind(49162);
          m_socket->listen(1); // Listen for only one connection at a time

          m_socket->setReceiveTimeout(m_connection_timeout);

          // int flags = fcntl(*m_socket, F_GETFL, 0);
          // flags |= O_NONBLOCK;
          // fcntl(*m_socket, F_SETFL, flags);

          spew(DTR("Server started..."));
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        }
        catch (std::runtime_error &e)
        {
          throw RestartNeeded(e.what(), 5);
        }
      }

      void
      onResourceRelease(void)
      {
        if (m_socket)
        {
          delete m_socket;
          m_socket = NULL;
        }
      }

      void
      consume(const IMC::GpsFix *msg)
      {
        // No change needed here.
        m_lat = Angles::degrees(msg->lat);
        m_lon = Angles::degrees(msg->lon);
        spew("lat and lon read!");
      }

      void
      consume(const IMC::EulerAngles *msg)
      {
        m_yaw = Angles::degrees(msg->psi);
        spew("Angles consume!");
      }

      void
      consume(const IMC::Temperature *msg)
      {
        // No change needed here.
        // inf("EHEHEHEH");
      }

      void
      consume(const IMC::VehicleState *msg)
      {
        m_asv_state = msg->op_mode;
        spew("Vehicle State: %d", m_asv_state);
      }

      void
      onMain(void)
      {
        // while (!stopping())
        // {
        //   waitForMessages(1.0);
        // }

        // Accept a new client connection.
        TCPSocket *client_socket = NULL;
        try
        {
          Address client_address;
          uint16_t client_port;
          client_socket = m_socket->accept(&client_address, &client_port);
          spew(DTR("Accepted connection from %s:%u"), client_address.c_str(), client_port);
        }
        catch (std::runtime_error &e)
        {
          if (client_socket)
          {
            delete client_socket;
            client_socket = NULL;
          }
          war(DTR("Failed to accept new connection: %s"), e.what());
          return;
        }

        // Communicate with the client.
        try
        {
          while (!stopping())
          {
            // Now we can read from and write to the client socket.
            waitForMessages(0.1);
            int rv = client_socket->read(m_buffer, sizeof(m_buffer));
            if (rv > 0)
            {
              // Process the received message.
              std::string received_message((char *)m_buffer, rv);
              inf("Received message: %s", received_message.c_str());

              // Check if the received message is a task and process it.
              if (received_message == "Task1\n") // goto message
              {
                // FIXME: chagne this to new coordinates

                std::string message;
                if (IMC::VehicleState::VS_SERVICE == m_asv_state)
                {
                  inf("Executing Task 1");
                  message = "ACK Task1";

                  m_current_task = 1;

                  IMC::Goto maneuver;
                  float x = 15, y = 3;

                  if ((x > 0) && (x <= 20) && (y > 0) & (y <= 20))
                  {
                    maneuver.lon = (x - 0) * (-8.7080671 - -8.70836583) / (20 - 0) - 8.70836583;
                    maneuver.lat = (y - 0) * (41.18388408 - 41.18365927) / (20 - 0) + 41.18365927;
                  }

                  //! Create Goto command in database
                  IMC::PlanGeneration m_gen;

                  m_gen.op = IMC::PlanGeneration::OP_REQUEST;
                  m_gen.plan_id = "go"; // Goto ID
                  // m_gen.params = "loc=;lat=" + std::to_string(maneuver.lat) + ";lon=" + std::to_string(maneuver.lon) + ";depth=0";
                  m_gen.params = "loc=;lat=" + std::to_string(41.17540997) + ";lon=" + std::to_string(-8.59899188) + ";depth=0";
                  m_gen.cmd = IMC::PlanGeneration::CMD_GENERATE;
                  dispatch(m_gen);

                  m_gen.cmd = IMC::PlanGeneration::CMD_EXECUTE;
                  dispatch(m_gen);
                }
                else
                {
                  inf("Task 1 Not Executed");
                  message = "ASV Busy";
                }

                client_socket->write(message.c_str(), message.size());
              }
              else if (received_message == "Task2\n") // Stop Maneuver
              {
                std::string message;
                if (IMC::VehicleState::VS_SERVICE == m_asv_state)
                {
                  inf("Executing Task 2");
                  message = "ACK Task2";
                  m_current_task = 0;
                }
                else
                {
                  inf("Task 2 Not Executed");
                  message = "ASV Busy";
                }

                client_socket->write(message.c_str(), message.size());
              }
              else if (received_message == "Task3\n") // Do .ini plan
              {

                std::string message;
                if (IMC::VehicleState::VS_SERVICE == m_asv_state)
                {

                  inf("Executing Task 3");
                  message = "ACK Task3";
                  m_current_task = 3;

                  // FIXME: not tested with App

                  IMC::PlanControl p_control; //= new IMC::PlanControl();
                  // IMC::PlanSpecification *ps = new IMC::PlanSpecification();

                  p_control.type = IMC::PlanControl::PC_REQUEST;
                  p_control.op = IMC::PlanControl::PC_START;
                  p_control.flags = IMC::PlanControl::FLG_IGNORE_ERRORS;
                  p_control.setDestination(getSystemId());
                  // p_control->setDestinationEntity(getEntityId());
                  p_control.plan_id = m_args.plan;
                  // p_control->arg.set(*ps);
                  // p_control->info = "Will this finally work?"; // Useless ahah
                  dispatch(p_control);
                }
                else
                {
                  inf("Task 3 Not Executed");
                  message = "ASV Busy";
                }

                client_socket->write(message.c_str(), message.size());
              }
              else if (received_message == "Task4\n")
              {
                std::string message;
                if (IMC::VehicleState::VS_SERVICE == m_asv_state)
                {
                  inf("Executing Task 4");
                  message = "ACK Task4";
                  m_current_task = 0;
                }
                else
                {
                  inf("Task 4 Not Executed");
                  message = "ASV Busy";
                }

                client_socket->write(message.c_str(), message.size());
              }
              else if (received_message == "Task5\n")
              {
                std::string message;
                if (IMC::VehicleState::VS_SERVICE == m_asv_state)
                {
                  inf("Executing Task 5");
                  message = "ACK Task5";
                  m_current_task = 0;
                }
                else
                {
                  inf("Task 5 Not Executed");
                  message = "ASV Busy";
                }

                client_socket->write(message.c_str(), message.size());
              }
              else if (received_message == "SendData\n")
              {
                // count++;
                // if (count % 4 == 0)
                // {
                //   m_asv_state++;
                // }
                // TODO: test this with Filipe and verify these values
                inf("Executing SendData");
                m_temp_cpu = 81;
                m_temp_batt = 40;
                // m_lat += 0.00005;
                // m_lon += 0.00005;
                m_batt_percentage = 77;
                // m_yaw += 0.33;

                std::string message = "cpu_temp=" + std::to_string(m_temp_cpu) + ",bat_temp=" + std::to_string(m_temp_batt) + ",lat=" + std::to_string(m_lat) + ",long=" + std::to_string(m_lon) + ",bat_perc=" + std::to_string(m_batt_percentage) + ",dir=" + std::to_string(m_yaw) + ",cur_task=" + std::to_string(m_current_task) + ",state=" + std::to_string(m_asv_state);
                client_socket->write(message.c_str(), message.size());
              }
              else if (received_message == "Stop\n") // Stop Maneuver
              {
                inf("Stopping");
                std::string message = "Stopped";

                IMC::VehicleCommand cmd;
                cmd.command = IMC::VehicleCommand::VC_STOP_MANEUVER;
                dispatch(cmd);

                client_socket->write(message.c_str(), message.size());
                m_current_task = 0;
              }
              else if (received_message == "Ping\n")
              {
                inf("Ping received -> Sending Pong");
                std::string message = "Pong";
                client_socket->write(message.c_str(), message.size());
              }
              else
              {
                spew("Ignoring message");
                std::string message = "WrongFormat";
                client_socket->write(message.c_str(), message.size());
              }
            }

            // Sleep briefly before reading the next message
            Delay::wait(0.1);
          }
        }
        catch (std::runtime_error &e)
        {
          spew(DTR("Failed to read from the socket: %s"), e.what());
        }

        // Close the client socket.
        if (client_socket)
        {
          delete client_socket;
          client_socket = NULL;
        }
      }
    };
  }
}

DUNE_TASK