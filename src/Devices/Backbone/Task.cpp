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
// Author: Ricardo Martins (GPS driver)                                     *
// Author: Luis Venancio (BasicDeviceDriver compatibility)                  *
// Author: Bernardo Gabriel (adaptation to Backbone)                        *
//***************************************************************************

// ISO C++ 98 headers.
#include <algorithm>
#include <cstddef>
#include <cstring>
// DUNE headers.
#include <DUNE/DUNE.hpp>
// Local headers.
#include "Reader.hpp"

namespace Devices
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Bernardo Gabriel
  namespace Backbone
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! IO device (URI).
      std::string io_dev;
      //! Input timeout in seconds.
      float inp_tout;
    };

    struct Task : public Hardware::BasicDeviceDriver
    {
      //! Serial port handle.
      IO::Handle *m_handle;
      //! Task arguments.
      Arguments m_args;
      //! Input watchdog.
      Time::Counter<float> m_wdog;
      //! Reader thread.
      Reader *m_reader;
      //! Buffer forEntityState
      char m_bufer_entity[64];

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string &name, Tasks::Context &ctx) : Hardware::BasicDeviceDriver(name, ctx),
                                                           m_handle(NULL),
                                                           m_reader(NULL)
      {
        // Define configuration parameters.
        paramActive(Tasks::Parameter::SCOPE_GLOBAL,
                    Tasks::Parameter::VISIBILITY_DEVELOPER,
                    true);

        param("IO Port - Device", m_args.io_dev)
            .defaultValue("")
            .description("IO device URI in the form \"uart://DEVICE:BAUD\"");

        param("Input Timeout", m_args.inp_tout)
            .units(Units::Second)
            .defaultValue("4.0")
            .minimumValue("0.0")
            .description("Input timeout");

        // Use wait for messages
        setWaitForMessages(1.0);

        bind<IMC::DevDataText>(this);
        bind<IMC::IoEvent>(this);
        bind<IMC::RemoteActions>(this);
      }

      ~Task() override
      {
        onDisconnect();
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      //! Try to connect to the device.
      //! @return true if connection was established, false otherwise.
      bool
      onConnect() override
      {
        try
        {
          m_handle = openDeviceHandle(m_args.io_dev);
          m_reader = new Reader(this, m_handle);
          m_reader->start();
          setEntityState(IMC::EntityState::ESTA_BOOT, Status::CODE_ACTIVATING);
          return true;
        }
        catch (...)
        {
          throw RestartNeeded(DTR(Status::getString(CODE_COM_ERROR)), 5);
        }

        return false;
      }

      //! Disconnect from device.
      void
      onDisconnect() override
      {
        if (m_reader != NULL)
        {
          m_reader->stopAndJoin();
          Memory::clear(m_reader);
        }

        Memory::clear(m_handle);
      }

      //! Initialize device.
      void
      onInitializeDevice() override
      {
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);

        // m_wdog.setTop(m_args.inp_tout);
      }

      void
      consume(const IMC::DevDataText *msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (msg->getDestinationEntity() != getEntityId())
          return;

        trace("%s", sanitize(msg->value).c_str());
      }

      void
      consume(const IMC::IoEvent *msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (msg->getDestinationEntity() != getEntityId())
          return;

        if (msg->type == IMC::IoEvent::IOV_TYPE_INPUT_ERROR)
          throw RestartNeeded(msg->error, 5);
      }

      void
      consume(const IMC::RemoteActions *msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        Utils::TupleList tuples(msg->actions);
        int motor = tuples.get("Motor", 0);
        int piston = tuples.get("Piston", 0);
        int rudder = tuples.get("Rudder", 0);

        trace("motor: %d | piston: %d | rudder: %d", motor, piston, rudder);

        char m_motor[64], m_piston[64], m_rudder[64];

        std::sprintf(m_motor, "MOTOR,%d,\r\n", motor);
        std::sprintf(m_piston, "PISTON,%d,\r\n", piston);
        std::sprintf(m_rudder, "RUDDER,%d,\r\n", rudder);

        m_handle->writeString(m_motor);
        m_handle->writeString(m_piston);
        m_handle->writeString(m_rudder);
      }

      //! Check for input timeout.
      //! Data is read in the DevDataText consume.
      //! @return true.
      bool
      onReadData() override
      {
        // if (m_wdog.overflow())
        // {
        //   setEntityState(IMC::EntityState::ESTA_ERROR, Status::CODE_COM_ERROR);
        //   throw RestartNeeded(DTR(Status::getString(CODE_COM_ERROR)), 5);
        // }

        return true;
      }
    };
  }
}

DUNE_TASK
