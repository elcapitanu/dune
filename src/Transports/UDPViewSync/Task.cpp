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
// Author: João Bogas                                                       *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local headers.
#include "Reader.hpp"

namespace Transports
{
  //! Implementation of a UDP-based View Synchronous Communication protocol,
  //! Based of https://www.researchgate.net/publication/200030099_Lightweight_Causal_and_Atomic_Group_Multicast.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Bernardo Gabriel
  //! @author João Bogas
  namespace UDPViewSync
  {
    using DUNE_NAMESPACES;

    //! Total number of group members.
    static const unsigned c_total_members = 2;

    //! Task arguments.
    struct Arguments
    {
      // UDP port.
      unsigned port;
    };

    //! Member struct.
    struct Member
    {
      // IPV4 address.
      Address address;
      // UDP port.
      unsigned port;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! UDP Socket.
      UDPSocket m_sock;
      //! Task arguments.
      Arguments m_args;
      //! Reader thread.
      Reader* m_reader;
      //! Array with group members.
      std::array<Member, c_total_members> m_members;
      //! Vector time.
      std::array<unsigned, c_total_members> m_vector_time;
      
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_reader(NULL),
        m_vector_time({0})
      {
        param("Port", m_args.port)
        .description("UDP port to listen on")
        .minimumValue("6000")
        .maximumValue("6063");

        for (unsigned i = 0; i < c_total_members; ++i)
        {
          std::string label = String::str("Member %u - Address", i);
          param(label, m_members[i].address)
          .description("Member address");

          label = String::str("Member %u - Port", i);
          param(label, m_members[i].port)
          .description("Member port");
        }

        bind<IMC::DevDataText>(this); 
        bind<IMC::IoEvent>(this);
        bind<IMC::Temperature>(this);
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
        m_sock.bind(m_args.port, Address::Any, false);

        m_reader = new Reader(*this, m_sock);
        m_reader->start();
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        if (m_reader != NULL)
        {
          m_reader->stopAndJoin();
          delete m_reader;
          m_reader = NULL;
        }
      }

      void
      consume(const IMC::DevDataText* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (msg->getDestinationEntity() != getEntityId())
          return;

        debug("%s", sanitize(msg->value).c_str());
      }

      void
      consume(const IMC::IoEvent* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (msg->getDestinationEntity() != getEntityId())
          return;

        if (msg->type == IMC::IoEvent::IOV_TYPE_INPUT_ERROR)
          throw RestartNeeded(msg->error, 5);
      }

      void
      consume(const IMC::Temperature* msg)
      {
        send_multicast(std::to_string(msg->value));
      }

      void
      send_multicast(const std::string msg)
      {        
        for (Member member: m_members)
        {
          if (member.address == "localhost" && member.port == m_args.port)
            continue;

          m_sock.write((const uint8_t*) msg.c_str(), msg.size(), member.address, member.port);
        }
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
