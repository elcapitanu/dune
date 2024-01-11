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

#include <unordered_map>

namespace Transports
{
  //! Implementation of a UDP-based View Synchronous Communication protocol,
  //! Based on https://www.researchgate.net/publication/200030099_Lightweight_Causal_and_Atomic_Group_Multicast.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Bernardo Gabriel
  //! @author João Bogas
  namespace UDPViewSync
  {
    using DUNE_NAMESPACES;

    //! Total number of group members.
    static const unsigned c_total_members = 3;

    enum UDPVS_state
    {
      IDLE,
      ACTIVE,
      ERROR
    };

    //! Member struct.
    struct Member
    {
      // IPV4 address.
      Address address;
      // UDP port.
      unsigned port;
    };

    //! Message struct.
    struct Message
    {
      // Time vector.
      std::array<unsigned, c_total_members> time_vector;
      // Process identifier.
      unsigned id;
      // Header of message.
      std::string header;
      // Content of message.
      std::string content;
    };

    //! Unstable Message struct.
    struct Unstable_Message
    {
      // Members who ack.
      std::array<bool, c_total_members> ack;
      // Message.
      std::string message;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! UDP Socket.
      UDPSocket m_sock;
      //! Reader thread.
      Reader* m_reader;
      // Process identifier.
      unsigned m_id;
      //! Array with group members.
      std::array<Member, c_total_members> m_members;
      //! Vector time.
      std::array<unsigned, c_total_members> m_time_vector;
      //! Message delay queue.
      std::vector<Message> m_queue;
      //! Current state.
      UDPVS_state m_state;

      std::unordered_map<unsigned, Unstable_Message> m_unstable_messages;
      
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_reader(NULL),
        m_time_vector({0}),
        m_state(IDLE)
      {        
        param("Id", m_id)
        .description("Process identifier");

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
        m_sock.bind(m_members[m_id].port , Address::Any, false);

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

        trace("%s", sanitize(msg->value).c_str());

        interpret_message(msg->value);
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
        if (m_state != ACTIVE)
          return;

        send_multicast("data", std::to_string(msg->value));
      }

      void
      deliver_message(const Message msg)
      {
        for (unsigned itr = 0; itr < c_total_members; itr++)
        {
          if (msg.time_vector[itr] > m_time_vector[itr])
          {
            m_time_vector[itr] = msg.time_vector[itr];
          }
        }
      }

      void
      check_queue()
      {
        unsigned messages = m_queue.size();

        if (!messages)
          return;

        for (unsigned itr = 0; itr < messages; itr++)
        {
          if (validate_time_vector(m_queue[itr], true))
          {
            deliver_message(m_queue[itr]);
            m_queue.erase(m_queue.begin() + itr);
            break;
          }
        }
      }

      std::string
      prepare_message(const std::string header, const std::string content)
      {
        std::string message = "[";

        for (unsigned time: m_time_vector)
          message += std::to_string(time) + "-";

        message.replace(message.end()-1, message.end(), 1, ']');

        message += "," + std::to_string(m_id) + "," + header + "," + content + ",*";

        // message += Algorithms::CRC16::compute((uint8_t*) message.c_str(), message.size()-1);

        message.push_back('\n');

        return message;
      }

      void
      send_ack(const unsigned dest, const unsigned msg)
      {
        std::string message = "ACK,";

        message += std::to_string(m_id) + "," + std::to_string(msg) + ",*";

        message.push_back('\n');

        m_sock.write((const uint8_t*) message.c_str(), message.size(), m_members[dest].address, m_members[dest].port);
      }

      void
      send_multicast(const std::string header, const std::string content)
      {
        m_time_vector[m_id]++;

        const std::string message = prepare_message(header, content);

        for (unsigned iter = 0; iter < c_total_members; iter++)
        {
          if (iter == m_id)
            continue;

          m_sock.write((const uint8_t*) message.c_str(), message.size(), m_members[iter].address, m_members[iter].port);
        }

        Unstable_Message msg;

        msg.message = message;

        msg.ack = {false};
        msg.ack[m_id] = true;

        m_unstable_messages.insert({m_time_vector[m_id], msg});
      }

      bool
      validate_time_vector(const Message message, const bool queue = false)
      {
        for (unsigned itr = 0; itr < c_total_members; itr++)
        {
          if (itr == message.id)
          {
            if (message.time_vector[itr] > m_time_vector[itr] + 1)
            {
              if (!queue)
                m_queue.push_back(message);
              return false;
            }
          }
          else
          {
            if (message.time_vector[itr] > m_time_vector[itr])
            {
              if (!queue)
                m_queue.push_back(message);
              return false;
            }
          }
        }

        return true;
      }

      std::array<unsigned, c_total_members>
      interpret_time_vector(const std::string time_vector)
      {
        std::vector<std::string> parts;
        String::split(time_vector, "-", parts);

        std::array<unsigned, c_total_members> tv;
        for (unsigned itr = 0; itr < c_total_members; itr++)
          tv[itr] = atoi(parts[itr].c_str());

        return tv;
      }

      void
      interpret_ack(const unsigned src, const unsigned message)
      {
        m_unstable_messages[message].ack[src] = true;
        
        for (bool itr: m_unstable_messages[message].ack)
        {
          if (!itr)
            return;
        }

        m_unstable_messages.erase(message);
      }

      void
      interpret_message(const std::string msg)
      {
        std::vector<std::string> parts;
        String::split(msg, ",", parts);

        if (parts[0] == "ACK")
        {
          interpret_ack(atoi(parts[1].c_str()), atoi(parts[2].c_str()));
          return;
        }

        if (parts.size() != 5)
          return;

        if (!String::startsWith(parts[0], "[") || !String::endsWith(parts[0], "]"))
          return;
        
        parts[0].erase(0,1);
        parts[0].pop_back();
        std::array<unsigned, c_total_members> tv = interpret_time_vector(parts[0]);

        Message message;
        message.time_vector = tv;
        message.id = atoi(parts[1].c_str());
        message.header = parts[2];
        message.content = parts[3];

        if (validate_time_vector(message))
          deliver_message(message);

        send_ack(message.id, message.time_vector[message.id]);
      }

      //! Main loop.
      void
      onMain(void)
      {
        while(!String::endsWith(Format::getTimeSafe(), "0"));

        m_state = ACTIVE;
        
        // Time::Counter<float> test(20.0);
        Time::Counter<float> p(1.0);
        
        while (!stopping())
        {
          // if (test.overflow())
            // m_state = IDLE;

          if (p.overflow())
          {
            // inf("%u | %u | %u", m_time_vector[0], m_time_vector[1], m_time_vector[2]);
            inf("%lu", m_unstable_messages.size());
            p.reset();
          }

          waitForMessages(0.1);

          check_queue();
        }

        for(unsigned t: m_time_vector)
          inf("%u", t);
      }
    };
  }
}

DUNE_TASK
