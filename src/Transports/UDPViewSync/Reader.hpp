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

#ifndef TRANSPORTS_UDPVIEWSYNC_READER_HPP_INCLUDED_
#define TRANSPORTS_UDPVIEWSYNC_READER_HPP_INCLUDED_

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Transports
{
  namespace UDPViewSync
  {
    using DUNE_NAMESPACES;

    class Reader: public Concurrency::Thread
    {
    public:
      Reader(Tasks::Task& task, UDPSocket& sock):
        m_task(task),
        m_sock(sock)
      {  }

    private:
      // Buffer capacity.
      static const int c_bfr_size = 65535;
      // Poll timeout in milliseconds.
      static const int c_poll_tout = 1000;
      // Parent task.
      Tasks::Task& m_task;
      // Reference to socket used for sending data.
      UDPSocket& m_sock;

      void
      run(void)
      {
        Address addr;
        double poll_tout = c_poll_tout / 1000.0;

        while (!isStopping())
        {
          try
          {
            if (!Poll::poll(m_sock, poll_tout))
              continue;

            uint8_t *bfr = new uint8_t[c_bfr_size];
            uint16_t rv = m_sock.read(bfr, c_bfr_size, &addr);

            if (rv)
              m_task.war("%s: %s", addr.c_str() , sanitize(String::str((char* ) bfr)).c_str());

            delete[] bfr;
          }
          catch (std::exception &e)
          {
            m_task.debug("error while unpacking message: %s", e.what());
          }
        }
      }
    };
  }
}

#endif
