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
// Author: Miguel Silva                                                     *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <DUNE/Coordinates/General.hpp>
#include <Maneuver/Multiplexer/Goto.hpp>
#include <DUNE/Control/PathController.hpp>

namespace MiniASV
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Miguel Silva
  namespace Mission1
  {
    using DUNE_NAMESPACES;

    struct Task : public DUNE::Tasks::Task
    {

      double x = 0, y = 0;

      // IMC::DesiredPath m_dpath;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string &name, Tasks::Context &ctx) : DUNE::Tasks::Task(name, ctx)
      {
        bind<IMC::Goto>(this);
        bind<IMC::EstimatedState>(this);
        bind<IMC::StateReport>(this);
        // bind<IMC::Temperature>(this);
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
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }

      void
      consume(const IMC::Goto *maneuver)
      {
        std::printf("X is %lf, y is %lf, z is %lf\n", maneuver->lat, maneuver->lon, maneuver->z);

        std::cout << std::fixed;
        std::cout << std::setprecision(30);
        std::cout << "(X,Y,Z) = " << maneuver->lat << " " << maneuver->lon << " " << maneuver->z << std::endl;

        // //! Start Goto
        // inf("Start (lat, lon): %f , %f || End (lat, lon): %f , %f", m_dpath.start_lat, m_dpath.start_lon, m_dpath.end_lat, m_dpath.end_lon);

        // m_dpath.start_z = 0;
        // m_dpath.start_lat = 0;
        // m_dpath.start_lon = 0;
        // m_dpath.end_lat = -0.15198;
        // m_dpath.end_lon = 0.71882;
        // dispatch(m_dpath);
      }

      // void
      // consume(const IMC::Temperature *maneuver)
      // {
      //   // inf("X is %lf, y is %lf, z is %lf\n", maneuver->lat, maneuver->lon, maneuver->z);

      //   // //! Start Goto
      //   std::printf("Start (lat, lon): %f , %f || End (lat, lon): %f , %f", m_dpath.start_lat, m_dpath.start_lon, m_dpath.end_lat, m_dpath.end_lon);

      //   inf("Mama noooooo");

      //   IMC::Goto m_goto;

      //   m_goto.lat = -0.15198;
      //   m_goto.lat = 0.71882;

      //   dispatch(m_goto);

      //   // m_dpath.start_z = 0;
      //   // m_dpath.start_lat = 0;
      //   // m_dpath.start_lon = 0;
      //   // m_dpath.end_lat = -0.15198;
      //   // m_dpath.end_lon = 0.71882;
      //   // dispatch(m_dpath);
      // }

      void
      consume(const IMC::EstimatedState *st)
      {
        // This is called way too much
        // std::printf("ESTSTATE -- Latitude: %f, LOngitude: %f\n", st->lat, st->lon);

        x = st->lat;
        y = st->lon;

        // std::cout << std::fixed;
        // std::cout << std::setprecision(30);
        // std::cout << "(X,Y) = " << st->lat << st->lon << std::endl;
      }

      // void
      // consume(const IMC::PathControlState *pct)
      // {
      //   inf("PCT -- Start Latitude & LOngitude: %f,%f\n", pct->start_lat, pct->start_lon);
      //   inf("PCT -- End Latitude & LOngitude: %f,%f\n", pct->end_lat, pct->end_lon);
      // }

      void
      consume(const IMC::StateReport *sr)
      {
        std::cout << std::fixed;
        std::cout << std::setprecision(30);
        std::cout << "(X,Y) = " << sr->latitude << sr->longitude << std::endl;
      }

      //! Main loop.
      void
      onMain(void)
      {

        while (!stopping())
        {

          waitForMessages(1.0);
          // std::cout << std::fixed;
          // std::cout << std::setprecision(30);
          // std::cout << "(X,Y) = " << x << y << std::endl;
        }
      }
    };
  }
}

DUNE_TASK
