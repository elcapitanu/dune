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
      IMC::EstimatedState m_eststate;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string &name, Tasks::Context &ctx) : DUNE::Tasks::Task(name, ctx)
      {
        bind<IMC::Goto>(this);
        // bind<IMC::EstimatedState>(this);
        bind<IMC::PlanSpecification>(this);
        bind<IMC::PlanManeuver>(this);

        // bind<IMC::StateReport>(this);
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

      // Goto 1: 41.18366783, -8.7083427
      // Goto 2: 41.18386553, -8.70834052
      // Goto 3: 41.18386377, -8.70828888
      // Goto 4: 41.18367465, -8.70828537
      // Goto 5: 41.18367452, -8.70810028
      // Goto 6: 41.18385948, -8.70810045

      /***************Corners***************/

      // IE: 41.18365977, -8.70836583
      // SE: 41.18388433, -8.70836548
      // ID: 41.18365927, -8.7080671
      // SD: 41.18388408, -8.70806743

      void
      consume(const IMC::Goto *maneuver)
      {
        inf("A GOTO message has been dispatched, tell me your timeline babbyyyy");
      }

      void
      consume(const IMC::PlanSpecification *ps)
      {
        inf("Plan Specification -- ID: %s || Descripttion: %s || Start man ID: %s", ps->plan_id.c_str(), ps->description.c_str(), ps->start_man_id.c_str());
      }

      void
      consume(const IMC::PlanManeuver *pm)
      {
        inf("Plan Maneuver -- Man ID: %s", pm->maneuver_id.c_str());
      }

      //! Main loop.
      void onMain(void)
      {
        IMC::LblEstimate lbl;

        lbl.x = 12.5;
        lbl.y = 1;

        inf("Dispatched lbl");

        dispatch(lbl);

        while (!stopping())
        {

          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
