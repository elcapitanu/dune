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

      IMC::PlanDB m_pdb;

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
        std::cout << "Help 6" << std::endl;
        bind<IMC::PlanDB>(this);
        std::cout << "Help 7" << std::endl;
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

      void
      consume(const IMC::PlanDB *pdb)
      {
        // std::cout << "Help 4" << std::endl;
        // m_pdb.op = pdb->op;
        // std::cout << "Help 10" << std::endl;
        // m_pdb.type = pdb->type;
        // std::cout << "Help 11" << std::endl;
        // m_pdb.plan_id = pdb->plan_id;

        // const IMC::PlanSpecification *ps = 0;

        // if (!pdb->arg.get(*ps))
        // {
        //   inf("no plan specification given");
        // }
        // std::cout << "Help 12" << std::endl;
        // std::printf("ps_id: %s", ps->plan_id.c_str());
        // // std::cout << "Help 13" << std::endl;
        // // m_pdb.arg.set(*ps);
        // // std::cout << "Help 14" << std::endl;
        // m_pdb.request_id = pdb->request_id;
        // std::cout << "Help 15" << std::endl;
        // std::cout << "Help 9" << std::endl;
      }

      //! Main loop.
      void onMain(void)
      {
        std::cout << "Help 1" << std::endl;
        while (!stopping())
        {
          waitForMessages(1.0);

          std::string input;
          std::cout << "Help 2" << std::endl;
          std::cin >> input;
          std::cout << "Help 3" << std::endl;
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
            // std::cout << "Comma is in " << comma << std::endl;
            float x = std::stof(input.substr(1, comma));
            float y = std::stof(input.substr(comma + 1, input.size()));

            // std::cout << "(x, y) " << x << ", " << y << std::endl;

            IMC::Goto maneuver;

            if ((x > 0) && (x <= 20) && (y > 0) & (y <= 20))
            {
              maneuver.lon = (x - 0) * (-8.7080671 - -8.70836583) / (20 - 0) - 8.70836583;
              maneuver.lat = (y - 0) * (41.18388408 - 41.18365927) / (20 - 0) + 41.18365927;
            }

            // maneuver.lon = DUNE::Math::Angles::radians(maneuver.lon);
            // maneuver.lat = DUNE::Math::Angles::radians(maneuver.lat);

            // spew("PC Values. Type - %d, Operation - %d, Req ID - %d, Flags - %d, ExtraInfo - %s", m_pc.type, m_pc.op, m_pc.request_id, m_pc.flags, m_pc.info.c_str());

            //! Create Goto command in database
            IMC::PlanGeneration m_gen;

            inf("Goto ID %d", IMC::Goto::getIdStatic());
            m_gen.op = IMC::PlanGeneration::OP_REQUEST;
            m_gen.plan_id = "go"; // Goto ID
            m_gen.params = "loc=;lat=" + std::to_string(maneuver.lat) + ";lon=" + std::to_string(maneuver.lon) + ";depth=0";
            m_gen.cmd = IMC::PlanGeneration::CMD_GENERATE;
            dispatch(m_gen);

            m_gen.cmd = IMC::PlanGeneration::CMD_EXECUTE;
            dispatch(m_gen);

            // IMC::PlanDB plan_db;

            // plan_db.type = IMC::PlanDB::DBT_REQUEST;
            // plan_db.op = IMC::PlanDB::DBOP_SET;
            // plan_db.plan_id = "mission1";
            // dispatch(plan_db);
            // plan_db.arg.set(m_gen);
            // plan_db.setDestination(m_gen.getSource());
            // plan_db.setDestinationEntity(m_gen.getSourceEntity());

            // IMC::PlanControl *p_control = new IMC::PlanControl();

            const IMC::PlanSpecification *ps = 0;

            if (!plan_db.arg.get(ps))
            {
              inf("no plan specification given");
            }

            // inf("ps_id: %s", ps->plan_id.c_str());

            inf("ps_id is dying lol");

            // if (ps->plan_id != m_gen.plan_id)
            // {
            //   inf("plan id mismatch, ps_id: %s", ps->plan_id.c_str());
            // }

            // p_control->type = IMC::PlanControl::PC_REQUEST;
            // p_control->op = IMC::PlanControl::PC_START;
            // p_control->flags = IMC::PlanControl::FLG_CALIBRATE;
            // p_control->request_id = m_pdb.request_id;
            // p_control->setDestination(ps->getSource());
            // p_control->setDestinationEntity(ps->getSourceEntity());
            // p_control->plan_id = ps->plan_id;
            // // p_control->arg.set(*ps);
            // p_control->info = "Will this finally work?"; // Useless ahah
            // dispatch(*p_control);

            // std::cout << std::fixed;
            // std::cout << std::setprecision(30);
            // std::cout << "(Lat,Lon) = " << maneuver.lat << maneuver.lon << std::endl;

            // maneuver.z = 0;
            // dispatch(maneuver);

            break;
          }
        }
      }
    };
  }
}

DUNE_TASK