//***************************************************************************
// Copyright 2007-2017 Universidade do Porto - Faculdade de Engenharia      *
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
// Author: Pedro Gonçalves                                                  *
//***************************************************************************

#ifndef SENSORS_OEMX_DRIVER_HPP_INCLUDED_
#define SENSORS_OEMX_DRIVER_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <cstring>
#include <cstdlib>

// DUNE headers.
#include <DUNE/DUNE.hpp>


namespace Sensors
{
  namespace OEMX
  {
    using DUNE_NAMESPACES;

    class DriverOEMX
    {
      public:

      struct CTDData
      {
        //! Temperature
        float m_temperature;
        //! Salinity
        float m_salinity;
        //! Pressure
        float m_pressure;
        //! Conductivity
        float m_conductivity;
        //! SoundSpeed
        float m_soundSpeed;
      };

      SerialPort* m_uart;
      Poll m_poll;

      DriverOEMX(DUNE::Tasks::Task* task, SerialPort* uart, Poll poll):
        m_task(task)
      {
        m_uart = uart;
        m_poll = poll;
        m_timeout_uart = 1.0f;
      }

      ~DriverOEMX(void)
      {}

      bool
      initCTD(int numberSamples)
      {
        if(numberSamples > 5 || numberSamples <= 0)
        {
          m_task->war("Incorrect Number of Samples, setting 1 sample/s");
          numberSamples = 1;
        }
        char cmdSample[32];
        char replySample[32];

        std::sprintf(cmdSample, "SET S %d /s\r", numberSamples);
        std::sprintf(replySample, ">SET S %d /s\r\n", numberSamples);

        m_uart->writeString("\r");
        Delay::wait(1.0);
        m_uart->flush();

        if (!sendCommand("\r", "\r\n"))
        {
          m_task->err(DTR("failed to enter command mode"));
          return false;
        }

        if (!sendCommand(cmdSample, replySample))
        {
          m_task->err(DTR("failed to set sampling rate"));
          return false;
        }

        if (!sendCommand("MONITOR\r", ">MONITOR\r\n"))
        {
          m_task->err(DTR("failed to enter monitor mode"));
          return false;
        }

        return true;
      }

      void
      getInfoOfCTD()
      {
        bool breakWhile = false;
        char usart_rec[2];

        while(!breakWhile && !m_task->isStopping())
        {
          if (sendCommand("\r", "\r\n"))
          {
            breakWhile = true;
          }
          else
          {
            Delay::wait(2.0);
            m_uart->flush();
          }
        }

        breakWhile = false;

        m_uart->writeString("display options\r\r");
        std::string txtRec;
        bool isFristTerminator = true;

        while(!breakWhile && !m_task->isStopping())
        {
          if (m_poll.poll(m_timeout_uart))
            if (m_poll.wasTriggered(*m_uart))
            {
              m_uart->read(usart_rec, 1);
              txtRec += usart_rec[0];

              if (usart_rec[0] == '>')
              {
                if (isFristTerminator)
                  isFristTerminator = false;
                else
                  breakWhile = true;
              }
            }
        }

        std::string delimiter = "\r\n\r\n";
        size_t pos = 0;
        size_t cnt = 0;
        std::string token[4];
        while ((pos = txtRec.find(delimiter)) != std::string::npos)
        {
          token[cnt] = txtRec.substr(0, pos);
          txtRec.erase(0, pos + delimiter.length());
          cnt++;
        }
        token[cnt] = txtRec;

        for(size_t i = 0; i <=3; i++)
          m_task->inf("\n%s\n", token[i].c_str());
      }

      bool
      sendCommand(const char* cmd, const char* reply)
      {
        char bfrUart[128];
        m_task->spew("Command: %s", cmd);
        m_uart->writeString(cmd);

        if (Poll::poll(*m_uart, m_timeout_uart))
        {
          m_uart->readString(bfrUart, sizeof(bfrUart));
          m_task->spew("Reply: %s", bfr);
          if (std::strcmp(bfrUart, reply) == 0)
            return true;
        }

        return false;
      }

      bool
      haveNewData(int m_numberSensors)
      {
        size_t rv = m_uart->readString(bfr, sizeof(bfr));

        if (rv == 0)
          throw RestartNeeded(DTR("I/O error"), 5);

        if (std::sscanf(bfr, " %f %f %f\r\n", &m_ctdData.m_conductivity,
                        &m_ctdData.m_pressure, &m_ctdData.m_temperature)
            != m_numberSensors)
          return false;

        m_ctdData.m_salinity = UNESCO1983::computeSalinity(
            m_ctdData.m_conductivity + 0.5, m_ctdData.m_pressure,
            m_ctdData.m_temperature);

        m_ctdData.m_soundSpeed = UNESCO1983::computeSoundSpeed(
            m_ctdData.m_salinity, m_ctdData.m_pressure,
            m_ctdData.m_temperature);

        return true;
      }

      CTDData m_ctdData;

      private:

      //! Parent task.
      DUNE::Tasks::Task* m_task;
      //! Timeout for new data in uart
      float m_timeout_uart;
      //! Buffer of uart
      char bfr[32];

    };
  }
}

#endif /* SENSORS_OEMX_DRIVER_HPP_INCLUDED_ */
