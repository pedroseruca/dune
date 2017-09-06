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
// Author: PGonçalves                                                       *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local Headers.
#include "Driver.hpp"

namespace Sensors
{
  namespace OEMX
  {
    using DUNE_NAMESPACES;

    static const float c_delay_startup = 5.0f;
    static const float c_timeout_uart = 1.0f;

    struct Arguments
    {
      //! Serial port device.
      std::string uart_dev;
      //! Serial port baud rate.
      unsigned uart_baud;
      //! Input timeout.
      double input_timeout;
      //! Input number samples/s
      unsigned int input_samples_number;
      //! List of sensors in primary mount.
      std::vector<std::string> primary_mount;
      //! List of sensors in secondary mount.
      std::vector<std::string> secondary_mount;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Serial port handle.
      SerialPort* m_uart;
      //! I/O Multiplexer.
      Poll m_poll;
      //! Task arguments
      Arguments m_args;
      //! Driver of CTD
      DriverOEMX *m_driver;
      //! Watchdog.
      Counter<double> m_wdog;
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
      //! Number of sensors plug in CTD
      size_t m_numberSensors;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_uart(NULL),
        m_driver(0),
        m_temperature(0),
        m_salinity(0),
        m_pressure(0),
        m_conductivity(0),
        m_soundSpeed(0),
        m_numberSensors(0)
      {
        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("")
        .description("Serial port device used to communicate with the sensor");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("38400")
        .description("Serial port baud rate");

        param("Input Timeout", m_args.input_timeout)
        .defaultValue("4.0")
        .minimumValue("1.0")
        .units(Units::Second)
        .description("Amount of seconds to wait for data before reporting an error");

        param("Number of Samples/s", m_args.input_samples_number)
        .defaultValue("1")
        .minimumValue("1")
        .maximumValue("5")
        .description("Amount of samples/s.");

        param("Primary Mount", m_args.primary_mount)
        .defaultValue("")
        .description("List of sensors in primary mount");

        param("Secondary Mount", m_args.secondary_mount)
        .defaultValue("")
        .description("List of sensors in secondary mount");
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
          m_uart->setCanonicalInput(true);
          m_uart->flush();
          m_poll.add(*m_uart);
          m_driver = new DriverOEMX(this, m_uart, m_poll, m_args.primary_mount.size(), m_args.secondary_mount.size());
        }
        catch (std::runtime_error& e)
        {
          throw RestartNeeded(e.what(), 10);
        }
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        Delay::wait(c_delay_startup);
        getInfoOfCTD();

        if(!m_driver->initCTD(m_args.input_samples_number))
          throw RestartNeeded(DTR("failed to init CTD"), 5, true);

        m_wdog.setTop(m_args.input_timeout);
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        if (m_uart != NULL)
        {
          m_poll.remove(*m_uart);
          Memory::clear(m_driver);
          Memory::clear(m_uart);
        }
      }

      void
      getInfoOfCTD()
      {
        m_numberSensors = m_args.primary_mount.size() + m_args.secondary_mount.size();

        inf("P: %d | S: %d | T: %d", (int)m_args.primary_mount.size(), (int)m_args.secondary_mount.size(), (int)m_numberSensors);

        m_driver->getInfoOfCTD();

        inf("Firmware version: %s  SN: %s", m_driver->m_ctdData.version.c_str(),
            m_driver->m_ctdData.serialCTD.c_str());

        if(m_args.primary_mount.size() > 0)
          inf("Primary Mount: %s", m_driver->m_ctdData.primaryMount.c_str());
        if(m_args.secondary_mount.size() > 0)
          inf("Secondary Mount: %s", m_driver->m_ctdData.secondayMount[0].c_str());
        if(m_args.secondary_mount.size() > 1)
          inf("Secondary Mount: %s", m_driver->m_ctdData.secondayMount[1].c_str());
      }


      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          //waitForMessages(1.0);
          consumeMessages();

          if (m_wdog.overflow())
          {
            setEntityState(IMC::EntityState::ESTA_ERROR, Status::CODE_COM_ERROR);
            throw RestartNeeded(DTR(Status::getString(CODE_COM_ERROR)), 5);
          }

          if (!Poll::poll(*m_uart, c_timeout_uart))
            continue;

          if(m_driver->haveNewData(m_numberSensors))
          {
            inf("C: %f | P: %f | T: %f | S: %f | V: %f",
                                m_driver->m_ctdData.m_conductivity, m_driver->m_ctdData.m_pressure,
                                m_driver->m_ctdData.m_temperature, m_driver->m_ctdData.m_salinity,
                                m_driver->m_ctdData.m_soundSpeed);

            setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
            m_wdog.reset();
          }
        }

        m_driver->sendCommand("\r", ">");
      }
    };
  }
}

DUNE_TASK
