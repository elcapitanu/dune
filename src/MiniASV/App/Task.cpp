#include <DUNE/DUNE.hpp>

namespace MiniASV
{
  namespace App
  {
    using DUNE_NAMESPACES;

    struct Task: public DUNE::Tasks::Task
    {
      // Socket handle.
      TCPSocket* m_socket;
      
      double lat = 0;
      double lon = 0;
      
      // Buffer for incoming messages
      uint8_t m_buffer[512];

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_socket(NULL)
      {
        bind<IMC::GpsFix>(this);
        bind<IMC::Temperature>(this);
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
          m_socket->listen(1);  // Listen for only one connection at a time

          inf(DTR("Server started..."));
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        }
        catch (std::runtime_error& e)
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
      }

      void
      consume(const IMC::Temperature *msg)
      {
        // No change needed here.
      }

   void
onMain(void)
{
  // Accept a new client connection.
  TCPSocket* client_socket = NULL;
  try
  {
    Address client_address;
    uint16_t client_port;
    client_socket = m_socket->accept(&client_address, &client_port);
    inf(DTR("Accepted connection from %s:%u"), client_address.c_str(), client_port);
  }
  catch (std::runtime_error& e)
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
      int rv = client_socket->read(m_buffer, sizeof(m_buffer));

      if (rv > 0)
      {
        // Process the received message.
        std::string received_message((char*)m_buffer, rv);
        inf("Received message: %s", received_message.c_str());

        // Check if the received message is a task and process it.
        if (received_message == "Task1")
        {
          inf("Executing Task 1");
          std::string message = "ACK Task1";
          client_socket->write(message.c_str(), message.size());
        }
        else if (received_message == "Task2")
        {
          inf("Executing Task 2");
          std::string message = "ACK Task2";
          client_socket->write(message.c_str(), message.size());
        }
        else if (received_message == "Task3")
        {
          inf("Executing Task 3");
          std::string message = "ACK Task3";
          client_socket->write(message.c_str(), message.size());
        }
        else if (received_message == "Task4")
        {
          inf("Executing Task 4");
          std::string message = "ACK Task4";
          client_socket->write(message.c_str(), message.size());
        }
        else if (received_message == "Ping")
        {
          inf("Ping received -> Sending Pong");
          std::string message = "Pong";
          client_socket->write(message.c_str(), message.size());
        }
        else
        {
          inf("Ignoring message");
          std::string message = "WrongFormat";
          client_socket->write(message.c_str(), message.size());
        }
      }


      // Sleep briefly before reading the next message
      Delay::wait(0.1);
    }
  }
  catch (std::runtime_error& e)
  {
    war(DTR("Failed to read from the socket: %s"), e.what());
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

