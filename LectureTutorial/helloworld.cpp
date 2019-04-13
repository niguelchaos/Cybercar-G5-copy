#include <iostream>
#include "PrimeChecker.hpp"
#include "cluon-complete-v0.0.121.hpp"
#include "messages.hpp"


int main(int /*argc*/, char** /*argv*/) {
   PrimeChecker pc;
   std::cout << "Hello World = " << pc.isPrime(43) << std::endl;

// Basic Sending data via UDP
   // cluon::UDPSender sender{"127.0.0.1", 1234};
   // sender.send("Hello World!");

// Sending via multicast port - >224.x.x.xxx
   // cluon::UDPSender sender{"225.0.0.111", 1236};


// Advanced Multicast Sending with encoding/decoding predefined messages
   // cluon::UDPSender sender{"225.0.0.111", 1238};
   //
   // uint16_t value;
   // std::cout << "Enter a number to check: ";
   // std::cin >> value;
   // MyTestMessage1 msg;
   // msg.myValue(value);
   // cluon::ToProtoVisitor encoder;
   // msg.accept(encoder);
   // std::string data{encoder.encodedData()};
   // sender.send(std::move(data));

// Sleep for 5 instead of 1 for multicast no idea why
   // using namespace std::literals::chrono_literals;
   // std::this_thread::sleep_for(5s);

// choose 1 for multicast receiving
   // cluon::UDPReceiver receiver("225.0.0.111", 1236,
   // cluon::UDPReceiver receiver("225.0.0.111", 1238,

// Basic Receiving data via UDP
   // cluon::UDPReceiver receiver("0.0.0.0", 1235,
   //      [](std::string &&data, std::string &&/*from*/,
   //         std::chrono::system_clock::time_point &&/*timepoint*/) noexcept {

// Decode messages and print/do stuff according to message received - only 1 message
   //      std::stringstream sstr{data};
   //      cluon::FromProtoVisitor decoder;
   //      decoder.decodeFrom(sstr);
   //      MyTestMessage1 receivedMsg;
   //      receivedMsg.accept(decoder);
   //      PrimeChecker pc;
   //      std::cout << receivedMsg.myValue() << " is " << (pc.isPrime(receivedMsg.myValue()) ? "" : "not") << " a prime." << std::endl;
   //  });

// Constantly sleep program to keep listening for data - basic listening
   // using namespace std::literals::chrono_literals;
   // while (receiver.isRunning()) {
   //      std::this_thread::sleep_for(1s);
   // }

// OD4 Session and messages do the UDP receiving and stuff already - encapsulated
// Handles multiple messages
   cluon::OD4Session od4(111,
        [](cluon::data::Envelope &&envelope) noexcept {
        if (envelope.dataType() == 2001) {
          MyTestMessage1 receivedMsg = cluon::extractMessage<MyTestMessage1>(std::move(envelope));

          PrimeChecker pc;
          std::cout << receivedMsg.myValue() << " is " << (pc.isPrime(receivedMsg.myValue()) ? "" : "not") << " a prime." << std::endl;
        }
    });

    uint16_t value;
    std::cout << "Enter a number to check: ";
    std::cin >> value;
    MyTestMessage1 msg;
    msg.myValue(value);

    od4.send(msg);

    return 0;
}
