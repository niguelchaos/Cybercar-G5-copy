/*
 * Copyright (C) 2019 Elsada Lagumdzic
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstdint>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include <string>

#include "cluon-complete.hpp"
#include "messages.hpp"

using namespace std;
using namespace cluon;

float currentCarSpeed = 0.0;
float currentSteering = 0.0;
bool stopCarSent = false;

void SetSpeed(cluon::OD4Session& od4, float speed, bool VERBOSE)
{
	opendlv::proxy::PedalPositionRequest pedalReq;
	pedalReq.position(speed);
	od4.send(pedalReq);
	if (VERBOSE) std::cout << "[ Speed: " << speed << " ] //	"<< std::endl;
}

void StopCar(cluon::OD4Session& od4, bool VERBOSE)
{
	SetSpeed(od4, 0.0, VERBOSE);
	// if (VERBOSE) { std::cout << "		[ Now stop ...] " << std::endl; }
}

void MoveForward(cluon::OD4Session& od4, float speed, bool VERBOSE)
{
	SetSpeed(od4, speed, VERBOSE);
	// if (VERBOSE) std::cout << "Now move forward ... " << std::endl;
}

void SetSteering(cluon::OD4Session& od4, float steer, bool VERBOSE)
{
	opendlv::proxy::GroundSteeringRequest steerReq;
        steerReq.groundSteering(steer);
        od4.send(steerReq);
        if (VERBOSE)
        {
            std::cout << "GroundSteeringRequest: " << steer << std::endl;
        }
}

void TurnLeft(cluon::OD4Session& od4, float steer, float speed, bool VERBOSE)
{
	SetSteering(od4, steer, VERBOSE);
	MoveForward(od4, speed, VERBOSE);
}

void TurnRight(cluon::OD4Session& od4, float steer, float speed, bool VERBOSE)
{
	SetSteering(od4, steer, VERBOSE);
	MoveForward(od4, speed, VERBOSE);
}

int32_t main(int32_t argc, char **argv) {

	// Parse the arguments from the command line
	auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

	if ( (0 == commandlineArguments.count("cid")) || (0 != commandlineArguments.count("help")) )
	{
		std::cerr << argv[0] << " is a first version of Kiwi car control. It is intended slowly move forward following the obstacle. " << std::endl;
		std::cerr << "Usage:  " << argv[0] << " --cid=<CID of your OD4Session> [--safetyDistance] [--speed] [--verbose] [--help]" << std::endl;
		std::cerr << "example:  " << argv[0] << " --cid=112 --speed=1.5 --safetyDistance=1.5 --speedIncrement=0.01 -- steerIncrement=0.01 --verbose" << std::endl;
		std::cerr << "example:  " << argv[0] << " --cid=112 --verbose" << std::endl;
		return -1;
   }
	else {
		cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

		if (0 == od4.isRunning()) {
		   std::cerr << "ERROR: No OD4Session running!!!" << std::endl;
		   return -2;
		}

	   const bool VERBOSE{commandlineArguments.count("verbose") != 0};
		const float STARTSPEED{(commandlineArguments["startspeed"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["startspeed"])) : static_cast<float>(0.10)};
		const float MAXSPEED{(commandlineArguments["maxspeed"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["maxspeed"])) : static_cast<float>(0.13)};

		const float MAXSTEER{(commandlineArguments["maxsteer"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["maxsteer"])) : static_cast<float>(0.4)};
		const float MINSTEER{(commandlineArguments["minsteer"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["minsteer"])) : static_cast<float>(-0.4)};
		const float SAFETYDISTANCE{(commandlineArguments["safetyDistance"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["safetyDistance"])) : static_cast<float>(0.01)};

      // A Data-triggered function to detect front obstacle and stop or move car accordingly
      float currentDistance{0.0};
      auto onFrontDistanceReading{ [&od4, SAFETYDISTANCE, VERBOSE, &currentDistance](cluon::data::Envelope &&envelope)
      { // &<variables> will be captured by reference (instead of value only)
	      auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
			// senderStamp 0 corresponds to front ultra-sound distance sensor
	      const uint16_t senderStamp = envelope.senderStamp();
	      currentDistance = msg.distance(); // Get the distance

		// proceed only if senderStamp is 0 (front sensor)
			if(senderStamp == 0) {
				if (VERBOSE) {
	            // std::cout << "Received DistanceReading message (senderStamp=" << senderStamp << "): " << currentDistance << std::endl;
	        	}
				if (currentDistance <= SAFETYDISTANCE) {
				StopCar(od4, VERBOSE); // Stop the car if obstacle is too close
				currentCarSpeed = 0.0;
				currentSteering = 0.0; // reset wheels too just in case
				SetSteering(od4, currentSteering, VERBOSE);
				std::cout << "Obstacle too close: " << currentDistance << std::endl;
				}
			}
       }
   };
	od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onFrontDistanceReading);

		// Receive StopCar message and stops the car. This is reaction on stop sign detection
// 		auto onStopCar{ [&od4, VERBOSE](cluon::data::Envelope&&) {
// 			if (!stopCarSent) {
// 				if (VERBOSE) {
// 					std::cout << "Received Stop car message: " << std::endl;
// 				}
// 				stopCarSent = true;
// 				StopCar(od4, VERBOSE);
// 			}
// 		}
// 	};
//   od4.dataTrigger(StopCarRequest::ID(), onStopCar);


	//Bool message for stoping the car
   auto onStopCar{[&od4, VERBOSE](cluon::data::Envelope &&envelope)
            {
		if (!stopCarSent) {
		auto msg = cluon::extractMessage<StopSignPresenceUpdate>(std::move(envelope));
		bool stopSignPresence = msg.stopSignPresence(); // Get the bool

			if (VERBOSE)
			{
		    		std::cout << "Received Stop car message: " << std::endl;
			}

			if (stopSignPresence==true){


			stopCarSent = true;
			StopCar(od4, VERBOSE);
			}


			/*if (stopSignPresence==false){

			MoveForward (od4, 0.13, VERBOSE);

			}*/
		}
	    }
        };
        od4.dataTrigger(StopSignPresenceUpdate::ID(), onStopCar);


// [Relative PID for speed correction]
	auto onSpeedCorrection{[&od4, VERBOSE, STARTSPEED, MAXSPEED](cluon::data::Envelope &&envelope)
	{
		if (!stopCarSent) {
			auto msg = cluon::extractMessage<SpeedCorrectionRequest>(std::move(envelope));
			float amount = msg.amount(); // Get the amount

			if (VERBOSE)
			{
		    		std::cout << "Received Speed Correction message: " << amount << std::endl;
			}
			if (amount == 1337) {
				if (currentCarSpeed >= STARTSPEED) {
					currentCarSpeed += -0.002; // car will eventually come to a stop
					cout << "Visual Lost, reducing speed" << endl;
				}
			}

			if ( currentCarSpeed < STARTSPEED && amount > 0) 	{ currentCarSpeed = STARTSPEED;	}// Set car speed to minimal moving car speed
			if ( currentCarSpeed < STARTSPEED && amount < 0) 	{ currentCarSpeed = 0.0;	} // automatically makes it 0, preventing car from moving backwards
			else {
				currentCarSpeed += amount;
				if (currentCarSpeed > MAXSPEED) 	{ currentCarSpeed = MAXSPEED;	} // limit the speed car can go
				if (currentCarSpeed < STARTSPEED){ currentCarSpeed = 0;			} // prevent the car from going backwards. Twice.
			}
			MoveForward(od4, currentCarSpeed, VERBOSE);
		}
	}
};
// (Data trigger below)
// [Absolute pid for speed was too fast]
// Absolute pid steering
	auto onSteeringCorrection{[&od4, VERBOSE, MAXSTEER, MINSTEER, MAXSPEED, STARTSPEED ](cluon::data::Envelope &&envelope)
	{
		if (!stopCarSent) {
			auto msg = cluon::extractMessage<SteeringCorrectionRequest>(std::move(envelope));
		  	float amount = msg.amount(); // Get the amount
			if (VERBOSE)
			{
				std::cout << "Absolute Steering Correction: " << amount << std::endl;
			}

			if (amount == 1337) { // slowly straighten back out the wheels if visual lost
				if (currentSteering < 0) {
					currentSteering += 0.05;
				}
				if (currentSteering > 0) {
					currentSteering += -0.05;
				}
			}

			// check if...
			if (amount >= -0.05 && amount < 0.05 && // and acc car is relatively in front...
			    (currentSteering <= -0.05 || currentSteering > 0.05) && // and wheels are not straight...
				currentCarSpeed < STARTSPEED - 0.05) // and car is stopped...
			{
				currentSteering = 0; // ...then reset wheels
				cout << "Steering Reset." << endl;
			}
			else {
				currentSteering = amount;
				if (currentSteering > MAXSTEER) {
					currentSteering = MAXSTEER;

					if (currentCarSpeed > STARTSPEED) { // slow down the car to give the car time to make it less blurry
						currentCarSpeed = currentCarSpeed - 0.003;
					}
				}
				if (currentSteering < MINSTEER) { currentSteering = MINSTEER; }
			}
			SetSteering(od4, currentSteering, VERBOSE);
		}
	}
};

// triggers - ordering is probably important
       od4.dataTrigger(SteeringCorrectionRequest::ID(), onSteeringCorrection); //check steering correction first
	    od4.dataTrigger(SpeedCorrectionRequest::ID(), onSpeedCorrection);

        while(od4.isRunning()) {
// only sends messages

		/*HelloWorld helloWorld;
		helloWorld.helloWorld("i HATE CAR");
		od4.send(helloWorld);
		if (VERBOSE) std::cout << "Hello World sent (i HATE CAR)" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		SpeedCorrectionRequest speedCorrection;
		speedCorrection.amount(1);
		od4.send(speedCorrection);

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		SteeringCorrectionRequest steeringCorrection;
		steeringCorrection.amount(-1);
		od4.send(steeringCorrection);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		StopCarRequest stopCar;
		od4.send(stopCar);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		SpeedCorrectionRequest speedCorrection2;
		speedCorrection2.amount(-1);
		od4.send(speedCorrection2);


		StopSignPresenceUpdate stopSign;
		stopSign.stopSignPresence(true);
		od4.send(stopSign);*/

	/*if (!stopCarSent){
	MoveForward (od4, 0.13, VERBOSE);
	}*/
		}
		return 0;
	}
}
