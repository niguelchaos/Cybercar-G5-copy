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
//bool stopCarSent = false;

std::chrono::time_point<std::chrono::system_clock> lastTimeZeroSpeed;
float previousSpeed = 0.1; // set previous speed to 0.1 as a start condition so that MoveForward function can see the change of speed to zero
bool standingStillForPeriodOfTime = false;

void SetSpeed(cluon::OD4Session& od4, float speed, bool VERBOSE)
{
	opendlv::proxy::PedalPositionRequest pedalReq;
	pedalReq.position(speed);
	od4.send(pedalReq);
	if (VERBOSE) std::cout << "[ Speed: " << speed << " ] //	" << std::endl;
}

void StopCar(cluon::OD4Session& od4, bool VERBOSE)
{
	SetSpeed(od4, 0.0, VERBOSE);
 	 if (VERBOSE) { std::cout << "		[ Now stop ...] " << std::endl; }
}

void MoveForward(cluon::OD4Session& od4, float speed, bool VERBOSE)
{
	SetSpeed(od4, speed, VERBOSE);
	// if (VERBOSE) std::cout << "Now move forward ... " << std::endl;
	
	if (standingStillForPeriodOfTime == false) { // The car was never still for a period of time
		if (previousSpeed != 0.0 && speed == 0.0) { 
			lastTimeZeroSpeed = std::chrono::system_clock::now(); // Take time stamp when car stopped
		} else if (previousSpeed == 0.0 && speed == 0.0) { 
			// Inspired by: https://en.cppreference.com/w/cpp/chrono/system_clock/now and https://en.cppreference.com/w/cpp/chrono
			auto now = std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = now-lastTimeZeroSpeed; // Calculate the time the car stands still
			if (elapsed_seconds.count() >= 7) {
				standingStillForPeriodOfTime = true;
	    			std::cout << "Not moving for 7 seconds. We are standing behind a car at the intersection. \n";
			}
		}

		previousSpeed = speed;
	}
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

void TurnLeft(cluon::OD4Session& od4, float steer, float speed, bool VERBOSE, int timer1, int timer2, int timer3)
{
	SetSteering(od4, 0.0, VERBOSE); //put wheels straight
	MoveForward(od4, speed, VERBOSE);
	std::this_thread::sleep_for(std::chrono::milliseconds(timer1));
	SetSteering(od4, steer, VERBOSE);
	std::this_thread::sleep_for(std::chrono::milliseconds(timer2));
	SetSteering(od4, 0.0, VERBOSE);
	std::this_thread::sleep_for(std::chrono::milliseconds(timer3));
	StopCar(od4, VERBOSE);
}


void TurnRight(cluon::OD4Session& od4, float steer, float speed, bool VERBOSE, int timer1, int timer2)
{
	steer = -steer; // GroundSteeringRequest received negative values for steering right. Argument steer must always be positive!
	SetSteering(od4, steer, VERBOSE);
	MoveForward(od4, speed, VERBOSE);
	std::this_thread::sleep_for(std::chrono::milliseconds(timer1));
	SetSteering(od4, 0.0, VERBOSE);
	std::this_thread::sleep_for(std::chrono::milliseconds(timer2));
	StopCar(od4, VERBOSE);
}

void GoStraight(cluon::OD4Session& od4, float speed, bool VERBOSE){

	SetSteering(od4, 0.0, VERBOSE); 		
	SetSpeed(od4, speed, VERBOSE);
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	StopCar(od4, VERBOSE);

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
		const float STARTSPEED{(commandlineArguments["startspeed"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["startspeed"])) : static_cast<float>(0.11)};
		const float MAXSPEED{(commandlineArguments["maxspeed"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["maxspeed"])) : static_cast<float>(0.25)};

		const float MAXSTEER{(commandlineArguments["maxsteer"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["maxsteer"])) : static_cast<float>(0.4)};
		const float MINSTEER{(commandlineArguments["minsteer"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["minsteer"])) : static_cast<float>(-0.4)};
		const float SAFETYDISTANCE{(commandlineArguments["safetyDistance"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["safetyDistance"])) : static_cast<float>(0.15)};

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
				MoveForward(od4, 0.0, VERBOSE); // Stop the car if obstacle is too close
				currentCarSpeed = 0.0;
				currentSteering = 0.0; // reset wheels too just in case
				SetSteering(od4, currentSteering, VERBOSE);
				std::cout << "Obstacle too close: " << currentDistance << std::endl;
				}
			}
       }
   };
	od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onFrontDistanceReading);


	//Bool message for stoping the car
   auto onStopCar{[&od4, VERBOSE](cluon::data::Envelope &&envelope)
            {
		//if (!stopCarSent) {
		auto msg = cluon::extractMessage<StopSignPresenceUpdate>(std::move(envelope));
		bool stopSignPresence = msg.stopSignPresence(); // Get the bool

			if (VERBOSE)
			{
		    		std::cout << "Received Stop car message: " << std::endl;
			}

			if (stopSignPresence==false){
				//stopCarSent = true;
				StopCar(od4, VERBOSE);
			}

		//}
	    }
        };
        od4.dataTrigger(StopSignPresenceUpdate::ID(), onStopCar);



// [Relative PID for speed correction]
	auto onSpeedCorrection{[&od4, VERBOSE, STARTSPEED, MAXSPEED](cluon::data::Envelope &&envelope)
	{
		if (!standingStillForPeriodOfTime) { // Don't listen corrections if car was still for period of time
			auto msg = cluon::extractMessage<SpeedCorrectionRequest>(std::move(envelope));
			float amount = msg.amount(); // Get the amount

			if (VERBOSE)
			{
		    		std::cout << "Received Speed Correction message: " << amount << std::endl;
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
		if (!standingStillForPeriodOfTime) { // Don't listen corrections if car was still for period of time
			auto msg = cluon::extractMessage<SteeringCorrectionRequest>(std::move(envelope));
		  	float amount = msg.amount(); // Get the amount
			if (VERBOSE)
			{
				std::cout << "Absolute Steering Correction: " << amount << std::endl;
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
				if (currentSteering > MAXSTEER) { currentSteering = MAXSTEER; }
				if (currentSteering < MINSTEER) { currentSteering = MINSTEER; }
			}
			SetSteering(od4, currentSteering, VERBOSE);
		}
	}
};

// triggers - ordering is probably important
       od4.dataTrigger(SteeringCorrectionRequest::ID(), onSteeringCorrection); //check steering correction first
	    od4.dataTrigger(SpeedCorrectionRequest::ID(), onSpeedCorrection);
	


	// Function to move forward to approach the stop line
	auto onCarOutOfSight{[&od4, VERBOSE, STARTSPEED ](cluon::data::Envelope &&envelope)
	{
		auto msg = cluon::extractMessage<CarOutOfSight>(std::move(envelope));
	  	
		if (standingStillForPeriodOfTime == true) {
			if (VERBOSE)
			{
				std::cout << "Car out of sight! Approach the stop line until stop sign is out of sight! " << std::endl;
			}

			SetSteering(od4, 0.0, VERBOSE);		
			MoveForward(od4, STARTSPEED, VERBOSE);
		}
	}};
	od4.dataTrigger(CarOutOfSight::ID(), onCarOutOfSight); 


		//Direction movments left /right /straight 
	auto onChooseDirectionRequest{[&od4, MAXSTEER, VERBOSE](cluon::data::Envelope &&envelope)
            {
		auto msg = cluon::extractMessage<ChooseDirectionRequest>(std::move(envelope));
		float direction = msg.direction(); // Get the amount

			if (VERBOSE)
			{
		    		std::cout << "Received Direction message: " << std::endl;
			}

			if (direction == 1) {			
			TurnRight(od4, MAXSTEER, 0.12, VERBOSE, 2000, 1500);
			}

			if (direction == 2) {
			GoStraight (od4, 0.12, VERBOSE);
			}
			
			else if (direction == 3) {
			TurnLeft(od4, MAXSTEER, 0.12, VERBOSE, 1500, 2000, 2000);
			}
	    }
        };
        od4.dataTrigger(ChooseDirectionRequest::ID(), onChooseDirectionRequest);


        while(od4.isRunning()) {
		
		}
		return 0;
	}
}
