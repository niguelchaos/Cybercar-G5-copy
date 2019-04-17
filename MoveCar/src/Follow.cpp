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

#include "cluon-complete.hpp"
#include "messages.hpp"

void SetSpeed(cluon::OD4Session& od4, float speed, bool VERBOSE)
{
	opendlv::proxy::PedalPositionRequest pedalReq;
	pedalReq.position(speed);
	od4.send(pedalReq);
	if (VERBOSE) std::cout << "Speed set to " << speed << std::endl;
}

void StopCar(cluon::OD4Session& od4, bool VERBOSE)
{
	SetSpeed(od4, 0.0, VERBOSE);
	if (VERBOSE) std::cout << "Now stop ..." << std::endl;
}

void MoveForward(cluon::OD4Session& od4, float speed, bool VERBOSE)
{
	SetSpeed(od4, speed, VERBOSE);
	if (VERBOSE) std::cout << "Now move forward ... " << std::endl;
}

void SetSteering(cluon::OD4Session& od4, float steer, bool VERBOSE)
{
	opendlv::proxy::GroundSteeringRequest steerReq;
        steerReq.groundSteering(steer);
        od4.send(steerReq);
        if (VERBOSE)
        {
            std::cout << "Sent GrounSeeringRequest message: " << steer << std::endl;
        }
}

void TurnLeft(cluon::OD4Session& od4, float steer, bool VERBOSE)
{
	SetSteering(od4, steer, VERBOSE);
}

void TurnRight(cluon::OD4Session& od4, float steer, bool VERBOSE)
{
	SetSteering(od4, steer, VERBOSE);
}

int32_t main(int32_t argc, char **argv) {

    // Parse the arguments from the command line
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

    if ( (0 == commandlineArguments.count("cid")) || (0 != commandlineArguments.count("help")) )
    {
        std::cerr << argv[0] << " is a first version of Kiwi car control. It is intended slowly move forward following the obstacle. " << std::endl;
        std::cerr << "Usage:  " << argv[0] << " --cid=<CID of your OD4Session> [--safetyDistance] [--speed] [--verbose] [--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << " --cid=112 --speed=1.5 --safetyDistance=1.5 --verbose" << std::endl;
	std::cerr << "example:  " << argv[0] << " --cid=112 --verbose" << std::endl;
        return -1;
    }
    else
    {
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

        if (0 == od4.isRunning())
        {
            std::cerr << "ERROR: No OD4Session running!!!" << std::endl;
            return -2;
        }
 
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
	const float SPEED{(commandlineArguments["speed"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["speed"])) : static_cast<float>(0.14)};
	const float SAFETYDISTANCE{(commandlineArguments["safetyDistance"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["safetyDistance"])) : static_cast<float>(0.15)};

        // A Data-triggered function to detect front obstacle and stop or move car accordingly
        float currentDistance{0.0};
        auto onFrontDistanceReading{[&od4, SPEED, SAFETYDISTANCE, VERBOSE, &currentDistance](cluon::data::Envelope &&envelope)
            // &<variables> will be captured by reference (instead of value only)
            {
                auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
                const uint16_t senderStamp = envelope.senderStamp(); // senderStamp 0 corresponds to front ultra-sound distance sensor
                currentDistance = msg.distance(); // Get the distance
                
		// proceed only if senderStamp is 0 (front sensor) 
		if(senderStamp == 0) {
			if (VERBOSE)
                	{
                    		std::cout << "Received DistanceReading message (senderStamp=" << senderStamp << "): " << currentDistance << std::endl;
                	}

			opendlv::proxy::PedalPositionRequest pedalReq;

			if(currentDistance > SAFETYDISTANCE) 
				MoveForward(od4, SPEED, VERBOSE);
			else 
				StopCar(od4, VERBOSE);
		}
            }
        };
        od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onFrontDistanceReading);

        while(od4.isRunning())
        {

	}
        return 0;
    }
}

