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



int32_t main(int32_t argc, char **argv) {
	// Parse the arguments from the command line
	auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

	if ( (0 == commandlineArguments.count("cid")) || (0 != commandlineArguments.count("help")) )
	{
		std::cerr << argv[0] << " is a service that handles the input direction.  " << std::endl;
		std::cerr << "Usage:  " << argv[0] << " --cid=<CID of your OD4Session> [--verbose] [--help]" << std::endl;
		return -1;
   	}

	// od4 session declarartion
	cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};		

	if (0 == od4.isRunning()) {
	   std::cerr << "ERROR: No OD4Session running!!!" << std::endl;
	   return -2;
	}

   	const bool VERBOSE{commandlineArguments.count("verbose") != 0};

	
	//Safe to go - choose direction
	auto onSafeToGo{[&od4, VERBOSE](cluon::data::Envelope &&envelope)
	    {
		auto msg = cluon::extractMessage<StopSignPresenceUpdate>(std::move(envelope));
		char input = '0';

		std::cout << "Please enter direction for kiwi car. " << std::endl << 
			    "Enter 1 for turn right, 2 for going straight and 3 for turning left: " << std::endl;
		std::cin >> input;

		while(input != '1' && input != '2' && input != '3') //check for invalid input
		{
			std::cout << "Invalid input! Please try again.";				
			std::cin >> input;
		}

		int option = 0; 
		if (input == '1') option = 1; //maping input value to integer that will be sent via message
		else if (input == '2') option = 2;
		else option = 3;

		//sends the direction input to the Move Car		
		ChooseDirectionRequest directionRequest; 
		directionRequest.direction(option);
		od4.send(directionRequest);
	    }
	};
	od4.dataTrigger(SafeToGo::ID(), onSafeToGo);
 


	while(od4.isRunning()) {
	}

	return 0;
}
