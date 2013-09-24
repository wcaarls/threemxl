// Dynamixel control code - header file
// Copyright (c) 2008 Erik Schuitema, Eelko van Breda
// Delft University of Technology
// www.dbl.tudelft.nl

#include "CDxlGeneric.h"
#include "dynamixel/Dynamixel.h"
#include "3mxl/3mxl.h"

/**
 * gDxlCreate creates the correct object based on a sting. This is easily used
 * for if you want to create an object based on an config file.
 * @param dxlTypeStr an std::string that describes the object to be made
 * @return generic Dxl pointer
 */
CDxlGeneric* gDxlCreate(const std::string& dxlTypeStr)
{
	CDxlGeneric* newDxl = NULL;

	if (dxlTypeStr == "Robotis_RXxx")
		newDxl = new CDynamixel();
	else if (dxlTypeStr == "3MXL")
		newDxl = new C3mxl();
	else
	  printf("[gDxlCreate] Unknown servo type \"%s\"!\n", dxlTypeStr.c_str());

	return newDxl;
}
