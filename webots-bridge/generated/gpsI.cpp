/*
 *    Copyright (C) 2026 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "gpsI.h"

GPSI::GPSI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
}

GPSI::~GPSI()
{
}

bool GPSI::getData(float &latitude, float &longitude, float &altitude, const Ice::Current&)
{
	if (!worker)
		throw std::runtime_error("Worker is null");
	#ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
	return worker->GPS_getData(latitude, longitude, altitude);
}

bool GPSI::getPos(float &x, float &y, float &z, const Ice::Current&)
{
	if (!worker)
		throw std::runtime_error("Worker is null");
	#ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
	return worker->GPS_getPos(x, y, z);
}

bool GPSI::getUTMData(int &xzone, std::string &yzone, double &eastering, double &northing, const Ice::Current&)
{
	if (!worker)
		throw std::runtime_error("Worker is null");
	#ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
	return worker->GPS_getUTMData(xzone, yzone, eastering, northing);
}

void GPSI::resetPos(const Ice::Current&)
{
	if (!worker)
		throw std::runtime_error("Worker is null");
	#ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
	worker->GPS_resetPos();
}
