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
#ifndef GPS_H
#define GPS_H

// Ice includes
#include <Ice/Ice.h>
#include <GPS.h>
#include <QPointer>

#include "../src/specificworker.h"


class GPSI : public virtual RoboCompGPS::GPS
{
public:
	GPSI(GenericWorker *_worker, const size_t id);
	~GPSI();

	bool getData(float &latitude, float &longitude, float &altitude, const Ice::Current&);
	bool getPos(float &x, float &y, float &z, const Ice::Current&);
	bool getUTMData(int &xzone, std::string &yzone, double &eastering, double &northing, const Ice::Current&);
	void resetPos(const Ice::Current&);

private:

	QPointer<GenericWorker> worker;
	size_t id;

};

#endif
