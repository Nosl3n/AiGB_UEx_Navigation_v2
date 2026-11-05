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
#include "fullposeestimationI.h"

FullPoseEstimationI::FullPoseEstimationI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	getFullPoseEulerHandlers = {
		[this]() -> RoboCompFullPoseEstimation::FullPoseEuler {if (worker != nullptr) return worker->FullPoseEstimation_getFullPoseEuler(); else throw std::runtime_error("Worker is null");}
	};

	getFullPoseMatrixHandlers = {
		[this]() -> RoboCompFullPoseEstimation::FullPoseMatrix {if (worker != nullptr) return worker->FullPoseEstimation_getFullPoseMatrix(); else throw std::runtime_error("Worker is null");}
	};

	setInitialPoseHandlers = {
		[this](auto &a, auto &b, auto &c, auto &d, auto &e, auto &f) {if (worker != nullptr) worker->FullPoseEstimation_setInitialPose(a, b, c, d, e, f); else throw std::runtime_error("Worker is null");}
	};

}

FullPoseEstimationI::~FullPoseEstimationI()
{
}

RoboCompFullPoseEstimation::FullPoseEuler FullPoseEstimationI::getFullPoseEuler(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getFullPoseEulerHandlers.at(id)();
}

RoboCompFullPoseEstimation::FullPoseMatrix FullPoseEstimationI::getFullPoseMatrix(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getFullPoseMatrixHandlers.at(id)();
}

void FullPoseEstimationI::setInitialPose(float x, float y, float z, float rx, float ry, float rz, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	setInitialPoseHandlers.at(id)(x, y, z, rx, ry, rz);
}