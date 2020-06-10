/*
 * Copyright 2009-2011 Maarten Baert
 * maarten-baert@hotmail.com
 * http://www.maartenbaert.be/
 * 
 * This file is part of ExtremePhysics.
 * 
 * ExtremePhysics is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * ExtremePhysics is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ExtremePhysics. If not, see <http://www.gnu.org/licenses/>.
 * 
 * File: ep_DistanceJoint_VelocityConstraints.h
 * Solves distance joint velocity constraints.
 * Included in ep_World_SolveVelocityConstraints.cpp.
 */

#ifndef EP_DISTANCEJOINT_VELOCITYCONSTRAINTS_H
#define EP_DISTANCEJOINT_VELOCITYCONSTRAINTS_H

#include "ExtremePhysics.h"

EP_FORCEINLINE void ep_DistanceJoint::InitVelocityConstraints() {
	
	// limits
	if(limit1_maxforce!=0.0 || limit2_maxforce!=0.0) {
		
		if(limit1_maxforce!=0.0 && limit2_maxforce!=0.0 && limit1_distance>=limit2_distance) {
			targetlimitvel = (limit1_velocity+limit2_velocity)*0.5-world->baumgarte_factor/world->timestep*(distance-limit1_distance);
			maxlimitforce = +limit1_maxforce;
			minlimitforce = -limit2_maxforce;
		} else {
			double currdistancevel =
			   ((body1->xvel+yy1*body1->rotvel)-(body2->xvel+yy2*body2->rotvel))*vx
			  +((body1->yvel-xx1*body1->rotvel)-(body2->yvel-xx2*body2->rotvel))*vy;
			if(limit1_maxforce!=0.0 && distance<limit1_distance+limit_contact_threshold) {
				maxlimitforce = +limit1_maxforce;
				targetlimitvel = limit1_velocity-world->baumgarte_factor/world->timestep*(distance-limit1_distance)-ep_min(0.0, currdistancevel+limit_velocity_threshold)*limit1_restitution;
			} else {
				maxlimitforce = 0.0;
			}
			if(limit2_maxforce!=0.0 && distance>limit2_distance-limit_contact_threshold) {
				minlimitforce = -limit2_maxforce;
				targetlimitvel = limit2_velocity-world->baumgarte_factor/world->timestep*(distance-limit2_distance)+ep_min(0.0, -currdistancevel+limit_velocity_threshold)*limit2_restitution;
			} else {
				minlimitforce = 0.0;
			}
		}
		
	} else {
		minlimitforce = 0.0;
		maxlimitforce = 0.0;
	}
	
}

EP_FORCEINLINE void ep_DistanceJoint::ApplyImpulses() {
	
	if(body1->storejointimpulses && body2->storejointimpulses) {
		
		// clamp stored force
		motorforce = ep_clamp(-maxmotorforce, +maxmotorforce, motorforce);
		limitforce = ep_clamp(minlimitforce,  maxlimitforce, limitforce);
		
		double f = motorforce+limitforce;
		if(f!=0.0) {
			double fx = f*vx;
			double fy = f*vy;
			body1->_ApplyImpulse(xx1, yy1, +fx, +fy);
			body2->_ApplyImpulse(xx2, yy2, -fx, -fy);
		}
		
	} else {
		motorforce = 0.0;
		limitforce = 0.0;
	}
	
}

EP_FORCEINLINE void ep_DistanceJoint::SolveVelocityConstraints() {
	
	// motor/limits
	double currdistancevel =
	   ((body1->xvel+yy1*body1->rotvel)-(body2->xvel+yy2*body2->rotvel))*vx
	  +((body1->yvel-xx1*body1->rotvel)-(body2->yvel-xx2*body2->rotvel))*vy;
	double combinedforce;
	
	if(maxmotorforce!=0.0) {
		
		combinedforce = (motorvel-currdistancevel)*motormass;
		
		// always clamp the accumulated impulse
		if(motorforce+combinedforce<-maxmotorforce) {
			combinedforce = -maxmotorforce-motorforce;
			motorforce = -maxmotorforce;
		} else if(motorforce+combinedforce>maxmotorforce) {
			combinedforce = +maxmotorforce-motorforce;
			motorforce = +maxmotorforce;
		} else {
			motorforce += combinedforce;
		}
		
		currdistancevel += combinedforce/motormass;
		
	} else {
		combinedforce = 0.0;
	}
	
	if(minlimitforce!=maxlimitforce) {
		
		double f = (targetlimitvel-currdistancevel)*motormass;
		
		// always clamp the accumulated impulse
		if(limitforce+f<minlimitforce) {
			f = minlimitforce-limitforce;
			limitforce = minlimitforce;
		} else if(limitforce+f>maxlimitforce) {
			f = maxlimitforce-limitforce;
			limitforce = maxlimitforce;
		} else {
			limitforce += f;
		}
		
		combinedforce += f;
		
	}
	
	if(combinedforce!=0.0) {
		double fx = vx*combinedforce;
		double fy = vy*combinedforce;
		body1->_ApplyImpulse(xx1, yy1, +fx, +fy);
		body2->_ApplyImpulse(xx2, yy2, -fx, -fy);
	}
	
}

#endif // EP_DISTANCEJOINT_VELOCITYCONSTRAINTS_H

