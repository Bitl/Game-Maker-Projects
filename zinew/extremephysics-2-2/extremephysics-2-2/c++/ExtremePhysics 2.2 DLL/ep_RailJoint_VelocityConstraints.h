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
 * File: ep_RailJoint_VelocityConstraints.h
 * Solves rail joint velocity constraints.
 * Included in ep_World_SolveVelocityConstraints.cpp.
 */

#ifndef EP_RAILJOINT_VELOCITYCONSTRAINTS_H
#define EP_RAILJOINT_VELOCITYCONSTRAINTS_H

#include "ExtremePhysics.h"

EP_FORCEINLINE void ep_RailJoint::InitVelocityConstraints() {
	
	// limits
	if(limit1_maxforce!=0.0 || limit2_maxforce!=0.0) {
		
		if(limit1_maxforce!=0.0 && limit2_maxforce!=0.0 && limit1_position>=limit2_position) {
			targetlimitvel = (limit1_velocity+limit2_velocity)*0.5-world->baumgarte_factor/world->timestep*(position-limit1_position);
			maxlimitforce = +limit1_maxforce;
			minlimitforce = -limit2_maxforce;
		} else {
			double currvel =
			   ((body1->xvel+yy1*body1->rotvel)-(body2->xvel+yy2*body2->rotvel))*vxx
			  +((body1->yvel-xx1*body1->rotvel)-(body2->yvel-xx2*body2->rotvel))*vyy;
			if(limit1_maxforce!=0.0 && position<limit1_position+limit_contact_threshold) {
				maxlimitforce = +limit1_maxforce;
				targetlimitvel = limit1_velocity-world->baumgarte_factor/world->timestep*(position-limit1_position)-ep_min(0.0, currvel+limit_velocity_threshold)*limit1_restitution;
			} else {
				maxlimitforce = 0.0;
			}
			if(limit2_maxforce!=0.0 && position>limit2_position-limit_contact_threshold) {
				minlimitforce = -limit2_maxforce;
				targetlimitvel = limit2_velocity-world->baumgarte_factor/world->timestep*(position-limit2_position)+ep_min(0.0, -currvel+limit_velocity_threshold)*limit2_restitution;
			} else {
				minlimitforce = 0.0;
			}
		}
		
	} else {
		minlimitforce = 0.0;
		maxlimitforce = 0.0;
	}
	
	// calculate mass
	{
		double r1 = yy1*vyy+xx1*vxx;
		double r2 = yy2*vyy+xx2*vxx;
		normalmass = (1.0+world->mass_bias)/(
		  body1->invmass+body2->invmass
		  +ep_sqr(r1)*body1->invinertia
		  +ep_sqr(r2)*body2->invinertia);
	}
	
}

EP_FORCEINLINE void ep_RailJoint::ApplyImpulses() {
	
	if(body1->storejointimpulses && body2->storejointimpulses) {
		
		// clamp stored force
		if(maxnormalforce!=0.0) {
			normalforce = ep_clamp(-maxnormalforce, +maxnormalforce, normalforce);
		}
		motorforce = ep_clamp(-maxmotorforce, +maxmotorforce, motorforce);
		limitforce = ep_clamp(minlimitforce,  maxlimitforce, limitforce);
		
		double f = motorforce+limitforce;
		double fx = f*vxx+normalforce*vyy;
		double fy = f*vyy-normalforce*vxx;
		body1->_ApplyImpulse(xx1, yy1, +fx, +fy);
		body2->_ApplyImpulse(xx2, yy2, -fx, -fy);
		
	} else {
		normalforce = 0.0;
		motorforce = 0.0;
		limitforce = 0.0;
	}
	
}

EP_FORCEINLINE void ep_RailJoint::SolveVelocityConstraints() {
	
	// motor/limits
	{
		double currvel =
		   ((body1->xvel+yy1*body1->rotvel)-(body2->xvel+yy2*body2->rotvel))*vxx
		  +((body1->yvel-xx1*body1->rotvel)-(body2->yvel-xx2*body2->rotvel))*vyy;
		double combinedforce;
		
		if(maxmotorforce!=0.0) {
			
			combinedforce = (motorvel-currvel)*motormass;
			
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
			
			currvel += combinedforce/motormass;
			
		} else {
			combinedforce = 0.0;
		}
		
		if(minlimitforce!=maxlimitforce) {
			
			double f = (targetlimitvel-currvel)*motormass;
			
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
			double fx = vxx*combinedforce;
			double fy = vyy*combinedforce;
			body1->_ApplyImpulse(xx1, yy1, +fx, +fy);
			body2->_ApplyImpulse(xx2, yy2, -fx, -fy);
		}
	}
	
	// normal constraint
	double currnormalvel =
	   ((body1->xvel+yy1*body1->rotvel)-(body2->xvel+yy2*body2->rotvel))*vyy
	  -((body1->yvel-xx1*body1->rotvel)-(body2->yvel-xx2*body2->rotvel))*vxx;
	double f = (targetnormalvel-currnormalvel)*normalmass;
	
	// f and normalforce are usually positive
	// always clamp the accumulated impulse
	if(maxnormalforce==0.0) {
		normalforce += f;
	} else {
		if(f<-maxnormalforce-normalforce) {
			f = -maxnormalforce-normalforce;
			normalforce = -maxnormalforce;
		} else if(f>+maxnormalforce-normalforce) {
			f = +maxnormalforce-normalforce;
			normalforce = +maxnormalforce;
		} else {
			normalforce += f;
		}
	}
	
	double fx = +f*vyy;
	double fy = -f*vxx;
	body1->_ApplyImpulse(xx1, yy1, +fx, +fy);
	body2->_ApplyImpulse(xx2, yy2, -fx, -fy);
	
}

#endif // EP_RAILJOINT_VELOCITYCONSTRAINTS_H

