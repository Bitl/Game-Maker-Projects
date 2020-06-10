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
 * File: ep_Contact_VelocityConstraints.h
 * Solves contact velocity constraints.
 * Included in ep_World_SolveVelocityConstraints.cpp.
 */

#ifndef EP_CONTACT_VELOCITYCONSTRAINTS_H
#define EP_CONTACT_VELOCITYCONSTRAINTS_H

#include "ExtremePhysics.h"

EP_FORCEINLINE void ep_Contact::InitCPVelocityConstraints(ep_ContactPoint* cp, double* r1, double* r2, double* factor) {
	
	// calculate normal mass
	// The derivation is at the bottom of the file.
	{
		*r1 = cp->yy1*nx-cp->xx1*ny;
		*r2 = cp->yy2*nx-cp->xx2*ny;
		*factor = body1->invmass+body2->invmass
			+ep_sqr(*r1)*body1->invinertia
			+ep_sqr(*r2)*body2->invinertia;
		cp->normalmass = (1.0+world->mass_bias)/(*factor);
	}
	
	// calculate tangent mass
	{
		double tr1 = cp->yy1*ny+cp->xx1*nx;
		double tr2 = cp->yy2*ny+cp->xx2*nx;
		cp->tangentmass = (1.0+world->mass_bias)/(
			body1->invmass+body2->invmass
			+ep_sqr(tr1)*body1->invinertia
			+ep_sqr(tr2)*body2->invinertia);
	}
	
	// init target velocities
	cp->normaltargetvel = shape1->normalvelocity+shape2->normalvelocity;
	cp->tangenttargetvel = shape1->tangentvelocity+shape2->tangentvelocity;
	
	// baumgarte
	{
		double sep =
			((body1->x+cp->xx1)-(body2->x+cp->xx2))*nx+
			((body1->y+cp->yy1)-(body2->y+cp->yy2))*ny;
		// Use a small 'reversed' bias factor so forces can't build
		// up. Usually 'sep' is negative though.
		cp->normaltargetvel -= world->baumgarte_factor/world->timestep*((sep<0.0)? sep : sep*0.01);
	}
	
	{
		// calculate current normal velocity
		double currnormalvel =
			((body1->xvel+cp->yy1*body1->rotvel)-(body2->xvel+cp->yy2*body2->rotvel))*nx+
			((body1->yvel-cp->xx1*body1->rotvel)-(body2->yvel-cp->xx2*body2->rotvel))*ny;
		// restitution
		cp->normaltargetvel -= ep_min(0.0, currnormalvel+world->velocity_threshold)*restitution;
		// store speed error
		cp->normalveldelta = cp->normaltargetvel-currnormalvel;
	}
	
	{
		// calculate current tangent velocity
		double currtangentvel =
			((body1->xvel+cp->yy1*body1->rotvel)-(body2->xvel+cp->yy2*body2->rotvel))*ny-
			((body1->yvel-cp->xx1*body1->rotvel)-(body2->yvel-cp->xx2*body2->rotvel))*nx;
		// store speed error
		cp->tangentveldelta = cp->tangenttargetvel-currtangentvel;
	}
	
}

EP_FORCEINLINE void ep_Contact::InitVelocityConstraints() {
	
	//world->mass_bias = 0.3;
	
	restitution = (double)(sqrt(shape1->restitution*shape2->restitution));
	friction = (double)(sqrt(shape1->friction*shape2->friction));
	
	double r1[2], r2[2], factor[3];
	if(contactpoints[0].active) {
		InitCPVelocityConstraints(&contactpoints[0], &r1[0], &r2[0], &factor[0]);
	}
	if(contactpoints[1].active) {
		InitCPVelocityConstraints(&contactpoints[1], &r1[1], &r2[1], &factor[1]);
	}
	
	// use block solver?

	if(contactpoints[0].active && contactpoints[1].active && (!body1->norotation || !body2->norotation)) {
		
		factor[2] = body1->invmass+body2->invmass
			+r1[0]*r1[1]*body1->invinertia
			+r2[0]*r2[1]*body2->invinertia;
		
		// invert (create mass matrix)
		// Theoretically, d is always positive if (r1[0]-r1[1])!=0 or (r2[0]-r2[1])!=0,
		// but in reality d could still become zero (or even negative) because of rounding errors.
		double d = factor[0]*factor[1]-ep_sqr(factor[2]);
		if(d<EP_EPSILON_MINBLOCKDET) {
			useblocksolver = false;
		} else {
			useblocksolver = true;
			cpfactor[0] = factor[2]/factor[0];
			cpfactor[1] = factor[2]/factor[1];
			d = (1.0+world->mass_bias)/d;
			massmatrix11 = d*factor[1];
			massmatrix12 = -d*factor[2];
			massmatrix22 = d*factor[0];
		}
		
	} else {
		useblocksolver = false;
	}
	
}

EP_FORCEINLINE void ep_Contact::ApplyCPImpulses(ep_ContactPoint* cp) {
	
	if(body1->storecontactimpulses && body2->storecontactimpulses) {
		
		// clamp stored impulses
		double d = friction*cp->normalforce;
		cp->tangentforce = ep_clamp(-d, d, cp->tangentforce);
		
		double fx = cp->normalforce*nx+cp->tangentforce*ny;
		double fy = cp->normalforce*ny-cp->tangentforce*nx;
		body1->_ApplyImpulse(cp->xx1, cp->yy1, +fx, +fy);
		body2->_ApplyImpulse(cp->xx2, cp->yy2, -fx, -fy);
		
	} else {
		cp->normalforce = 0.0;
		cp->tangentforce = 0.0;
	}
	
}

EP_FORCEINLINE void ep_Contact::ApplyImpulses() {
	
	if(contactpoints[0].active) {
		ApplyCPImpulses(&contactpoints[0]);
	}
	if(contactpoints[1].active) {
		ApplyCPImpulses(&contactpoints[1]);
	}
	
}

EP_FORCEINLINE void ep_Contact::BlockSolveCPNormalVelocityConstraints() {
	
	/*
	The block solver solves two normal constraints at the same time. There are
	2 possible solutions for every contact point:
	- F>=0 and V==0
	- F==0 and V>=0
	For two contact points, there are four possible solutions.
	*/
	
	// first try to solve the 2x2 problem
	double normalvelerror[2];
	normalvelerror[0] = contactpoints[0].normaltargetvel-(
	   ((body1->xvel+contactpoints[0].yy1*body1->rotvel)-(body2->xvel+contactpoints[0].yy2*body2->rotvel))*nx
	  +((body1->yvel-contactpoints[0].xx1*body1->rotvel)-(body2->yvel-contactpoints[0].xx2*body2->rotvel))*ny);
	normalvelerror[1] = contactpoints[1].normaltargetvel-(
	   ((body1->xvel+contactpoints[1].yy1*body1->rotvel)-(body2->xvel+contactpoints[1].yy2*body2->rotvel))*nx
	  +((body1->yvel-contactpoints[1].xx1*body1->rotvel)-(body2->yvel-contactpoints[1].xx2*body2->rotvel))*ny);
	double ff[2];
	ff[0] = normalvelerror[0]*massmatrix11+normalvelerror[1]*massmatrix12,
	ff[1] = normalvelerror[0]*massmatrix12+normalvelerror[1]*massmatrix22;
	
	// are the forces positive?
	if(contactpoints[0].normalforce+ff[0]>=0.0 && contactpoints[1].normalforce+ff[1]>=0.0) {
		// F>=0 for both contact points.
		// This solution is valid so we're done.
		// Just apply the forces.
		if(ff[0]!=0.0) {
			double fx = ff[0]*nx;
			double fy = ff[0]*ny;
			body1->_ApplyImpulse(contactpoints[0].xx1, contactpoints[0].yy1, +fx, +fy);
			body2->_ApplyImpulse(contactpoints[0].xx2, contactpoints[0].yy2, -fx, -fy);
			contactpoints[0].normalforce += ff[0];
		}
		if(ff[1]!=0.0) {
			double fx = ff[1]*nx;
			double fy = ff[1]*ny;
			body1->_ApplyImpulse(contactpoints[1].xx1, contactpoints[1].yy1, +fx, +fy);
			body2->_ApplyImpulse(contactpoints[1].xx2, contactpoints[1].yy2, -fx, -fy);
			contactpoints[1].normalforce += ff[1];
		}
		return;
	}
	
	// is the first force negative?
	if(contactpoints[0].normalforce+ff[0]<0.0) {
		// The force of contact point 1 is too low: reduce the
		// force to zero, drop the constraint and solve for
		// contact point 2 only. The velocity at contact
		// point 1 is guaranteed to be positive.
		double f = normalvelerror[1]*contactpoints[1].normalmass+contactpoints[0].normalforce*cpfactor[1];
		// is the second force positive?
		if(contactpoints[1].normalforce+f>=0.0) {
			// This solution is valid so we're done.
			// Reduce force 1 to zero and apply this force to contact point 2.
			if(contactpoints[0].normalforce!=0.0) {
				double fx = -contactpoints[0].normalforce*nx;
				double fy = -contactpoints[0].normalforce*ny;
				body1->_ApplyImpulse(contactpoints[0].xx1, contactpoints[0].yy1, +fx, +fy);
				body2->_ApplyImpulse(contactpoints[0].xx2, contactpoints[0].yy2, -fx, -fy);
				contactpoints[0].normalforce = 0.0;
			}
			if(f!=0.0) {
				double fx = f*nx;
				double fy = f*ny;
				body1->_ApplyImpulse(contactpoints[1].xx1, contactpoints[1].yy1, +fx, +fy);
				body2->_ApplyImpulse(contactpoints[1].xx2, contactpoints[1].yy2, -fx, -fy);
				contactpoints[1].normalforce += f;
			}
			return;
		}
	}
	
	// is the second force negative?
	if(contactpoints[1].normalforce+ff[1]<0.0) {
		// The force of contact point 2 is too low: reduce the
		// force to zero, drop the constraint and solve for
		// contact point 1 only. The velocity at contact
		// point 2 is guaranteed to be positive.
		double f = normalvelerror[0]*contactpoints[0].normalmass+contactpoints[1].normalforce*cpfactor[0];
		if(contactpoints[0].normalforce+f>=0.0) {
			// This solution is valid so we're done.
			// Reduce force 1 to zero and apply this force to contact point 2.
			if(contactpoints[1].normalforce!=0.0) {
				double fx = -contactpoints[1].normalforce*nx;
				double fy = -contactpoints[1].normalforce*ny;
				body1->_ApplyImpulse(contactpoints[1].xx1, contactpoints[1].yy1, +fx, +fy);
				body2->_ApplyImpulse(contactpoints[1].xx2, contactpoints[1].yy2, -fx, -fy);
				contactpoints[1].normalforce = 0.0;
			}
			if(f!=0.0) {
				double fx = f*nx;
				double fy = f*ny;
				body1->_ApplyImpulse(contactpoints[0].xx1, contactpoints[0].yy1, +fx, +fy);
				body2->_ApplyImpulse(contactpoints[0].xx2, contactpoints[0].yy2, -fx, -fy);
				contactpoints[0].normalforce += f;
			}
			return;
		}
	}
	
	// just reduce the forces to zero
	if(contactpoints[0].normalforce!=0.0) {
		double fx = -contactpoints[0].normalforce*nx;
		double fy = -contactpoints[0].normalforce*ny;
		body1->_ApplyImpulse(contactpoints[0].xx1, contactpoints[0].yy1, +fx, +fy);
		body2->_ApplyImpulse(contactpoints[0].xx2, contactpoints[0].yy2, -fx, -fy);
		contactpoints[0].normalforce = 0.0;
	}
	if(contactpoints[1].normalforce!=0.0) {
		double fx = -contactpoints[1].normalforce*nx;
		double fy = -contactpoints[1].normalforce*ny;
		body1->_ApplyImpulse(contactpoints[1].xx1, contactpoints[1].yy1, +fx, +fy);
		body2->_ApplyImpulse(contactpoints[1].xx2, contactpoints[1].yy2, -fx, -fy);
		contactpoints[1].normalforce = 0.0;
	}
	
}

EP_FORCEINLINE void ep_Contact::SolveCPNormalVelocityConstraint(ep_ContactPoint* cp) {
	
	// normal contact constraints
	double currnormalvel =
	   ((body1->xvel+cp->yy1*body1->rotvel)-(body2->xvel+cp->yy2*body2->rotvel))*nx
	  +((body1->yvel-cp->xx1*body1->rotvel)-(body2->yvel-cp->xx2*body2->rotvel))*ny;
	double f = (cp->normaltargetvel-currnormalvel)*cp->normalmass;
	
	// f and normalforce are usually positive
	// always clamp the accumulated impulse
	if(f<-cp->normalforce) {
		f = -cp->normalforce;
		cp->normalforce = 0.0;
	} else {
		cp->normalforce += f;
	}
	
	if(f!=0.0) {
		double fx = f*nx;
		double fy = f*ny;
		body1->_ApplyImpulse(cp->xx1, cp->yy1, +fx, +fy);
		body2->_ApplyImpulse(cp->xx2, cp->yy2, -fx, -fy);
	}
	
}

EP_FORCEINLINE void ep_Contact::SolveCPTangentVelocityConstraint(ep_ContactPoint* cp) {
	
	// tangent contact constraints (friction)
	double currtangentvel =
	   ((body1->xvel+cp->yy1*body1->rotvel)-(body2->xvel+cp->yy2*body2->rotvel))*ny
	  -((body1->yvel-cp->xx1*body1->rotvel)-(body2->yvel-cp->xx2*body2->rotvel))*nx;
	double f = (cp->tangenttargetvel-currtangentvel)*cp->tangentmass;
	
	// always clamp the accumulated impulse
	double bound = friction*cp->normalforce;
	if(cp->tangentforce+f<-bound) {
		f = -cp->tangentforce-bound;
		cp->tangentforce = -bound;
	} else if(cp->tangentforce+f>+bound) {
		f = -cp->tangentforce+bound;
		cp->tangentforce = +bound;
	} else {
		cp->tangentforce += f;
	}
	
	if(f!=0.0) {
		double fx = +f*ny;
		double fy = -f*nx;
		body1->_ApplyImpulse(cp->xx1, cp->yy1, +fx, +fy);
		body2->_ApplyImpulse(cp->xx2, cp->yy2, -fx, -fy);
	}
	
}

EP_FORCEINLINE void ep_Contact::SolveVelocityConstraints() {
	
	// normal contact constraints
	if(useblocksolver) {
		BlockSolveCPNormalVelocityConstraints();
	} else {
		if(contactpoints[0].active) {
			SolveCPNormalVelocityConstraint(&contactpoints[0]);
		}
		if(contactpoints[1].active) {
			SolveCPNormalVelocityConstraint(&contactpoints[1]);
		}
	}
	
	if(contactpoints[0].active) {
		SolveCPTangentVelocityConstraint(&contactpoints[0]);
	}
	if(contactpoints[1].active) {
		SolveCPTangentVelocityConstraint(&contactpoints[1]);
	}
	
}

/*
Contact point derivation notes:

normal constraint: relative speed at contact point <dot> contact normal is always zero or greater than zero

fx = f*nx;
fy = f*ny;

delta_xspeed = fx/mass1+fx/mass2+( yy1)*(fx*yy1-fy*xx1)/inertia1+( yy2)*(fx*yy2-fy*xx2)/inertia2;
delta_yspeed = fy/mass1+fy/mass2+(-xx1)*(fx*yy1-fy*xx1)/inertia1+(-xx2)*(fx*yy2-fy*xx2)/inertia2;

delta_normalspeed = nx*delta_xspeed+ny*delta_yspeed;
delta_normalspeed =
  nx*(f*nx/mass1+f*nx/mass2+( yy1)*(f*nx*yy1-f*ny*xx1)/inertia1+( yy2)*(f*nx*yy2-f*ny*xx2)/inertia2)+
  ny*(f*ny/mass1+f*ny/mass2+(-xx1)*(f*nx*yy1-f*ny*xx1)/inertia1+(-xx2)*(f*nx*yy2-f*ny*xx2)/inertia2);
delta_normalspeed = f*(
  nx*nx/mass1+nx*nx/mass2+( yy1)*nx*(nx*yy1-ny*xx1)/inertia1+( yy2)*nx*(nx*yy2-ny*xx2)/inertia2)+
  ny*ny/mass1+ny*ny/mass2+(-xx1)*ny*(nx*yy1-ny*xx1)/inertia1+(-xx2)*ny*(nx*yy2-ny*xx2)/inertia2));

delta_normalspeed = f*(
  (nx*nx+ny*ny)/mass1+
  (nx*nx+ny*ny)/mass2+
  (yy1*nx-xx1*ny)*(yy1*nx-xx1*ny)/inertia1+
  (yy2*nx-xx2*ny)*(yy2*nx-xx2*ny)/inertia2);

nx*nx+ny*ny is always 1

delta_normalspeed = f*(
1/mass1+
1/mass2+
sqr(yy1*nx-xx1*ny)/inertia1+
sqr(yy2*nx-xx2*ny)/inertia2);

tangent constraint: identical to normal constraint but nx is replaced by ny and ny is replaced by -nx
*/

#endif // EP_CONTACT_VELOCITYCONSTRAINTS_H

