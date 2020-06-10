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
 * File: ep_Contact_PositionConstraints.h
 * Solves contact position constraints.
 * Included in ep_World_SolvePositionConstraints.cpp.
 */

#ifndef EP_CONTACT_POSITIONCONSTRAINTS_H
#define EP_CONTACT_POSITIONCONSTRAINTS_H

#include "ExtremePhysics.h"

EP_FORCEINLINE void ep_Contact::InitPositionConstraints() {
	
	contactpoints[0].normalpseudoforce = contactpoints[0].normalforce;
	contactpoints[1].normalpseudoforce = contactpoints[1].normalforce;
	
}

EP_FORCEINLINE void ep_Contact::SolveCPNormalPositionConstraint(ep_ContactPoint* cp) {
	
	// update local points
	double current_xx1, current_yy1, current_xx2, current_yy2;
	current_xx1 = ep_transform_x(body1->rot_sin, body1->rot_cos, cp->x1-body1->xoff, cp->y1-body1->yoff);
	current_yy1 = ep_transform_y(body1->rot_sin, body1->rot_cos, cp->x1-body1->xoff, cp->y1-body1->yoff);
	current_xx2 = ep_transform_x(body2->rot_sin, body2->rot_cos, cp->x2-body2->xoff, cp->y2-body2->yoff);
	current_yy2 = ep_transform_y(body2->rot_sin, body2->rot_cos, cp->x2-body2->xoff, cp->y2-body2->yoff);
	
	// calculate pseudoforce
	double r1 = current_yy1*nx-current_xx1*ny;
	double r2 = current_yy2*nx-current_xx2*ny;
	double f =
	  (((body2->x+current_xx2)-(body1->x+current_xx1))*nx+
	   ((body2->y+current_yy2)-(body1->y+current_yy1))*ny
	   -world->contact_threshold)*world->position_factor/(
	  body1->invmass+body2->invmass
	  +ep_sqr(r1)*body1->invinertia
	  +ep_sqr(r2)*body2->invinertia);
	
	// clamp pseudoforce
	if(f<-cp->normalpseudoforce) {
		f = -cp->normalpseudoforce;
		cp->normalpseudoforce = 0.0;
	} else {
		cp->normalpseudoforce += f;
	}
	
	// apply pseudoforce
	if(f!=0.0) {
		double fx = f*nx;
		double fy = f*ny;
		body1->_ApplyPseudoImpulse(current_xx1, current_yy1, +fx, +fy);
		body2->_ApplyPseudoImpulse(current_xx2, current_yy2, -fx, -fy);
	}
	
}

EP_FORCEINLINE void ep_Contact::SolvePositionConstraints() {
	
	// normal contact constraints
	if(contactpoints[0].active) SolveCPNormalPositionConstraint(&contactpoints[0]);
	if(contactpoints[1].active) SolveCPNormalPositionConstraint(&contactpoints[1]);
	
}

#endif // EP_CONTACT_POSITIONCONSTRAINTS_H

