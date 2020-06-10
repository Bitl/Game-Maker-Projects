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
 * File: ep_World_IntegratePosition.cpp
 * Integrates the position of the bodies in the world.
 */

#include "ExtremePhysics.h"

void ep_World::IntegratePosition() {
	
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		if(body->issleeping) continue;
		if(body->xvel==0.0 && body->yvel==0.0 && body->rotvel==0.0) continue;
		
		body->x += body->xvel*timestep;
		body->y += body->yvel*timestep;
		body->rot += body->rotvel*timestep;
		body->rot_sin = sin(body->rot);
		body->rot_cos = cos(body->rot);
		body->Moved();
		
	}
	
}

