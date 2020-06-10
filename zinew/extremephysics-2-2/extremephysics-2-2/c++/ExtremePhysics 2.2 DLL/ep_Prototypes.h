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
 * File: ep_Prototypes.h
 * Prototypes of all ExtremePhysics classes.
 * Included in ExtremePhysics.h.
 */

#ifndef EP_PROTOTYPES_H
#define EP_PROTOTYPES_H

struct ep_CollisionData;

class ep_Callback;
class ep_Main;

#if EP_COMPILE_MULTIPOLY
struct ep_Multipoly_Vertex;
struct ep_Multipoly_Task;
struct ep_Multipoly_Cut;
#endif // EP_COMPILE_MULTIPOLY
class ep_World;

struct ep_PolygonVertex;
class ep_Polygon;

class ep_Group;
class ep_Body;

struct ep_ContactLink;
struct ep_ContactPoint;
class ep_Contact;

struct ep_HingeJointLink;
class ep_HingeJoint;

struct ep_DistanceJointLink;
class ep_DistanceJoint;

struct ep_RailJointLink;
class ep_RailJoint;

struct ep_SliderJointLink;
class ep_SliderJoint;

//JOINTS//

class ep_View;

class ep_Water;

class ep_Shape;

class ep_Force;

#endif // EP_PROTOTYPES_H

