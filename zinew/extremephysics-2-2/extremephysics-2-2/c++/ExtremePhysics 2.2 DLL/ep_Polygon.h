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
 * File: ep_Polygon.h
 * Definition of ep_Polygon.
 * Included in ExtremePhysics.h.
 */

#include "ep_Definitions.h"

#ifndef EP_POLYGON_H
#define EP_POLYGON_H

struct ep_PolygonVertex {
	double x, y;
	double nx, ny, len, dist, pos;	
};

#if EP_USE_IDHASHTABLE
class ep_Polygon : private ep_IdHashTableEntry {
#else
class ep_Polygon {
#endif
	
	friend class ep_Main;
	friend class ep_World;
	friend class ep_Body;
	friend class ep_Shape;
	
	// variables
	private:
	
	ep_Main *main;
	ep_World *world;
	ep_Polygon *prev, *next;
#if !EP_USE_IDHASHTABLE
	unsigned long id;
#endif
	
	double xoff, yoff;
	double mass;
	double inertia;
	
	unsigned long vertexcount;
	ep_PolygonVertex *vertices;
	
#if EP_COMPILE_DEBUGCHECKS
	unsigned long referencecount;
	bool initialized;
#endif
	
	// private functions
	private:
	
	void Init(ep_World* _world, unsigned long _vertexcount, unsigned long _id);
	void DeInit();
	
	// public functions
	public:
	
	/// Sets the coordinates of a vertex of this polygon.
	/// @param index The index of the vertex, starting from 0.
	/// @param x The x coordinate of the vertex.
	/// @param y The y coordinate of the vertex.
	/// @return Returns whether successful.
	bool SetVertex(unsigned long index, double x, double y);
	/// Initializes this polygon.
	/// @return Returns whether successful.
	bool Initialize();
	
	// inline functions
	public:
	
	/// Returns the id of this polygon.
	inline unsigned long GetID() { return id; }
	/// Caches the id of this polygon to speed up the next FindPolygon call (if the id matches the id of this polygon).
	inline void CacheID() { world->current_polygon = this; }
	/// Returns a pointer to the main ExtremePhysics object associated with this polygon.
	inline ep_Main* GetMain() { return main; }
	/// Returns a pointer to the world associated with this polygon.
	inline ep_World* GetWorld() { return world; }
	/// Returns a pointer to the previous polygon.
	inline ep_Polygon* GetPrevious() { return prev; }
	/// Returns a pointer to the next polygon.
	inline ep_Polygon* GetNext() { return next; }
	/// Returns a pointer to the userdata memory block.
	inline void* GetUserData() { return (void*)((ep_PolygonVertex*)(this+1)+vertexcount); }
	
	/// Returns the number of vertices of this polygon.
	unsigned long GetVertexCount() { return vertexcount; }
	/// Returns the x coordinate of the vertex with the given index.
	double GetVertexX(unsigned long index) {
#if EP_COMPILE_DEBUGCHECKS
		if(index>=vertexcount) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Could not get x coordinate of vertex %lu of polygon %lu in world %lu, the polygon has only %lu vertices.", index, id, world->id, vertexcount);
#endif
			return 0.0;
		}
#endif
		return vertices[index].x;
	}
	/// Returns the y coordinate of the vertex with the given index.
	double GetVertexY(unsigned long index) {
#if EP_COMPILE_DEBUGCHECKS
		if(index>=vertexcount) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Could not get y coordinate of vertex %lu of polygon %lu in world %lu, the polygon has only %lu vertices.", index, id, world->id, vertexcount);
#endif
			return 0.0;
		}
#endif
		return vertices[index].y;
	}
	/// Returns the x component of the normal vector of the vertex with the given index.
	double GetVertexNormalX(unsigned long index) {
#if EP_COMPILE_DEBUGCHECKS
		if(index>=vertexcount) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Could not get x component of normal vector of vertex %lu of polygon %lu in world %lu, the polygon has only %lu vertices.", index, id, world->id, vertexcount);
#endif
			return 0.0;
		}
#endif
		return vertices[index].nx;
	}
	/// Returns the y component of the normal vector of the vertex with the given index.
	double GetVertexNormalY(unsigned long index) {
#if EP_COMPILE_DEBUGCHECKS
		if(index>=vertexcount) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Could not get y component of normal vector of vertex %lu of polygon %lu in world %lu, the polygon has only %lu vertices.", index, id, world->id, vertexcount);
#endif
			return 0.0;
		}
#endif
		return vertices[index].ny;
	}
	/// Returns the length of the edge with the given index.
	double GetEdgeLength(unsigned long index) {
#if EP_COMPILE_DEBUGCHECKS
		if(index>=vertexcount) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Could not get length of edge %lu of polygon %lu in world %lu, the polygon has only %lu vertices.", index, id, world->id, vertexcount);
#endif
			return 0.0;
		}
#endif
		return vertices[index].len;
	}
	
};

#endif // EP_POLYGON_H

