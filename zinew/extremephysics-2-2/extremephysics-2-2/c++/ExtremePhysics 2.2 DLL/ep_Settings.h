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
 * File: ep_Settings.h
 * This file contains a list of all settings and constants used by ExtremePhysics.
 * Included in ExtremePhysics.h.
 */

#ifndef EP_SETTINGS_H
#define EP_SETTINGS_H

// ##### settings #####

// debugging
// You can disable debug checks in release mode but your program
// might crash if you do something wrong. A few examples:
// - using a polygon that has not been initialized
// - using a polygon that belongs to a different world
// - trying to connect a body to itself with a hinge joint
// - trying to connect bodies that belong to a different world
// - ...
// Memory allocation is always checked, even if you disable this.
#define EP_COMPILE_DEBUGCHECKS    1  // enable debug checks, e.g. 'can not create hinge joint, both bodies are static'
#define EP_COMPILE_DEBUGMESSAGES  1  // compile debug messages (without this the debug checks are still performed but no messages will be shown)
#define EP_COMPILE_DEBUGDRAW      1  // compile debug drawing

// polygon decomposition (extension)
#define EP_COMPILE_MULTIPOLY 1

// box chains (extension)
#define EP_COMPILE_BOXCHAIN 1

// serialize (extension)
#define EP_COMPILE_SERIALIZE 1

// Enable this if you want to be able to throw exceptions in callback functions. If you
// don't enable this and an exception is thrown, there could be memory leaks.
#define EP_USE_EXCEPTIONS 0

// id hash table settings
// The id hash table can significantly improve id lookup times.
// If this is disabled, ExtremePhysics will fall back to the original id lookup system
// (i.e. traversing the entire list). You can disable the id hash table if you're not
// using the id system frequently.
#define EP_USE_IDHASHTABLE           1  // enable the id hash table
#define EP_IDHASHTABLE_BUCKETS_BITS  8  // the actual number of buckets is 2 to the power of EP_IDHASHTABLE_BUCKETS_BITS

// ##### functions #####

// You can replace this with your own allocation functions. If memory allocation
// fails, the function should return NULL. If your allocator throws exceptions
// (e.g. std::bad_alloc), you should enable EP_USE_EXCEPTIONS to avoid memory leaks.
// ep_free should never throw
inline void* ep_malloc(unsigned long size) { return malloc(size); }
inline void* ep_realloc(void* mem, unsigned long size) { return realloc(mem, size); }
inline void ep_free(void* mem) { free(mem); }

// ##### constants #####

// epsilon values used in floating-point calculations
#define EP_EPSILON_MINPOLYMASS       1.0e-10   // minimum mass (i.e. aera) for polygons
#define EP_EPSILON_MINPOLYINERTIA    1.0e-10   // minimum moment of inertia for polygons
#define EP_EPSILON_MINBODYMASS       1.0e-20   // minimum mass for bodies
#define EP_EPSILON_MINBODYINERTIA    1.0e-20   // minimum moment of inertia for bodies
#define EP_EPSILON_MINTIMESTEP       1.0e-20   // minimum length of a time step
#define EP_EPSILON_MINCROSSPRODUCT   1.0e-4    // minimum cross product of adjacent edges (convexity check)
#define EP_EPSILON_MINRAILLENGTH     1.0e-6    // minimum length of rail joint 
#define EP_EPSILON_MINPOLYEDGELEN    1.0e-6    // minimum length of polygon edges
#define EP_EPSILON_MINNORMALVECTOR   1.0e-6    // minimum length of a vector that will be normalized
#define EP_EPSILON_MINCONTACTPOINTS  1.0e-4    // minimum distance between two contact points
#define EP_EPSILON_MINCIRCLEVERTEX   1.0e-4    // minimum distance between the center of the circle and a vertex during collisions
#define EP_EPSILON_MINBLOCKDET       1.0e-50   // minimum value of the determinant of a block solver matrix

// epsilon values for multipolygons (extension)
#define EP_MULTIPOLY_EPSILON_MINCROSSPRODUCT  1.0e-6  // this value should be smaller than EP_EPSILON_MAXCROSSPRODUCT
#define EP_MULTIPOLY_EPSILON_MINPOLYEDGELEN   1.0e-4  // this value should be greater than EP_EPSILON_MINPOLYEDGELEN

// 32-bit integer types
// used for radix sort
#define ep_int32   signed long
#define ep_uint32  unsigned long

// maximum number of free contacts that will be stored to avoid additional memory allocations
#define EP_MAXFREECONTACTS 100

// size of the buffer used to create debug messages
#define EP_MESSAGE_BUFFERSIZE 2000

// debug message levels
#define EP_MESSAGELEVEL_EXPLICIT         0  // this level of message is sent only when you explicitly ask for it
#define EP_MESSAGELEVEL_ERROR            1  // an error occurred
#define EP_MESSAGELEVEL_IMPORTANTCHANGE  2  // an important change occurred (create, destroy, ...)
#define EP_MESSAGELEVEL_NORMALCHANGE     3  // a normal change occurred (position changed, step, ...)

// shape types
#define EP_SHAPETYPE_BOX      1
#define EP_SHAPETYPE_CIRCLE   2
#define EP_SHAPETYPE_POLYGON  3

// forceinline
#if defined(_MSC_VER)
#define EP_FORCEINLINE __forceinline
#elif defined(__GNUC__)
#define EP_FORCEINLINE inline
#else
#define EP_FORCEINLINE inline
#endif

#endif // EP_SETTINGS_H

