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
 * File: ep_Macros.h
 * This file contains a list of all macros used by ExtremePhysics.
 * Included in ExtremePhysics.h.
 */

#ifndef EP_MACROS_H
#define EP_MACROS_H

#define ep_sqr(x) ((x)*(x))
#define ep_max(a, b) (((a)>(b))? (a) : (b))
#define ep_min(a, b) (((a)<(b))? (a) : (b))
#define ep_clamp(a, b, val) (((val)<(a))? (a) : (((val)>(b))? (b) : (val)))
#define ep_round(x) (floor(0.5+(x)))

// rotates a vector
// [ x' ] = [  cos(a) ,  sin(a) ] * [ x ]
// [ y' ]   [ -sin(a) ,  cos(a) ]   [ y ]
#define ep_transform_x(sin, cos, x, y) (+(cos)*(x)+(sin)*(y))
#define ep_transform_y(sin, cos, x, y) (-(sin)*(x)+(cos)*(y))

// rotates a vector in the opposite direction
// cos(-a) = cos(a)
// sin(-a) = -sin(a)
#define ep_invtransform_x(sin, cos, x, y) (+(cos)*(x)-(sin)*(y))
#define ep_invtransform_y(sin, cos, x, y) (+(sin)*(x)+(cos)*(y))

// combines two transformations
// sin(a+b) = sin(a)*cos(b)+cos(a)*sin(b)
// cos(a+b) = cos(a)*cos(b)-sin(a)*sin(b)
#define ep_totaltransform_sin(sin1, cos1, sin2, cos2) ((sin1)*(cos2)+(cos1)*(sin2))
#define ep_totaltransform_cos(sin1, cos1, sin2, cos2) ((cos1)*(cos2)-(sin1)*(sin2))

// combines a transformation and an inverse transformation
// sin(a-b) = sin(a)*cos(b)-cos(a)*sin(b)
// cos(a-b) = cos(a)*cos(b)+sin(a)*sin(b)
#define ep_totalinvtransform_sin(sin1, cos1, sin2, cos2) ((sin1)*(cos2)-(cos1)*(sin2))
#define ep_totalinvtransform_cos(sin1, cos1, sin2, cos2) ((cos1)*(cos2)+(sin1)*(sin2))

// shuffles indices in an array
// count 5 -> n 5, {0,1,2,3,4} -> {1,3,0,2,4}
// count 6 -> n 7, {0,1,2,3,4,5} -> {1,3,5,0,2,4}
// count 7 -> n 7, {0,1,2,3,4,5,6} -> {1,3,5,0,2,4,6}
// count 8 -> n 9, {0,1,2,3,4,5,6,7} -> {1,3,5,7,0,2,4,6}
#define ep_shuffle_getn(count) ((count)|1)
#define ep_shuffle(iii, nnn) ((((iii)<<1)|1)%(nnn))

#endif // EP_MACROS_H

