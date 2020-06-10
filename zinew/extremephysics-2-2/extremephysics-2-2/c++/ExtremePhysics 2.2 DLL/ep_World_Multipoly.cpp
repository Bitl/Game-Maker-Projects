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
 * File: ep_World_Multipoly.cpp
 * Polygon decomposition algorithm (extension).
 */

#include "ExtremePhysics.h"

#if EP_COMPILE_MULTIPOLY

inline bool ep_Multipoly_IsVertexConvex(double nx1, double ny1, double nx2, double ny2) {
	return (nx1*ny2-ny1*nx2>=EP_MULTIPOLY_EPSILON_MINCROSSPRODUCT);
}
inline double ep_Multipoly_PointAboveLine(double x, double y, double x1, double y1, double x2, double y2) {
	return (x-x1)*(y2-y1)-(y-y1)*(x2-x1);
}
bool ep_Multipoly_GenerateSegmentNormal(ep_Multipoly_Vertex *current, ep_Multipoly_Vertex *next) {
	double vx, vy, d;
	vx = current->y-next->y;
	vy = next->x-current->x;
	d = sqrt(ep_sqr(vx)+ep_sqr(vy));
	if(d<EP_MULTIPOLY_EPSILON_MINPOLYEDGELEN) return false;
	current->segment_nx = vx/d;
	current->segment_ny = vy/d;
	return true;
}
bool ep_Multipoly_GenerateNormal(ep_Multipoly_Vertex *prev, ep_Multipoly_Vertex *current) {
	double vx, vy, d;
	vx = prev->segment_nx+current->segment_nx;
	vy = prev->segment_ny+current->segment_ny;
	d = sqrt(ep_sqr(vx)+ep_sqr(vy));
	if(d<EP_EPSILON_MINNORMALVECTOR) return false;
	current->nx = vx/d;
	current->ny = vy/d;
	return true;
}

bool ep_World::Multipoly_Decompose(bool showerrors) {
	
	if(multipoly_vertexcount>4000) {
#if EP_COMPILE_DEBUGMESSAGES
		if(showerrors) main->Message(EP_MESSAGELEVEL_ERROR, "Decomposition of multipolygon in world %lu has failed, the number of vertices is higher than 4000.", id);
#endif
		return false;
	}
	
	// initialize
	for(unsigned int i = 0; i<multipoly_vertexcount; ++i) {
		unsigned int prev = (i+multipoly_vertexcount-1)%multipoly_vertexcount;
		unsigned int next = (i+1)%multipoly_vertexcount;
		multipoly_vertices[i].prev = prev;
		multipoly_vertices[i].next = next;
		if(!ep_Multipoly_GenerateSegmentNormal(&multipoly_vertices[i], &multipoly_vertices[next])) {
#if EP_COMPILE_DEBUGMESSAGES
			if(showerrors) main->Message(EP_MESSAGELEVEL_ERROR, "Decomposition of multipolygon in world %lu has failed, the distance between vertex %lu and %lu is zero.", id, i, next);
#endif
			return false;
		}
	}
	for(unsigned int i = 0; i<multipoly_vertexcount; ++i) {
		if(!ep_Multipoly_GenerateNormal(multipoly_vertices+i, multipoly_vertices+multipoly_vertices[i].next)) {
#if EP_COMPILE_DEBUGMESSAGES
			if(showerrors) main->Message(EP_MESSAGELEVEL_ERROR, "Decomposition of multipolygon in world %lu has failed, the multipolygon is degenerate near vertex %lu.", id, i);
#endif
			return false;
		}
		multipoly_vertices[i].convex = ep_Multipoly_IsVertexConvex(multipoly_vertices[multipoly_vertices[i].prev].segment_nx, multipoly_vertices[multipoly_vertices[i].prev].segment_ny, multipoly_vertices[i].segment_nx, multipoly_vertices[i].segment_ny);
	}
	
	// allocate enough space for the worst case
	ep_Multipoly_Cut *cuts = (ep_Multipoly_Cut*)(ep_malloc(multipoly_vertexcount*(multipoly_vertexcount-1)/2*sizeof(ep_Multipoly_Cut)));
	if(cuts==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		if(showerrors) main->Message(EP_MESSAGELEVEL_ERROR, "Decomposition of multipolygon in world %lu has failed, memory allocation failed.", id);
#endif
		return false;
	}
	ep_Multipoly_Task *tasks = (ep_Multipoly_Task*)(ep_malloc((multipoly_vertexcount-2)*sizeof(ep_Multipoly_Task)));
	if(cuts==NULL) {
		ep_free(cuts);
#if EP_COMPILE_DEBUGMESSAGES
		if(showerrors) main->Message(EP_MESSAGELEVEL_ERROR, "Decomposition of multipolygon in world %lu has failed, memory allocation failed.", id);
#endif
		return false;
	}
	
	// find all cuts
	unsigned long cuts_found = 0;
	{
		unsigned long i = 0;
		do {
			
			if(!multipoly_vertices[i].convex) { // ignore convex vertices
				unsigned long j = multipoly_vertices[multipoly_vertices[i].next].next;
				unsigned long stop = multipoly_vertices[i].prev;
				do {
					if(j>i || multipoly_vertices[j].convex) { // avoid duplicates
						if(Multipoly_IsValidCut(i, j)) {
							cuts[cuts_found].i = i;
							cuts[cuts_found].j = j;
							Multipoly_CalculateRating(cuts+cuts_found);
							++cuts_found;
						}
					}
					j = multipoly_vertices[j].next;
				} while(j!=stop);
			}
			
			i = multipoly_vertices[i].next;
		} while(i!=0);
	}
	
	// create linked list
	if(cuts_found==0) {
		ep_free(cuts);
		cuts = NULL; // this is an empty linked list, after all
	} else {
		for(unsigned long c = 0; c<cuts_found; ++c) {
			cuts[c].next = cuts+c+1;
		}
		cuts[cuts_found-1].next = NULL;
	}
	
	// decompose
	multipoly_first_polygon = NULL;
	multipoly_last_polygon = NULL;
#if EP_USE_EXCEPTIONS
	try {
#endif
	{
		ep_Polygon *p;
		int result = Multipoly_MakeTask(tasks, cuts, 0, 0, &p, showerrors);
		if(result==0) {
			if(multipoly_first_polygon==NULL) multipoly_first_polygon = p;
			multipoly_last_polygon = p;
			ep_free(cuts);
			ep_free(tasks);
			return true;
		}
		if(result==-1) {
			Multipoly_CancelDecompose();
			ep_free(cuts);
			ep_free(tasks);
			return false;
		}
	}
	unsigned long taskcount = 1;
	while(taskcount>0) {
		ep_Multipoly_Task *t = tasks+taskcount-1;
		unsigned long i = t->i, j = t->j;
		ep_Multipoly_Cut *list1 = t->cuts_list1, *list2 = t->cuts_list2;
		if(t->part==1) {
			++(t->part);
			t->a = multipoly_vertices[i].next;
			t->b = multipoly_vertices[j].prev;
			multipoly_vertices[i].next = j;
			multipoly_vertices[j].prev = i;
			ep_Polygon *p;
			int result = Multipoly_MakeTask(tasks+taskcount, list2, j, i, &p, showerrors);
			if(result==0) {
				if(multipoly_first_polygon==NULL) multipoly_first_polygon = p;
				multipoly_last_polygon = p;
			}
			if(result==1) {
				++taskcount;
			}
			if(result==-1) {
				Multipoly_CancelDecompose();
				ep_free(cuts);
				ep_free(tasks);
				return false;
			}
		} else {
			--taskcount;
			multipoly_vertices[i].next = t->a;
			multipoly_vertices[j].prev = t->b;
			multipoly_vertices[j].next = i;
			multipoly_vertices[i].prev = j;
			ep_Polygon *p;
			int result = Multipoly_MakeTask(tasks+taskcount, list1, i, j, &p, showerrors);
			if(result==0) {
				if(multipoly_first_polygon==NULL) multipoly_first_polygon = p;
				multipoly_last_polygon = p;
			}
			if(result==1) {
				++taskcount;
			}
			if(result==-1) {
				Multipoly_CancelDecompose();
				ep_free(cuts);
				ep_free(tasks);
				return false;
			}
		}
	}
#if EP_USE_EXCEPTIONS
	}
	catch(...) {
		Multipoly_CancelDecompose();
		ep_free(cuts);
		ep_free(tasks);
		throw;
	}
#endif
	
	ep_free(cuts);
	ep_free(tasks);
	return true;
}

void ep_World::Multipoly_CancelDecompose() {
	ep_Polygon *next;
	for(; multipoly_first_polygon!=NULL; multipoly_first_polygon = next) {
		next = multipoly_first_polygon->next;
		DestroyPolygon(multipoly_first_polygon);
	}
	multipoly_last_polygon = NULL;
}

int ep_World::Multipoly_MakeTask(ep_Multipoly_Task* result, ep_Multipoly_Cut* cuts, unsigned long first, unsigned long last, ep_Polygon** p, bool showerrors) {
	
	// update edge vertices
	if(first!=last) {
		if(!ep_Multipoly_GenerateSegmentNormal(&multipoly_vertices[first], &multipoly_vertices[multipoly_vertices[first].next])) {
#if EP_COMPILE_DEBUGMESSAGES
			if(showerrors) main->Message(EP_MESSAGELEVEL_ERROR, "Decomposition of multipolygon in world %lu has failed, the distance between vertex %lu and %lu is zero.", id, first, multipoly_vertices[first].next);
#endif
			return -1;
		}
		if(!ep_Multipoly_GenerateSegmentNormal(&multipoly_vertices[last], &multipoly_vertices[multipoly_vertices[last].next])) {
#if EP_COMPILE_DEBUGMESSAGES
			if(showerrors) main->Message(EP_MESSAGELEVEL_ERROR, "Decomposition of multipolygon in world %lu has failed, the distance between vertex %lu and %lu is zero.", id, last, multipoly_vertices[last].next);
#endif
			return -1;
		}
		if(!ep_Multipoly_GenerateNormal(&multipoly_vertices[multipoly_vertices[first].prev], &multipoly_vertices[first])) {
#if EP_COMPILE_DEBUGMESSAGES
			if(showerrors) main->Message(EP_MESSAGELEVEL_ERROR, "Decomposition of multipolygon in world %lu has failed, the multipolygon is degenerate near vertex %lu.", id, first);
#endif
			return -1;
		}
		if(!ep_Multipoly_GenerateNormal(&multipoly_vertices[multipoly_vertices[last].prev], &multipoly_vertices[last])) {
#if EP_COMPILE_DEBUGMESSAGES
			if(showerrors) main->Message(EP_MESSAGELEVEL_ERROR, "Decomposition of multipolygon in world %lu has failed, the multipolygon is degenerate near vertex %lu.", id, last);
#endif
			return -1;
		}
		multipoly_vertices[first].convex = ep_Multipoly_IsVertexConvex(multipoly_vertices[multipoly_vertices[first].prev].segment_nx, multipoly_vertices[multipoly_vertices[first].prev].segment_ny, multipoly_vertices[first].segment_nx, multipoly_vertices[first].segment_ny);
		multipoly_vertices[last].convex = ep_Multipoly_IsVertexConvex(multipoly_vertices[multipoly_vertices[last].prev].segment_nx, multipoly_vertices[multipoly_vertices[last].prev].segment_ny, multipoly_vertices[last].segment_nx, multipoly_vertices[last].segment_ny);
	}
	
	// count the vertices
	unsigned long count = 0;
	unsigned long i = first;
	do {
		++count;
		i = multipoly_vertices[i].next;
	} while(i!=first);
	
	// is this polygon convex?
	bool convex = true;
	i = first;
	do {
		if(!multipoly_vertices[i].convex) {
			convex = false;
			break;
		}
		i = multipoly_vertices[i].next;
	} while(i!=first);
	if(convex) {
		// make sure the polygon has no loops
		{
			long counter = 0;
			i = first;
			do {
				if((multipoly_vertices[multipoly_vertices[i].prev].y>multipoly_vertices[i].y)!=(multipoly_vertices[i].y>multipoly_vertices[multipoly_vertices[i].next].y)) {
					++counter;
				}
				i = multipoly_vertices[i].next;
			} while(i!=first);
			if(counter!=2) {
#if EP_COMPILE_DEBUGMESSAGES
				if(showerrors) main->Message(EP_MESSAGELEVEL_ERROR, "Decomposition of multipolygon in world %lu has failed, the multipolygon contains loops.", id, last);
#endif
				return -1;
			}
		}
		ep_Polygon *poly = CreatePolygon(count);
		if(poly==NULL) return -1;
		count = 0;
		i = first;
		do {
			poly->SetVertex(count, multipoly_vertices[i].x, multipoly_vertices[i].y);
			++count;
			i = multipoly_vertices[i].next;
		} while(i!=first);
		if(!poly->Initialize()) return -1;
		*p = poly;
		return 0;
	}
	
	// make sure there are at least 4 vertices
	if(count<4) {
#if EP_COMPILE_DEBUGMESSAGES
		if(showerrors) main->Message(EP_MESSAGELEVEL_ERROR, "Decomposition of multipolygon in world %lu has failed, the multipolygon is not valid (a sub-polygon has less than 4 vertices and is not convex).", id, last);
#endif
		return -1;
	}
	
	// find best cut
	int best_rating1 = -1;
	double best_rating2;
#ifdef __GNUC__
	best_rating2 = 0.0;
#endif
	ep_Multipoly_Cut *best_c = NULL;
	if(first!=last) {
		ep_Multipoly_Cut *prev = NULL;
		for(ep_Multipoly_Cut *c = cuts; c!=NULL; c = c->next) {
			if(c->i==first || c->i==last || c->j==first || c->j==last) {
				if(multipoly_vertices[c->i].convex && multipoly_vertices[c->j].convex) {
					// this cut is useless now, remove it
					if(prev==NULL) {
						cuts = c->next;
					} else {
						prev->next = c->next;
					}
					continue; // prev should not be updated
				}
				Multipoly_CalculateRating(c); // the normals have changed
			}
			if(c->rating1>best_rating1 || (c->rating1==best_rating1 && c->rating2<best_rating2)) {
				best_rating1 = c->rating1;
				best_rating2 = c->rating2;
				best_c = c;
			}
			prev = c;
		}
	} else {
		for(ep_Multipoly_Cut *c = cuts; c!=NULL; c = c->next) {
			if(c->rating1>best_rating1 || (c->rating1==best_rating1 && c->rating2<best_rating2)) {
				best_rating1 = c->rating1;
				best_rating2 = c->rating2;
				best_c = c;
			}
		}
	}
	if(best_c==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		if(showerrors) main->Message(EP_MESSAGELEVEL_ERROR, "Decomposition of multipolygon in world %lu has failed, the multipolygon is not valid (a sub-polygon is not convex and has no valid cuts).", id, last);
#endif
		return -1;
	}
	
	// split the rest into two lists (and discard some of them)
	unsigned long best_i = best_c->i, best_j = best_c->j;
	if(best_i>best_j) { unsigned long temp = best_i; best_i = best_j; best_j = temp; }
	ep_Multipoly_Cut *list1 = NULL, *list2 = NULL;
	while(cuts!=NULL) {
		ep_Multipoly_Cut *next = cuts->next;
		if(cuts!=best_c) {
			if(cuts->i>=best_i && cuts->i<=best_j && cuts->j>=best_i && cuts->j<=best_j) {
				cuts->next = list1;
				list1 = cuts;
			} else if((cuts->i<=best_i || cuts->i>=best_j) && (cuts->j<=best_i || cuts->j>=best_j)) {
				cuts->next = list2;
				list2 = cuts;
			}
		}
		cuts = next;
	}
	
	result->i = best_i;
	result->j = best_j;
	result->cuts_list1 = list1;
	result->cuts_list2 = list2;
	result->part = 1;
	
	return 1;
}

bool ep_World::Multipoly_IsValidCut(unsigned long i, unsigned long j) {
	
	double x1, y1, x2, y2;
	x1 = multipoly_vertices[i].x;
	y1 = multipoly_vertices[i].y;
	x2 = multipoly_vertices[j].x;
	y2 = multipoly_vertices[j].y;
	double d = sqrt(ep_sqr(x1-x2)+ep_sqr(y1-y2));
	
	if(multipoly_vertices[i].convex) {
		if(ep_Multipoly_PointAboveLine(x2, y2, multipoly_vertices[multipoly_vertices[i].next].x, multipoly_vertices[multipoly_vertices[i].next].y, x1, y1)<=0.0f ||
		   ep_Multipoly_PointAboveLine(x2, y2, x1, y1, multipoly_vertices[multipoly_vertices[i].prev].x, multipoly_vertices[multipoly_vertices[i].prev].y)<=0.0f) return false;
	} else {
		if(ep_Multipoly_PointAboveLine(x2, y2, multipoly_vertices[multipoly_vertices[i].next].x, multipoly_vertices[multipoly_vertices[i].next].y, x1, y1)<=0.0f &&
		   ep_Multipoly_PointAboveLine(x2, y2, x1, y1, multipoly_vertices[multipoly_vertices[i].prev].x, multipoly_vertices[multipoly_vertices[i].prev].y)<=0.0f) return false;
	}
	if(multipoly_vertices[j].convex) {
		if(ep_Multipoly_PointAboveLine(x1, y1, multipoly_vertices[multipoly_vertices[j].next].x, multipoly_vertices[multipoly_vertices[j].next].y, x2, y2)<=0.0f ||
		   ep_Multipoly_PointAboveLine(x1, y1, x2, y2, multipoly_vertices[multipoly_vertices[j].prev].x, multipoly_vertices[multipoly_vertices[j].prev].y)<=0.0f) return false;
	} else {
		if(ep_Multipoly_PointAboveLine(x1, y1, multipoly_vertices[multipoly_vertices[j].next].x, multipoly_vertices[multipoly_vertices[j].next].y, x2, y2)<=0.0f &&
		   ep_Multipoly_PointAboveLine(x1, y1, x2, y2, multipoly_vertices[multipoly_vertices[j].prev].x, multipoly_vertices[multipoly_vertices[j].prev].y)<=0.0f) return false;
	}
	
	unsigned long k;
	bool left1;
	
	k = multipoly_vertices[i].next;
	left1 = (ep_Multipoly_PointAboveLine(multipoly_vertices[k].x, multipoly_vertices[k].y, x1, y1, x2, y2)>EP_MULTIPOLY_EPSILON_MINPOLYEDGELEN*d);
	k = multipoly_vertices[k].next;
	while(k!=j) {
		bool left2 = (ep_Multipoly_PointAboveLine(multipoly_vertices[k].x, multipoly_vertices[k].y, x1, y1, x2, y2)>EP_MULTIPOLY_EPSILON_MINPOLYEDGELEN*d);
		if(left1 && !left2) {
			unsigned long kprev = multipoly_vertices[k].prev;
			if(ep_Multipoly_PointAboveLine(x1, y1, multipoly_vertices[kprev].x, multipoly_vertices[kprev].y, multipoly_vertices[k].x, multipoly_vertices[k].y)<0.0 &&
				ep_Multipoly_PointAboveLine(x2, y2, multipoly_vertices[kprev].x, multipoly_vertices[kprev].y, multipoly_vertices[k].x, multipoly_vertices[k].y)>0.0) return false;
		}
		if(!left1 && left2) {
			unsigned long kprev = multipoly_vertices[k].prev;
			if(ep_Multipoly_PointAboveLine(x1, y1, multipoly_vertices[k].x, multipoly_vertices[k].y, multipoly_vertices[kprev].x, multipoly_vertices[kprev].y)<0.0 &&
				ep_Multipoly_PointAboveLine(x2, y2, multipoly_vertices[k].x, multipoly_vertices[k].y, multipoly_vertices[kprev].x, multipoly_vertices[kprev].y)>0.0) return false;
		}
		left1 = left2;
		k = multipoly_vertices[k].next;
	}
	
	k = multipoly_vertices[j].next;
	left1 = (ep_Multipoly_PointAboveLine(multipoly_vertices[k].x, multipoly_vertices[k].y, x2, y2, x1, y1)>EP_MULTIPOLY_EPSILON_MINPOLYEDGELEN*d);
	k = multipoly_vertices[k].next;
	while(k!=i) {
		bool left2 = (ep_Multipoly_PointAboveLine(multipoly_vertices[k].x, multipoly_vertices[k].y, x2, y2, x1, y1)>EP_MULTIPOLY_EPSILON_MINPOLYEDGELEN*d);
		if(left1 && !left2) {
			unsigned long kprev = multipoly_vertices[k].prev;
			if(ep_Multipoly_PointAboveLine(x2, y2, multipoly_vertices[kprev].x, multipoly_vertices[kprev].y, multipoly_vertices[k].x, multipoly_vertices[k].y)<0.0 &&
				ep_Multipoly_PointAboveLine(x1, y1, multipoly_vertices[kprev].x, multipoly_vertices[kprev].y, multipoly_vertices[k].x, multipoly_vertices[k].y)>0.0) return false;
		}
		if(!left1 && left2) {
			unsigned long kprev = multipoly_vertices[k].prev;
			if(ep_Multipoly_PointAboveLine(x2, y2, multipoly_vertices[k].x, multipoly_vertices[k].y, multipoly_vertices[kprev].x, multipoly_vertices[kprev].y)<0.0 &&
				ep_Multipoly_PointAboveLine(x1, y1, multipoly_vertices[k].x, multipoly_vertices[k].y, multipoly_vertices[kprev].x, multipoly_vertices[kprev].y)>0.0) return false;
		}
		left1 = left2;
		k = multipoly_vertices[k].next;
	}
	
	return true;
}

void ep_World::Multipoly_CalculateRating(ep_Multipoly_Cut* c) {
	const double normal_weight_factor = 0.75;
	const double concave_rating2_weight = 2.0;
	unsigned int i = c->i, j = c->j;
	c->rating1 = 0;
	double dsqr = ep_sqr(multipoly_vertices[i].x-multipoly_vertices[j].x)+ep_sqr(multipoly_vertices[i].y-multipoly_vertices[j].y);
	double d = sqrt(dsqr);
	if(!multipoly_vertices[i].convex &&
	   ep_Multipoly_PointAboveLine(multipoly_vertices[j].x, multipoly_vertices[j].y, multipoly_vertices[multipoly_vertices[i].next].x, multipoly_vertices[multipoly_vertices[i].next].y, multipoly_vertices[i].x, multipoly_vertices[i].y)>EP_MULTIPOLY_EPSILON_MINCROSSPRODUCT*dsqr &&
	   ep_Multipoly_PointAboveLine(multipoly_vertices[j].x, multipoly_vertices[j].y, multipoly_vertices[i].x, multipoly_vertices[i].y, multipoly_vertices[multipoly_vertices[i].prev].x, multipoly_vertices[multipoly_vertices[i].prev].y)>EP_MULTIPOLY_EPSILON_MINCROSSPRODUCT*dsqr) {
		c->rating1 = 1;
	}
	if(!multipoly_vertices[j].convex &&
	   ep_Multipoly_PointAboveLine(multipoly_vertices[i].x, multipoly_vertices[i].y, multipoly_vertices[multipoly_vertices[j].next].x, multipoly_vertices[multipoly_vertices[j].next].y, multipoly_vertices[j].x, multipoly_vertices[j].y)>EP_MULTIPOLY_EPSILON_MINCROSSPRODUCT*dsqr &&
	   ep_Multipoly_PointAboveLine(multipoly_vertices[i].x, multipoly_vertices[i].y, multipoly_vertices[j].x, multipoly_vertices[j].y, multipoly_vertices[multipoly_vertices[j].prev].x, multipoly_vertices[multipoly_vertices[j].prev].y)>EP_MULTIPOLY_EPSILON_MINCROSSPRODUCT*dsqr) {
		c->rating1 += 1;
	}
	c->rating2 =
	  ((multipoly_vertices[i].convex)? 1.0 : concave_rating2_weight)*
	  (d-(multipoly_vertices[i].nx*(multipoly_vertices[j].x-multipoly_vertices[i].x)+multipoly_vertices[i].ny*(multipoly_vertices[j].y-multipoly_vertices[i].y))*normal_weight_factor)
	  +((multipoly_vertices[j].convex)? 1.0 : concave_rating2_weight)*
	  (d-(multipoly_vertices[j].nx*(multipoly_vertices[i].x-multipoly_vertices[j].x)+multipoly_vertices[j].ny*(multipoly_vertices[i].y-multipoly_vertices[j].y))*normal_weight_factor);
}

bool ep_World::MultipolyBegin(unsigned long vertexcount) {
#if EP_COMPILE_DEBUGCHECKS
	if(vertexcount<3) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not begin multipolygon in world %lu, vertex count is smaller than 3.", id);
#endif
		return false;
	}
#endif
	ep_free(multipoly_vertices);
	multipoly_vertices = (ep_Multipoly_Vertex*)(ep_malloc(vertexcount*sizeof(ep_Multipoly_Vertex)));
	if(multipoly_vertices==NULL) {
		multipoly_vertexcount = 0;
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not begin multipolygon in world %lu, memory allocation failed.", id);
#endif
		return false;
	}
	multipoly_vertexcount = vertexcount;
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Started multipolygon in world %lu.", id);
#endif
	return true;
}

bool ep_World::MultipolyEnd(bool showerrors) {
#if EP_COMPILE_DEBUGCHECKS
	if(multipoly_vertices==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not end multipolygon in world %lu, the multipolygon was never started.", id);
#endif
		return false;
	}
#endif
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Started decomposition of multipolygon in world %lu.", id);
#endif
	if(!Multipoly_Decompose(showerrors)) {
		ep_free(multipoly_vertices);
		multipoly_vertexcount = 0;
		multipoly_vertices = NULL;
		return false;
	}
	ep_free(multipoly_vertices);
	multipoly_vertexcount = 0;
	multipoly_vertices = NULL;
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Ended multipolygon in world %lu.", id);
#endif
	return true;
}

bool ep_World::MultipolySetVertex(unsigned long index, double x, double y) {
#if EP_COMPILE_DEBUGCHECKS
	if(multipoly_vertices==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not set vertex %lu of multipolygon in world %lu, the multipolygon was never started.", index, id);
#endif
		return false;
	}
	if(index>=multipoly_vertexcount) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not set vertex %lu of multipolygon in world %lu, the multipolygon has only %lu vertices.", index, id, multipoly_vertexcount);
#endif
		return false;
	}
#endif
	multipoly_vertices[index].x = x;
	multipoly_vertices[index].y = y;
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Set vertex %lu of multipolygon in world %lu.", index, id);
#endif
	return true;
}

#endif // EP_COMPILE_MULTIPOLY

