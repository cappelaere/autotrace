/* output-osm.c - output in OSM format

   Copyright (C) 2012 Pat Cappelaere, Vightel Corporation

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public License
   as published by the Free Software Foundation; either version 2.1 of
   the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
   USA. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif /* Def: HAVE_CONFIG_H */
#include <math.h>
#include <proj_api.h>

#include "spline.h"
#include "color.h"
#include "output-osm.h"

typedef struct  {
	int 	height;
	projPJ	sourcePrj;
	projPJ	targetPrj;
} ProjStruct;

/*===========================================================================
 Convert Bezier Spline
===========================================================================*/
static at_real bezpnt(at_real t, at_real z1, at_real z2, at_real z3, at_real z4)
{
  at_real temp, t1;
  /* Determine ordinate on Bezier curve at length "t" on curve */
  if (t < (at_real) 0.0) { t = (at_real) 0.0; }
  if (t > (at_real) 1.0) { t = (at_real) 1.0; }
  t1 = ((at_real) 1.0 - t);
  temp = t1*t1*t1*z1 + (at_real)3.0*t*t1*t1*z2 + (at_real)3.0*t*t*t1*z3 + t*t*t*z4;
  return(temp);
}

/*===========================================================================
  Print a point
===========================================================================*/
static void print_coord(FILE* f, long id, ProjStruct* pj, at_output_opts_type * opts, at_real x, at_real y)
{
	// use (0,0) UpperLeft Corner
	double xm = opts->osm_min_x + x * opts->osm_res;
	double ym = opts->osm_max_y  - (pj->height-y) * opts->osm_res;
	
	int p = pj_transform(pj->sourcePrj, pj->targetPrj, 1, 1, &xm, &ym, NULL );

    xm *= RAD_TO_DEG;
    ym *= RAD_TO_DEG;
	
   fprintf(f, "\n\t<node id='%ld' lat='%f' lon='%f' visible='true' />",	id, ym, xm );
}

static void
out_splines (FILE * file, spline_list_array_type shape, ProjStruct* pj, at_output_opts_type * opts)
{
  	unsigned this_list;
  	spline_list_type list;
  	at_color last_color = {0,0,0};
	long id 			= -10000;
	long start_node_id 	= id;
	long end_node_id 	= id;
	long change_set		= 0;

	for (this_list = 0; this_list < SPLINE_LIST_ARRAY_LENGTH (shape); this_list++)   {
		unsigned this_spline;
		spline_type first;

		list = SPLINE_LIST_ARRAY_ELT (shape, this_list);
		first = SPLINE_LIST_ELT (list, 0);

//      if (this_list == 0 || !COLOR_EQUAL(list.color, last_color)) {
//          if (this_list > 0) {
//              if (!(shape.centerline || list.open)) fputs("z", file);
//              fputs("\"/>\n", file);
//          }
//          fprintf(file, "<path style=\"%s:#%02x%02x%02x; %s:none;\" d=\"",
//		    (shape.centerline || list.open) ? "stroke" : "fill",
//		    list.color.r, list.color.g, list.color.b,
//		    (shape.centerline || list.open) ? "fill" : "stroke");
//      }
		
		//
		// do the nodes
		//
		start_node_id = id;
	
		print_coord(file, id--, pj, opts, START_POINT(first).x, START_POINT(first).y);
	
		for (this_spline = 0; this_spline < SPLINE_LIST_LENGTH (list); this_spline++) {
			spline_type s = SPLINE_LIST_ELT (list, this_spline);

			if (SPLINE_DEGREE(s) == LINEARTYPE) {
				print_coord(file, id--, pj, opts, END_POINT(s).x, END_POINT(s).y);
			} else {
				at_real temp;
				at_real dt = (at_real) (1.0/7.0);
				for( temp=dt; temp<=1.0; temp+=dt ){
					print_coord(file, id--, pj, opts,
						bezpnt(temp,START_POINT(s).x,CONTROL1(s).x,CONTROL2(s).x,END_POINT(s).x),
						bezpnt(temp,START_POINT(s).y,CONTROL1(s).y,CONTROL2(s).y,END_POINT(s).y));
				}
			}
			last_color = list.color;
		}
		end_node_id = id;

		// do the ways	
		fprintf(file, "\n\t<way id='%ld' visible='true'>", id-- );
		for( long i= start_node_id; i> end_node_id; i-- ) {
			fprintf(file, "\n\t\t<nd ref='%ld'/>", i );		
		}
		fprintf(file, "\n\t\t<tag k='name' v='water' />");		
		fprintf(file, "\n\t</way>\n" );

		// do the relations
		fprintf(file, "\n\t<relation id='%ld' visible='true' changeset='%ld'>\n", id--, change_set );
		long wayid = id + 2;
		fprintf(file, "\t\t<member type='way' ref='%ld' />\n", wayid );
		fprintf(file, "\t</relation>\n");	

	 }

	//if (!(shape.centerline || list.open)) fputs("z", file);
	//  if (SPLINE_LIST_ARRAY_LENGTH(shape) > 0)
	//    fputs("\"/>\n", file);
}

int output_osm_writer(FILE* file, gchar* name,
		int llx, int lly, int urx, int ury, 
		at_output_opts_type * opts,
		spline_list_array_type shape,
		at_msg_func msg_func, 
		gpointer msg_data,
		gpointer user_data) 
{
    int width  = urx - llx;
    int height = ury - lly;

	// Setup the projection structure
	ProjStruct pj;

	pj.height 			= height;
	
	// Source OSM Projection 3857
	pj.sourcePrj 		= pj_init_plus("+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +wktext  +no_defs");

	// Destination WGS84
	pj.targetPrj 		= pj_init_plus("+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs");
	
	// we need to get the bounding rectangle
	// UpperLeft  = {min_x, max_y} 
	// LowerRight = {max_x, min_y}
	// let's convert those (x,y) to (lon,lat)
	
	double ulx = opts->osm_min_x;
	double uly = opts->osm_max_y;
	double lrx = opts->osm_max_x;
	double lry = opts->osm_min_y;
	
   
	int p1 = pj_transform(pj.sourcePrj, pj.targetPrj, 1, 1, &ulx, &uly, NULL );
	int p2 = pj_transform(pj.sourcePrj, pj.targetPrj, 1, 1, &lrx, &lry, NULL );

	ulx *= RAD_TO_DEG;
    uly *= RAD_TO_DEG;
    lrx *= RAD_TO_DEG;
    lry *= RAD_TO_DEG;

	printf("(min_x: %f, max_y:%f) -> (ul_lon: %f, ul_lat: %f)\n", opts->osm_min_x, opts->osm_max_y, ulx, uly);
	printf("(max_x: %f, min_y:%f) -> (lr_lon: %f, lr_lat: %f)\n", opts->osm_max_x, opts->osm_min_y, lrx, lry);
	printf("resolution %f\n", opts->osm_res);
	
    fputs("<?xml version=\"1.0\" encoding='utf-8' ?>\n", file);

	fprintf(file, "<osm generator=\"geobliki\" version=\"0.6\">\n");
	fprintf(file, "<bounds maxlat=\"%f\" maxlon=\"%f\" minlat=\"%f\" minlon=\"%f\" />",
		uly, lrx, lry, ulx );
	
    out_splines(file, shape, &pj, opts);

    fputs("</osm>\n", file);
    return 0;
}

