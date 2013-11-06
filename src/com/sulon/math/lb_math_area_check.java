package com.sulon.math;

import com.sulon.datatypes.lb_vec2.vec2f;

public class lb_math_area_check {
	/*
	 * lb_math_area_check.h
	 *
	 *  Created on: Feb 25, 2009
	 *      Author: mahisorn
	 *
	 *  Copyright (c) <2009> <Mahisorn Wongphati>
	 *  Permission is hereby granted, free of charge, to any person
	 *  obtaining a copy of this software and associated documentation
	 *  files (the "Software"), to deal in the Software without
	 *  restriction, including without limitation the rights to use,
	 *  copy, modify, merge, publish, distribute, sublicense, and/or sell
	 *  copies of the Software, and to permit persons to whom the
	 *  Software is furnished to do so, subject to the following
	 *  conditions:
	 *
	 *  The above copyright notice and this permission notice shall be
	 *  included in all copies or substantial portions of the Software.
	 *
	 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
	 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
	 *  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
	 *  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
	 *  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
	 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
	 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
	 *  OTHER DEALINGS IN THE SOFTWARE.
	 */


	public boolean lb_point_is_inside_polygon(vec2f[] polygon, vec2f point)
	{
	    double xt = point.x;
	    double yt = point.y;

	    int n_point = polygon.length;
	    if(n_point < 3) return false;

	    boolean inside = false;

	    double xold = polygon[n_point-1].x;
	    double yold = polygon[n_point-1].y;

	    double xnew, ynew, x1, y1, x2, y2;
	    for(int i = 0; i < polygon.length; i++) {
	        xnew = polygon[i].x;
	        ynew = polygon[i].y;

	        if(xnew > xold) {
	            x1 = xold;
	            x2 = xnew;
	            y1 = yold;
	            y2 = ynew;
	        } else {
	            x1 = xnew;
	            x2 = xold;
	            y1 = ynew;
	            y2 = yold;
	        }

	        if(((xnew < xt) == (xt <= xold)) && (((yt-y1)*(x2-x1)) < ((y2-y1)*(xt-x1))))
	            inside=!inside;
	        xold=xnew;
	        yold=ynew;
	    }
	    return inside;
	}


}
