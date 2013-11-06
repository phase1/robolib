package com.sulon.object_detection;

import com.sulon.datatypes.lb_vec2.vec2f;

public class lb_regression {
	
	public static void lb_line_define(vec2f[] points,double a,double b,double c)	{
	    int end_idx = points.length - 1;

	    //Ax+By+C=0
	    double m1,m2;
	    m1=points[0].y - points[end_idx].y;
	    m2=points[0].x - points[end_idx].x;

	    a = m1;
	    b = -m2;
	    c = m2*points[end_idx].y - m1*points[end_idx].x;
	}

	
	public static void lb_liner_regression(vec2f[] points, double m, double b, double r)	{
	    int n = points.length;
	    if(n > 0) {
	    	double sum_x = 0;
	    	double sum_y = 0;
	        for(int i = 0; i < n; i++) {
	            sum_x += points[i].x;
	            sum_y += points[i].y;
	        }

	        double mean_x = sum_x / n;
	        double mean_y = sum_y / n;

	        double x_err[] = new double[n];
	        double y_err[] = new double[n];

	        for(int i = 0; i < n; i++) {
	            x_err[i] = points[i].x - mean_x;
	            y_err[i] = points[i].y - mean_y;
	        }

	        double A = 0;
	        double sum_err_xy = 0;
	        for (int i = 0; i < n; i++) {
	            A = A + (float) (Math.pow(x_err[i],2) - Math.pow(y_err[i],2));
	            sum_err_xy += (x_err[i] * y_err[i]);
	        }

	        if (sum_err_xy == 0)
	            sum_err_xy = 1e-6f;

	        A = A / sum_err_xy;

	        // m^2 + A*m - 1 = 0
	        double m1 = ((-A + Math.sqrt(A*A + 4)) * 0.5);
	        double m2 = ((-A - Math.sqrt(A*A + 4)) * 0.5);

	        double b1 = mean_y - (m1 * mean_x);
	        A = m1;
	        double B = -1;
	        double C = b1;

	        //calculate the maximum error on m1
	        r = 0;
	        double aux = Math.sqrt(A*A + B*B);
	        double dist;
	        for (int i = 0; i < n; i++) {
	            dist = Math.abs((points[i].x*A + points[i].y*B + C)/aux);
	            if (dist > r) r = dist;
	        }

	        double r1 = r;
	        double b2 = mean_y - (m2 * mean_x);
	        A = m2;
	        B = -1;
	        C = b2;

	        //calculate the maximum error on m2
	        r = 0;
	        aux = Math.sqrt(A*A + B*B);
	        for (int i = 0; i < n; i++) {
	            dist = Math.abs((points[i].x*A + points[i].y*B + C)/aux);
	            if (dist > r) r = dist;
	        }

	        double r2 = r;
	        if (r1 > r2) {
	            m = m2; b = b2; r = r2;
	        } else {
	            m = m1; b = b1; r = r1;
	        }
	    } else {
	        m = b = r = 0;
	        //debug("%s number of point == 0", __FUNCTION__);
	    }
	}
}
