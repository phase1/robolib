package com.sulon.math;

import com.sulon.math.*;;

public class lb_misc_functions {

	/** \defgroup g_user_template User-friendly template functions */


	public static void lb_build_angle_table(double start, double end, int step, double[] table){

		if(table.length != step){
			macro_functions.LB_RESIZE(table, step);
		}
		double angle_step = (end - start) / step;
		for(int i = 0; i < step; i++) {
			table[i] = start + (i * angle_step);
		}
	}

	public static void lb_build_cos_sin_table(double start, double end, int step, double[] cos_table, double[] sin_table) {
		if(cos_table.length != (int)step) macro_functions.LB_RESIZE(cos_table, step);
		if(sin_table.length != (int)step) macro_functions.LB_RESIZE(sin_table, step);
		double angle_step = (end - start) / step;
		for(int i = 0; i < step; i++) {
			cos_table[i] = Math.cos(start + (i * angle_step));
			sin_table[i] = Math.sin(start + (i * angle_step));
		}
	}

	public static void lb_build_cos_sin_table(double[] angle_table, double[] cos_table, double[] sin_table){
		int step = angle_table.length;
		if(cos_table.length != (int)step) macro_functions.LB_RESIZE(cos_table, step);
		if(sin_table.length != (int)step) macro_functions.LB_RESIZE(sin_table, step);
		for(int i = 0; i < step; i++) {
			cos_table[i] = Math.cos(angle_table[i]);
			sin_table[i] = Math.sin(angle_table[i]);
		}
	}

	/**
	 * Normalize the angle to \f$(-\pi, \pi)\f$ radian unit
	 * \param a input angle value
	 * \return normalized angle
	 */

	 public static double lb_normalize_angle(double a) {
		 int m = (int)(a / (2.0*macro_functions.M_PI));
		 a = a - (m * macro_functions.M_PI * 2.0);
		 if (a < (-macro_functions.M_PI))
			 a += (2.0 * macro_functions.M_PI);
		 if (a >= macro_functions.M_PI)
			 a -= (2.0 * macro_functions.M_PI);
		 return a;
	 }

	 /**
	  * Find the minimum angle distance between two input angle in radian unit
	  * \param a input angle value
	  * \param b input angle value
	  * \return minimum angle distance from a -> b
	  */

	 public static double lb_minimum_angle_distance(double a, double b) {
		 a = lb_normalize_angle(a);
		 b = lb_normalize_angle(b);

		 double diff = b - a;

		 if(Math.abs(diff) > macro_functions.M_PI) {
			 if(diff < 0)
				 diff += (2 * macro_functions.M_PI);
			 else
				 diff -= (2 * macro_functions.M_PI);
		 }
		 return diff;
	 }


}
