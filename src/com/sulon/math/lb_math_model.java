package com.sulon.math;

import com.sulon.datatypes.*;

public class lb_math_model {
	/**
	 * Measurement model for range sensor from CH6 of Probabilistic Robotics book.
	 * http://robots.stanford.edu/probabilistic-robotics/ \n
	 * @param x measurement data
	 * @param x_hit expected measurement range
	 * @param x_max maximum possible measurement range
	 * @param var_hit variance of the measurement
	 * @param rate_short rate of exponential distribution
	 * @param z[4] weighted average for z_hit, z_short, z_max. z_rand and z[0]+z[1]+z[2]+z[3]=1;
	 * @return
	 */
	public double lb_beam_range_finder_measurement_model(double x, double x_hit, double x_max, double var_hit, double rate_short, double z[])	{
		
		double p_hit =   ((x > 0) && (x <= x_max)) ? lb_statistic_function.lb_pdf_normal_dist(var_hit, x_hit, x) : 0.0;
		double p_short = ((x > 0) && (x <= x_hit)) ? lb_statistic_function.lb_pdf_exponential_dist(rate_short, x) : 0.0;
		p_short *= 1.0 / (1.0 - Math.exp(-rate_short * x_hit));
		double p_max =   (x <= 0 || x >= x_max ? 1.0 : 0.0);
		double p_rand =  ((x > 0) && (x < x_max)) ? 1.0/x_max : 0.0;
		return (z[0] * p_hit) + (z[1] * p_short) + (z[2] * p_max)  + (z[3] * p_rand);
	}


	/**
	 * Sample base velocity motion model for sampling pose \f$x_t = (x',y',\theta')^T\f$
	 * from give initial pose and control.
	 * @param ut control \f$(v, w)^T\f$
	 * @param p initial pose \f$(x, y, \theta)^T\f$
	 * @param dt update time
	 * @param var robot specific motion error parameters \f$(\sigma_1 ... \sigma_6) \f$
	 * @return pose \f$x_t = (x',y',\theta')^T\f
	 */

	public pose2f lb_velocity_motion_model_sample( vec2f ut,
			pose2f p,
			double dt,
			double var[])
	{
		double v2 = LB_SQR(ut.x);
		double w2 = LB_SQR(ut.y);
		double v = ut.x + lb_statistic_function.lb_sample_normal_dist(var[0]*v2 + var[1]*w2);
		double w = ut.y + lb_statistic_function.lb_sample_normal_dist(var[2]*v2 + var[3]*w2);
		double r = lb_statistic_function.lb_sample_normal_dist(var[4]*v2 + var[5]*w2);

		double x, y, a, v_w;
		if(w != 0) {
			v_w = v/w;
			x = p.x - v_w*sin(p.a) + v_w*sin(p.a + w*dt);
			y = p.y + v_w*cos(p.a) - v_w*cos(p.a + w*dt);
			a = lb_normalize_angle(p.a + w*dt + r*dt);
		} else {
			x = p.x + v*cos(p.a);
			y = p.y + v*sin(p.a);
			a = lb_normalize_angle(p.a + r*dt);
		}
		return pose2f(x, y, a);
	}

	/**
	 * Closed form velocity motion model for compute \f$p(x_t|u_t, x_{t-1})\f$
	 * @param pt    hypothesized pose \f$(x', y', \theta')^T\f$
	 * @param ut    control \f$(v, w)^T\f$
	 * @param p     initial pose \f$(x, y, \theta)^T\f$
	 * @param dt    update time
	 * @param var   robot specific motion error parameters \f$(\sigma_1 ... \sigma_6) \f$
	 * @return \f$p(x_t|u_t, x_{t-1})\f$
	 */

	public double velocity_motion(pose2f pt,
			vec2f ut,
			pose2f p,
			double dt,
			double var[])
	{
		double x_x = p.x - pt.x;
		double y_y = p.y - pt.y;
		double tmp0 = ((x_x*cos(p.a)) + (y_y*sin(p.a)));
		double tmp1 = ((y_y*cos(p.a)) - (x_x*sin(p.a)));
		double u, xx, yy, rr, aa, v, w;

		u = 0.0;
		if(tmp1 != 0) {
			u = 0.5 * (tmp0/tmp1);
		}

		//compute center and radius of the circle
		xx = ((p.x + pt.x) * 0.5) + (u*(p.y - pt.y));
		yy = ((p.y + pt.y) * 0.5) + (u*(pt.x - p.x));
		rr = sqrt(LB_SQR(pt.x - xx) + LB_SQR(pt.y - yy));

		aa = lb_minimum_angle_distance(atan2(p.y - yy, p.x - xx), atan2(pt.y - yy, pt.x - xx));
		v = aa/dt * rr;

		if(ut.x >= 0) {
			if(macro_functions.LB_SIGN(yy) == macro_functions.LB_SIGN(aa))
				v = LB_SIGN(ut.x) * Math.abs(v);   //check sign with control input
			else
				return 0;
		} else {
			if(macro_functions.LB_SIGN(yy) != macro_functions.LB_SIGN(aa))
				v = LB_SIGN(ut.x) * Math.abs(v);   //check sign with control input
			else
				return 0;
		}

		w = aa/dt;

		double v2 = v*v;
		double w2 = w*w;
		double r = lb_minimum_angle_distance(w, (pt.a - p.a)/dt);


		double p0 = lb_statistic_function.lb_pdf_normal_dist(var[0]*v2 + var[1]*w2, 0.0, v - ut.x);
		double p1 = lb_statistic_function.lb_pdf_normal_dist(var[2]*v2 + var[3]*w2, 0.0, lb_minimum_angle_distance(ut.y, w));
		double p2 = lb_statistic_function.lb_pdf_normal_dist(var[4]*v2 + var[5]*w2, 0.0, r);

		return p0 * p1 * p2;
	}


	/**
	 * Sample based odometry motion model
	 * @param u_pt
	 * @param u_p
	 * @param p
	 * @param var
	 * @return
	 */

	public pose2f lb_odometry_motion_model_sample( pose2f u_pt,
			pose2f u_p,
			pose2f p,
			double var[])
	{
		double rot1 = lb_minimum_angle_distance(u_p.a, atan2(u_pt.y - u_p.y, u_pt.x - u_p.x));
		double tran = (u_pt.get_vec2() - u_p.get_vec2()).size();
		double rot2 = lb_minimum_angle_distance(rot1, lb_minimum_angle_distance(u_p.a, u_pt.a));

		double rot1_sqr = macro_functions.LB_SQR(rot1);
		double tran_sqr = macro_functions.LB_SQR(tran);
		double rot2_sqr = macro_functions.LB_SQR(rot2);

		double nrot1 = rot1 + lb_statistic_function.lb_sample_normal_dist(var[0]*rot1_sqr + var[1]*tran_sqr);
		double ntran = tran + lb_statistic_function.lb_sample_normal_dist(var[2]*tran_sqr + var[3]*rot1_sqr + var[3]*rot2_sqr);
		double nrot2 = rot2 + lb_statistic_function.lb_sample_normal_dist(var[0]*rot2_sqr + var[1]*tran_sqr);

		double x = p.x + ntran*cos(p.a + nrot1);
		double y = p.y + ntran*sin(p.a + nrot1);
		double a = lb_normalize_angle(p.a + nrot1 + nrot2);
		return pose2f(x, y, a);
	}

	/**
	 * Odometry motion model
	 * @param pt
	 * @param u_pt
	 * @param u_p
	 * @param p
	 * @param var
	 * @return
	 */
	public double odometry_motion(pose2f pt,
			pose2f u_pt,
			pose2f u_p,
			pose2f p,
			double var[])
	{
		double rot1 = lb_minimum_angle_distance(u_p.a, atan2(u_pt.y - u_p.y, u_pt.x - u_p.x));
		double tran = (u_pt.get_vec2() - u_p.get_vec2()).size();
		double rot2 = lb_minimum_angle_distance(rot1, lb_minimum_angle_distance(u_p.a, u_pt.a));

		double nrot1 = lb_minimum_angle_distance(p.a, atan2(pt.y - p.y, pt.x - p.x));
		double ntran = (pt.get_vec2() - p.get_vec2()).size();
		double nrot2 = lb_minimum_angle_distance(nrot1, lb_minimum_angle_distance(p.a, pt.a));

		double nrot1_sqr = macro_functions.LB_SQR(nrot1);
		double ntran_sqr = macro_functions.LB_SQR(ntran);
		double nrot2_sqr = macro_functions.LB_SQR(nrot2);

		double p0 = lb_statistic_function.lb_pdf_normal_dist(var[0]*nrot1_sqr + var[1]*ntran_sqr, 0.0, lb_misc_functions.lb_minimum_angle_distance(nrot1, rot1));
		double p1 = lb_statistic_function.lb_pdf_normal_dist(var[2]*ntran_sqr + var[3]*nrot1_sqr + var[3]*nrot2_sqr , 0.0, tran - ntran);
		double p2 = lb_statistic_function.lb_pdf_normal_dist(var[0]*nrot2_sqr + var[1]*ntran_sqr, 0.0, lb_misc_functions.lb_minimum_angle_distance(nrot2, rot2));

		return p0 * p1 * p2;
	}

}


