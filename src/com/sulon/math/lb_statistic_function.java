package com.sulon.math;

import java.util.Random;

import com.sulon.math.*;

public class lb_statistic_function {
	
	
	/*
	 * Probability density function (PDF) of the normal distribution.
	 * @param v variance
	 * @param m mean
	 * @param x
	 * @return PDF(x)
	 */
	public static double lb_pdf_normal_dist(double v, double m, double x) {
	    if(v <= 0) return 0;
	    return (1.0/Math.sqrt(2*macro_functions.M_PI*v)) * Math.exp(-macro_functions.LB_SQR(x - m) / (2.0 * v));
	}

	/**
	 * Probability density function (PDF) of the triangular distribution.
	 * @param v
	 * @param m
	 * @param x
	 * @return PDF(x)
	 */
	public static double lb_pdf_normal_triangular_dist(double v, double m, double x) {
	    //SQRT6 = 2.449489743
	    if(v <= 0) return 0;
	    double p = (1.0/(2.449489743 * Math.sqrt(v))) - (Math.abs(x - m)/(6.0 * v));
	    return macro_functions.LB_MAX(0.0, p);
	}

	/**
	 * PDF of the exponential distribution.
	 * @param rate
	 * @param x
	 * @return PDF(x)
	 */
	public static double lb_pdf_exponential_dist(double rate, double x) {
	    if(x < 0) return 0;
	    return rate * Math.exp(-rate * x);
	}

	/**
	 * Use a specific srand initialization to avoid multi-threading problems
	 * (executed only one time for a single program).
	 */
	

	
	public static void lb_srand() {
		Random generator = new Random(System.currentTimeMillis());
	    boolean first_run = true;
	    if(first_run) {
	    	generator.nextDouble();
	    	double rand_mem = 1+generator.nextDouble()%2048;
	    	Random tmpRnd = new Random((long) (generator.nextDouble() + rand_mem));
	        first_run = false;
	    }
	}

	/**
	 * Return a random variable between \f$[0,1]\f$ with respect to an uniform distribution.
	 */
	public static double lb_rand() {
		Random r = new Random();
		double d = r.nextFloat();
		return d;
	}

	
	/**
	 * Return a random variable between \f$[-1,1]\f$ with respect to an uniform distribution.
	 */
	public static double lb_crand() {
	    return 1.0 - (2.0 * lb_rand());
	}

	/**
	 * Sample a random value from (approximate) normal distribution with zero mean.
	 * @param v variance
	 * @return random sample from normal distribution with zero mean
	 */
	
	public static double lb_sample_normal_dist(double v) {
	    lb_srand();
	    double sum = 0;
	    for(int i = 0; i < 12; i++) {
	        sum += (lb_crand() * v);
	    }
	    return sum/2.0;
	}

	/**
	 * Sample a random value from (approximate) triangular distribution with zero mean.
	 * @param v variance
	 * @return random sample from triangular distribution with zero mean
	 */
	public static double lb_sample_triangular_dist(double v) {
	    //Math.sqrt(6) / 2 = 1.224744871
	    lb_srand();
	    return 1.224744871 * ((lb_crand()*v) +  (lb_crand()*v));
	}

	/**
	 * Sample a random value from uniform distribution in a circle.
	 * (http://www.comnets.uni-bremen.de/itg/itgfg521/per_eval/p001.html)
	 * @param a angle result \f$[-\pi, \pi]\f$
	 * @param r radius result \f$[-1, 1]\f$
	 */
	public static void lb_sample_circle_uniform_dist(double a, double r) {
	    lb_srand();
	    a = lb_crand() * macro_functions.M_PI;
	    r = Math.sqrt(lb_rand());
	}


	public static void lb_stratified_random(double[] v, int n) {
	    double k = 1.0/n;
	    double k_2 = k * 0.5;
	    if(v.length != n) macro_functions.LB_RESIZE(v, n);
	    for(int i = 0; i < n; i++) {
	        v[i] = (k_2 + (k*i)) + (lb_rand() * k) - k_2;
	    }
	}

	/**
	 * Compute a cumulative summation.
	 * @param v number vector
	 * @param sum output vector
	 */

	public static void lb_cumulative_sum(double[] v, double[] sum) {
	    int n = v.length;
	    if(n == 0) return;
	    if(sum.length != v.length)  macro_functions.LB_RESIZE(sum, v.length);
	    sum[0] = v[0];
	    for(int i = 1; i < n; i++) {
	        sum[i] = sum[i-1] + v[i];
	    }
	}

	/**
	 * Compute a square summation
	 * @param v input
	 * @return square sum of all value in v
	 */
	public static double lb_square_sum(double[] v) {
	    int n = v.length;
	    double sum = 0;
	    for(int i = 0; i < n; i++) {
	        sum +=  macro_functions.LB_SQR(v[i]);
	    }
	    return sum;
	}

	public static double lb_mean(double[] v) {
	    int n = v.length;
	    if(n == 0) return 0;
	    double sum = 0;
	    for(int i = 0; i < n; i++) {
	        sum += v[i];
	    }
	    return sum / n;
	}


	public static double lb_stdev(double[] v, double m) {
	    int n = v.length;
	    if(n < 2) return 0;
	    double sum_rms_err = 0;
	    for(int i = 0; i < n; i++) {
	        sum_rms_err += macro_functions.LB_SQR((v[i] - m));
	    }
	    return Math.sqrt(sum_rms_err / (n - 1));
	}

}
