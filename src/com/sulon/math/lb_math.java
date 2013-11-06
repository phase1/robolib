package com.sulon.math;

import java.util.Arrays;

public class lb_math{

	public static double lb_mean(double[] data)
	{
		double sum = 0.0;
		int size = data.length;
		for(double a : data)
			sum += a;
		return sum/size;
	}

	public static double lb_variance(double[] data)
	{
		double mean = lb_mean(data);
		double temp = 0;
		int size = data.length;
		for(double a :data)
			temp += (mean-a)*(mean-a);
		return temp/size;
	}

	public static double lb_stdev(double[] data)
	{
		return Math.sqrt(lb_variance(data));
	}

	public static double lb_median(double[] data) 
	{
		double[] b = new double[data.length];
		System.arraycopy(data, 0, b, 0, b.length);
		Arrays.sort(b);

		if (data.length % 2 == 0) 
		{
			return (b[(b.length / 2) - 1] + b[b.length / 2]) / 2.0;
		} 
		else 
		{
			return b[b.length / 2];
		}
	}

}