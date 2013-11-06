package com.sulon.math;

public class macro_functions {
	public static double M_E       = 2.7182818284590452354;   /* e */
	public static double M_LOG2E   = 1.4426950408889634074;   /* log_2 e */
	public static double M_LOG10E  = 0.43429448190325182765;  /* log_10 e */
	public static double M_LN2     = 0.69314718055994530942;  /* log_e 2 */
	public static double M_LN10    = 2.30258509299404568402;  /* log_e 10 */
	public static double M_PI      = 3.14159265358979323846; /* pi */
	public static double M_PI_2    = 1.57079632679489661923;  /* pi/2 */
	public static double M_PI_4    = 0.78539816339744830962;  /* pi/4 */
	public static double M_1_PI    = 0.31830988618379067154;  /* 1/pi */
	public static double M_2_PI    = 0.63661977236758134308;  /* 2/pi */
	public static double M_2_SQRTPI= 1.12837916709551257390;  /* 2/sqrt(pi) */
	public static double M_SQRT2   = 1.41421356237309504880;  /* sqrt(2) */
	public static double M_SQRT1_2 = 0.70710678118654752440;  /* 1/sqrt(2) */

	public static double LB_IS_ZERO = 1e-8;
	public static double LB_VERY_SMALL = 1e-6;

	public static double LB_SIGN (double A) {
		return ((A) >= 0.0 ? (1.0) : (-1.0));
	}

	public static boolean LB_SIGN_BOOL(double A) {
		return ((A) >= 0.0 ? (true) : (false));
	}

	public static double LB_ROUND(double x){
		return ((x) < 0 ? Math.ceil((x)-0.5):Math.floor((x)+0.5));
	}

	//public static double MY_LB_ROUND(x)             ((x) < 0 ? ceil((x)-0.5):floor((x)+0.5))
	public static double LB_SQR(double x) {
		return ((x)*(x));
	}

	public static double LB_SQUARE(double x){ 
		return (LB_SQR(x));
	}

	public static double LB_DEG2RAD(double x){
		return (((x)/180.0) * M_PI);
	}

	public static double LB_RAD2DEG(double x) { 
		return (((x)/M_PI) * 180.0);
	}

	public static double LB_SIZE(double x, double y) { 
		return (Math.sqrt(LB_SQR(x) + LB_SQR(y))); 
	}

	public static double LB_SQR_SIZE(double x, double y) { return (LB_SQR(x) + LB_SQR(y)); }

	//compare
	public static double LB_MIN(double a, double b) { 
		return ((a) <= (b) ? (a) : (b));
	}

	public static boolean LB_MIN_BOOL(double a, double b)  { 
		return ((a) <= (b) ? true : false);
	}

	public static double LB_MIN3(double a,double b,double c) { 
		return (Math.min(Math.min(a,b), c));
	}

	public static  double LB_MAX(double a, double b) {
		return (((a) >= (b)) ? (a) : (b));
	}

	public static  boolean LB_MAX_BOOL(double a, double b) { 
		return (((a) >= (b)) ? true : false);
	}

	public static double LB_MAX3(double a, double b, double c) { 
		return (Math.max(Math.max(a,b), c));
	}


	//simple print
	public static void LB_PRINT_VAR(double x) {
		System.out.println(x);
	}

	public static void LB_PRINT_VAL(double x){
		System.out.println(x);
	}

	public static void LB_PRINT_VEC(double[] x)	{ 
		for(int idx = 0; idx < x.length; idx++) {
			System.out.println(idx + " : " + x[idx]);
		} 
	}

	public static void LB_PRINT_2D_VEC(double[][] x) 
	{ 
		for(int idx1 = 0; idx1 < x.length; idx1++) { 
			for(int idx2 = 0; idx2 < x[idx1].length; idx2++) { 
				System.out.print("(" + x[idx1][idx2] + "),"); 
			} 
			System.out.println();
		}
	}

	
	public static void LB_RESIZE (double[] v, int x){
		 v = (double[])resizeArray(v, x);
	}
		
	//vector resize
	public static void LB_RESIZE_2D_VEC(double[][] v, int x, int y){ 
		  v = (double[][])resizeArray(v, x);
		    // new array is [x][old y]
		  for (int i=0; i<v.length; i++) {
		     if (v[i] == null){
		        v[i] = new double[y];
		     } else { 
		    	 v[i] = (double[])resizeArray(v[i], y); 
		     }
		  }
		   // new array is [x][y]
	}

	private static Object resizeArray (Object oldArray, int newSize) {
		int oldSize = java.lang.reflect.Array.getLength(oldArray);
		Class elementType = oldArray.getClass().getComponentType();
		Object newArray = java.lang.reflect.Array.newInstance(
				elementType, newSize);
		int preserveLength = Math.min(oldSize, newSize);
		if (preserveLength > 0)
			System.arraycopy(oldArray, 0, newArray, 0, preserveLength);
		return newArray; 
	}


	//check size
	public static boolean LB_CHK_SIZE2(double[] v0, double[] v1) { 
		return (v0.length == v1.length);
	}
	
	public static boolean LB_CHK_SIZE3(double[] v0, double[] v1, double[] v2){ 
		return (LB_CHK_SIZE2(v0,v1) && LB_CHK_SIZE2(v0,v2));
	}
	
	public static boolean LB_CHK_SIZE4(double[] v0, double[] v1, double[] v2, double[] v3){ 
		return (LB_CHK_SIZE3(v0, v1, v2) && LB_CHK_SIZE2(v0, v3));
	}
	
	public static boolean LB_CHK_SIZE5(double[] v0, double[] v1, double[] v2, double[] v3, double[] v4) { 
		return (LB_CHK_SIZE4(v0, v1, v2, v3) && LB_CHK_SIZE2(v0, v4));
	}

	

}