package com.sulon.datatypes;

import java.lang.Math;

import com.sulon.math.macro_functions;

public class lb_vec2{
	public static class vec2f {
		public double x;
		public double y;

		//Empty Constructor
		public vec2f(){
			this.x = 0.0;
			this.y = 0.0;
		}
		//Constructor
		public vec2f(double x, double y){
			this.x = x;
			this.y = y;
		}
		
		//Copy constructor
		public vec2f(vec2f v){
			this.x = v.x;
			this.y = v.y;
		}

		///Unary minus
		public vec2f not(){
			this.x = -x;
			this.y = -y;
			return this;
		}

		///Addition for vector/vector
		public vec2f add(vec2f v) {
			vec2f c = new vec2f();
			c.x = x + v.x;
			c.y = y + v.y;
			return c;
		}

		///Subtraction for vector/vector
		public vec2f minus(vec2f v) {
			vec2f c = new vec2f();
			c.x = x - v.x;
			c.y = y - v.y;
			return c;
		}

		///Basic assignment for vector/vector
		public vec2f equal(vec2f v) {
			x = v.x;
			y = v.y;
			return this;
		}

		///Assignment by addition for vector/vector
		public vec2f plus_equal(vec2f v) {
			x += v.x;
			y += v.y;
			return this;
		}

		///Assignment by subtraction for vector/vector
		public vec2f minus_equal(vec2f v) {
			x -= v.x;
			y -= v.y;
			return this;
		}

		///Dot product between two vectors
		public double dot (vec2f v) {
			return  ((x * v.x) + (y * v.y));
		}

		///Cross product between two vectors
		public double cross(vec2f v) {
			return ((x * v.y) - (y * v.x));
		}

		///Multiply by scalar value
		public vec2f times(double s) {
			this.x = x * s;
			this.y = y * s;
			return this;
		}

		///Divide by scalar value
		public vec2f divide(double s) {
			this.x = x / s;
			this.y = y / s;
			return this;
		}

		///Assignment by multiply with scalar value
		public vec2f times_equal(double s) {
			x *= s;
			y *= s;
			return (this);
		}

		///Assignment by divide with scalar value
		public vec2f divide_equal(double s) {
			x /= s;
			y /= s;
			return this;
		}

		///Get size
		public double size() {
			return Math.sqrt(x * x + y * y);
		}

		///Get square size
		public double sqr_size() {
			return (x * x + y * y);
		}

		///Get angle in \f$(-\pi, \pi)\f$ radian range
		public double theta() {
			return Math.atan2(y, x);
		}

		///Get angle in \f$(0, 2\pi)\f$ radian range
		public double theta_2PI() {
			double tmp = Math.atan2(y, x);
			if (tmp < 0)
				tmp += 2 * macro_functions.M_PI;
			return tmp;
		}

		///Get angle in \f$(-180, 180)\f$ degree range
		public double degree() {
			return Math.toDegrees(theta());
		}

		///Get angle in \f$(0, 360)\f$ degree range
		public double degree_360() {
			double tmp = Math.atan2(y, x);
			if (tmp < 0)
				tmp += 2 * macro_functions.M_PI;
			return Math.toDegrees(tmp);
		}

		///Check for zero size vector
		public boolean is_zero() {
			return ((x == 0) && (y == 0));
		}

		//Get normalized vector
		public vec2f get_normalize() {
			vec2f c = new vec2f();
			c = this.divide(size());
			return c;

		}

		///Normalize the vector
		public vec2f normalize() {
			return this.divide(size());
		}

		/**
		 * Get rotated vector
		 * \param angle rotate angle
		 * \param angle unit select (true for radian, false for degree)
		 */
		public vec2f get_rotate(double angle, boolean rad ) {
			rad = true;
			double tmp = angle;
			if (!rad)
				tmp = (float) Math.toDegrees(angle);
			double c = Math.cos(tmp);
			double s = Math.sin(tmp);

			vec2f d = new vec2f();
			d.x = (x * c - y * s);
			d.y = (x * s + y * c);

			return d;
		}

		/**
		 * Rotate the vector
		 * \param angle rotate angle
		 * \param angle unit select (true for radian, false for degree)
		 */
		public vec2f rotate(double angle, boolean rad) {
			double a = angle;
			if (!rad)
				a = Math.toDegrees(angle);

			double c = Math.cos(angle);
			double s = Math.sin(angle);
			double tmpx = (x * c - y * s);
			this.y = (x * s + y * c);
			this.x = tmpx;
			return this;
		}

		///Support for output stream operator
		public String getVec2() {
			return (this.x + " " + this.y);
		}
	}

	public class vec2i{
		private float M_PI = 3.14159265359f;

		public int x;

		public int y;

		//Empty Constructor
		public vec2i(){
			this.x = 0;
			this.y = 0;
		}

		//Constructor
		public vec2i(int x, int y){
			this.x = x;
			this.y =y;
		}

		//Copy constructor
		public vec2i(vec2i v){
			this.x = v.x;
			this.y = v.y;
		}

		///Unary minus
		public vec2i not(){
			this.x = -x;
			this.y = -y;
			return this;
		}

		///Addition for vector/vector
		public vec2i add(vec2i v) {
			vec2i c = new vec2i();
			c.x = x + v.x;
			c.y = y + v.y;
			return c;
		}

		///Subtraction for vector/vector
		public vec2i minus(vec2i v) {
			vec2i c = new vec2i();
			c.x = x - v.x;
			c.y = y - v.y;
			return c;
		}

		///Basic assignment for vector/vector
		public vec2i equal(vec2i v) {
			x = v.x;
			y = v.y;
			return this;
		}

		///Assignment by addition for vector/vector
		public vec2i plus_equal(vec2i v) {
			x += v.x;
			y += v.y;
			return this;
		}

		///Assignment by subtraction for vector/vector
		public vec2i minus_equal(vec2i v) {
			x -= v.x;
			y -= v.y;
			return this;
		}

		///Dot product between two vectors
		public float dot (vec2i v) {
			return  ((x * v.x) + (y * v.y));
		}

		///Cross product between two vectors
		public float cross(vec2i v) {
			return ((x * v.y) - (y * v.x));
		}

		///Multiply by scalar value
		public vec2i times(int s) {
			vec2i c = new vec2i();
			c.x = x * s;
			c.y = y * s;
			return c;
		}

		///Divide by scalar value
		public vec2i divide(int s) {
			vec2i c = new vec2i();
			c.x = x / s;
			c.y = y / s;
			return c;
		}

		///Assignment by multiply with scalar value
		public vec2i times_equal(int s) {
			x *= s;
			y *= s;
			return (this);
		}

		///Assignment by divide with scalar value
		public vec2i divide_equal(int s) {
			x /= s;
			y /= s;
			return this;
		}

		///Get size
		public int size() {
			return (int) Math.sqrt(x * x + y * y);
		}

		///Get square size
		public int sqr_size() {
			return (x * x + y * y);
		}

		///Get angle in \f$(-\pi, \pi)\f$ radian range
		public int theta() {
			return (int) Math.atan2(y, x);
		}

		///Get angle in \f$(0, 2\pi)\f$ radian range
		public int theta_2PI() {
			double tmp = Math.atan2(y, x);
			if (tmp < 0)
				tmp += 2 * M_PI;
			return (int) tmp;
		}

		///Get angle in \f$(-180, 180)\f$ degree range
		public int degree() {
			return (int) Math.toDegrees(theta());
		}

		///Get angle in \f$(0, 360)\f$ degree range
		public int degree_360() {
			float tmp = (float) Math.atan2(y, x);
			if (tmp < 0)
				tmp += 2 * M_PI;
			return (int)Math.toDegrees(tmp);
		}

		///Check for zero size vector
		public boolean is_zero() {
			return ((x == 0) && (y == 0));
		}

		//Get normalized vector
		public vec2i get_normalize() {
			vec2i c = new vec2i();
			c = this.divide(size());
			return c;

		}

		/**
		 * Get rotated vector
		 * \param angle rotate angle
		 * \param angle unit select (true for radian, false for degree)
		 */
		public vec2i get_rotate(float angle, boolean rad ) {
			double tmp = angle;
			if (!rad)
				tmp = (float) Math.toDegrees(angle);
			double c = Math.cos(tmp);
			double s = Math.sin(tmp);

			vec2i d = new vec2i();
			d.x = (int) (x * c - y * s);
			d.y = (int) (x * s + y * c);

			return d;
		}

		/**
		 * Rotate the vector
		 * \param angle rotate angle
		 * \param angle unit select (true for radian, false for degree)
		 */
		public vec2i rotate(float angle, boolean rad) {
			double a = angle;
			if (!rad)
				a = Math.toDegrees(angle);

			double c = Math.cos(angle);
			double s = Math.sin(angle);
			double tmpx = (x * c - y * s);
			this.y = (int) (x * s + y * c);
			this.x = (int) tmpx;
			return this;
		}

		///Support for output stream operator
		public String getVec2(vec2i v) {
			return (this.x + " " + this.y);
		}




	}

	public boolean vec2_angle_compare(vec2i i, vec2i j) {
		double ai = Math.atan2(i.y, i.x);
		double aj = Math.atan2(j.y, j.x);
		return ai < aj;
	}

	public boolean vec2_size_compare(vec2i i, vec2i j) {
		double si = i.size();
		double sj = j.size();
		return si < sj;
	}


	public vec2i vec2_average(vec2i[] v) {
		int n = v.length;
		if(n == 0) return new vec2i();
		if(n == 1) return v[0];
		float x = 0;
		float y = 0;
		for(int i = 0; i <  n; i++) {
			x += v[i].x;
			y += v[i].y;
		}
		return (new vec2i((int)x/n, (int)y/n));
	}

	public boolean vec2_angle_compare(vec2f i, vec2f j) {
		double ai = Math.atan2(i.y, i.x);
		double aj = Math.atan2(j.y, j.x);
		return ai < aj;
	}

	public boolean vec2_size_compare(vec2f i, vec2f j) {
		double si = i.size();
		double sj = j.size();
		return si < sj;
	}


	public vec2f vec2_average(vec2f[] v) {
		int n = v.length-1;
		if(n == 0) return new vec2f();
		if(n == 1) return v[0];
		float x = 0;
		float y = 0;
		for(int i = 0; i <  n; i++) {
			x += v[i].x;
			y += v[i].y;
		}
		return (new vec2f(x/n, y/n));
	}

}



