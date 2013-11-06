package com.sulon.datatypes;

import com.sulon.datatypes.lb_vec2;
import com.sulon.datatypes.lb_vec2.vec2f;


public class lb_pose2f {

	public double x;    //!< X position
	public double y;    //!< Y position
	public double a;    //!< direction

	///Constructor
	public lb_pose2f(){  
		this.x = 0;
		this.y = 0;
		this.a = 0;
	}

	///Constructor
	public lb_pose2f( double xx, double yy,  double aa) { 
		this.x = (xx);
		this.y = (yy);
		this.a = (aa);
	}

	///Constructor
	public lb_pose2f( lb_vec2.vec2f p,  double aa){
		this.x = p.x;
		this.y = p.y;
		this.a = aa;
	}

	///Copy constructor
	public lb_pose2f (lb_pose2f p2){ 
		this.x = p2.x;
		this.y = p2.y;
		this.a = p2.a;
	}


	///Addition
	/*public pose2f add(pose2f p) {
		return (new pose2f(x + p.x, y + p.y, lb_normalize_angle(a + p.a)));
	}*/

	///Addition
	public lb_pose2f add(lb_vec2.vec2f v) {
		return (new lb_pose2f(x + v.x, y + v.y, a));
	}

	///Subtraction
	/*public pose2f minus(pose2f p)  {
		return (new pose2f(x - p.x, y - p.y, lb_normalize_angle(a - p.a)));
	}*/

	///Subtraction
	public lb_pose2f minus(lb_vec2.vec2f v)  {
		return (new lb_pose2f(x - v.x, y - v.y, a));
	}

	public lb_vec2.vec2f get_vec2()  {
		lb_vec2 lb_v2 = new lb_vec2();
		lb_vec2.vec2f v2 = new vec2f(x,y);
		return (v2);
	}


	public lb_vec2.vec2f get_vec2_to( lb_pose2f p)  {
		lb_vec2 lb_v2 = new lb_vec2();
		lb_vec2.vec2f v2 = new vec2f(p.x - x, p.y - y);
		return (v2);
	}


	public double dist_to(lb_pose2f p) {
		return (double)Math.sqrt(Math.pow((p.x - x) , 2)+ Math.pow((p.y - y),2));
	}


/*	public double angle_to(pose2f p) {
		return minimum_angle_distance(a, p.a);
	}*/


}
