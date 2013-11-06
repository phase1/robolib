package com.sulon.object_detection;

import com.sulon.datatypes.lb_data_type;
import com.sulon.datatypes.lb_vec2;
import com.sulon.datatypes.lb_data_type.lrf_object;
import com.sulon.datatypes.lb_vec2.vec2f;

public class test_main {
	public static void main(String [ ] args)
	{
		lb_data_type tt = new lb_data_type();
		
		lrf_object lo= tt.new lrf_object() ;
		p(lo.type.toString());
		
		lb_vec2 tstvec2 = new lb_vec2();
		lb_vec2.vec2f[] tst = new lb_vec2.vec2f[3];
		
		lo.points = tst;
		
		lb_vec2.vec2f vec2_1 = tstvec2.new vec2f(4.0f, 4.0f);
		lb_vec2.vec2f vec2_2 = tstvec2.new vec2f(8.0f, 4.0f);
		lb_vec2.vec2f vec2_3 = tstvec2.new vec2f(4.0f, 4.0f);
		
		lo.points[0] = vec2_1;
		lo.points[1] = vec2_2;
		lo.points[2] = vec2_3;
		
		p("Angle 1 greater than Angle 2: "+tstvec2.vec2_angle_compare(vec2_2, vec2_3));
		p("Vector Size - 4, 4: "+lo.points[0].getVec2());
		
		p("Vector Size: "+vec2_1.size());
		
		p("Theta - "+vec2_1.theta());
		p("Theta 2PI Range - "+vec2_1.theta_2PI());
		p("Degrees - "+vec2_1.degree());
		
		p("Average of 4,8,4 and 4,4,4: "+tstvec2.vec2_average(lo.points).getVec2());
		p("Adding 4,4 with 4,4: " + lo.points[0].add(vec2_1).getVec2());
		
		p("Rotate by 4 radian: " + vec2_1.get_rotate(4, true).getVec2());
		
		p("Not : " + vec2_1.not().getVec2());
		
	}
	
	
	
	





	public static void p(String in){
		System.out.println(in);
		
	}
	
}
