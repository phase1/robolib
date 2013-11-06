package com.sulon.object_detection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.logging.Logger;

import com.sulon.datatypes.lb_data_type;
import com.sulon.datatypes.lb_vec2;
import com.sulon.datatypes.lb_data_type.lrf_object;
import com.sulon.datatypes.lb_data_type.lrf_object_type;
import com.sulon.datatypes.lb_vec2.vec2f;
import com.sulon.math.lb_math;
import com.sulon.math.macro_functions;

public class lb_lrf_object_detect {
	
	public int lb_lrf_recusive_line_fitting(vec2f[] points, lrf_object[] objects, float min_length, float err_threshold, int min_point)	{
		int n = points.length;
		int id = -1;
		
		//check total point
		if(n < min_point) {
			//debug("%s: not enough point", __FUNCTION__);
			return 0;
		}

		//check length (simple)
		if((points[0].minus(points[n - 1])).size() < min_length) {
			//debug("%s: not enough length", __FUNCTION__);
			return 0;
		}

		float m = 0, b = 0, r = 0;
		lb_regression.lb_liner_regression(points, m, b, r);

		//check result
		if(m == 0 && b == 0 && r == 0) {
			//debug("%s: linear regression fail", __FUNCTION__);
			return 0;
		}

		if(r < err_threshold) {
			//LB_PRINT_VAL("single line");
			double theta= Math.atan2(-1/m,1);

			if ( ((m > 0) && (b >0)) || ((m < 0) && (b < 0)) ) {
				theta =- (macro_functions.M_PI - theta);
			};

			double rho =  Math.abs(b/Math.sqrt(Math.pow(m*m,2)+1));
			vec2f p = new vec2f(rho*Math.cos(theta), rho*Math.sin(theta));
			vec2f o = new vec2f(Math.cos(theta), Math.sin(theta));

			//add line
			lrf_object line = null;
			line.points = points;
			line.type = (int) lrf_object_type.LRF_OBJ_LINE.getObjectType();
			line.extra_point[0] = o;    //line direction
			line.extra_point[1] = p;                                  //vector to line normal
			line.extra_param[0] = m;
			line.extra_param[1] = b;
			line.extra_param[2] = r;
			line.segment_id = id;
			List<lrf_object> x = Arrays.asList(objects);
			x.add(line);
			objects = (lrf_object [])x.toArray() ;
			return 1;
		} else {
			//LB_PRINT_VAL("recursive check");
			double A = 0, B = 0, C = 0, ss = 0;
			lb_regression.lb_line_define(points, A, B, C);
			ss = macro_functions.LB_SIZE(A, B);

			// search for lines break
			int n_break = 0;
			float dist, dist_max = -1.0f;
			for (int i = 0; i < n; i++) {
				dist = (float) Math.abs( (points[i].x*A + points[i].y*B + C)/ss);
				if (dist > dist_max) {
					dist_max = dist;
					n_break = i;
				}
			}

			if (dist_max > err_threshold) {
				//First half
				vec2f[] first_beam = null;
				
				ArrayList<vec2f> first_list = new ArrayList<vec2f>(Arrays.asList(first_beam));
				
				for(int i = 0; i < n_break; i++) {

					first_list.add(points[i]);
				}
				
				first_list.toArray(first_beam);
				
				int line1 = lb_lrf_recusive_line_fitting(first_beam,
						objects,
						min_length,
						err_threshold,
						min_point);

				//Second half
				vec2f[] second_beam = null;
				ArrayList<vec2f> second_list = new ArrayList<vec2f>(Arrays.asList(second_beam));
				for(int i = n_break; i < n; i++) {
					second_list.add(points[i]);
				}

				second_list.toArray(second_beam);
				
				int line2 = lb_lrf_recusive_line_fitting(second_beam,
						objects,
						min_length,
						err_threshold,
						min_point);
				
				return line1 + line2;
			}//if
		}//if

		return 0;
	}


	public boolean lb_lrf_arc_fiting(
			vec2f[] points,
			lrf_object[] objects,
			float min_angle,
			float max_angle,
			float max_stdev,
			float arc_ratio,
			float is_line_error,
			float is_line_stdev, 
			float min_diameter, 
			float max_diameter, 
			int id, 
			int min_point)	{

		min_diameter = -1f;
		id = -1;
		min_point = 10;
		max_diameter = Float.MAX_VALUE;

		ArrayList<lrf_object> obj_list = new ArrayList<lrf_object>(Arrays.asList(objects));

		int n = points.length;
		if(n < min_point) {
			//debug.("%s: not enough point", __FUNCTION__);
			return false;
		}

		vec2f middle = points[n >> 1];
		vec2f right = points[0];
		vec2f left = points[n - 1];
		vec2f center = (right.add(left)).times(0.5f);

		//LB_PRINT_VAR(left);
		//LB_PRINT_VAR(middle);
		//LB_PRINT_VAR(right);

		double dist_lr = (left.minus(right)).size();
		double dist_mc = (middle.minus(center)).size();

		if(dist_lr <= min_diameter) {
			//debug("%s: too small", __FUNCTION__);
			return false;
		}

		if(dist_lr >= max_diameter) {
			//debug("%s: too big", __FUNCTION__);
			return false;
		}

		//LB_PRINT_VAR(dist_lr);
		//LB_PRINT_VAR(dist_mc);
		//LB_PRINT_VAR(dist_mc/dist_lr);

		boolean check_ratio = dist_mc > (arc_ratio * dist_lr);
		boolean check_size = dist_mc < dist_lr;

		//LB_PRINT_VAR(check_ratio);
		//LB_PRINT_VAR(check_size);

		if(!(check_ratio && check_size)) {
			//debug("%s: arc ratio and size not correct", __FUNCTION__);
			return false;
		}


		int n_angle = n - 3;
		double ma = 0, mb = 0;
		double[] angles = new double[n_angle];

		// angle inside arc
		vec2f xy_temp;
		for (int i = 0; i < n_angle; i++) {
			xy_temp = points[i+1];
			ma = (left.minus(xy_temp)).theta();
			mb = (right.minus(xy_temp)).theta();
			//angles[i] = Math.abs(lb_normalize_angle(ma - mb));
		}

		//LB_PRINT_VEC(slopes);
		double average = lb_math.lb_mean(angles);
		double std_dev = lb_math.lb_stdev(angles);

		//LB_PRINT_VAR(average);
		//LB_PRINT_VAR(std_dev);

		//check if is line
		if ((Math.abs(average - macro_functions.M_PI) < is_line_error) && (std_dev < is_line_stdev)) {
			//debug("%s: is a line", __FUNCTION__);
			return false;// is a line
		}

		//LB_PRINT_VAR(std_dev);

		//check arc
		if (std_dev < max_stdev) {
			vec2f tmp = right.minus(left);
			double angle_to_rotate =  Math.atan2( tmp.y, tmp.x);
			tmp = tmp.rotate(-angle_to_rotate, true);
			double mid = tmp.x / 2.0f;
			double q = average - (macro_functions.M_PI/2.0f);
			double height = mid * Math.tan(q);
			vec2f center1 = new vec2f(mid, height);
			double radius = center1.size();
			center1 = center1.rotate(angle_to_rotate, true);
			center = center1.add(left);

			if(average < 0.f) {
				average = (2 * macro_functions.M_PI) + average;
			}

			if((average >= min_angle) && (average <= max_angle)) {
				//LB_PRINT_VAR(center);
				//LB_PRINT_VAR(radius);
				lrf_object arc = null;
				arc.points = points;
				arc.type = lb_data_type.lrf_object_type.LRF_OBJ_ARC.getObjectType();
				arc.extra_point[0] = center;
				arc.extra_param[0] =  radius;
				arc.extra_param[1] =  average;
				arc.extra_param[2] = std_dev;
				arc.segment_id = id;
				obj_list.add(arc);
				
				obj_list.toArray(objects);
				
				return true;
			} else {
				System.out.println("Average angle outside range of arc");
				//debug("%s: average angle outside range", __FUNCTION__);
			}
		} else {
			System.out.println("angle standard deviation > max_stdev ");
			//debug("%s: angle standard deviation > max_stdev ", __FUNCTION__);
		}

		return false;
	}

	public int lb_lrf_leg_detect(vec2f[] points,
			lrf_object[] objects,
			float min_size,
			float max_size,
			float leg_arc_ratio,
			int id ,
			boolean do_ratio_check ,
			int min_points)	{
		id = -1;
		do_ratio_check = false;
		min_points = 5;
		int n = points.length;
		
		
		ArrayList<lrf_object> obj_list = new ArrayList<lrf_object>(Arrays.asList(objects));
		
		
		if(n < min_points) {
			//debug("%s: not enough point", __FUNCTION__);
			System.out.println("Not enough points!");
			return 0;
		}

		vec2f middle = points[n >> 1];
		vec2f right = points[0];
		vec2f left = points[n - 1];
		vec2f center = (right.add(left)).times(0.5);

		float dist_lr = (float) (left.minus(right)).size();

		if(dist_lr < min_size) {
			//debug("%s: too small", __FUNCTION__);

			return 0;
		}

		if(dist_lr > (2.0 * max_size)) {
			//debug("%s: too big", __FUNCTION__);
			return 0;
		}

		float dist_mc = (float) (middle.minus(center)).size();
		boolean check_ratio = false;

		if(dist_lr <= max_size) {
			//LB_PRINT_VAL("check 1 leg");
			check_ratio = dist_mc > (leg_arc_ratio * dist_lr);
			check_ratio |= (!do_ratio_check);

			if(check_ratio) {
				//LB_PRINT_VAL("add 1 leg");
				lrf_object leg = null;
				leg.type = lb_data_type.lrf_object_type.LRF_OBJ_LEG.getObjectType();
				leg.points = points;
				leg.extra_point[0] = center;
				leg.extra_param[0] = dist_lr * 0.5; //radius
				leg.segment_id = id;
				obj_list.add(leg);
				obj_list.toArray(objects);
				return 1;
			} else {
				//debug("%s: 1 leg ratio fail", __FUNCTION__);
				System.out.println("1 leg ratio fail");
				return 0;
			}

		} else {
			//LB_PRINT_VAL("check 2 leg");
			int cnt = 0;
			vec2f right_middle = points[n >> 2];
			vec2f right_center = (right.add(middle)).times(0.5);
			float dist_mr = (float) (middle.minus(right)).size();
			float dist_rm_rc = (float) (right_middle.minus(right_center)).size();

			check_ratio = dist_rm_rc > (leg_arc_ratio * dist_mr);
			check_ratio |= (!do_ratio_check);
			if(check_ratio) {
				//add right leg
				lrf_object leg = null;
				
				ArrayList<vec2f> leg_list = new ArrayList<vec2f>(Arrays.asList(leg.points));
				
				for(int i = 0; i < (n >> 1); i++) {
					leg_list.add(points[i]);
				}
				leg.type = lb_data_type.lrf_object_type.LRF_OBJ_LEG.getObjectType();
				leg.extra_point[0] = right_center;
				leg.extra_param[0] = dist_mr / 2; //radius
				leg.segment_id = id;
				leg_list.toArray(leg.points);
				obj_list.add(leg);
				cnt++;
			} else {
				//debug("%s: right leg ratio fail", __FUNCTION__);
			}


			vec2f left_middle = points[(n >> 2) + (n >> 1)];
			vec2f left_center = (left.add(middle)).times(0.5);
			float dist_ml = (float) (middle.minus(left)).size();
			float dist_lm_lc = (float) (left_middle.minus(left_center)).size();

			check_ratio = dist_lm_lc > (leg_arc_ratio * dist_ml);
			check_ratio |= (!do_ratio_check);
			if(check_ratio) {
				//add left leg
				lrf_object leg = null;
				
				ArrayList<vec2f> leg_list = new ArrayList<vec2f>(Arrays.asList(leg.points));
				
				for(int i = (n >> 1); i < n; i++) {
					leg_list.add(points[i]);
				}
				leg.type = lb_data_type.lrf_object_type.LRF_OBJ_LEG.getObjectType();
				leg.extra_point[0] = left_center;
				leg.extra_param[0] = dist_ml * 0.5; //radius
				leg.segment_id = id;
				
				leg.points = (vec2f [])leg_list.toArray();
				
				obj_list.add(leg);
				cnt++;
			} else {
				System.out.println("Left leg ratio fail");
				//debug("%s: left leg ratio fail", __FUNCTION__);
			}

			if(cnt == 2) {
				//remove last two leg
				obj_list.remove(obj_list.size()-1);
				obj_list.remove(obj_list.size()-1);

				lrf_object leg2 = null;
				leg2.points = points;
				leg2.type = lb_data_type.lrf_object_type.LRF_OBJ_LEG2.getObjectType();
				leg2.extra_point[0] = center;
				leg2.extra_param[0] = dist_lr * 0.5;
				leg2.segment_id = id;
				obj_list.add(leg2);
			}
			
			obj_list.toArray(objects);
			return cnt;
		}
	}

	public boolean lb_lrf_group_detect(vec2f[] points,
			lrf_object[] objects,
			float min_size,
			float max_size,
			int id,
			int min_points)	{
		id = -1;
		min_points = 5;
		int n = points.length;
		
		ArrayList<lrf_object> obj_list = new ArrayList<lrf_object>(Arrays.asList(objects));
		
		if(n < min_points) {
			//debug("%s: not enough point", __FUNCTION__);
			return false;
		}

		vec2f middle = points[n >> 1];
		vec2f right = points[0];
		vec2f left = points[n - 1];
		vec2f center = (right.add(left)).times( 0.5);

		float dist_lr = (float) (left.minus(right)).size();
		float dist_mc = (float) (middle.minus(center)).size();
		double dist_max = macro_functions.LB_MAX(dist_lr, dist_mc);


		if(dist_max < min_size) {
			//	debug("%s: too small", __FUNCTION__);
			return false;
		}

		if(dist_max > max_size) {
			//	debug("%s: too big", __FUNCTION__);
			return false;
		}

		double sum_x = 0;
		double sum_y = 0;
		for(int i = 0; i < n; i++) {
			sum_x += points[i].x;
			sum_y += points[i].y;
		}
		sum_x /= n;
		sum_y /= n;
		vec2f v2 = new vec2f(sum_x, sum_y);

		lrf_object group = null;
		group.points = points;
		group.type = lb_data_type.lrf_object_type.LRF_OBJ_GROUP.getObjectType();
		group.extra_point[0] = v2;
		group.extra_param[0] = dist_max / 2; //radius
		group.segment_id = id;
		obj_list.add(group);

		obj_list.toArray(objects);
		return true;
	}

	public int lb_lrf_object_human_check(lrf_object[] objects,
			float max_leg_distance,
			float min_group_size,
			float max_group_size,
			boolean allow_one_leg)
	{
		allow_one_leg = false;
		int n = objects.length;

		if(n == 0) return 0;

		int human_cnt = 0;
		ArrayList<lrf_object> obj_list = new ArrayList<lrf_object>(Arrays.asList(objects));
		ArrayList<lrf_object> leg_object = new ArrayList<lrf_object>();
		Iterator<lrf_object> it = leg_object.iterator();
		double leg_dist;
		for(int i = 0; i < n; i++) {
			if(objects[i].type == lb_data_type.lrf_object_type.LRF_OBJ_LEG2.getObjectType()) {
				//add human
				//LB_PRINT_VAL("add 2 leg");
				lrf_object human = objects[i];
				human.type = lb_data_type.lrf_object_type.LRF_OBJ_HUMAN.getObjectType();
				obj_list.add(human);
				human_cnt++;
			} else if(objects[i].type == lb_data_type.lrf_object_type.LRF_OBJ_LEG.getObjectType()) {
				if(leg_object.size() > 0) {
					//check current leg with leg_list
					while(it.hasNext()) {
						//check distance
						lrf_object curr_leg = it.next();
						leg_dist = curr_leg.extra_point[0].minus(objects[i].extra_point[0]).size();
						if(leg_dist < max_leg_distance) {
							//LB_PRINT_VAL("add 1+1 leg");
							//add human
							lrf_object human;
							human.points = curr_leg.points;
							human.points.insert( human.points.end(),
									objects[i].points.begin(),
									objects[i].points.end());
							human.extra_point[0] =
									curr_leg.extra_point[0].add(objects[i].extra_point[0]).times(0.5);
							human.type = lb_data_type.lrf_object_type.LRF_OBJ_HUMAN.getObjectType();
							obj_list.add(human);
							human_cnt++;


							//remove
							leg_object.remove(curr_leg);
							break;
						} else {
							//LB_PRINT_VAL("add new leg to the list");
							leg_object.add(objects[i]);
						}
					}
				} else {
					//LB_PRINT_VAL("get 1st leg");
					leg_object.add(objects[i]);
				}
			}
		}


		//check all orphan leg
		int pass = 0;
		int group_idx = -1;
		while(!leg_object.isEmpty() && allow_one_leg) {
			//check 1 leg condition
			it = leg_object.iterator();
			pass = -10;
			group_idx = -1;
			for(int i = 0; i < n; i++) {
				lrf_object curr_leg = it.next();
				if(objects[i].segment_id == curr_leg.segment_id) {
					switch(objects[i].type) {
					case lb_data_type.lrf_object_type.LRF_OBJ_LINE.getObjectType() : pass--; break;
					case lb_data_type.lrf_object_type.LRF_OBJ_ARC.getObjectType() : pass--; break;
					case lb_data_type.lrf_object_type.LRF_OBJ_GROUP.getObjectType() :
						if(min_group_size <= objects[i].extra_param[0] &&
						max_group_size >= objects[i].extra_param[0])
						{
							pass++;
							group_idx = i;
						}
					break;
					default:
						break;
					}
				}
			}

			if(pass >= 2) {
				//LB_PRINT_VAL("add 1 leg human");
				if(group_idx == -1) {
					lrf_object human = it.next();
					human.type = lb_data_type.lrf_object_type.LRF_OBJ_HUMAN.getObjectType();
					obj_list.add(human);
				} else {
					lrf_object human = objects[group_idx];
					human.type = lb_data_type.lrf_object_type.LRF_OBJ_HUMAN.getObjectType();
					obj_list.add(human);
				}
				human_cnt++;
			}
			leg_object.remove(0);
		}

		return human_cnt;
	}
}
