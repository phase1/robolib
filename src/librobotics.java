

public class librobotics {

	//base
	#include "src/lb_common.h" //Don't know what this is!
	#include "src/lb_simple_gui.h" //Don't need gui
	#include "src/lb_cimg_draw.h" //Don't need gui
	#include "src/lb_macro_function.h" /** FINSIHED */
	#include "src/lb_misc_function.h" /** FINSIHED */
	#include "src/lb_regression.h" /**FINISHED */
	#include "src/lb_tools.h" /** FINSIHED */
	#include "src/lb_log_file.h" /* NOT STARTED */
	#include "src/lb_statistic_function.h" /* STARTED */
	#include "src/lb_data_type.h" /** FINISHED */
	//#include "src/lb_lrf_psm.h"
	#include "src/lb_math_model.h" /* STARTED */
	#include "src/lb_particle_function.h" /* NOT STARTED */
	#include "src/lb_math_area_check.h" /* NOT STARTED */

	//low level control function
	#include "src/lb_simple_control.h" /* NOT STARTED */

	//sensor function
	#include "src/lb_lrf_basic.h" /* NOT STARTED */
	#include "src/lb_lrf_object_detect.h" /* STARTED */

	//object tracker
	#include "src/lb_kalman_tracker2.h" /* NOT STARTED */

	//path planning
	#include "src/lb_vfh.h" /* NOT STARTED */

	//Localization and/or Mapping
	#include "src/lb_map2_grid.h" /* NOT STARTED */
	#include "src/lb_mcl2.h" /* NOT STARTED */

	/*-----------------------------------------------------------
	 *
	 * Definition of the LibRobotics: Polar Scan Match Algorithm
	 * (http://www.irrc.monash.edu.au/adiosi/downloads.html)
	 *
     -----------------------------------------------------------
	 */


	struct lrf_psm_cfg {
		lrf_psm_cfg() :
			scale(1000.0),         //scale factor from 1 m
			maxError(1*scale),
			searchWndAngle(LB_DEG2RAD(20)),
			lrfMaxRange(8*scale),
			lrfMinRange(0.1*scale),
			minValidPts(50),
			maxIter(20),
			smallCorrCnt(5)

			{ }
		double scale;
		double maxError;
		double searchWndAngle;
		double lrfMaxRange;
		double lrfMinRange;
		int minValidPts;
		int maxIter;
		int smallCorrCnt;

	};

	/**
	 * Polar Scan-match function for Albert Diosi
	 * http://www.irrc.monash.edu.au/adiosi/downloads.html
	 *
	 * @param ref_robot_pose
	 * @param ref_laser_pose
	 * @param ref_scan_ranges
	 * @param refbad
	 * @param refseg
	 * @param act_robot_pose
	 * @param act_laser_pose
	 * @param act_scan_ranges
	 * @param actbad
	 * @param actseg
	 * @param pm_fi
	 * @param pm_co
	 * @param pm_si
	 * @param cfg
	 * @param rel_laser_pose
	 * @param rel_robot_pose
	 * @param force_check
	 * @return
	 */

	template <typename T, typename T2>
	bool lrf_psm(const pose2<T>& ref_robot_pose,
			const pose2<T>& ref_laser_pose,
			const std::vector<T2>& ref_scan_ranges,
			const std::vector<unsigned int>& refbad,
			const std::vector<int>& refseg,
			const pose2<T>& act_robot_pose,
			const pose2<T>& act_laser_pose,
			const std::vector<T2>& act_scan_ranges,
			const std::vector<unsigned int>& actbad,
			const std::vector<int>& actseg,
			const std::vector<T>& pm_fi,      //angle lookup table
			const std::vector<T>& pm_co,      //cosine lookup table
			const std::vector<T>& pm_si,      //sine lookup table
			const lrf_psm_cfg& cfg,
			pose2<T>& rel_laser_pose,           //scan match result
			pose2<T>& rel_robot_pose,
			bool force_check = true)           //scan match result)
	{

		vec2<T> relRbPose = ref_robot_pose.get_vec2_to(act_robot_pose).rotate(-ref_robot_pose.a);
		//vec2<T> relRbPose = ref_robot_pose.vec_to(act_robot_pose).rot(-ref_robot_pose.a);
		T relRbTheta = act_robot_pose.a -ref_robot_pose.a;

		//transformation of actual scan laser scanner coordinates into reference
		//laser scanner coordinates
		//vec2<T> actLrfPose = relRbPose + act_laser_pose.vec().rot(relRbTheta);
		//T actLrfTheta = relRbTheta + act_laser_pose.a;
		vec2<T> actLrfPose = relRbPose + act_laser_pose.get_vec2().rotate(relRbTheta);
		T actLrfTheta = relRbTheta + act_laser_pose.a;

		vec2<T> relLrfPose = actLrfPose - ref_laser_pose.get_vec2();
		//vec2<T> relLrfPose = actLrfPose - ref_laser_pose.vec();
		//T relLrfTheta = norm_a_rad(actLrfTheta - ref_laser_pose.a);

		T relLrfTheta = lb_normalize_angle(actLrfTheta - ref_laser_pose.a);
		std::vector<T2> refranges(ref_scan_ranges);
		std::vector<T2> actranges(act_scan_ranges);

		//some variables
		size_t nPts = refranges.size();
		std::vector<T> r(nPts, 0);
		std::vector<T> fi(nPts, 0);
		std::vector<T> new_r(nPts, 0);
		std::vector<unsigned int> new_bad(nPts, LRF_COND_EMPTY);
		std::vector<int> index(nPts, 0);

		double angleStep = pm_fi[1] - pm_fi[0];
		int small_corr_cnt = 0;
		int iter = -1;
		int n = 0;//, n2 = 0;
		T dx = 0, dy = 0, dth = 0;
		T ax = relLrfPose.x,  ay = relLrfPose.y, ath = relLrfTheta;
		T delta = 0, x = 0, y = 0, xr = 0, yr = 0;
		T abs_err = 0;
		T ri = 0;
		int idx = 0;
		size_t i = 0;
		T C = LB_SQR(0.7 * cfg.scale);

		while((++iter < cfg.maxIter) && (small_corr_cnt < cfg.smallCorrCnt)) {

			if(iter > 10) C = (1 * cfg.scale);

			T corr = fabs(dx)+fabs(dy)+fabs(dth);

			if(corr < (0.001 * cfg.scale)) {
				small_corr_cnt++;
			}
			else
				small_corr_cnt = 0;

			// convert range readings into ref frame
			// this can be speeded up, by connecting it with the interpolation
			for(i = 0; i < nPts; i++)
			{
				delta   = ath + pm_fi[i];
				xr = (act_scan_ranges[i] * cos(delta));
				yr = (act_scan_ranges[i] * sin(delta));
				x       = xr + ax;
				y       = yr + ay;
				r[i]    = sqrt((x*x)+(y*y));
				fi[i]   = atan2(y,x);

				new_r[i]  = 1e6;            //initialize big interpolated r;
				new_bad[i] = LRF_COND_EMPTY;
			}//for i
			//------------------------INTERPOLATION------------------------
			//calculate/interpolate the associations to the ref scan points

			for(i = 1; i < nPts; i++) {
				// i and i-1 has to be in the same segment, both shouldn't be bad
				// and they should be bigger than 0
				if( actseg[i] >= 0 &&                           //is a segment
						actseg[i] == actseg[i-1] &&                 //same segment
						actranges[i] > 0 &&                         //has a measurement
						(actbad[i] == 0) && (actbad[i-1] == 0))     //is a good measurement
				{
					//calculation of the "whole" parts of the angles
					T fi0 = 0, fi1 = 0;
					T r0 = 0, r1 = 0, a0 = 0, a1 = 0;
					bool occluded = false;
					if( fabsf(fi[i]-fi[i-1]) >= M_PI ) ///TODO: replace this hack with proper fix where we don't loose points.
						continue;
					//are the points visible?
					if(fi[i] > fi[i-1]) {
						occluded = false;
						a0  = fi[i-1];
						a1  = fi[i];
						fi0 = fi[i-1];//fi0 is the meas. angle!
						fi1 = fi[i];
						r0  = r[i-1];
						r1  = r[i];
					} else {
						//invisible - still have to calculate to filter out points which
						occluded = true; //are covered up by these!
						//flip the points-> easier to program
						a0  = fi[i];
						a1  = fi[i-1];
						fi0 = fi[i];
						fi1 = fi[i-1];
						r0  = r[i];
						r1  = r[i-1];
					}

					//interpolate for all the measurement bearings between fi0 and fi1			
					while(fi0 <= fi1)//if at least one measurement point difference, then ...
					{
						//linear interpolate by r and theta ratio
						ri = (((r1-r0)/(a1-a0))*(fi0 - a0)) + r0;

						//if fi0 -> falls into the measurement range and ri is shorter
						//than the current range then overwrite it

						idx = (int)ceil(fi0/angleStep);					
						idx += (nPts/2);

						//if(idx<0)cout<<"be4 Orient"<<"idx "<<idx<<" idx "<<fi0/angleStep<<"npt/2 "<<nPts/2<<endl;
						if((idx >= 0 && idx < (int)nPts) && new_r[idx]>ri) {//modified with idx in the vector range by david
							new_r[idx]    = ri; //overwrite the previous reading							
							index[idx]    = i;  //store which reading was used at index fi0							
							new_bad[idx]  &= ~LRF_COND_EMPTY;    //clear the empty flag

							//check if it was occluded
							if(occluded) {
								new_bad[idx] = LRF_COND_OCCLUDED;//set the occluded flag
							} else {
								new_bad[idx] = LRF_COND_NONE;
							}
							//if((idx>=1081)|| (idx<=0))	cout<<"in Inter "<<"iter "<<iter<<" i "<<i<<endl;
							//the new range reading also it has to inherit the other flags

							new_bad[idx] |= actbad[i];
							new_bad[idx] |= actbad[i-1];

						}
						fi0 += angleStep;//check the next measurement angle!

					}//while
				}//if
			}//for

			//---------------ORIENTATION SEARCH-----------------------------------
			//search for angle correction using cross correlation
			if((iter % 2) == 0) {
				T e;
				std::vector<T> err;         // the error rating
				std::vector<T> beta;        // angle for the corresponding error
				int ii = 0;
				int min_i = 0, max_i = 0;
				int wnd = (int)(cfg.searchWndAngle / angleStep);
				T dr;

				for(int di = -wnd ; di <= wnd; di++) {
					n = 0; e = 0;
					min_i = LB_MAX(-di, 0);
					max_i = LB_MIN(nPts, nPts-di);
					for(ii = min_i; ii < max_i; ii++)//searching through the actual points
					{
						if((new_bad[ii] == 0) &&
								(refbad[ii+di] == 0))
						{
							dr = fabs(new_r[ii]-refranges[ii+di]);
							e += dr;
							n++;
						}
					}//for i

					if(n > 0)
						err.push_back(e/n);
					else
						err.push_back(1e6); //very large error
					beta.push_back(di*angleStep);
				}//for di
				//if((i>=1081)|| (i<=0))cout<<"be4 trans"<<"iter "<<iter<<" i "<<i<<endl;
				//now search for the global minimum
				//assumption: monomodal error function!
				T emin = 1e6;
				int imin = 0;
				for(i = 0; i < err.size(); i++) {
					if(err[i] < emin) {
						emin = err[i];
						imin = i;
					}
				}

				if(err[imin] >= 1e6)
				{
					warn("lrf_psm: orientation search failed: %f", err[imin]);
					dx = 10;
					dy = 10;
					if(force_check) {
						warn("lrf_psm: force check");
						continue;
					}
					else
						return false;
				} else {
					dth = beta[imin];
					if(imin >= 1 && (imin < (int)(beta.size()-1)) &&
							err[imin-1] < 1e6 && err[imin+1] < 1e6 ) //is it not on the extreme?
					{//lets try interpolation
						T D = err[imin-1] + err[imin+1] - 2.0*err[imin];
						T d = 1000;
						if((fabs(D) > 0.01) &&
								(err[imin-1] > err[imin]) &&
								(err[imin+1] > err[imin]))
						{
							d = ((err[imin-1] - err[imin+1]) / D) / 2.0;
							//                            warn("lrf_psm: Orientation refinement: %f ", d);
						}
						if(fabs(d) < 1) {
							dth += d * angleStep;
						}
						ath += dth;
					}
				}
				continue;
			}//if
			//if((i>=1081)|| (i<=0)||(iter ==1))cout<<"be4 trans"<<"iter "<<iter<<" i "<<i<<endl;
			//-----------------translation-------------
			// do the weighted linear regression on the linearized ...
			// include angle as well
			T hi1, hi2, hwi1, hwi2, hw1 = 0, hw2 = 0, hwh11 = 0;
			T hwh12 = 0, hwh21 = 0, hwh22 = 0, w;
			T dr;
			abs_err = 0;
			n = 0;
			for (i = 0; i < nPts; i++) {
				dr = refranges[i] - new_r[i];

				//weight calculation
				if (refbad[i] == 0 &&
						new_bad[i] == 0 &&
						refranges[i] > 0 &&
						new_r[i] > 0 &&
						new_r[i] < cfg.lrfMaxRange &&
						new_r[i] > cfg.lrfMinRange &&
						fabs(dr) < cfg.maxError )
				{
					n++;
					abs_err += fabs(dr);
					w = C / (dr * dr + C);

					//proper calculations of the jacobian
					hi1 = pm_co[i];
					hi2 = pm_si[i];

					hwi1 = hi1 * w;
					hwi2 = hi2 * w;

					//par = (H^t*W*H)^-1*H^t*W*dr
					hw1 += hwi1 * dr;//H^t*W*dr
					hw2 += hwi2 * dr;

					//H^t*W*H
					hwh11 += hwi1 * hi1;
					hwh12 += hwi1 * hi2;
					//hwh21 += hwi2*hi1; //should take adv. of symmetricity!!
					hwh22 += hwi2 * hi2;
				}
			}//for i

			if(n < cfg.minValidPts) {
				warn("lrf_psm: Not enough points for linearize: %d", n);
				dx = 10;
				dy = 10;
				if(force_check){
					warn("Polar Match: force check");
					continue;
				}
				else
					return false;
			}

			//calculation of inverse
			T D;//determinant
			T inv11,inv21,inv12,inv22;//inverse matrix
			D = (hwh11*hwh22) - (hwh12*hwh21);
			if(D < 0.001)
			{
				warn("lrf_psm: Determinant too small: %f", D);
				dy = 10;
				dy = 10;
				if(force_check){
					warn("lrf_psm: force check");
					continue;
				}
				else
					return false;
			}

			inv11 =  hwh22/D;
			inv12 = -hwh12/D;
			inv21 = -hwh12/D;
			inv22 =  hwh11/D;

			dx = inv11*hw1+inv12*hw2;
			dy = inv21*hw1+inv22*hw2;

			ax += dx;
			ay += dy;
		}//while

		rel_laser_pose.x = ax;
		rel_laser_pose.y = ay;
		rel_laser_pose.a = ath;
		LB_PRINT_VAR(rel_laser_pose);

		//warn("!!!lrf_psm: rel_robot_pose still not compute!!!");

		return true;
	}
}
