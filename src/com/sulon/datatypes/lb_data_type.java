package com.sulon.datatypes;

import com.sulon.datatypes.lb_vec2.vec2f;
import com.sulon.datatypes.lb_vec2.vec2i;

public class lb_data_type{
		
	public class bbox2i {
		lb_vec2.vec2i min,max;
	}
	public class bbox2f {
		lb_vec2.vec2f min,max;
	}
	public static enum lrf_range_condition {
	        LRF_COND_NONE		(0x00),
	        LRF_COND_FAR		(0x01),     //too far
	        LRF_COND_MOVE		(0x02),     //moving object
	        LRF_COND_MIXED		(0x04),     //mixed pixel
	        LRF_COND_OCCLUDED	(0x08),     //occluded
	        LRF_COND_EMPTY		(0x10);     //no measurement
	        private int value;
	
	        private lrf_range_condition(int value) {
	                this.value = value;
	        } 
	        
	        public int getLRFCondition(){
		    	return value;
		    }
	}
	
	public static enum lrf_object_type {
		LRF_OBJ_SEGMENT     	(0x00),
	    LRF_OBJ_LINE     	    (0x01),
	    LRF_OBJ_ARC    	        (0x02),
	    LRF_OBJ_CORNER  	    (0x04),
	    LRF_OBJ_GROUP       	(0x08),
	    LRF_OBJ_LEG          	(0x10),
	    LRF_OBJ_LEG2        	(0x20),
	    LRF_OBJ_HUMAN        	(0x40);
	    private int	value;
	
	    private lrf_object_type(int value) {
	            this.value = value;
	    } 
	    
	    public int getObjectType(){
	    	return value;
	    }
		
		
	}
	
	 
	public class lrf_segment {
		float[] ranges;
		lb_vec2.vec2f[] points;
	}
	
	public class lrf_object {
		public lb_vec2.vec2f[] points;
		public int type;
		public lb_vec2.vec2f[] extra_point = new lb_vec2.vec2f[3];
		public double[] extra_param = new double[3];
		public double  segment_id;
	
		public lrf_object() {
			type = lrf_object_type.LRF_OBJ_SEGMENT.getObjectType();
			segment_id = -1;
		}
	}


	

public boolean lrf_object_id_compare(lrf_object i,lrf_object j) {
        return i.segment_id < j.segment_id;
}




}









