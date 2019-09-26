package com.deepsouthrobotics.data;

/**
|--------------------------------------------------------------------------
| Copyright (C) Wayne Baswell 2019 -- GPL version 3 or later
|
| Cartesian Position
|--------------------------------------------------------------------------
|
| Represents an (x,y) position on a Cartesian plane. When we do the math to
| build the mission, it's often easier to work in terms of (x,y) on a
| Cartesian plane (rather than the wonderful but unintuitive
| WSG-84 (longitude,latitude) system).
|
| If you're approaching this code without much context as to what problem
| we're solving, please keep this in mind: this code is supporting
| building centimeter-level missions for land-based robots. So the
| length/breadth of the missions is usually well under 1 kilometer
| and we can pretend that the distance between WSG-84 (lat,long)
| coordinate pairs is consistent for our entire mission -- in
| other words, WSG-84 distances between coordinates are not
| linearly consistent (i.e. the distance measured in meters
| between two WSG-84 (lat,long) coordinates near the north
| pole is much different than the distance between two
| WSG-84 coordinates with the same delta between the
| coordinates measured at the Equator).
*/
public class CartesianPosition extends Position
{
    public Double x;
    public Double y;
    public Long autopilotTimeRecordedMillis;
    public Long localTimeReceivedMillis;
    
    public static final String SERIALIZE_CART_X = "X"; //+North
    public static final String SERIALIZE_CART_Y = "Y"; //+East
    public static final String SERIALIZE_CART_AUTOPILOT_TIME_MILLIS = "AUTOPILOT_TIME_MILLIS";
    public static final String SERIALIZE_CART_LOCAL_TIME_MILLIS = "LOCAL_TIME_MILLIS";
    
    public CartesianPosition() {
		super();
	}
    
	public CartesianPosition(Double x, Double y) {
		super();
		this.x = x;
		this.y = y;
	}
    
	public CartesianPosition(Double x, Double y, Long autopilotTimeRecordedMillis, Long localTimeReceivedMillis) {
		super();
		this.x = x;
		this.y = y;
		this.autopilotTimeRecordedMillis = autopilotTimeRecordedMillis;
		this.localTimeReceivedMillis = localTimeReceivedMillis;
	}

	public Double getX() {
        return x;
    }

    public void setX(Double x) {
        this.x = x;
    }

    public Double getY() {
        return y;
    }

    public void setY(Double y) {
        this.y = y;
    }

    public Long getAutopilotTimeRecordedMillis() {
        return autopilotTimeRecordedMillis;
    }

    public void setAutopilotTimeRecordedMillis(Long autopilotTimeRecordedMillis) {
        this.autopilotTimeRecordedMillis = autopilotTimeRecordedMillis;
    }

    public Long getLocalTimeReceivedMillis() {
        return localTimeReceivedMillis;
    }

    public void setLocalTimeReceivedMillis(Long localTimeReceivedMillis) {
        this.localTimeReceivedMillis = localTimeReceivedMillis;
    }
    
    public static CartesianPosition buildFromKeyValArray(String[] keyVals)
    {
    	CartesianPosition cp = new CartesianPosition();
    	
    	for(String keyValString : keyVals)
    	{
    		String[] keyValPair = keyValString.split(Constants.SERIALIZE_KEY_VAL_SEPARATOR);
    		if(SERIALIZE_CART_X.equalsIgnoreCase(keyValPair[0]))
    		{
    			Double x = Double.parseDouble(keyValPair[1]);
    			cp.setX(x);
    		}
    		else if(SERIALIZE_CART_Y.equalsIgnoreCase(keyValPair[0]))
    		{
    			Double y = Double.parseDouble(keyValPair[1]);
    			cp.setY(y);
    		}
    		else if(SERIALIZE_CART_AUTOPILOT_TIME_MILLIS.equalsIgnoreCase(keyValPair[0]))
    		{
    			Long autopilotTimeRecordedMillis = Long.parseLong(keyValPair[1]);
    			cp.setAutopilotTimeRecordedMillis(autopilotTimeRecordedMillis);
    		}
    		else if(SERIALIZE_CART_LOCAL_TIME_MILLIS.equalsIgnoreCase(keyValPair[0]))
    		{
    			Long localTimeReceivedMillis = Long.parseLong(keyValPair[1]);
    			cp.setLocalTimeReceivedMillis(localTimeReceivedMillis);
    		}
    	}
    	
    	return cp;
    }
    
    public String toKeyValString()
    {
    	String keyValString = 
    			SERIALIZE_CART_X + Constants.SERIALIZE_KEY_VAL_SEPARATOR + this.getX() +
    			Constants.SERIALIZE_LINE_VAL_SEPARATOR + SERIALIZE_CART_Y + Constants.SERIALIZE_KEY_VAL_SEPARATOR + this.getY() +
    			Constants.SERIALIZE_LINE_VAL_SEPARATOR + SERIALIZE_CART_AUTOPILOT_TIME_MILLIS + Constants.SERIALIZE_KEY_VAL_SEPARATOR + this.getAutopilotTimeRecordedMillis() +
    			Constants.SERIALIZE_LINE_VAL_SEPARATOR + SERIALIZE_CART_LOCAL_TIME_MILLIS + Constants.SERIALIZE_KEY_VAL_SEPARATOR + this.getLocalTimeReceivedMillis();
    	return keyValString;
    }

    /*
     * Expected line is of the form:
     * 
     * POINT 32 CARTESIAN X:-2.72 Y:0.57 AUTOPILOT_TIME_MILLIS:131559 LOCAL_TIME_MILLIS:1487620496519
     * POINT 1007 CARTESIAN X:-4.19 Y:-22.26 AUTOPILOT_TIME_MILLIS:166281 LOCAL_TIME_MILLIS:1487620531231
     * 
     */
    public String buildPointString(Integer recordIndex)
    {
		return Constants.SERIALIZE_POINT_TYPE +
				Constants.SERIALIZE_LINE_VAL_SEPARATOR + recordIndex.toString() +
				Constants.SERIALIZE_LINE_VAL_SEPARATOR + Constants.SERIALIZE_CARTESIAN_TYPE +
				Constants.SERIALIZE_LINE_VAL_SEPARATOR + this.toKeyValString();

    }
    
	/*
	 * Expected line format:
	 * 
	 * GUIDEPOINT_CARTESIAN CARTESIAN X:-4.57 Y:-1.68 AUTOPILOT_TIME_MILLIS:126446 LOCAL_TIME_MILLIS:1487617141467
	 * 
	 */
    public String buildGuidePointString()
    {
		return Constants.SERIALIZE_GUIDEPOINT_CARTESIAN_TYPE +
				Constants.SERIALIZE_LINE_VAL_SEPARATOR + Constants.SERIALIZE_CARTESIAN_TYPE +
				Constants.SERIALIZE_LINE_VAL_SEPARATOR + this.toKeyValString();
    }
}
