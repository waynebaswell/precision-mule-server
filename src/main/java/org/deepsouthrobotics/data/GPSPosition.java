package org.deepsouthrobotics.data;

/**
|--------------------------------------------------------------------------
| Copyright (C) Wayne Baswell 2019 -- GPL version 3 or later
|
| GPS Coordinate Space
|--------------------------------------------------------------------------
|
| Holds GPS (more generally, GNSS) position data
|
*/
public class GPSPosition implements Cloneable
{
    public Double latitude;
    public Double longitude;
    public Integer fixType;
    public Long autopilotTimeRecordedMillis;
    public Long localTimeReceivedMillis;

    public static final String SERIALIZE_GPS_LAT = "LAT";
    public static final String SERIALIZE_GPS_LON = "LON";
    public static final String SERIALIZE_GPS_FIX = "FIX_TYPE";
    
    public GPSPosition() {
    }
   

    public GPSPosition(Double latitude, Double longitude, Integer fixType) {
        this.latitude = latitude;
        this.longitude = longitude;
        this.fixType = fixType;
    }

    public Double getLatitude() {
        return latitude;
    }

    public void setLatitude(Double latitude) {
        this.latitude = latitude;
    }

    public Double getLongitude() {
        return longitude;
    }

    public void setLongitude(Double longitude) {
        this.longitude = longitude;
    }

    public Integer getFixType() {
        return fixType;
    }

    public void setFixType(Integer fixType) {
        this.fixType = fixType;
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
    
    public static GPSPosition buildFromKeyValArray(String[] keyVals)
    {
    	GPSPosition gps = new GPSPosition();
    	
    	for(String keyValString : keyVals)
    	{
    		String[] keyValPair = keyValString.split(Constants.SERIALIZE_KEY_VAL_SEPARATOR);
    		if(SERIALIZE_GPS_LAT.equalsIgnoreCase(keyValPair[0]))
    		{
    			Double lat = Double.parseDouble(keyValPair[1]);
    			gps.setLatitude(lat);
    		}
    		else if(SERIALIZE_GPS_LON.equalsIgnoreCase(keyValPair[0]))
    		{
    			Double lon = Double.parseDouble(keyValPair[1]);
    			gps.setLongitude(lon);
    		}
    		else if(SERIALIZE_GPS_FIX.equalsIgnoreCase(keyValPair[0]))
    		{
    			Integer fixType = Integer.parseInt(keyValPair[1]);
    			gps.setFixType(fixType);
    		}
    	}
    	
    	return gps;
    }
    
    public String toKeyValString()
    {
    	String keyValString = 
    			SERIALIZE_GPS_LAT + Constants.SERIALIZE_KEY_VAL_SEPARATOR + this.getLatitude() +
    			Constants.SERIALIZE_LINE_VAL_SEPARATOR + SERIALIZE_GPS_LON + Constants.SERIALIZE_KEY_VAL_SEPARATOR + this.getLongitude() +
    			Constants.SERIALIZE_LINE_VAL_SEPARATOR + SERIALIZE_GPS_FIX + Constants.SERIALIZE_KEY_VAL_SEPARATOR + this.getFixType();
    	return keyValString;
    }
    
    
	/*
	 * Expected line format:
	 * 
	 * POINT 201 GPS LAT:30.5636255 LON:-87.6780056 FIX_TYPE:3
	 * 
	 */
    public String buildPointString(Integer recordIndex)
    {
		return Constants.SERIALIZE_POINT_TYPE +
				Constants.SERIALIZE_LINE_VAL_SEPARATOR + recordIndex.toString() +
				Constants.SERIALIZE_LINE_VAL_SEPARATOR + Constants.SERIALIZE_GPS_TYPE +
				Constants.SERIALIZE_LINE_VAL_SEPARATOR + this.toKeyValString();
    }
    
	/*
	 * Expected line format:
	 * 
	 * GUIDEPOINT_GPS GPS LAT:30.5635939 LON:-87.6780064 FIX_TYPE:3
     *
	 */
    public String buildGuidePointString()
    {
		return Constants.SERIALIZE_GUIDEPOINT_GPS_TYPE +
				Constants.SERIALIZE_LINE_VAL_SEPARATOR + Constants.SERIALIZE_GPS_TYPE +
				Constants.SERIALIZE_LINE_VAL_SEPARATOR + this.toKeyValString();
    }
    
}
