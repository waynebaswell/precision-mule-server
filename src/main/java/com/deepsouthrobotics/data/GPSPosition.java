package com.deepsouthrobotics.data;

import java.awt.geom.Point2D;

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
public class GPSPosition extends Point2D.Double implements Cloneable, Comparable
{
    public double latitude;
    public double longitude;

    public Integer fixType;
    public Long autopilotTimeRecordedMillis;
    public Long localTimeReceivedMillis;

    public static final String SERIALIZE_GPS_LAT = "LAT";
    public static final String SERIALIZE_GPS_LON = "LON";
    public static final String SERIALIZE_GPS_FIX = "FIX_TYPE";
    
    public GPSPosition() {}

    public GPSPosition(double latitude, double longitude)
    {
        this.latitude = latitude;
        this.longitude = longitude;
    }

    public GPSPosition(double latitude, double longitude, Integer fixType)
    {
        this.latitude = latitude;
        this.longitude = longitude;
        this.fixType = fixType;
    }

    public GPSPosition(double latitude, double longitude, double x, double y)
    {
        this.latitude = latitude;
        this.longitude = longitude;

        super.x = x;
        super.y = y;

    }

    public double getLatitude() {
        return latitude;
    }

    public void setLatitude(double latitude) {
        this.latitude = latitude;
    }

    public double getLongitude() {
        return longitude;
    }

    public void setLongitude(double longitude) {
        this.longitude = longitude;
    }

//    public Integer getFixType() {
//        return fixType;
//    }
//
//    public void setFixType(Integer fixType) {
//        this.fixType = fixType;
//    }
//
//    public Long getAutopilotTimeRecordedMillis() {
//        return autopilotTimeRecordedMillis;
//    }
//
//    public void setAutopilotTimeRecordedMillis(Long autopilotTimeRecordedMillis) {
//        this.autopilotTimeRecordedMillis = autopilotTimeRecordedMillis;
//    }
//
//    public Long getLocalTimeReceivedMillis() {
//        return localTimeReceivedMillis;
//    }
//
//    public void setLocalTimeReceivedMillis(Long localTimeReceivedMillis) {
//        this.localTimeReceivedMillis = localTimeReceivedMillis;
//    }
    
//    public static GPSPosition buildFromKeyValArray(String[] keyVals)
//    {
//    	GPSPosition gps = new GPSPosition();
//
//    	for(String keyValString : keyVals)
//    	{
//    		String[] keyValPair = keyValString.split(Constants.SERIALIZE_KEY_VAL_SEPARATOR);
//    		if(SERIALIZE_GPS_LAT.equalsIgnoreCase(keyValPair[0]))
//    		{
//    			Double lat = Double.parseDouble(keyValPair[1]);
//    			gps.setLatitude(lat);
//    		}
//    		else if(SERIALIZE_GPS_LON.equalsIgnoreCase(keyValPair[0]))
//    		{
//    			Double lon = Double.parseDouble(keyValPair[1]);
//    			gps.setLongitude(lon);
//    		}
//    		else if(SERIALIZE_GPS_FIX.equalsIgnoreCase(keyValPair[0]))
//    		{
//    			Integer fixType = Integer.parseInt(keyValPair[1]);
//    			gps.setFixType(fixType);
//    		}
//    	}
//
//    	return gps;
//    }
    
//    public String toKeyValString()
//    {
//    	String keyValString =
//    			SERIALIZE_GPS_LAT + Constants.SERIALIZE_KEY_VAL_SEPARATOR + this.getLatitude() +
//    			Constants.SERIALIZE_LINE_VAL_SEPARATOR + SERIALIZE_GPS_LON + Constants.SERIALIZE_KEY_VAL_SEPARATOR + this.getLongitude() +
//    			Constants.SERIALIZE_LINE_VAL_SEPARATOR + SERIALIZE_GPS_FIX + Constants.SERIALIZE_KEY_VAL_SEPARATOR + this.getFixType();
//    	return keyValString;
//    }

//    public LatLng toLatLng()
//    {
//        return new LatLng(this.latitude, this.longitude);
//    }
//
    
//	/*
//	 * Expected line format:
//	 *
//	 * POINT 201 GPS LAT:30.5636255 LON:-87.6780056 FIX_TYPE:3
//	 *
//	 */
//    public String buildPointString(Integer recordIndex)
//    {
//		return Constants.SERIALIZE_POINT_TYPE +
//				Constants.SERIALIZE_LINE_VAL_SEPARATOR + recordIndex.toString() +
//				Constants.SERIALIZE_LINE_VAL_SEPARATOR + Constants.SERIALIZE_GPS_TYPE +
//				Constants.SERIALIZE_LINE_VAL_SEPARATOR + this.toKeyValString();
//    }
//
//	/*
//	 * Expected line format:
//	 *
//	 * GUIDEPOINT_GPS GPS LAT:30.5635939 LON:-87.6780064 FIX_TYPE:3
//     *
//	 */
//    public String buildGuidePointString()
//    {
//		return Constants.SERIALIZE_GUIDEPOINT_GPS_TYPE +
//				Constants.SERIALIZE_LINE_VAL_SEPARATOR + Constants.SERIALIZE_GPS_TYPE +
//				Constants.SERIALIZE_LINE_VAL_SEPARATOR + this.toKeyValString();
//    }

    public double distanceToAnotherGPSPositionViaXY(GPSPosition otherGPS)
    {
        double xComponent = Math.pow(this.x-otherGPS.x, 2);
        double yComponent = Math.pow(this.y-otherGPS.y, 2);
        return Math.sqrt(xComponent+yComponent);
    }

    @Override
    public int compareTo(Object o)
    {
        if(o instanceof GPSPosition)
        {
            GPSPosition gps2 = (GPSPosition)o;
            if(this.latitude == gps2.latitude)
            {
                if(this.longitude == gps2.longitude)
                {
                    return 0;
                }
                else
                {
                    return java.lang.Double.compare(this.longitude, gps2.longitude);
                }
            }
            else
            {
                return java.lang.Double.compare(this.latitude, gps2.latitude);
            }
        }
        else
        {
            return -1;
        }
    }
}
