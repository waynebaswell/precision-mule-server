package com.deepsouthrobotics.util;

import com.deepsouthrobotics.data.GPSPosition;

/**
|--------------------------------------------------------------------------
| Copyright (C) Wayne Baswell 2019 -- GPL version 3 or later
|
| Geography
|--------------------------------------------------------------------------
|
| Credit: Adapted from https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Math/location.cpp
|
| Functions that help measure distances between points on Earth
*/
public class Geo
{
	// scaling factor from 1e-7 degrees to meters at equator
	// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
	public static final float LOCATION_SCALING_FACTOR = 0.011131884502145034f;
	
	// inverse of LOCATION_SCALING_FACTOR
	public static final float LOCATION_SCALING_FACTOR_INV = 89.83204953368922f;
	
	public static final double  M_PI = 3.14159265358979323846;
	public static final double DEG_TO_RAD = (M_PI / 180.0);
	public static final double RAD_TO_DEG   =   (180.0f / M_PI);

	public static void main(String[] args)
	{
		Geo geo = new Geo();

		double latDistance = geo.latLongDistance(30.5633, 0, 30.5634, 0);
		double lonDistance = geo.latLongDistance(0, -87.6780, 0, -87.6781);
		System.out.println("Latitude distance: " + latDistance);
		System.out.println("Longitude distance: " + lonDistance);
	}

	/*
	 * Calculate distance between two points in latitude and longitude taking
	 * into account height difference. If you are not interested in height
	 * difference pass 0.0. Uses Haversine method as its base.
	 * 
	 * lat1, lon1 Start point lat2, lon2 End point el1 Start altitude in meters
	 * el2 End altitude in meters
	 * @returns Distance in Meters
	 * 
	 * http://stackoverflow.com/questions/3694380/calculating-distance-between-two-points-using-latitude-longitude-what-am-i-doi
	 */
	public double latLongDistanceWithHeight(double lat1, double lon1, double lat2,
	        double lon2, double el1, double el2) 
	{

	    final int R = 6371; // Radius of the earth

	    Double latDistance = Math.toRadians(lat2 - lat1);
	    Double lonDistance = Math.toRadians(lon2 - lon1);
	    Double a = Math.sin(latDistance / 2) * Math.sin(latDistance / 2)
	            + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2))
	            * Math.sin(lonDistance / 2) * Math.sin(lonDistance / 2);
	    Double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
	    double distance = R * c * 1000; // convert to meters

	    double height = el1 - el2;

	    distance = Math.pow(distance, 2) + Math.pow(height, 2);

	    return Math.sqrt(distance);
	}
	
	public double latLongDistance(double lat1, double lon1, double lat2, double lon2) 
	{

	    return latLongDistanceWithHeight(lat1, lon1, lat2, lon2, 0, 0);
	}
	
	/*
	 *  extrapolate latitude/longitude given distances north and east in Meters
	 */
	public GPSPosition offset(GPSPosition gpsLoc, double X_ofs_north, double Y_ofs_east)
	{
		//
		double long107x = gpsLoc.longitude*1.0e+7;
		double lat107x = gpsLoc.latitude*1.0e+7;
		GPSPosition loc = new GPSPosition(lat107x, long107x, gpsLoc.fixType);
		
	    if (!(X_ofs_north == 0.0) || !(Y_ofs_east == 0.0)) 
	    {
	        double dlat = X_ofs_north * LOCATION_SCALING_FACTOR_INV;
	        double dlng = (Y_ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(loc);
	        loc.latitude += dlat;
	        loc.longitude += dlng;
	    }
	    loc.latitude = loc.latitude*1.0e-7;
	    loc.longitude = loc.longitude*1.0e-7;
	    return loc;
	}
	
	public double longitude_scale(GPSPosition loc)
	{
	    double scale = Math.cos(loc.getLatitude() * 1.0e-7 * DEG_TO_RAD);
	    return constrain_float(scale, 0.01, 1.0);
	}
	
	public double constrain_float(double amt, double low, double high)
	{
	    if (amt < low) 
	    {
	        return low;
	    }
	    if (amt > high) {
	        return high;
	    }
	    
	    return amt;
	}
	
}