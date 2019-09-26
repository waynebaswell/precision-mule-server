package com.deepsouthrobotics.data;

import com.deepsouthrobotics.util.Geo;

/**
|--------------------------------------------------------------------------
| Copyright (C) Wayne Baswell 2019 -- GPL version 3 or later
|
| GPS Cartesian Coordinate Space
|--------------------------------------------------------------------------
|
| Try to simplify the process of mapping back and forth between
| GPS (WSG-84 (lat,long)) and cartesian (x,y) coordinates
*/
public class GPSCartesianCoordinateSpace 
{
	public GPSPosition center;
	
	/*
	 * Pass 1 position's representation as GPS and Cartesian to the constructor -- from this we'll get GPS position's Lat/Long center
	 * and we'll be able to convert future cartesian X Y offsets to GPSPosition
	 */
	public GPSCartesianCoordinateSpace(GPSPosition gpsPosition)
	{
		Double distanceFromZeroX = gpsPosition.x;
		Double distanceFromZeroY = gpsPosition.y;
		
		Geo geo = new Geo();
		center = geo.offset(gpsPosition, -distanceFromZeroX, -distanceFromZeroY);
	}
	
	public GPSPosition gpsPositionGivenDistanceFromZeroZero(double x, double y)
	{
		Geo geo = new Geo();
		GPSPosition g = geo.offset(center, x, y);
		g.x = x;
		g.y = y;
		return g;
	}
}
