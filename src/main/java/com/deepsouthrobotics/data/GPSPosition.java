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
    
    public GPSPosition() {}

    public GPSPosition(double latitude, double longitude)
    {
        this.latitude = latitude;
        this.longitude = longitude;
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

    public double distanceToAnotherGPSPositionViaXY(GPSPosition otherGPS)
    {
        double xComponent = Math.pow(this.x-otherGPS.x, 2);
        double yComponent = Math.pow(this.y-otherGPS.y, 2);
        return Math.sqrt(xComponent+yComponent);
    }

//    @Override
//    public Object clone()
//    {
//        GPSPosition clone = new GPSPosition();
//        clone.latitude = this.latitude;
//        clone.longitude = this.longitude;
//        clone.x = this.x;
//        clone.y = this.y;
//        return clone;
//    }

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
