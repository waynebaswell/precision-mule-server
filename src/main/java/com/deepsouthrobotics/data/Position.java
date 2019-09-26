package com.deepsouthrobotics.data;

/**
|--------------------------------------------------------------------------
| Copyright (C) Wayne Baswell 2019 -- GPL version 3 or later
|
| Wrapper class holding both GPS and Cartesian position
|--------------------------------------------------------------------------
|
| Container for GPS and Cartesian position data
|
*/
public class Position
{
    public PositionType type;
	public Integer recordIndex;
    public GPSPosition gps;
    public CartesianPosition cartesian;
    
    public Position()
    {
    	
    }
    
    public Position(PositionType type, GPSPosition gps, CartesianPosition cartesian)
    {
        this.type = type;
        this.gps = gps;
        this.cartesian = cartesian;
    }
    
    public Position(PositionType type, Integer recordIndex, GPSPosition gps, CartesianPosition cartesian)
    {
        this.type = type;
        this.recordIndex = recordIndex;
        this.gps = gps;
        this.cartesian = cartesian;
    }
    
    public Boolean isGPS()
    {
    	return PositionType.GPS == this.getType();
    }
    
    public Boolean isCartesian()
    {
    	return PositionType.Cartesian == this.getType();
    }

    public static Position buildGpsPosition(GPSPosition gps)
    {
        return new Position(PositionType.GPS, gps, null);
    }

    public static Position buildCartesianPosition(CartesianPosition cartesian)
    {
        return new Position(PositionType.Cartesian, null, cartesian);
    }

    public PositionType getType() {
        return type;
    }

    public void setType(PositionType type) {
        this.type = type;
    }

    public GPSPosition getGps() {
        return gps;
    }

    public void setGps(GPSPosition gps) {
        this.gps = gps;
    }

    public CartesianPosition getCartesian() {
        return cartesian;
    }

    public void setCartesian(CartesianPosition cartesian) {
        this.cartesian = cartesian;
    }
    
    /**
     * 
     * @return Order received while recording mission plan
     */
    public Integer getRecordIndex() {
		return recordIndex;
	}

	public void setRecordIndex(Integer recordIndex) {
		this.recordIndex = recordIndex;
	}
	

}
