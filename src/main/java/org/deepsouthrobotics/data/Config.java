package org.deepsouthrobotics.data;

/**
|--------------------------------------------------------------------------
| Copyright (C) Wayne Baswell 2019 -- GPL version 3 or later
|
| Configuration
|--------------------------------------------------------------------------
|
| Various configuration values that we use in the application
*/
public class Config 
{
    // when building a mission, line distance threshold at which we end mission
    public static final Double minMowingLineDistanceMeters = 2.0;

    // when building a mission, adjust flexible point by this much until boundary reached
    // basically, this is a hack and it exists until I make the algorithm less dumb
    public static final Double boundaryPushAdjustmentMeters = 0.01;
    
    public static final int MISSION_BUILDING_SERVICE_TIMEOUT = 7200; //number of seconds to wait for mission building service to create mission from recording
}
