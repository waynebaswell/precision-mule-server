package org.deepsouthrobotics.brain;

import org.deepsouthrobotics.data.*;
import org.deepsouthrobotics.util.Geo;
import org.json.JSONArray;
import org.json.JSONObject;

import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
|--------------------------------------------------------------------------
| Copyright (C) Wayne Baswell 2019 -- GPL version 3 or later
|
| Mission Brain (i.e. the Orchestrator for the entire app)
|--------------------------------------------------------------------------
|
| Builds the series of waypoints inside a boundary that's represented as a
| series of (lat,lng) coordinates.
*/
public class MissionBrain
{
    public List<Position> positions;
    public CartesianPosition guidePointCartesian;
    public GPSPosition guidePointGPS;
    
    public MissionBrain()
    {
    	positions = new ArrayList<>();
    }

	/**
	  Builds mission waypoints from InputStream
	 */
    public List<Position> buildMissionWaypointsFromInputStream(InputStream missionPlanRecordingInputStream, Double mowingPathWidthInMeters)
    {
    	positions = new ArrayList<>();
    	this.loadPositionsFromMissionPlanRecordingFromInputStream(missionPlanRecordingInputStream);

        scaleMinXAndMinYToZero();
        Path2D missionBoundary = missionBoundary();
        		
    	Point2D.Double start = new Point2D.Double(this.getStartX(), this.getStartY());
    	Point2D.Double guide = new Point2D.Double(this.getGuidePointCartesian().x, this.getGuidePointCartesian().y);
    	
    	GPSCartesianCoordinateSpace space = new GPSCartesianCoordinateSpace(this.getGuidePointGPS(), this.getGuidePointCartesian());
    	
    	List<Position> missionWaypoints = new ArrayList<Position>();
    	//List<CartesianPosition> missionWaypointsCartesian = new ArrayList<CartesianPosition>(); //Just for debugging
    	
    	Position startPosition = new Position(PositionType.GPS, -1, this.getStartGPS(), this.getStartCartesian());
    	Position guidePosition = new Position(PositionType.GPS, -1, this.getGuidePointGPS(), this.getGuidePointCartesian());
    	
    	//Calculate which way to commence building the mission after hitting the guidepoint
    	int navigateDirection = directionToNavigateAfterGuideLine(start, guide, missionBoundary);
    	
    	//We use the perpendicular norm for finding next turn points
    	Point2D.Double normPerpXY = getPerpendicularNorm(start, guide, mowingPathWidthInMeters);
    	normPerpXY.x *= navigateDirection;
    	normPerpXY.y *= navigateDirection;
    	
    	//We use the parallel norm for pushing/pulling the mission line
    	Point2D.Double normParallel = getParallelNorm(start, guide, Config.boundaryPushAdjustmentMeters);
    	normParallel.x *= -1;//navigateDirection;
    	normParallel.y *= -1;//navigateDirection;
    	
    	//Convenient way to push/pull the mission line opposite direction of above
    	Point2D.Double normParallelNegative = new Point2D.Double();
    	normParallelNegative.x  = normParallel.x * -1;
    	normParallelNegative.y  = normParallel.y * -1;
    	
		Point2D.Double currentTopPoint = new Point2D.Double(guide.x, guide.y);
		Point2D.Double currentBottomPoint = new Point2D.Double(start.x, start.y);
		
		Point2D.Double lastTopPoint = new Point2D.Double(currentTopPoint.x, currentTopPoint.y);
		Point2D.Double lastBottomPoint = new Point2D.Double(currentBottomPoint.x, currentBottomPoint.y);
		
		GPSPosition lastTopGPS = this.getGuidePointGPS();
		GPSPosition lastBottomGPS = this.getStartGPS();
		
		Geo geo = new Geo();
		
    	//The first mission point will always be the start position
    	missionWaypoints.add(startPosition);
    	
    	//The second mission point will always be the guide point adjusted to ensure next turn is within boundary
		Boolean ADD_ANOTHER_PATH_LINE = adjustTurnInitiationPointSoThatNextOrthogonalPointIsWithinBoundary(currentBottomPoint, currentTopPoint, normPerpXY, missionBoundary);
		
		GPSPosition adjustedGuideGPS = space.gpsPositionGivenDistanceFromZeroZero(currentTopPoint.x, currentTopPoint.y);
		CartesianPosition adjustedGuideCart = new CartesianPosition(currentTopPoint.x, currentTopPoint.y);
		Position adjustedGuidePosition = new Position(PositionType.GPS, -1, adjustedGuideGPS, adjustedGuideCart);
		missionWaypoints.add(adjustedGuidePosition);
		
		for(int i = 1; ADD_ANOTHER_PATH_LINE; i++)
		{
			if(i%2 != 0) //we are at the top -- so set the top point to the perpendicular offset and push the bottom point to missionBoundary
			{
				currentTopPoint.x += normPerpXY.x;
				currentTopPoint.y += normPerpXY.y;
				
				currentBottomPoint.x = currentTopPoint.x;
				currentBottomPoint.y = currentTopPoint.y;

				pushLineToBoundary(currentTopPoint, currentBottomPoint, normParallel, missionBoundary);
				ADD_ANOTHER_PATH_LINE = adjustTurnInitiationPointSoThatNextOrthogonalPointIsWithinBoundary(currentTopPoint, currentBottomPoint, normPerpXY, missionBoundary);
				
				double topOffsetX = currentTopPoint.x - lastTopPoint.x;
				double topOffsetY = currentTopPoint.y - lastTopPoint.y;
				
				double bottomOffsetX = currentBottomPoint.x - currentTopPoint.x;
				double bottomOffsetY = currentBottomPoint.y - currentTopPoint.y;
				
				if(ADD_ANOTHER_PATH_LINE)
				{
					GPSPosition newTopGPS = space.gpsPositionGivenDistanceFromZeroZero(currentTopPoint.x, currentTopPoint.y);
					//GPSPosition newTopGPS = geo.offset(lastTopGPS, topOffsetX, topOffsetY);
					CartesianPosition cart = new CartesianPosition(currentTopPoint.x, currentTopPoint.y);
					missionWaypoints.add(new Position(PositionType.GPS, -1, newTopGPS, cart));
					System.out.println(i+"Top "+ newTopGPS.latitude + " " + newTopGPS.longitude + " Distance: " + geo.latLongDistance(lastTopGPS.latitude, lastTopGPS.longitude, newTopGPS.latitude, newTopGPS.longitude));
					//System.out.println(i+"A Top "+ gps.latitude + " " + gps.longitude + " Distance: " + geo.latLongDistance(lastTopGPS.latitude, lastTopGPS.longitude, gps.latitude, gps.longitude));
					lastTopGPS = newTopGPS;
					
					if(Math.abs(currentTopPoint.distance(currentBottomPoint)) >= Config.minMowingLineDistanceMeters)
					{
						GPSPosition newBottomGPS = space.gpsPositionGivenDistanceFromZeroZero(currentBottomPoint.x, currentBottomPoint.y);
						//GPSPosition newBottomGPS = geo.offset(newTopGPS, bottomOffsetX, bottomOffsetY);
						cart = new CartesianPosition(currentBottomPoint.x, currentBottomPoint.y);
						missionWaypoints.add(new Position(PositionType.GPS, -1, newBottomGPS, cart));
						lastBottomGPS = newBottomGPS;
					}
					else
					{
						ADD_ANOTHER_PATH_LINE = false;
					}
				}
					
				lastTopPoint.x = currentTopPoint.x;
				lastTopPoint.y = currentTopPoint.y;
				
				lastBottomPoint.x = currentBottomPoint.x;
				lastBottomPoint.y = currentBottomPoint.y;
				
			}
			else ////we are at the bottom -- so set the bottom point to the perpendicular offset  and push the top point to missionBoundary
			{
				currentBottomPoint.x += normPerpXY.x;
				currentBottomPoint.y += normPerpXY.y;
				
				currentTopPoint.x = currentBottomPoint.x;
				currentTopPoint.y = currentBottomPoint.y;
				
				pushLineToBoundary(currentBottomPoint, currentTopPoint, normParallelNegative, missionBoundary);
				ADD_ANOTHER_PATH_LINE = adjustTurnInitiationPointSoThatNextOrthogonalPointIsWithinBoundary(currentBottomPoint, currentTopPoint, normPerpXY, missionBoundary);
				
				double topOffsetX = currentTopPoint.x - currentBottomPoint.x;
				double topOffsetY = currentTopPoint.y - currentBottomPoint.y;
				
				double bottomOffsetX = currentBottomPoint.x - lastBottomPoint.x;
				double bottomOffsetY = currentBottomPoint.y - lastBottomPoint.y;
				
				if(ADD_ANOTHER_PATH_LINE)
				{
					GPSPosition newBottomGPS = space.gpsPositionGivenDistanceFromZeroZero(currentBottomPoint.x, currentBottomPoint.y);
					//GPSPosition newBottomGPS = geo.offset(lastBottomGPS, bottomOffsetX, bottomOffsetY);
					CartesianPosition cart = new CartesianPosition(currentBottomPoint.x, currentBottomPoint.y);
					missionWaypoints.add(new Position(PositionType.GPS, -1, newBottomGPS, cart));
					System.out.println(i+"Bottom lat:"+ newBottomGPS.latitude + " lon:" + newBottomGPS.longitude + " x: " + cart.x + " y:" + cart.y +" Distance: " + geo.latLongDistance(lastBottomGPS.latitude, lastBottomGPS.longitude, newBottomGPS.latitude, newBottomGPS.longitude));
					//System.out.println(i+"B Bottom "+ gps.latitude + " " + gps.longitude + " Distance: " + geo.latLongDistance(lastBottomGPS.latitude, lastBottomGPS.longitude, gps.latitude, gps.longitude));
					lastBottomGPS = newBottomGPS;
					
					if(Math.abs(currentTopPoint.distance(currentBottomPoint)) >= Config.minMowingLineDistanceMeters)
					{
						GPSPosition newTopGPS = space.gpsPositionGivenDistanceFromZeroZero(currentTopPoint.x, currentTopPoint.y);
						//GPSPosition newTopGPS = geo.offset(newBottomGPS, topOffsetX, topOffsetY);
						cart = new CartesianPosition(currentTopPoint.x, currentTopPoint.y);
						missionWaypoints.add(new Position(PositionType.GPS, -1, newTopGPS, cart));
						lastTopGPS = newTopGPS;
					}
					else
					{
						ADD_ANOTHER_PATH_LINE = false;
					}
				}
				
				lastTopPoint.x = currentTopPoint.x;
				lastTopPoint.y = currentTopPoint.y;
				
				lastBottomPoint.x = currentBottomPoint.x;
				lastBottomPoint.y = currentBottomPoint.y;
			}

		}
		
		return missionWaypoints;
    }

    public static void main(String[] args)
    {
    	MissionBrain mpr = new MissionBrain();
    	String testWaypointFormatStatic = "11	0	3	16	0	0	0	0	30.563625	-87.678372	100.000000	1";
    	GPSPosition gps = new GPSPosition();
    	gps.setLongitude(-87.678372);
    	gps.setLatitude(30.563625);
    	String testWaypointFormatDynamic = mpr.buildWaypointString(11, gps);
    	System.out.println("Waypoint static string:  " + testWaypointFormatStatic);
    	System.out.println("Waypoint dynamic string: " + testWaypointFormatDynamic);
    	System.out.println("Waypoing string generator functioning correctly: " + testWaypointFormatDynamic.contentEquals(testWaypointFormatStatic));
    }

	/**
	 *
	 * @param posHome Home position (in lat,long terms) for this mission --
	 * this is just a reference point we use when building the mission
	 * -- so another way of explaining what we're doing is this: we
	 * can say that we're going to tream the Home GPS lat,lng
	 * position as being (0,0) on our (x,y) map -- and for
	 * all other GPS positions, we just find the distance
	 * between them and the Home GPS position and the
	 * x,y components of this distance are the x,y
	 * values for the point on our Cartesian
	 * plane. If that doesn't make sense
	 * it's likely that I'm not
	 * explaining a simple
	 * idea very well.
	 * @param pos Some other (lat,lng) position that we'd like to build
	 * an (x, y) position for
	 * @return CartesianPosition whose x and y values indicate the meters
	 * North and East of the home position
	 */
	public CartesianPosition buildCPFromGPSPositionByDiffingLatAndLonDistanceFromHome(GPSPosition posHome, GPSPosition pos)
	{
		Geo geo = new Geo();
		CartesianPosition cp = new CartesianPosition();
		double lonMeters = geo.latLongDistance(posHome.latitude,posHome.longitude,posHome.latitude,pos.longitude);
		double latMeters = geo.latLongDistance(posHome.latitude,posHome.longitude,pos.latitude,posHome.longitude);

		if(posHome.latitude > pos.latitude)
			latMeters*=-1;

		if(posHome.longitude > pos.longitude)
			lonMeters*=-1;

		cp.x = latMeters;
		cp.y = lonMeters;
		return cp;
	}

	/**
	 *
	 * @param latlngArray JSON Array of coordinate tuples that represent the
	 * vertices of the polygon that we're building a mission inside of. Just
	 * so we're on the same page -- this Array must contain at least 3
	 * points to make sense (i.e. 2 points would be a straight line, 1
	 * point would be a point, and 0 points is, well, you know..)
	 * @param mowingPathWidthInMeters Distance between waypoints -- for a
	 * mowing application, for example, this will be slightly less than
	 * the width of the mowing deck to allow a little overlap
	 * @return List of waypoints which define the mission within the
	 * polygon. The list is ordered such that the first point will
	 * be the starting coordinate and the
	 */
	public List<Position> buildMissionWaypointsFromLatLngBoundsJSONArray(JSONArray latlngArray, Double mowingPathWidthInMeters)
	{
		List<GPSPosition> gpsPositionList = new ArrayList<>();

		for (int i = 0; i < latlngArray.length(); i++)
		{
			JSONObject latLng = (JSONObject) latlngArray.get(i);
			Double lat = latLng.getDouble("lat");
			Double lng = latLng.getDouble("lng");
			GPSPosition gpsPosition = new GPSPosition();
			gpsPosition.latitude = lat;
			gpsPosition.longitude = lng;
			gpsPosition.fixType = 3;
			gpsPosition.autopilotTimeRecordedMillis = 1l;
			gpsPosition.localTimeReceivedMillis = 1l;
			gpsPositionList.add(gpsPosition);
		}

		//Now gpsPositionList contains the gps positions
		System.out.println("Now let's do some KARATE on the gps positions...");

		//We'll say that the first item in gpsPositionList is the home position (0,0)
		CartesianPosition cpZero = new CartesianPosition();
		cpZero.x = 0.0;
		cpZero.y = 0.0;
		cpZero.localTimeReceivedMillis = 1l;
		cpZero.autopilotTimeRecordedMillis = 1l;

		List<CartesianPosition> cartesionPositionList = new ArrayList<>();
		cartesionPositionList.add(cpZero);

		for(int x = 1; x < gpsPositionList.size(); x++)
		{
			CartesianPosition cp = buildCPFromGPSPositionByDiffingLatAndLonDistanceFromHome(gpsPositionList.get(0), gpsPositionList.get(x));
			cp.localTimeReceivedMillis = 1l;
			cp.autopilotTimeRecordedMillis = 1l;
			cartesionPositionList.add(cp);
		}
		StringBuffer mprBuffer = new StringBuffer();
		//WE've gotta have at least 3 coordinates in order for
		//any of this logic to make sense (i.e. if you've only got 2 points
		//then you've got a line, which doesn't have area and
		//won't contain waypoints by definition)

		if(gpsPositionList.size() >= 3)
		{
			CartesianPosition guidepointC = cartesionPositionList.get(1);
			GPSPosition guidepointG = gpsPositionList.get(1);

			mprBuffer.append(guidepointC.buildGuidePointString()+"\n"); //Add Cartesian guidepoint
			mprBuffer.append(guidepointG.buildGuidePointString()+"\n"); //Add GPS guidepoint

			for(int x = 0, y=0; x < gpsPositionList.size(); x++)
			{
				mprBuffer.append(gpsPositionList.get(x).buildPointString(y++)+"\n");
				mprBuffer.append(cartesionPositionList.get(x).buildPointString(y++)+"\n");
			}
		}

		InputStream mprInputStream = new ByteArrayInputStream(mprBuffer.toString().getBytes(StandardCharsets.UTF_8));

		List<Position> waypoints = this.buildMissionWaypointsFromInputStream(mprInputStream, mowingPathWidthInMeters);
		System.out.println("Now we've built those beautiful cartesian positions...");
		return waypoints;
	}

	/**
	 *
	 * @param index
	 * @param gps
	 * @return
	 */
    public String buildWaypointString(int index, GPSPosition gps)
    {
    	return index + "\t0\t3\t16\t0\t0\t0\t0\t" + gps.getLatitude() + "\t" + gps.getLongitude() + "\t100.000000\t1";
    }
    
    private void loadPositionsFromMissionPlanRecordingFromInputStream(InputStream inputStreamMissionPlanRecordingLines)
    {
    	try
    	{
        	// Open the file
        	BufferedReader br = new BufferedReader(new InputStreamReader(inputStreamMissionPlanRecordingLines));
        		
        	String line;
        	
        	//Read file line by line
        	while ((line = br.readLine()) != null)   {
        		
        		String[] vals = line.split(Constants.SERIALIZE_LINE_VAL_SEPARATOR);
        		
        		if(vals != null && vals.length > 0)
        		{
            		if(Constants.SERIALIZE_GUIDEPOINT_CARTESIAN_TYPE.equalsIgnoreCase(vals[0]))
            		{
            			/*
            			 * Expected line format:
            			 * 
            			 * GUIDEPOINT_CARTESIAN CARTESIAN X:-4.57 Y:-1.68 AUTOPILOT_TIME_MILLIS:126446 LOCAL_TIME_MILLIS:1487617141467
            			 * 
            			 */
            			this.guidePointCartesian = loadCartesianGuidepoint(vals);
            		}
            		else if(Constants.SERIALIZE_GUIDEPOINT_GPS_TYPE.equalsIgnoreCase(vals[0]))
            		{
            			/*
            			 * Expected line format:
            			 * 
            			 * GUIDEPOINT_GPS GPS LAT:30.5635939 LON:-87.6780064 FIX_TYPE:3
 						 *
            			 */
            			this.guidePointGPS = loadGPSGuidepoint(vals);
            		}
            		else if(Constants.SERIALIZE_POINT_TYPE.equalsIgnoreCase(vals[0]))
            		{
            			/*
            			 * Expected line format:
            			 * 
            			 * POINT 199 CARTESIAN X:-1.12 Y:-0.89 AUTOPILOT_TIME_MILLIS:103865 LOCAL_TIME_MILLIS:1487617118875
            			 * -----or-----
            			 * POINT 201 GPS LAT:30.5636255 LON:-87.6780056 FIX_TYPE:3
            			 * 
            			 */

            			Position position = buildPositionFromMissionPlanRecordingLineArray(vals);
            			this.addPosition(position);
            			
            		}
        		}

        	}

        	//Close the input stream
        	br.close();
    	}
    	catch(FileNotFoundException e1)
    	{
    		System.out.println("FileNotFoundException --> MissionDAO.java --> " + e1.toString());
    	}
    	catch(IOException e2)
    	{
    		System.out.println("IOException --> MissionDAO.java --> " + e2.toString());
    	}
    }
    
    /*
     * Expected line is of the form:
     * 
	 * GUIDEPOINT_CARTESIAN CARTESIAN X:-4.57 Y:-1.68 AUTOPILOT_TIME_MILLIS:126446 LOCAL_TIME_MILLIS:1487617141467
	 *
     */
    private CartesianPosition loadCartesianGuidepoint(String[] vals)
    {
    	String[] positionVals = Arrays.copyOfRange(vals, 2, vals.length);
    	return CartesianPosition.buildFromKeyValArray(positionVals);
    }
    
    /*
     * Expected line is of the form:
     * 
	 *	GUIDEPOINT_GPS GPS LAT:30.5635939 LON:-87.6780064 FIX_TYPE:3
	 *
     */
    private GPSPosition loadGPSGuidepoint(String[] vals)
    {
    	String[] positionVals = Arrays.copyOfRange(vals, 2, vals.length);
    	return GPSPosition.buildFromKeyValArray(positionVals);
    }
    
    /*
     * Expected array is of the form:
     * 
     * [POINT] [32] [CARTESIAN] [X:-2.72] [Y:0.57] [AUTOPILOT_TIME_MILLIS:131559] [LOCAL_TIME_MILLIS:1487620496519]
     * 
     * ------or------
     * 
     * [POINT] [24] [GPS] [LAT:30.5636187] [LON:-87.6780116] [FIX_TYPE:3]
	 *
     */
    private Position buildPositionFromMissionPlanRecordingLineArray(String[] vals)
    {
    	
		Integer recordIndex = Integer.parseInt(vals[1]);
    	PositionType positionType = guidePointTypeFromString(vals[2]);
    	String[] positionVals = Arrays.copyOfRange(vals, 2, vals.length);
    	
    	Position position = new Position();
    	position.setType(positionType);
    	position.setRecordIndex(recordIndex);
    	
    	if(positionType == PositionType.Cartesian)
    	{
    		CartesianPosition cp = CartesianPosition.buildFromKeyValArray(positionVals);
        	position.setCartesian(cp);
    	}
    	else if(positionType == PositionType.GPS)
    	{
    		GPSPosition gps = GPSPosition.buildFromKeyValArray(positionVals);
        	position.setGps(gps);
    	}
    	
    	return position;
    }
    
    private PositionType guidePointTypeFromString(String str)
    {
    	PositionType positionType = 
    			Constants.SERIALIZE_GPS_TYPE.equalsIgnoreCase(str) ? PositionType.GPS :
    				Constants.SERIALIZE_CARTESIAN_TYPE.equalsIgnoreCase(str) ? PositionType.Cartesian : null;
    	return positionType;
    }
    
    /*
     * Returns FALSE if we didn't adjust the line for the next orthogonal point since doing so would have
     * made the current line's value less than Config.Config.minMowingLineDistanceMeters.  In other words,
     * if we get back FALSE from this function, it can be assumed that we've reached the end of the mission
     * and should not build any further lines.
     */
    private Boolean adjustTurnInitiationPointSoThatNextOrthogonalPointIsWithinBoundary(Point2D.Double lineStartPoint, Point2D.Double lineTurnPoint, Point2D.Double normPerp, Path2D missionBoundary)
    {
    	//Next point is the perpendicular norm from lineTurnPoint
    	
    	Point2D.Double tempLineTurnPoint = new Point2D.Double(lineTurnPoint.x, lineTurnPoint.y);
    	Point2D.Double pushXY = getParallelNorm(lineStartPoint, lineTurnPoint, Config.boundaryPushAdjustmentMeters);
    	
    	Boolean LINE_TOO_SHORT = false;
    	
    	//If perpendicular norm is not within missionBoundary, then we
    	//reduce lineTurnPoint (pull it back towards lineStartPoint) until either
    	// A. Perpendicular norm is within mission Boundary
    	// B. Line from startTurnPoint to lineTurnPoint reduced below Config.minMowingLineDistanceMeters
    	Point2D.Double nextLineStartPoint = new Point2D.Double(lineTurnPoint.x+normPerp.x, lineTurnPoint.y+normPerp.y);
    	
    	while(!missionBoundary.contains(nextLineStartPoint) && !LINE_TOO_SHORT)
    	{
    		if(Math.abs(lineStartPoint.distance(tempLineTurnPoint))<Config.minMowingLineDistanceMeters)
    		{
    			LINE_TOO_SHORT = true;
    		}
    		tempLineTurnPoint.x -= pushXY.x;
    		tempLineTurnPoint.y -= pushXY.y;
    		
    		nextLineStartPoint.x -= pushXY.x;
    		nextLineStartPoint.y -= pushXY.y;
    	}
    	
    	//if tempLineTurnPoint adjusted until perpendicular point was within 
    	//missionBoundary and we didn't violate the length contract of Config.minMowingLineDistanceMeters
    	//then adjust the lineTurnPoint to tempLineTurnPoint
    	if(!LINE_TOO_SHORT)
    	{
    		Double nextLineLength = lineLengthToBoundary(nextLineStartPoint, pushXY, missionBoundary);
    		if(nextLineLength > Config.minMowingLineDistanceMeters)
    		{
        		lineTurnPoint.x = tempLineTurnPoint.x;
        		lineTurnPoint.y = tempLineTurnPoint.y;
    		}
    	}
    	
    	return !LINE_TOO_SHORT;
    }
    
    private Double lineLengthToBoundary(Point2D.Double start, Point2D.Double normParallel, Path2D missionBoundary)
    {
    	Point2D.Double parallelBoundaryPoint = new Point2D.Double(start.x, start.y);
    	while(missionBoundary.contains(parallelBoundaryPoint))
    	{
    		parallelBoundaryPoint.x -= normParallel.x;
    		parallelBoundaryPoint.y -= normParallel.y;
    	}
    	
    	Double distanceFromStartToParallelPointBoundary = start.distance(parallelBoundaryPoint);
    	return distanceFromStartToParallelPointBoundary;
    }
    
    private void pushLineToBoundary(Point2D.Double startFixed, Point2D.Double endFlexible, Point2D.Double normParallel, Path2D missionBoundary)
    {
    	Point2D.Double pushXY = new Point2D.Double(normParallel.x, normParallel.y);
    	
    	while(missionBoundary.contains(endFlexible))
    	{
    		endFlexible.x += pushXY.x;
    		endFlexible.y += pushXY.y;
    	}
    }
    
    /*
     * Returns direction (+1 or -1) to build mission from guideline.  Positive or Negative is just a standard to know how
     * to build the mission out.
     */
    private int directionToNavigateAfterGuideLine(Point2D.Double start, Point2D.Double guide, Path2D missionBoundary)
    {
    	int maxDistanceMetersToLookFromGuideLine = 10000000;
    	int pointsPositivePerpendicularWithinBounds = 0;
    	int pointsNegativePerpendicularWithinBounds = 0;
    	
    	
        for(int i = 1; i < maxDistanceMetersToLookFromGuideLine; i++)
        {
        	Point2D perpPoint = getPointPerpendicularToMiddleOfLine(start, guide, (double)i);
        	if(missionBoundary.contains(perpPoint))
        	{
        		pointsPositivePerpendicularWithinBounds += 1;
        	}
        	else
        	{
        		break;
        	}
        }
        
        for(int i = -1; i >= -maxDistanceMetersToLookFromGuideLine; i--)
        {
        	Point2D perpPoint = getPointPerpendicularToMiddleOfLine(start, guide, (double)i);
        	if(missionBoundary.contains(perpPoint))
        	{
        		pointsNegativePerpendicularWithinBounds += 1;
        	}
        	else
        	{
        		break;
        	}
        }

        return pointsPositivePerpendicularWithinBounds > pointsNegativePerpendicularWithinBounds ? 1 : -1;
    }
    
    public Path2D missionBoundary()
    {
    	Path2D missionBoundary = new Path2D.Double();
    	
    	//  loop trough all position points, starting at the beginning
    	//  and add Cartesian points to graph
    	
    	Boolean FIRST_LINE = true;
    	
    	for(int i = 0; i < this.getPoints().size(); i++)
    	{
    		Position position = this.getPoints().get(i);
    		if(position.isCartesian())
    		{
    			CartesianPosition cartesianPosition = position.getCartesian();
    			Double x = cartesianPosition.getX();
    			Double y = cartesianPosition.getY();
    			
            	if(FIRST_LINE)
            	{
            		FIRST_LINE = false;
            		missionBoundary.moveTo(x, y);
            	}
            	else
            	{
            		missionBoundary.lineTo(x, y);
            	}
    		}
    		
    	}
    	missionBoundary.closePath();
    	return missionBoundary;
    }
    
    private Point2D.Double getParallelNorm(Point2D.Double start, Point2D.Double stop, double distance)
    {
    	Double nX = (stop.x-start.x);
    	Double nY = (stop.y-start.y);

        double norm_length = (double) Math.sqrt((nX * nX) + (nY * nY));
        nX /= norm_length;
        nY /= norm_length;
        
        double xDistance = (distance*nX);
        double yDistance = (distance*nY);
        return new Point2D.Double(xDistance, yDistance);
    }
    
    private Point2D.Double getPerpendicularNorm(Point2D.Double start, Point2D.Double stop, double distance)
    {
    	Double perpX = (start.x-stop.x);
    	Double perpY = (start.y-stop.y);
    	
    	Double nX = -perpY;
    	Double nY = perpX;
    	
        double norm_length = (double) Math.sqrt((nX * nX) + (nY * nY));
        nX /= norm_length;
        nY /= norm_length;
        
        double xDistance = (distance*nX);
        double yDistance = (distance*nY);
        return new Point2D.Double(xDistance, yDistance);
    }
    
    private Point2D.Double getPointPerpendicularToMiddleOfLine(Point2D.Double start, Point2D.Double stop, double distance)
    {
    	Double midpointX = (start.x+stop.x)/2;
    	Double midpointY = (start.y+stop.y)/2;

    	Double perpX = (start.x-stop.x);
    	Double perpY = (start.y-stop.y);
    	
    	Double nX = -perpY;
    	Double nY = perpX;
    	
        
        double norm_length = (double) Math.sqrt((nX * nX) + (nY * nY));
        nX /= norm_length;
        nY /= norm_length;
        
        double newX = midpointX + (distance*nX);
        double newY = midpointY + (distance*nY);
        return new Point2D.Double(newX, newY);
    }

    public void addPosition(Position p)
    {
    	positions.add(p);
    }
    
    public GPSPosition getStartGPS()
    {
    	for(int i=0; i<positions.size(); i++)
    	{
    		if(positions.get(i).isGPS())
    		{
    			return positions.get(i).getGps();
    		}
    	}
    	return null;
    }
    
    public CartesianPosition getStartCartesian()
    {
    	for(int i=0; i<positions.size(); i++)
    	{
    		if(positions.get(i).isCartesian())
    		{
    			return positions.get(i).getCartesian();
    		}
    	}
    	return null;
    }
    
    public Double getStartX()
    {
    	CartesianPosition pos = getStartCartesian();
    	if(pos != null)
    		return pos.x;
    	return null;
    }
    
    public Double getStartY()
    {
    	CartesianPosition pos = getStartCartesian();
    	if(pos != null)
    		return pos.y;
    	return null;
    }
    
    public CartesianPosition getFinishCartesian()
    {
    	for(int i=positions.size()-1; i>=0; i--)
    	{
    		if(positions.get(i).isCartesian())
    		{
    			return positions.get(i).getCartesian();
    		}
    	}
    	return null;
    }
    
    public Double getFinishX()
    {
    	CartesianPosition pos = getFinishCartesian();
    	if(pos != null)
    		return pos.x;
    	return null;
    }
    
    public Double getFinishY()
    {
    	CartesianPosition pos = getFinishCartesian();
    	if(pos != null)
    		return pos.y;
    	return null;
    }

    public CartesianPosition getLastCartesian()
    {
        for(int x = positions.size()-1; x >= 0; x--)
        {
            if(positions.get(x).getType() == PositionType.Cartesian)
            {
                return positions.get(x).getCartesian();
            }
        }
        return null;
    }

    public GPSPosition getLastGps()
    {
        for(int x = positions.size()-1; x >= 0; x--)
        {
            if(positions.get(x).getType() == PositionType.GPS)
            {
                return positions.get(x).getGps();
            }
        }
        return null;
    }
    
    public Double getMinX()
    {
    	if(positions != null && positions.size() > 0)
    	{
    		Double minX = 0.0;
    		
    		for(Position position : positions)
    		{
    			if(position.isCartesian())
    			{
    				if(position.getCartesian().getX() < minX)
    					minX = position.getCartesian().getX();
    			}
    		}
    		return minX;
    	}
    	return null;
    }
    
    public Double getMinY()
    {
    	if(positions != null && positions.size() > 0)
    	{
    		Double minY = 0.0;
    		
    		for(Position position : positions)
    		{
    			if(position.isCartesian())
    			{
    				if(position.getCartesian().getY() < minY)
    					minY = position.getCartesian().getY();
    			}
    		}
    		return minY;
    	}
    	return null;
    }
    
    public Double getMaxX()
    {
    	if(positions != null && positions.size() > 0)
    	{
    		Double maxX = 0.0;
    		
    		for(Position position : positions)
    		{
    			if(position.isCartesian())
    			{
    				if(position.getCartesian().getX() > maxX)
    					maxX = position.getCartesian().getX();
    			}
    		}
    		return maxX;
    	}
    	return null;
    }
    
    public Double getMaxY()
    {
    	if(positions != null && positions.size() > 0)
    	{
    		Double maxY = 0.0;
    		
    		for(Position position : positions)
    		{
    			if(position.isCartesian())
    			{
    				if(position.getCartesian().getY() > maxY)
    					maxY = position.getCartesian().getY();
    			}
    		}
    		return maxY;
    	}
    	return null;
    }
    
    //Adjust Cartesian coordinates so that they all fall in the positive space
    //This is helpful for testing -- specifically graphing in java 
    public void scaleMinXAndMinYToZero()
    {
    	Double minX = getMinX();
    	Double minY = getMinY();
    	
    	for(Position p : this.positions)
    	{
    		if(p.isCartesian())
    		{
    			p.cartesian.x -= minX;
    			p.cartesian.y -= minY;
    		}
    	}
    	guidePointCartesian.x -= minX;
    	guidePointCartesian.y -= minY;
    }
    
    private void fileWriterSilentClose(FileWriter writer)
    {
        try
        {
            writer.flush();
            writer.close();
        }
        catch (Exception ex)
        {
            System.out.println(ex.toString());
        }
    }

    public List<Position> getPoints() {
        return positions;
    }

    public void setPoints(List<Position> points) {
        this.positions = points;
    }

    public CartesianPosition getGuidePointCartesian() {
        return guidePointCartesian;
    }

    public void setGuidePointCartesian(CartesianPosition guidePointCartesian) {
        this.guidePointCartesian = guidePointCartesian;
    }

    public GPSPosition getGuidePointGPS() {
        return guidePointGPS;
    }

    public void setGuidePointGPS(GPSPosition guidePointGPS) {
        this.guidePointGPS = guidePointGPS;
    }

}
