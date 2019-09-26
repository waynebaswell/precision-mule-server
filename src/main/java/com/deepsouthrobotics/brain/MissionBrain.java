package com.deepsouthrobotics.brain;

import com.deepsouthrobotics.data.Config;
import com.deepsouthrobotics.data.GPSCartesianCoordinateSpace;
import com.deepsouthrobotics.data.GPSPosition;
import com.deepsouthrobotics.util.Geo;

import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.io.FileWriter;
import java.util.ArrayList;
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
    //public List<Position> positions;
    //public CartesianPosition guidePointCartesian;
    //public GPSPosition guidePointGPS;


    public MissionBrain()
    {
    	//positions = new ArrayList<>();
    }

    public List<List<Point2D.Double>> additionalValidMissionPointsOnTheGivenPointPathAndHeading(
			Path2D.Double missionBoundary,
			Point2D.Double originalPointPath,
			Double headingRadians,
			Double minX,
			Double maxX,
			Double minY,
			Double maxY
	)
	{
		List<List<Point2D.Double>> listOfPointsLists = new ArrayList<>();
		Point2D.Double endPointAtEdgeOfBoundary = new Point2D.Double(originalPointPath.x, originalPointPath.y);
		//First push the start point 'till it's at the edge of the missionBoundary
		//on the given heading
		while(missionBoundary.contains(endPointAtEdgeOfBoundary))
		{
			endPointAtEdgeOfBoundary.x += Math.cos(headingRadians) * .01;
			endPointAtEdgeOfBoundary.y += Math.sin(headingRadians) * .01;
		}
		//In the while... statement above we pushed the start point just beyond
		//the boundary, so lets bring it back to the last point within
		//the boundary and then begin looking for more valid points
		//on the same heading
		endPointAtEdgeOfBoundary.x -= Math.cos(headingRadians) * .01;
		endPointAtEdgeOfBoundary.y -= Math.sin(headingRadians) * .01;


		//Define a new variable "point" that we'll work with -- at first point
		//will be equal to the endPointAtEdeOfBoundary, but we'll push
		//it along the path's heading in search of a valid
		//section of polygon out in the distance
		Point2D.Double point = new Point2D.Double(endPointAtEdgeOfBoundary.x, endPointAtEdgeOfBoundary.y);

		//Now we push beyond the current boundary in increments of (Config.
		//minMowingLineDistanceMeters - 1cm) 'till we're beyond one of the
		//the edge values of our missionBoundary (i.e. the "edge"
		//values are passed in as minX,maxX,minY,maxY)
		//checking to see if there's another
		//section of the mission on
		//this heading
		while(point.x >= minX && point.x <= maxX && point.y >= minY && point.y <= maxY)
		{
			Double xComponentAdd = Math.cos(headingRadians) * (Config.minMowingLineDistanceMeters-.01);
			Double yComponentAdd = Math.sin(headingRadians) * (Config.minMowingLineDistanceMeters-.01);
			point.x += xComponentAdd;
			point.y += yComponentAdd;
			if(missionBoundary.contains(point))
			{
				//once we find a point within the polygon we're only part of the way to knowing if
				//this is a valid mission point -- in order to know if it's valid we have to back
				//up to the polygon edge where the line containing this point begins -- and then
				//push from that point 'till we find the end of the line and figure out
				//if the distance is sufficient to consider this
				//a valid mission line
				Point2D.Double newBeginPoint = new Point2D.Double(point.x, point.y);
				Double xCentimeterBump = Math.cos(headingRadians) * .01;
				Double yCentimeterBump = Math.sin(headingRadians) * .01;
				while(missionBoundary.contains(newBeginPoint))
				{
					newBeginPoint.x -= xCentimeterBump;
					newBeginPoint.y -= yCentimeterBump;
				}
				newBeginPoint.x += xCentimeterBump;
				newBeginPoint.y += yCentimeterBump;
				Point2D.Double newEndPoint = polygonEdgePointByFollowingGivenStartingPointAndHeading(
						missionBoundary, newBeginPoint, headingRadians);
				if(newBeginPoint.distance(newEndPoint) >= 2.0)
				{
					//Well, congratulations folks, we've got a valid mission point out beyond where the
					//polygon boundary had previously stopped -- now that we know the begin
					//and end of this new mission point, we've got to chart a path
					//beginning with the original point and following the
					//mission boundary around 'till we
					//get to this new point
					List<Point2D.Double> pointsFromOriginalPointToNewStartingPoint = new ArrayList<>();
					if(originalPointPath.distance(endPointAtEdgeOfBoundary) != 0)
					{
						pointsFromOriginalPointToNewStartingPoint.add(endPointAtEdgeOfBoundary);
					}
					tracePathAlongMissionBoundaryFromOnePointToAnotherPointAddingVerticesOfTheShortestPath(
							missionBoundary, endPointAtEdgeOfBoundary, newBeginPoint,
							pointsFromOriginalPointToNewStartingPoint);
					pointsFromOriginalPointToNewStartingPoint.add(newEndPoint);
					//Now pointsFromOriginalPointToNewStartingPoint is a big beautiful
					//List of points from the originalPointPath point that we called
					//this method with tracing around the bondary 'till the next
					//valid point start and then containing that point's end

					//So let's go ahead and add it to the List of Lists
					//that we will return
					listOfPointsLists.add(pointsFromOriginalPointToNewStartingPoint);

					//Next up -- figure out how to look beyond the newEndPoint to see
					//if there are additional places to mow out in the distance...
				}
			}
		}

		return listOfPointsLists;
	}

	public void tracePathAlongMissionBoundaryFromOnePointToAnotherPointAddingVerticesOfTheShortestPath(
			Path2D.Double missionBoundary,
			Point2D.Double point1,
			Point2D.Double point2,
			List<Point2D.Double> listOfPointsToAppendThePathTo
	)
	{
		//First off we need to find out EITHER 1. if point1
		//is a vertex on missionBoundary ...OR... 2.
		//which 2 vertices on the missionBoundary
		//that point1 lies between
		List<Point2D.Double> missionBoundaryPoints = getPointsOnPath(missionBoundary);

		Boolean point1IsVertex = false;
		int point1MissionBoundaryVertexIndex = -1;
		int point1LeftMissionBoundaryVertexIndex = -1;
		int point1RightMissionBoundaryVertexIndex = -1;

		for(int x = 0; x < missionBoundaryPoints.size(); x++)
		{
			Point2D.Double vertex = missionBoundaryPoints.get(x);
			if (vertex.distance(point1) == 0)
			{
				//so it turns out that point1 was a vertex of the polygon
				//so now we need to:
				//1. Find the shortest path from point1 to point2 along
				//the mission boundary (i.e. there are 2 paths so our
				//aim is to find the shortest of those 2)
				point1IsVertex = true;
				point1MissionBoundaryVertexIndex = x;
			}
			else
			{
				if(x == missionBoundaryPoints.size()-1)
				{
					//we're at the end of the list, so we need to check if the
					//point lies between the last element (vertex) of
					//the list and the first element

					Point2D.Double leftPoint = missionBoundaryPoints.get(x);
					Point2D.Double rightPoint = missionBoundaryPoints.get(0);
					Line2D.Double line = new Line2D.Double(leftPoint, rightPoint);
					if(line.contains(point1))
					{
						point1LeftMissionBoundaryVertexIndex = x;
						point1RightMissionBoundaryVertexIndex = 0;
					}
				}
				else
				{
					//check if point lies between the current vertex and the
					//next vertex in the list
					Point2D.Double leftPoint = missionBoundaryPoints.get(x);
					Point2D.Double rightPoint = missionBoundaryPoints.get(x+1);
					Line2D.Double line = new Line2D.Double(leftPoint, rightPoint);
					if(line.contains(point1))
					{
						point1LeftMissionBoundaryVertexIndex = x;
						point1RightMissionBoundaryVertexIndex = x+1;
					}
				}
			}
		}

		if(point1IsVertex == false && point1LeftMissionBoundaryVertexIndex == -1 && point1RightMissionBoundaryVertexIndex == -1)
		{
			//we should never get to this point -- it means that we didn't
			//find the point anywhere on the perimeter of the polygon
			System.out.println("Houston we have a problem: point1 is bogus in MissionBrain::tracePathAlongMissionBoundaryFromOnePointToAnotherPointAddingVerticesOfTheShortestPath(..)");
		}

		//Now that we know where point1 is on the missionBoundary,
		//it's high time to do the same logic for point2

		//We effectively duplicate the code above for point1
		Boolean point2IsVertex = false;
		int point2MissionBoundaryVertexIndex = -1;
		int point2LeftMissionBoundaryVertexIndex = -1;
		int point2RightMissionBoundaryVertexIndex = -1;

		for(int x = 0; x < missionBoundaryPoints.size(); x++)
		{
			Point2D.Double vertex = missionBoundaryPoints.get(x);
			if (vertex.distance(point2) == 0)
			{
				point2IsVertex = true;
				point2MissionBoundaryVertexIndex = x;
			}
			else
			{
				if(x == missionBoundaryPoints.size()-1)
				{
					Point2D.Double leftPoint = missionBoundaryPoints.get(x);
					Point2D.Double rightPoint = missionBoundaryPoints.get(0);
					Line2D.Double line = new Line2D.Double(leftPoint, rightPoint);
					if(line.contains(point2))
					{
						point2LeftMissionBoundaryVertexIndex = x;
						point2RightMissionBoundaryVertexIndex = 0;
					}
				}
				else
				{
					//check if point lies between the current vertex and the
					//next vertex in the list
					Point2D.Double leftPoint = missionBoundaryPoints.get(x);
					Point2D.Double rightPoint = missionBoundaryPoints.get(x+1);
					Line2D.Double line = new Line2D.Double(leftPoint, rightPoint);
					if(line.contains(point2))
					{
						point2LeftMissionBoundaryVertexIndex = x;
						point2RightMissionBoundaryVertexIndex = x+1;
					}
				}
			}
		}

		if(point2IsVertex == false && point2LeftMissionBoundaryVertexIndex == -1 && point2RightMissionBoundaryVertexIndex == -1)
		{
			//we should never get to this point -- it means that we didn't
			//find the point anywhere on the perimeter of the polygon
			System.out.println("Houston we have a problem: point2 is bogus in MissionBrain::tracePathAlongMissionBoundaryFromOnePointToAnotherPointAddingVerticesOfTheShortestPath(..)");
		}

		//Now that we know where point1 and point2 are on the polygon perimeter,
		//it's time to move along the mission boundary both ways and see
		//which path is the shortest -- this will be the route we
		//choose to follow (i.e. we'll append those points
		//to listOfPointsToAppendThePathTo)

		Double directionLeftDistance = 0.0;
		List<Point2D.Double> leftPathVertices = new ArrayList<>();

		Double directionRightDistance = 0.0;
		List<Point2D.Double> rightPathVertices = new ArrayList<>();

		if(point1IsVertex)
		{
			if(point2IsVertex)
			{
				//point1 and point2 are both vertices

				//First find the distance by moving from point1 to point2
				//by **subtracting** the vertices' index from point1 --
				//I'll call moving "left" -- here we move
				//along the boundary by subtracting
				//from the point1 index 'till
				//we reach point2's index
				int nextIndexLeft = point1MissionBoundaryVertexIndex == 0 ? missionBoundaryPoints.size()-1 : point1MissionBoundaryVertexIndex-1;
				Point2D.Double previousPoint = missionBoundaryPoints.get(point1MissionBoundaryVertexIndex);
				boolean done = false;
				while(!done)
				{
					Point2D.Double nextPoint = missionBoundaryPoints.get(nextIndexLeft);
					leftPathVertices.add(nextPoint);
					Double previousToNextPointDistance = previousPoint.distance(nextPoint);
					directionLeftDistance += previousToNextPointDistance;
					nextIndexLeft = nextIndexLeft == 0 ? missionBoundaryPoints.size()-1 : nextIndexLeft-1;
					previousPoint = nextPoint;

					if(nextIndexLeft == point2MissionBoundaryVertexIndex)
					{
						//Next point would be point2, so we're done with the loop --
						//we need to add the distance from
						//previousPoint to point2
						done = true;
						previousToNextPointDistance = previousPoint.distance(point2);
						directionLeftDistance += previousToNextPointDistance;
						//Note that we take care of adding point2 to the listOfPointsToAppendThePathTo
						//at the very end of this method
					}
				}

				//Now find the distance by moving from point1 to point2
				//by **adding** the vertices' index from point1
				//'till we reach point2's index
				int nextIndexRight = point1MissionBoundaryVertexIndex == missionBoundaryPoints.size()-1 ? 0 : point1MissionBoundaryVertexIndex+1;
				previousPoint = missionBoundaryPoints.get(point1MissionBoundaryVertexIndex);
				done = false;
				while(!done)
				{
					Point2D.Double nextPoint = missionBoundaryPoints.get(nextIndexRight);
					rightPathVertices.add(nextPoint);
					Double previousToNextPointDistance = previousPoint.distance(nextPoint);
					directionRightDistance += previousToNextPointDistance;
					nextIndexRight = nextIndexRight == missionBoundaryPoints.size()-1 ? 0 : nextIndexRight+1;
					previousPoint = nextPoint;

					if(nextIndexRight == point2MissionBoundaryVertexIndex)
					{
						done=true;
						previousToNextPointDistance = previousPoint.distance(point2);
						directionRightDistance += previousToNextPointDistance;
					}
				}

			}
			else
			{
				//point1 is a vertex, but point2 lies between 2 vertices -- specifically,
				//point2 lies between point2LeftMissionBoundaryVertexIndex
				//and point2RightMissionBoundaryVertexIndex
				int nextIndexLeft = point1MissionBoundaryVertexIndex == 0 ? missionBoundaryPoints.size()-1 : point1MissionBoundaryVertexIndex-1;
				Point2D.Double previousPoint = missionBoundaryPoints.get(point1MissionBoundaryVertexIndex);
				boolean done = false;
				while(!done)
				{
					Point2D.Double nextPoint = missionBoundaryPoints.get(nextIndexLeft);
					leftPathVertices.add(nextPoint);
					Double previousToNextPointDistance = previousPoint.distance(nextPoint);
					directionLeftDistance += previousToNextPointDistance;
					nextIndexLeft = nextIndexLeft == 0 ? missionBoundaryPoints.size()-1 : nextIndexLeft-1;
					previousPoint = nextPoint;

					if(nextIndexLeft == point2LeftMissionBoundaryVertexIndex ||
						nextIndexLeft == point2RightMissionBoundaryVertexIndex)
					{
						//Next point would be a vertex of the line containing point2, so we're done
						//with the loop -- we need to 1. add the distance from previousPoint to
						//the vertex at point2LeftMissionBoundaryVertexIndex and 2. then from
						//the vertex at point2LeftMissionBoundaryVertexIndex to point2

						done = true;
						Point2D.Double leftBoundaryPoint = missionBoundaryPoints.get(nextIndexLeft);
						directionLeftDistance += nextPoint.distance(leftBoundaryPoint);
						directionLeftDistance += leftBoundaryPoint.distance(point2);

						//we'll need to add the Point2D.Double at point2LeftMissionBoundaryVertexIndex
						//to the leftPathVertices
						leftPathVertices.add(leftBoundaryPoint);
					}
				}

				//Now repeat the logic above, but chart the "right" path and it's distance
				//Pick up right here Friday, September 27!
			}
		}
		else
		{
			if(point2IsVertex)
			{
				//point1 lies between 2 vertices, but point2 is a vertex

			}
			else
			{
				//point1 and point2 both lie between 2 vertices

			}
		}

		//Now that we've calculated the length of both the left and right paths
		//we follow the shorter path by appending it's points
		//to listOfPointsToAppendThePathTo
		if(directionLeftDistance <= directionRightDistance)
		{
			for(int x = 0; x < leftPathVertices.size(); x++)
			{
				listOfPointsToAppendThePathTo.add(leftPathVertices.get(x));
			}
		}
		else
		{
			for(int x = 0; x < rightPathVertices.size(); x++)
			{
				listOfPointsToAppendThePathTo.add(rightPathVertices.get(x));
			}
		}

		//in this method we assume that the calling code has already added
		//point1 to the listOfPointsToAppendThePathTo...but,
		//we do need to add point2 at the end
		listOfPointsToAppendThePathTo.add(point2);
	}


	//Credit:
	//https://github.com/gurkenlabs/litiengine/blob/master/src/de/gurkenlabs/litiengine/util/geom/GeometricUtilities.java
	public static List<Point2D.Double> getPointsOnPath(final Path2D path)
	{
		final PathIterator pi = path.getPathIterator(null);
		final double[] coordinates = new double[22];
		final List<Point2D.Double> points = new ArrayList<>();
		while (!pi.isDone())
		{
			pi.next();

			pi.currentSegment(coordinates);
			final Point2D.Double currentPoint = new Point2D.Double(coordinates[0], coordinates[1]);
			points.add(currentPoint);
		}

		return points;
	}

    public Point2D.Double polygonEdgePointByFollowingGivenStartingPointAndHeading(
			Path2D missionBoundary, Point2D.Double start, Double headingRadians)
	{
		Double edgeX = start.x;
		Double edgeY = start.y;

		//Starting at the start point, move within the missionBoundary path
		//in the direction specified by headingRadians 1 cm at a time
		//'till we reach a point outside of missionBoundary
		Double xComponentAdd = Math.cos(headingRadians) * .01;
		Double yComponentAdd = Math.sin(headingRadians) * .01;

		while(missionBoundary.contains(edgeX, edgeY))
		{
			edgeX += xComponentAdd;
			edgeY += yComponentAdd;
		}

		//Note that we have to subtract a centimeter off the x and y component
		//here before returning because the while.. statement above goes
		//'till we hit a point that's a centimeter outside of
		//the boundary -- in other words, by subtracting
		//the centimeter (.01) we end up returning
		//the last point we evaluated that
		//the missionBoundary actually
		//contained
		Point2D.Double edge = new Point2D.Double(edgeX-xComponentAdd, edgeY-yComponentAdd);
		return edge;
	}

	public void adjustStartingPointIfFirstLineIsTooShort(GPSPosition start,
														 Path2D missionBoundary,
														 Double headingRadians)
	{
		Point2D.Double guide = polygonEdgePointByFollowingGivenStartingPointAndHeading(missionBoundary, start, headingRadians);
		//If line length is too short then adjust the starting location
		double distance = start.distance(guide);
		if(distance == 0)
		{
			//Add some x,y value that's perpendicular to the start value --
			//find the perpendicular value out by adding then subtracting
			//pi/2 to the headingRadians value and then using that
			//value to get x/y components to add to the start
			//value and seeing if it's within
			//the missionBoundary
			Double perpHeading = headingRadians + Math.PI/2;

			Double xComponentPerp = Math.cos(perpHeading) * .01;
			Double yComponentPerp = Math.sin(perpHeading) * .01;

			Point2D.Double perpPoint = new Point2D.Double(start.x+xComponentPerp, start.y+yComponentPerp);
			if(missionBoundary.contains(perpPoint))
			{
				start.x = perpPoint.x;
				start.y = perpPoint.y;
			}
			else
			{
				perpPoint = new Point2D.Double(start.x-xComponentPerp, start.y-yComponentPerp);
				if(missionBoundary.contains(perpPoint))
				{
					start.x = perpPoint.x;
					start.y = perpPoint.y;
				}
				else
				{
					//Well, the perpendicular points on either side of the start line aren't
					//in the mission polygon so i think something is pretty
					//jacked up
				}
			}
		}
		guide = polygonEdgePointByFollowingGivenStartingPointAndHeading(missionBoundary, start, headingRadians);
		int navigateDirection = directionToNavigateAfterGuideLine(start, guide, missionBoundary);

		while(start.distance(guide) < Config.minMowingLineDistanceMeters)
		{
			Point2D.Double normPerpXY = getPerpendicularNorm(start, guide, .01);
			start.x += normPerpXY.x*navigateDirection;
			start.y += normPerpXY.y*navigateDirection;
			guide = polygonEdgePointByFollowingGivenStartingPointAndHeading(missionBoundary, start, headingRadians);
			Double theDistance = start.distance(guide);
			System.out.println("Hello friends");
		}
	}

	/**
	  Builds mission waypoints from InputStream
	 */
    public List<GPSPosition> buildMissionWaypointsFromInputStream(List<GPSPosition> missionBoundaryGPSPositionList,
															   Double mowingPathWidthInMeters,
															   Double headingDegrees,
															   GPSPosition startGPSPosition)
    {
		Double[] minXandMinY = scaleMinXAndMinYToZero(missionBoundaryGPSPositionList);

		//If the startGPSPosition is not the first element in the missionBoundaryGPSPositionList then
		//that means that we didn't scale the Min X and Min Y values in the method call
		//above -- just for the record, the scenario where startGPSPosition wouldn't
		//be the first element in the list is where startGPSPosition is where
		//the user has dragged the start marker inside the
		//missionBoundaryGPSPositionList polygon
		if(startGPSPosition.compareTo(missionBoundaryGPSPositionList.get(0)) != 0)
		{
			startGPSPosition.x -= minXandMinY[0];
			startGPSPosition.y -= minXandMinY[1];
		}

		//Go ahead and pull the min x, max x, min y and max y values --
		//we'll use these to hopefully save a few cpu
		//cycles when building the mission

		Double minX = getMinX(missionBoundaryGPSPositionList); // should always be zero
		Double maxX = getMaxX(missionBoundaryGPSPositionList);

		Double minY = getMinY(missionBoundaryGPSPositionList); // should always be zero
		Double maxY = getMaxY(missionBoundaryGPSPositionList);

		//Not entirely sure we need this space variable --
		//the idea is you're passing in some coordinate
		// other than the home coordinate --
		//before we got rid of "GuidePoint" stuff we were
		//passing in the guidepoint coordinate
		GPSCartesianCoordinateSpace space = new GPSCartesianCoordinateSpace(missionBoundaryGPSPositionList.get(1));

		Double headingRadians = Math.toRadians(headingDegrees);
        Path2D missionBoundary = missionBoundary(missionBoundaryGPSPositionList);

		adjustStartingPointIfFirstLineIsTooShort(startGPSPosition, missionBoundary, headingRadians);
		startGPSPosition = space.gpsPositionGivenDistanceFromZeroZero(startGPSPosition.x, startGPSPosition.y);

		//Build a "guidepoint" to shoe-horn into our old code/logic --
		//the guidepoint is the endpoint of our first mission line --
		//in other words, it's the second waypoint in our mission (i.e.
		//the startGPSPosition is the first waypoint)
		Point2D.Double guide = polygonEdgePointByFollowingGivenStartingPointAndHeading(missionBoundary, startGPSPosition, headingRadians);

    	List<GPSPosition> missionWaypoints = new ArrayList<>();

    	//Calculate which way to commence building the mission after hitting the guidepoint
    	int navigateDirection = directionToNavigateAfterGuideLine(startGPSPosition, guide, missionBoundary);
    	
    	//We use the perpendicular norm for finding next turn points
    	Point2D.Double normPerpXY = getPerpendicularNorm(startGPSPosition, guide, mowingPathWidthInMeters);
    	normPerpXY.x *= navigateDirection;
    	normPerpXY.y *= navigateDirection;
    	
    	//We use the parallel norm for pushing/pulling the mission line
    	Point2D.Double normParallel = getParallelNorm(startGPSPosition, guide, Config.boundaryPushAdjustmentMeters);
    	normParallel.x *= -1;//navigateDirection;
    	normParallel.y *= -1;//navigateDirection;
    	
    	//Convenient way to push/pull the mission line opposite direction of above
    	Point2D.Double normParallelNegative = new Point2D.Double();
    	normParallelNegative.x  = normParallel.x * -1;
    	normParallelNegative.y  = normParallel.y * -1;

		Point2D.Double currentTopPoint = new Point2D.Double(guide.x, guide.y);
		Point2D.Double currentBottomPoint = new Point2D.Double(startGPSPosition.x, startGPSPosition.y);
		
		Point2D.Double lastTopPoint = new Point2D.Double(currentTopPoint.x, currentTopPoint.y);
		Point2D.Double lastBottomPoint = new Point2D.Double(currentBottomPoint.x, currentBottomPoint.y);
		
		GPSPosition lastTopGPS = space.gpsPositionGivenDistanceFromZeroZero(guide.x, guide.y); //this.getGuidePointGPS();
		GPSPosition lastBottomGPS = new GPSPosition(startGPSPosition.latitude,
				startGPSPosition.longitude, startGPSPosition.x, startGPSPosition.y);//this.getStartGPS();
		
		Geo geo = new Geo();
		
    	//The first mission point will always be the start position
    	missionWaypoints.add(startGPSPosition);
    	
    	//The second mission point will always be the guide point adjusted to ensure next turn is within boundary
		Boolean ADD_ANOTHER_PATH_LINE = adjustTurnInitiationPointSoThatNextOrthogonalPointIsWithinBoundary(currentBottomPoint, currentTopPoint, normPerpXY, missionBoundary);
		
		GPSPosition adjustedGuideGPS = space.gpsPositionGivenDistanceFromZeroZero(currentTopPoint.x, currentTopPoint.y);
		//CartesianPosition adjustedGuideCart = new CartesianPosition(currentTopPoint.x, currentTopPoint.y);
		//Position adjustedGuidePosition = new Position(PositionType.GPS, -1, adjustedGuideGPS, adjustedGuideCart);
		missionWaypoints.add(adjustedGuideGPS);

		//baswell begin new fancy look-beyond-boundary logic

		//first you have to push beyond the immediate boundary of the current
		//polygon -- the adjustTurnInitiation... logic above will make
		//it such that our guidepoint may not be at the end



		//baswell end new fancy look-beyond-boundary logic
		
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
					//CartesianPosition cart = new CartesianPosition(currentTopPoint.x, currentTopPoint.y);
					missionWaypoints.add(newTopGPS);
					System.out.println(i+"Top "+ newTopGPS.latitude + " " + newTopGPS.longitude + " Distance: " + geo.latLongDistance(lastTopGPS.latitude, lastTopGPS.longitude, newTopGPS.latitude, newTopGPS.longitude));
					//System.out.println(i+"A Top "+ gps.latitude + " " + gps.longitude + " Distance: " + geo.latLongDistance(lastTopGPS.latitude, lastTopGPS.longitude, gps.latitude, gps.longitude));
					lastTopGPS = newTopGPS;
					
					if(Math.abs(currentTopPoint.distance(currentBottomPoint)) >= Config.minMowingLineDistanceMeters)
					{
						GPSPosition newBottomGPS = space.gpsPositionGivenDistanceFromZeroZero(currentBottomPoint.x, currentBottomPoint.y);
						//GPSPosition newBottomGPS = geo.offset(newTopGPS, bottomOffsetX, bottomOffsetY);
						//cart = new CartesianPosition(currentBottomPoint.x, currentBottomPoint.y);
						missionWaypoints.add(newBottomGPS);
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
			else //we are at the bottom -- so set the bottom point to the perpendicular offset and push the top point to missionBoundary
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
					//CartesianPosition cart = new CartesianPosition(currentBottomPoint.x, currentBottomPoint.y);
					missionWaypoints.add(newBottomGPS);
					System.out.println(i+"Bottom lat:"+ newBottomGPS.latitude + " lon:" + newBottomGPS.longitude + " x: " + newBottomGPS.x + " y:" + newBottomGPS.y +" Distance: " + geo.latLongDistance(lastBottomGPS.latitude, lastBottomGPS.longitude, newBottomGPS.latitude, newBottomGPS.longitude));
					//System.out.println(i+"B Bottom "+ gps.latitude + " " + gps.longitude + " Distance: " + geo.latLongDistance(lastBottomGPS.latitude, lastBottomGPS.longitude, gps.latitude, gps.longitude));
					lastBottomGPS = newBottomGPS;
					
					if(Math.abs(currentTopPoint.distance(currentBottomPoint)) >= Config.minMowingLineDistanceMeters)
					{
						GPSPosition newTopGPS = space.gpsPositionGivenDistanceFromZeroZero(currentTopPoint.x, currentTopPoint.y);
						//GPSPosition newTopGPS = geo.offset(newBottomGPS, topOffsetX, topOffsetY);
						//cart = new CartesianPosition(currentTopPoint.x, currentTopPoint.y);
						missionWaypoints.add(newTopGPS);
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
    	//Path2D p1 = new Path2D.Double();
		//Path2D p2 = new Path2D.Double();


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
	 * @param gpsPosition GPS position (in lat,long terms) for this mission --
	 * this is just a reference point we use when building the mission
	 * -- so another way of explaining what we're doing is this: we
	 * we're going to treat the given gpsPosition as being (0,0)
	 * on our (x,y) map -- and for all other GPS positions, we
	 * just find the distance between them and this GPS
	 * position and the x,y components of this
	 * distance are the x,y values for the
	 * point on our Cartesian plane. If
	 * that doesn't make sense it
	 * is likely that I'm not
	 * explaining a simple
	 * idea very well.
	 *
	 * @param pos Some other (lat,lng) position that we'd like to build
	 * an (x, y) position for
	 */
	public void setXLatandYLngMetersByDiffingLatAndLonDistanceFromGPSPosition(GPSPosition gpsPosition, GPSPosition pos)
	{
		Geo geo = new Geo();
		double lngMeters = geo.latLongDistance(gpsPosition.latitude,gpsPosition.longitude,gpsPosition.latitude,pos.longitude);
		double latMeters = geo.latLongDistance(gpsPosition.latitude,gpsPosition.longitude,pos.latitude,gpsPosition.longitude);

		if(gpsPosition.latitude > pos.latitude)
			latMeters*=-1;

		if(gpsPosition.longitude > pos.longitude)
			lngMeters*=-1;

		pos.x = latMeters;
		pos.y = lngMeters;
	}

//	public List<LatLng> GPSPositionListToLatLngList(List<GPSPosition> gpsList)
//	{
//		List<LatLng> llList = new ArrayList<>();
//		for(int x = 0; x < gpsList.size(); x++)
//		{
//			llList.add(gpsList.get(x).toLatLng());
//		}
//		return llList;
//	}

	/**
	 *
	 * @param missionBoundaryGPSPositionList List of coordinates that represent the
	 * vertices of the polygon that we're building a mission inside of. Just
	 * so we're on the same page -- this Array must contain at least 3
	 * points to make sense (i.e. 2 points would be a straight line, 1
	 * point would be a point, and 0 points is, well, you know..)
	 *
	 * @param mowingPathWidthInMeters Distance between waypoints -- for a
	 * mowing application, for example, this will be slightly less than
	 * the width of the mowing deck to allow a little overlap
	 *
	 * @return List of waypoints which define the mission within the
	 * polygon. The list is ordered such that the first point will
	 * be the starting coordinate and the
	 *
	 */
	public List<GPSPosition> buildMissionWaypointsFromLatLngBoundsJSONArray(
			List<GPSPosition> missionBoundaryGPSPositionList,
			List<List<GPSPosition>> polyObstaclesGPSPositionList,
			Double mowingPathWidthInMeters,
			GPSPosition startGPSPositionUnchecked,
			Double heading)
	{
		//New method begin

		//Build LatLng objects from the given GPSPosition objects
//		List<LatLng> missionLatLngList = GPSPositionListToLatLngList(missionBoundaryGPSPositionList);
//		List<List<LatLng>> polyObstaclesLatLngList = new ArrayList<>();
//		for(List<GPSPosition> list : polyObstaclesGPSPositionList)
//		{
//			polyObstaclesLatLngList.add(GPSPositionListToLatLngList(list));
//		}
//		LatLng startLatLng = startGPSPosition.toLatLng();
		//Now figure out if the startLatLng is within the mission polygon -- if it is
		//then that will be the mission starting point -- if it's not then
		//find the nearest vertex on the mission polygon to the point
		//and set the point's LatLng to that vertex -- in
		//other words we'll begin building the
		//mission at that vertex

		startGPSPositionUnchecked.x = 0;
		startGPSPositionUnchecked.y = 0;

		for(int x = 0; x < missionBoundaryGPSPositionList.size(); x++)
		{
			setXLatandYLngMetersByDiffingLatAndLonDistanceFromGPSPosition(
					startGPSPositionUnchecked, missionBoundaryGPSPositionList.get(x));
		}


		//Path2D missionBoundaryPath2D = missionBoundary(missionBoundaryGPSPositionList);

		GPSPosition startGPSPosition = getClosestPolygonVertexIfPointNotWithinPolygon(startGPSPositionUnchecked, missionBoundaryGPSPositionList);
		//startGPSPosition = ((DSRLatLng)startLatLng).toGPSPosition();
		//New method end

		//Old method begin
		//Now gpsPositionList contains the gps positions
		System.out.println("Now let's do some KARATE on the gps positions...");

//		//We'll say that the first item in gpsPositionList is the home position (0,0)
//		CartesianPosition cpZero = new CartesianPosition();
//		cpZero.x = 0.0;
//		cpZero.y = 0.0;
//		cpZero.localTimeReceivedMillis = 1l;
//		cpZero.autopilotTimeRecordedMillis = 1l;
//
//		List<CartesianPosition> cartesianPositionList = new ArrayList<>();
//		cartesianPositionList.add(cpZero);

		//If startGPSPosition is one of the mission boundary polygon vertices,
		//this moves it to the beginning of the list
		moveStartGPSPositionToBeginningOfList(missionBoundaryGPSPositionList, startGPSPosition);
		startGPSPosition.x = 0;
		startGPSPosition.y = 0;

		for(int x = 1; x < missionBoundaryGPSPositionList.size(); x++)
		{
			setXLatandYLngMetersByDiffingLatAndLonDistanceFromGPSPosition(startGPSPosition, missionBoundaryGPSPositionList.get(x));
		}

		//StringBuffer mprBuffer = new StringBuffer();
		//WE've gotta have at least 3 coordinates in order for
		//any of this logic to make sense (i.e. if you've only got 2 points
		//then you've got a line, which doesn't have area and
		//won't contain waypoints by definition)

//		if(missionGPSPositionList.size() >= 3)
//		{
//			//CartesianPosition guidepointC = cartesianPositionList.get(1);
//			GPSPosition guidepointG = missionGPSPositionList.get(1);
//
//			mprBuffer.append(guidepointC.buildGuidePointString()+"\n"); //Add Cartesian guidepoint
//			mprBuffer.append(guidepointG.buildGuidePointString()+"\n"); //Add GPS guidepoint
//
//			for(int x = 0, y=0; x < missionGPSPositionList.size(); x++)
//			{
//				mprBuffer.append(missionGPSPositionList.get(x).buildPointString(y++)+"\n");
//				//mprBuffer.append(cartesianPositionList.get(x).buildPointString(y++)+"\n");
//			}
//		}

//		InputStream mprInputStream = new ByteArrayInputStream(mprBuffer.toString().getBytes(StandardCharsets.UTF_8));
//		String text = "";
//		try
//		{
//			text = IOUtils.toString(mprInputStream, StandardCharsets.UTF_8.name());
//		}
//		catch (IOException e)
//		{
//			e.printStackTrace();
//		}
//
		List<GPSPosition> waypoints = this.buildMissionWaypointsFromInputStream(
				missionBoundaryGPSPositionList, mowingPathWidthInMeters, heading, startGPSPosition);

		System.out.println("Now we've built those beautiful cartesian positions...");
		return waypoints;
	}

	private void moveStartGPSPositionToBeginningOfList(List<GPSPosition> mission, GPSPosition startGPSPosition)
	{
		int startGPSPositionIndex = -1;
		for(int x = 0; x < mission.size(); x++)
		{
			if(startGPSPosition.equals(mission.get(x)))
			{
				startGPSPositionIndex = x;
				break;
			}
		}

		//Now that we know the index of the start position, pop the elements off the
		//head of the list and append them to the tail until we
		//get to the starting element
		for(int x = 0; x < startGPSPositionIndex; x++)
		{
			GPSPosition popMe = mission.remove(0);
			mission.add(popMe);
		}
	}

	/**
	 * The motivation for this method is if the user specifies a starting point
	 * outside of the mission polygon -- in this case we'll look for
	 * the closest point on the polygon and begin
	 * the mission at this point
	 *
	 * @param startPointUnchecked Location that we'll check for containment within polygon
	 * @param gpsPositionList List of points that make up the mission polygon
	 * @return If the polygon contains the given point, then simply return the
	 * point; otherwise, we'll look at all polygon vertices and
	 * return the vertex that's closest to the point
	 */
	public GPSPosition getClosestPolygonVertexIfPointNotWithinPolygon(GPSPosition startPointUnchecked, List<GPSPosition> gpsPositionList)
	{
		//Build a missionBoundaryPath2D for easy x,y point containment checking
		Path2D missionBoundaryPath2D = missionBoundary(gpsPositionList);
		if(missionBoundaryPath2D.contains(startPointUnchecked.x, startPointUnchecked.y))
		{
			//startPointUnchecked is within the mission boundary, so we can simply send
			//it back to the user as the starting point
			return startPointUnchecked;
		}
		else
		{
			//startPointUnchecked is not within the mission boundary, so we have to
			//loop through all mission vertices and find the vertex that's closest
			//to the startPointUnchecked and return it
			double distance = Double.MAX_VALUE;
			GPSPosition closestVertex = null;
			for(int x = 0; x < gpsPositionList.size(); x++)
			{
				double currentPointComparisonDistance = startPointUnchecked.distanceToAnotherGPSPositionViaXY(gpsPositionList.get(x));
				if(currentPointComparisonDistance < distance)
				{
					distance = currentPointComparisonDistance;
					closestVertex = gpsPositionList.get(x);
				}
			}
			return closestVertex;
		}

		//Old code:
//		if(PolyUtil.containsLocation(point, polygon, true))
//		{
//			return point;
//		}
//
//		double distanceBetweenPoints = Double.MAX_VALUE;
//		LatLng closestPoint = null;
//
//		for(LatLng vertex : polygon)
//		{
//			double thisDistance = SphericalUtil.computeDistanceBetween(point, vertex);
//			if(thisDistance < distanceBetweenPoints)
//			{
//				distanceBetweenPoints = thisDistance;
//				closestPoint = vertex;
//			}
//		}
//		return closestPoint;
	}

	/**
	 *
	 * Builds a string with (what I think is) the default format that ArduPilot expects
	 *
	 * @param index Index of waypoint
	 * @param gps Waypoint GPS location
	 * @return ArduPilot waypoint string
	 */
    public String buildWaypointString(int index, GPSPosition gps)
    {
    	return index + "\t0\t3\t16\t0\t0\t0\t0\t" + gps.getLatitude() + "\t" + gps.getLongitude() + "\t100.000000\t1";
    }
    
//    private void loadPositionsFromMissionPlanRecordingFromInputStream(InputStream inputStreamMissionPlanRecordingLines)
//    {
//    	try
//    	{
//        	// Open the file
//        	BufferedReader br = new BufferedReader(new InputStreamReader(inputStreamMissionPlanRecordingLines));
//
//        	String line;
//
//        	//Read file line by line
//        	while ((line = br.readLine()) != null)   {
//
//        		String[] vals = line.split(Constants.SERIALIZE_LINE_VAL_SEPARATOR);
//
//        		if(vals != null && vals.length > 0)
//        		{
//            		if(Constants.SERIALIZE_GUIDEPOINT_CARTESIAN_TYPE.equalsIgnoreCase(vals[0]))
//            		{
//            			/*
//            			 * Expected line format:
//            			 *
//            			 * GUIDEPOINT_CARTESIAN CARTESIAN X:-4.57 Y:-1.68 AUTOPILOT_TIME_MILLIS:126446 LOCAL_TIME_MILLIS:1487617141467
//            			 *
//            			 */
//            			this.guidePointCartesian = loadCartesianGuidepoint(vals);
//            		}
//            		else if(Constants.SERIALIZE_GUIDEPOINT_GPS_TYPE.equalsIgnoreCase(vals[0]))
//            		{
//            			/*
//            			 * Expected line format:
//            			 *
//            			 * GUIDEPOINT_GPS GPS LAT:30.5635939 LON:-87.6780064 FIX_TYPE:3
// 						 *
//            			 */
//            			this.guidePointGPS = loadGPSGuidepoint(vals);
//            		}
//            		else if(Constants.SERIALIZE_POINT_TYPE.equalsIgnoreCase(vals[0]))
//            		{
//            			/*
//            			 * Expected line format:
//            			 *
//            			 * POINT 199 CARTESIAN X:-1.12 Y:-0.89 AUTOPILOT_TIME_MILLIS:103865 LOCAL_TIME_MILLIS:1487617118875
//            			 * -----or-----
//            			 * POINT 201 GPS LAT:30.5636255 LON:-87.6780056 FIX_TYPE:3
//            			 *
//            			 */
//
//            			Position position = buildPositionFromMissionPlanRecordingLineArray(vals);
//            			this.addPosition(position);
//
//            		}
//        		}
//
//        	}
//
//        	//Close the input stream
//        	br.close();
//    	}
//    	catch(FileNotFoundException e1)
//    	{
//    		System.out.println("FileNotFoundException --> MissionDAO.java --> " + e1.toString());
//    	}
//    	catch(IOException e2)
//    	{
//    		System.out.println("IOException --> MissionDAO.java --> " + e2.toString());
//    	}
//    }
    
//    /*
//     * Expected line is of the form:
//     *
//	 * GUIDEPOINT_CARTESIAN CARTESIAN X:-4.57 Y:-1.68 AUTOPILOT_TIME_MILLIS:126446 LOCAL_TIME_MILLIS:1487617141467
//	 *
//     */
//    private CartesianPosition loadCartesianGuidepoint(String[] vals)
//    {
//    	String[] positionVals = Arrays.copyOfRange(vals, 2, vals.length);
//    	return CartesianPosition.buildFromKeyValArray(positionVals);
//    }
    
//    /*
//     * Expected line is of the form:
//     *
//	 *	GUIDEPOINT_GPS GPS LAT:30.5635939 LON:-87.6780064 FIX_TYPE:3
//	 *
//     */
//    private GPSPosition loadGPSGuidepoint(String[] vals)
//    {
//    	String[] positionVals = Arrays.copyOfRange(vals, 2, vals.length);
//    	return GPSPosition.buildFromKeyValArray(positionVals);
//    }
    
//    /*
//     * Expected array is of the form:
//     *
//     * [POINT] [32] [CARTESIAN] [X:-2.72] [Y:0.57] [AUTOPILOT_TIME_MILLIS:131559] [LOCAL_TIME_MILLIS:1487620496519]
//     *
//     * ------or------
//     *
//     * [POINT] [24] [GPS] [LAT:30.5636187] [LON:-87.6780116] [FIX_TYPE:3]
//	 *
//     */
//    private Position buildPositionFromMissionPlanRecordingLineArray(String[] vals)
//    {
//
//		Integer recordIndex = Integer.parseInt(vals[1]);
//    	PositionType positionType = guidePointTypeFromString(vals[2]);
//    	String[] positionVals = Arrays.copyOfRange(vals, 2, vals.length);
//
//    	Position position = new Position();
//    	position.setType(positionType);
//    	position.setRecordIndex(recordIndex);
//
//    	if(positionType == PositionType.Cartesian)
//    	{
//    		CartesianPosition cp = CartesianPosition.buildFromKeyValArray(positionVals);
//        	position.setCartesian(cp);
//    	}
//    	else if(positionType == PositionType.GPS)
//    	{
//    		GPSPosition gps = GPSPosition.buildFromKeyValArray(positionVals);
//        	position.setGps(gps);
//    	}
//
//    	return position;
//    }
    
//    private PositionType guidePointTypeFromString(String str)
//    {
//    	PositionType positionType =
//    			Constants.SERIALIZE_GPS_TYPE.equalsIgnoreCase(str) ? PositionType.GPS :
//    				Constants.SERIALIZE_CARTESIAN_TYPE.equalsIgnoreCase(str) ? PositionType.Cartesian : null;
//    	return positionType;
//    }
    
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
     * Returns direction (+1 or -1) to build mission from initial path.
     * Positive or Negative is just a standard to know how
     * to build the mission out.
     */
    private int directionToNavigateAfterGuideLine(Point2D.Double start, Point2D.Double guide, Path2D missionBoundary)
    {
    	int maxDistanceMetersToLookFromGuideLine = Config.maxMissionBoundaryMeters;
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
    
    public Path2D.Double missionBoundary(List<GPSPosition> missionBoundaryGPSPositionList)
    {
    	Path2D.Double missionBoundary = new Path2D.Double();
    	
    	//  loop trough all position points, starting at the beginning
    	//  and add Cartesian points to graph
    	
    	Boolean FIRST_LINE = true;
    	
    	for(int i = 0; i < missionBoundaryGPSPositionList.size(); i++)
    	{
    		GPSPosition position = missionBoundaryGPSPositionList.get(i);

			Double x = position.x;
			Double y = position.y;

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
    
    public Double getMinX(List<GPSPosition> missionBoundaryGPSPositionList)
    {
    	if(missionBoundaryGPSPositionList != null && missionBoundaryGPSPositionList.size() > 0)
    	{
    		Double minX = 0.0;
    		
    		for(GPSPosition position : missionBoundaryGPSPositionList)
    		{
				if(position.x < minX)
					minX = position.x;
    		}
    		return minX;
    	}
    	return null;
    }
    
    public Double getMinY(List<GPSPosition> missionBoundaryGPSPositionList)
    {
    	if(missionBoundaryGPSPositionList != null && missionBoundaryGPSPositionList.size() > 0)
    	{
    		Double minY = 0.0;
    		
    		for(GPSPosition position : missionBoundaryGPSPositionList)
    		{
				if(position.y < minY)
					minY = position.y;
    		}
    		return minY;
    	}
    	return null;
    }
    
    public Double getMaxX(List<GPSPosition> missionBoundaryGPSPositionList)
    {
    	if(missionBoundaryGPSPositionList != null && missionBoundaryGPSPositionList.size() > 0)
    	{
    		Double maxX = 0.0;
    		
    		for(GPSPosition position : missionBoundaryGPSPositionList)
    		{
				if(position.x > maxX)
					maxX = position.x;
    		}
    		return maxX;
    	}
    	return null;
    }
    
    public Double getMaxY(List<GPSPosition> missionBoundaryGPSPositionList)
    {
    	if(missionBoundaryGPSPositionList != null && missionBoundaryGPSPositionList.size() > 0)
    	{
    		Double maxY = 0.0;
    		
    		for(GPSPosition position : missionBoundaryGPSPositionList)
    		{
				if(position.y > maxY)
					maxY = position.y;
    		}
    		return maxY;
    	}
    	return null;
    }
    
    //Adjust Cartesian coordinates so that they all fall in the positive space
    //This is helpful for testing -- specifically graphing in java 
    public Double[] scaleMinXAndMinYToZero(List<GPSPosition> missionBoundaryGPSPositionList)
    {
    	Double minX = getMinX(missionBoundaryGPSPositionList);
    	Double minY = getMinY(missionBoundaryGPSPositionList);
    	
    	for(GPSPosition p : missionBoundaryGPSPositionList)
    	{
			p.x -= minX;
			p.y -= minY;
    	}

    	return new Double[] {minX, minY};
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

//    public List<Position> getPoints() {
//        return positions;
//    }

//    public void setPoints(List<Position> points) {
//        this.positions = points;
//    }
//
//    public CartesianPosition getGuidePointCartesian() {
//        return guidePointCartesian;
//    }
//
//    public void setGuidePointCartesian(CartesianPosition guidePointCartesian) {
//        this.guidePointCartesian = guidePointCartesian;
//    }
//
//    public GPSPosition getGuidePointGPS() {
//        return guidePointGPS;
//    }
//
//    public void setGuidePointGPS(GPSPosition guidePointGPS) {
//        this.guidePointGPS = guidePointGPS;
//    }

}
