package com.deepsouthrobotics.brain;

import com.deepsouthrobotics.data.Config;
import com.deepsouthrobotics.data.GPSCartesianCoordinateSpace;
import com.deepsouthrobotics.data.GPSPosition;
import com.deepsouthrobotics.data.GPSPositionArea;
import com.deepsouthrobotics.util.Geo;

import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
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
    public MissionBrain()
    {
    }

	/**
	 * This is the fancy look-ahead logic that takes a given edge in the mission and
	 * asks (and attempts to answer) the question "are there valid sections
	 * out in the distance beyond the current boundary?"
	 *
	 * @param missionBoundaryPath
	 * @param missionBoundaryPoints
	 * @param originalPointPath
	 * @param headingRadians
	 * @param minX
	 * @param maxX
	 * @param minY
	 * @param maxY
	 *
	 * @return List of Lists that are the additional valid mission points on this heading --
	 * truth be told, I'm not sure if there's any reason why I'm returning this as
	 * a List of Lists (i.e. rather than just flattening the guy rather
	 * than making the calling method do the flattinging)
	 */
    public List<List<Point2D.Double>> additionalValidMissionPointsOnTheGivenPointPathAndHeading(
			Path2D.Double missionBoundaryPath,
			List<GPSPosition> missionBoundaryPoints,
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

		//Save the Sine/Cosine values since we'll be using them often
		Double cos = Math.cos(headingRadians);
		Double sin = Math.sin(headingRadians);

		boolean pushed = false;

		//First push the start point 'till it's at the edge of
		//the missionBoundary on the given heading
		while(missionBoundaryPath.contains(endPointAtEdgeOfBoundary))
		{
			pushed = true;
			endPointAtEdgeOfBoundary.x += cos * .01;
			endPointAtEdgeOfBoundary.y += sin * .01;
		}

		//In the while... statement above we pushed the start point just beyond
		//the boundary, so lets bring it back to the last point within
		//the boundary and then begin looking for more valid points
		//on the same heading
		if(pushed)
		{
			endPointAtEdgeOfBoundary.x -= cos * .01;
			endPointAtEdgeOfBoundary.y -= sin * .01;
		}

		//Define a new variable "point" that we'll work with -- at first "point"
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
			Double xComponentAdd = cos * (Config.minMowingLineDistanceMeters-.01);
			Double yComponentAdd = sin * (Config.minMowingLineDistanceMeters-.01);
			point.x += xComponentAdd;
			point.y += yComponentAdd;
			if(missionBoundaryPath.contains(point))
			{
				//once we find a point within the polygon we're only part of the way to knowing
				//if this is a valid mission point -- in order to know if it's valid we have
				//to back up to the polygon edge where the line containing this point
				//begins -- and then push from that point 'till we find the end
				//of the line and figure out if the distance is sufficient
				//to consider this a valid mission line
				Point2D.Double newBeginPoint = new Point2D.Double(point.x, point.y);
				Double xCentimeterBump = cos * .01;
				Double yCentimeterBump = sin * .01;
				while(missionBoundaryPath.contains(newBeginPoint))
				{
					newBeginPoint.x -= xCentimeterBump;
					newBeginPoint.y -= yCentimeterBump;
				}
				newBeginPoint.x += xCentimeterBump;
				newBeginPoint.y += yCentimeterBump;
				Point2D.Double newEndPoint = polygonEdgePointByFollowingGivenStartingPointAndHeading(
						missionBoundaryPath, newBeginPoint, headingRadians);
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
						pointsFromOriginalPointToNewStartingPoint.add(new Point2D.Double(endPointAtEdgeOfBoundary.x, endPointAtEdgeOfBoundary.y));
					}
					tracePathAlongMissionBoundaryFromOnePointToAnotherPointAddingVerticesOfTheShortestPath(
							missionBoundaryPoints, endPointAtEdgeOfBoundary, newBeginPoint,
							pointsFromOriginalPointToNewStartingPoint);
					pointsFromOriginalPointToNewStartingPoint.add(new Point2D.Double(newEndPoint.x, newEndPoint.y));
					//Now pointsFromOriginalPointToNewStartingPoint is a big beautiful
					//List of points from the originalPointPath point that we called
					//this method with tracing around the boundary 'till the next
					//valid point start and then containing that point's end

					//So let's go ahead and add it to the List of Lists
					//that we will return
					listOfPointsLists.add(pointsFromOriginalPointToNewStartingPoint);

					//Now do some setup so we'll be able to see if there are
					//additional places to mow out in the distance...

					//We're just setting up the variables to mirror
					//how they are set up at the beginning
					//of this while loop
					endPointAtEdgeOfBoundary.x = newEndPoint.x;
					endPointAtEdgeOfBoundary.y = newEndPoint.y;

					//Do the logic to ensure we're at the end of the mission boundary
					//before we resume looking for new points out in the distance
					while(missionBoundaryPath.contains(endPointAtEdgeOfBoundary))
					{
						endPointAtEdgeOfBoundary.x += cos * .01;
						endPointAtEdgeOfBoundary.y += sin * .01;
					}
					//In the while... statement above we pushed the start point just beyond
					//the boundary, so lets bring it back to the last point within
					//the boundary and then begin looking for more valid points
					//on the same heading
					endPointAtEdgeOfBoundary.x -= cos * .01;
					endPointAtEdgeOfBoundary.y -= sin * .01;

					//Remember that the "point" variable is the pioneering variable that we push
					//out into the distance looking for valid mission points -- the
					//"endPointAtEdgeOfBoundary" is the edge of the boundary
					//where we last recorded a valid mission point
					point.x = endPointAtEdgeOfBoundary.x;
					point.y = endPointAtEdgeOfBoundary.y;
				}
			}
		}

		return listOfPointsLists;
	}

	/**
	 *
	 * @param point1 Point to check to see if it's a vertex in mission boundary
	 * @param missionBoundaryPoints mission boundary list
	 * @return Vertex index in the missionBoundary list if this point is a
	 * vertex -- if it's not a vertex, then return -1
	 */
	public int pointVertexIndex(Point2D.Double point1, List<? extends Point2D.Double> missionBoundaryPoints)
	{
		for(int x = 0; x < missionBoundaryPoints.size(); x++)
		{
			if(point1.x == missionBoundaryPoints.get(x).x &&
					point1.y == missionBoundaryPoints.get(x).y)
			{
				return x;
			}
		}
		return -1;
	}

	/**
	 * Baswell -- this method needs to be updated to address the situation where point1
	 * or point2 are a vertex (i.e. they are equal to some value in the
	 * missionBoundaryPoints List) -- that logic is not working
	 * but I've gotten away with it 'till now as this
	 * is sortof an edge case I think
	 *
	 * @param missionBoundaryPoints
	 * @param point1 The first point on the edge of the mission that we're looking
	 *               to chart a path along the mission perimeter from
	 * @param point2 The second point on the edge of the mission that we're looking
	 *               to chart a path along the mission perimeter to
	 * @param listOfPointsToAppendThePathTo After doing the fancy logic to find the
	 *               shortest perimeter path from point1 to point2, we append
	 *               the beautiful points of said path to this list
	 */
	public void tracePathAlongMissionBoundaryFromOnePointToAnotherPointAddingVerticesOfTheShortestPath(
			List<GPSPosition> missionBoundaryPoints,
			Point2D.Double point1,
			Point2D.Double point2,
			List<Point2D.Double> listOfPointsToAppendThePathTo
	)
	{
		//First off we need to find out EITHER 1. if point1
		//is a vertex on missionBoundary ...OR... 2.
		//which 2 vertices on the missionBoundary
		//that point1 lies between
		//List<Point2D.Double> missionBoundaryPoints = getPointsOnPath(missionBoundary);

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
					Double pointLineDistance = line.ptLineDist(point1);
					if(pointLineDistance <= .01)
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
					Double pointLineDistance = line.ptLineDist(point1);
					if(pointLineDistance <= .01)
					{
						point1LeftMissionBoundaryVertexIndex = x;
						point1RightMissionBoundaryVertexIndex = x+1;
						break;//we've found the line containing this point -- mission accomplished, so proceed
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
					Double pointLineDistance = line.ptLineDist(point2);
					if(pointLineDistance <= .01)
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
					Double pointLineDistance = line.ptLineDist(point2);
					if(pointLineDistance <= .01)
					{
						point2LeftMissionBoundaryVertexIndex = x;
						point2RightMissionBoundaryVertexIndex = x+1;
						break; //we've found the line containing this point -- mission accomplished, so proceed
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
						//with the loop -- we need to 1. add the distance from previousPoint
						//to the vertex at nextIndexLeft and 2. then from
						//the vertex at nextIndexLeft to point2

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
				//point1 is a vertex, but point2 lies between 2 vertices -- specifically,
				//point2 lies between point2LeftMissionBoundaryVertexIndex
				//and point2RightMissionBoundaryVertexIndex
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

					if(nextIndexRight == point2LeftMissionBoundaryVertexIndex ||
							nextIndexRight == point2RightMissionBoundaryVertexIndex)
					{
						//Next point would be a vertex of the line containing point2, so we're done
						//with the loop -- we need to 1. add the distance from previousPoint to
						//the vertex at nextIndexRight and 2. then add the distance
						//from the vertex at nextIndexRight to point2

						done = true;
						Point2D.Double rightBoundaryPoint = missionBoundaryPoints.get(nextIndexRight);
						directionRightDistance += nextPoint.distance(rightBoundaryPoint);
						directionRightDistance += rightBoundaryPoint.distance(point2);

						//we'll also need to add the rightBoundaryPoint to the rightPathVertices
						rightPathVertices.add(rightBoundaryPoint);
					}
				}
			}
		}
		else //point1 is NOT a vertex -- i.e. it lies between point1LeftMissionBoundaryVertexIndex
		{	 //and point1RightMissionBoundaryVertexIndex
			if(point2IsVertex)
			{
				//point1 lies between 2 vertices, but point2 is a vertex

				//First find the distance by moving from point1 to point2
				//by **subtracting** the vertices' index from point1 --
				//I'll call moving "left" -- here we move
				//along the boundary by subtracting
				//from the point1 index 'till
				//we reach point2's index
				int nextIndexLeft = point1LeftMissionBoundaryVertexIndex == 0 ? missionBoundaryPoints.size()-1 : point1LeftMissionBoundaryVertexIndex-1;
				Point2D.Double previousPoint = missionBoundaryPoints.get(point1LeftMissionBoundaryVertexIndex);
				leftPathVertices.add(previousPoint);
				directionLeftDistance += point1.distance(previousPoint);
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
				int nextIndexRight = point1RightMissionBoundaryVertexIndex == missionBoundaryPoints.size()-1 ? 0 : point1RightMissionBoundaryVertexIndex+1;
				previousPoint = missionBoundaryPoints.get(point1RightMissionBoundaryVertexIndex);
				rightPathVertices.add(previousPoint);
				directionRightDistance += point1.distance(previousPoint);
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
				//point1 and point2 both lie between 2 vertices

				//First find the distance by moving from point1 to point2
				//by **subtracting** the vertices' index from point1 --
				//I'll call moving "left" -- here we move
				//along the boundary by subtracting
				//from the point1 index 'till
				//we reach point2's index
				int currentIndexLeft = point1LeftMissionBoundaryVertexIndex;
				Point2D.Double currentPoint = missionBoundaryPoints.get(point1LeftMissionBoundaryVertexIndex);

				directionLeftDistance += point1.distance(currentPoint);
				boolean done = false;

				//there's a special case we need to take care of -- when there's only
				//1 point on the line between point1 and point2 then we
				//have a situation where point1 and point2 share
				//a vertex -- this "if" takes care of that
				if(point1LeftMissionBoundaryVertexIndex == point2LeftMissionBoundaryVertexIndex ||
						point1LeftMissionBoundaryVertexIndex == point2RightMissionBoundaryVertexIndex)
				{
					done = true;
					leftPathVertices.add(currentPoint);
					directionLeftDistance += currentPoint.distance(point2);
				}

				while(!done)
				{
					leftPathVertices.add(currentPoint);
					if(currentIndexLeft == point2LeftMissionBoundaryVertexIndex ||
							currentIndexLeft == point2RightMissionBoundaryVertexIndex)
					{
						//The next point is a vertex of the line containing point2, so we're done
						//with the loop -- we need to 1. add the distance from vertex to
						//point2 and 2. set done=true

						done = true;
						directionLeftDistance += currentPoint.distance(point2);
					}
					else
					{
						currentIndexLeft = currentIndexLeft == 0 ? missionBoundaryPoints.size()-1 : currentIndexLeft-1;
						Point2D.Double nextBoundaryPoint = missionBoundaryPoints.get(currentIndexLeft);
						Double currentToNextPointDistance = currentPoint.distance(nextBoundaryPoint);
						directionLeftDistance += currentToNextPointDistance;
						currentPoint = nextBoundaryPoint;
					}
				}

				//Now find the distance by moving from point1 to point2
				//by **adding** the vertices' index from point1
				//'till we reach point2's index

				int currentIndexRight = point1RightMissionBoundaryVertexIndex;
				currentPoint = missionBoundaryPoints.get(point1RightMissionBoundaryVertexIndex);

				directionRightDistance = point1.distance(currentPoint);

				done = false;

				if(point1RightMissionBoundaryVertexIndex == point2LeftMissionBoundaryVertexIndex ||
						point1RightMissionBoundaryVertexIndex == point2RightMissionBoundaryVertexIndex)
				{
					done = true;
					rightPathVertices.add(currentPoint);
					directionRightDistance += currentPoint.distance(point2);
				}

				while(!done)
				{
					rightPathVertices.add(currentPoint);

					if(currentIndexRight == point2LeftMissionBoundaryVertexIndex ||
							currentIndexRight == point2RightMissionBoundaryVertexIndex)
					{
						//The next point is a vertex of the line containing point2, so we're done
						//with the loop -- we need to 1. add the distance from vertex to
						//point2 and 2. set done=true

						done = true;
						directionRightDistance += currentPoint.distance(point2);
					}
					else
					{
						currentIndexRight = currentIndexRight == missionBoundaryPoints.size()-1 ? 0 : currentIndexRight+1;
						Point2D.Double nextBoundaryPoint = missionBoundaryPoints.get(currentIndexRight);
						Double currentToNextPointDistance = currentPoint.distance(nextBoundaryPoint);
						directionRightDistance += currentToNextPointDistance;
						currentPoint = nextBoundaryPoint;
					}
				}
			}
		}

		//Now that we've calculated the length of both the left and right paths
		//we follow the shorter path by appending it's points
		//to listOfPointsToAppendThePathTo
		if(directionLeftDistance <= directionRightDistance)
		{
			for(int x = 0; x < leftPathVertices.size(); x++)
			{
				listOfPointsToAppendThePathTo.add(new Point2D.Double(leftPathVertices.get(x).x, leftPathVertices.get(x).y));
			}
		}
		else
		{
			for(int x = 0; x < rightPathVertices.size(); x++)
			{
				listOfPointsToAppendThePathTo.add(new Point2D.Double(rightPathVertices.get(x).x, rightPathVertices.get(x).y));
			}
		}

		//in this method we assume that the calling code has already added
		//point1 to the listOfPointsToAppendThePathTo...but,
		//we do need to add point2 at the end
		listOfPointsToAppendThePathTo.add(new Point2D.Double(point2.x, point2.y));
	}

	/**
	 * 	Starting at the start point, move within the missionBoundary path
	 * 	in the direction specified by headingRadians 1 cm at a time
	 * 	'till we reach a point outside of missionBoundary
	 * @param missionBoundary Outer perimeter of our mission
	 * @param start Point at which to to begin pushing on the given heading
	 * @param headingRadians Heading (i.e. direction) to push toward in search of boundary
	 * @return Point at the edge of the missionMoundary beginning at the given
	 * start point on the given heading
	 */
    public Point2D.Double polygonEdgePointByFollowingGivenStartingPointAndHeading(
			Path2D missionBoundary, Point2D.Double start, Double headingRadians)
	{
		Double xComponentAdd = Math.cos(headingRadians) * .01;
		Double yComponentAdd = Math.sin(headingRadians) * .01;

		Double edgeX = start.x + xComponentAdd;
		Double edgeY = start.y + yComponentAdd;

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
														 List<GPSPosition> missionBoundaryGPSPositionList,
														 Double headingRadians)
	{
		Point2D.Double guide = polygonEdgePointByFollowingGivenStartingPointAndHeading(missionBoundary, start, headingRadians);
		//If line length is too short then adjust the starting location
		double distance = start.distance(guide);
		if (distance < Config.minMowingLineDistanceMeters)
		{
			int pointVertexIndex = pointVertexIndex(start, missionBoundaryGPSPositionList);
			//if the polygon is at a vertex, we start at the starting point vertex and move
			//toward the vertex on either side of it, and make a guess based
			//on the path ahead (as we move toward either vertex)

			if (pointVertexIndex != -1)
			{
				int minusVertexIndex; //vertex index to one side of the point
				int plusVertexIndex; //vertex index to the other side of the point

				if (pointVertexIndex == 0)
				{
					minusVertexIndex = missionBoundaryGPSPositionList.size() - 1;
					plusVertexIndex = 1;
				} else if (pointVertexIndex == missionBoundaryGPSPositionList.size() - 1)
				{
					minusVertexIndex = pointVertexIndex - 1;
					plusVertexIndex = 0;
				} else
				{
					minusVertexIndex = pointVertexIndex - 1;
					plusVertexIndex = pointVertexIndex + 1;
				}

				Point2D.Double minusVertex = missionBoundaryGPSPositionList.get(minusVertexIndex);
				Point2D.Double plusVertex = missionBoundaryGPSPositionList.get(plusVertexIndex);

				//Now that we know the "minus" and "plus" vertices (i.e. this is what we're calling
				//the vertices to either side of our point/vertex on the mission boundary) it's
				//time to move along either of them and see if we can find some fertile
				//ground

				//Get the point one half meter along the path to either vertex
				Point2D.Double minusNormHalfMeter = getParallelNorm(start, minusVertex, 0.5);
				Point2D.Double plusNormHalfMeter = getParallelNorm(start, plusVertex, 0.5);

				Point2D.Double minusNormVenturePoint = new Point2D.Double(
						start.x + minusNormHalfMeter.x, start.y + minusNormHalfMeter.y);

				Point2D.Double plusNormVenturePoint = new Point2D.Double(
						start.x + plusNormHalfMeter.x, start.y + plusNormHalfMeter.y);

				Point2D.Double minusNormCentimeter = getParallelNorm(start, minusVertex, 0.01);
				Point2D.Double plusNormCentimeter = getParallelNorm(start, plusVertex, 0.01);

				//baswell Friday -- taking out this missionBoundary.contains... check because I think we may be getting
				//some floating point precision issues where the minusNormVenturePoint or plusNormVenturePoint
				//are a sub-millimeters outside of the missionBoundary -- the method that works off of those
				//points (polygonEdgePointByFollowingGivenStartingPointAndHeading..) actually adds a
				//centimeter before beginning it's checks so it should get around this funkiness
				//(in other words, even though one of the "venture" points is technically
				//a few micrometers outside of the mission boundary right now,
				//by the time the polygonEdgePoint method begins checking
				//them within it's logic it's already added a
				//centimeter in the direction of the
				//user-requested heading
				//if(missionBoundary.contains(minusNormVenturePoint) && missionBoundary.contains(plusNormVenturePoint))
				//{
				//the only situation I can think of where the missionBoundary would not contain
				//both points 1 meter away would be where the distance between a point
				//and our vertex was less than 1 meter -- let's assume this isn't
				//a very likely scenario for now
				Point2D.Double minusNormEndPoint = polygonEdgePointByFollowingGivenStartingPointAndHeading(missionBoundary, minusNormVenturePoint, headingRadians);
				Point2D.Double plusNormEndPoint = polygonEdgePointByFollowingGivenStartingPointAndHeading(missionBoundary, plusNormVenturePoint, headingRadians);

				Double minusNormDistance = minusNormVenturePoint.distance(minusNormEndPoint);
				Double plusNormDistance = plusNormVenturePoint.distance(plusNormEndPoint);

				if (plusNormDistance >= minusNormDistance)
				{
					//Plus norm was longer, so let's assume this is the point to push toward to
					//find a good starting point
					if (plusNormDistance > Config.minMowingLineDistanceMeters)
					{
						//Line length is too long, so let's pull the new starting point back toward the original
						//starting point 'till we get to line length of Config.minMowingLineDistanceMeters
						boolean adjusted = false;
						while (plusNormDistance > Config.minMowingLineDistanceMeters)
						{
							adjusted = true;
							plusNormVenturePoint.x -= plusNormCentimeter.x;
							plusNormVenturePoint.y -= plusNormCentimeter.y;
							plusNormEndPoint = polygonEdgePointByFollowingGivenStartingPointAndHeading(missionBoundary, plusNormVenturePoint, headingRadians);
							plusNormDistance = plusNormVenturePoint.distance(plusNormEndPoint);
						}
						if (adjusted)
						{
							plusNormVenturePoint.x += plusNormCentimeter.x;
							plusNormVenturePoint.y += plusNormCentimeter.y;
						}
						start.x = plusNormVenturePoint.x;
						start.y = plusNormVenturePoint.y;
					} else
					{
						//Line length is too short, so let's push the new starting point back
						//toward the plusNormEndPoint 'till we get to line length
						//of Config.minMowingLineDistanceMeters
						while (plusNormDistance < Config.minMowingLineDistanceMeters)
						{
							plusNormVenturePoint.x += plusNormCentimeter.x;
							plusNormVenturePoint.y += plusNormCentimeter.y;
							plusNormEndPoint = polygonEdgePointByFollowingGivenStartingPointAndHeading(missionBoundary, plusNormVenturePoint, headingRadians);
							plusNormDistance = plusNormVenturePoint.distance(plusNormEndPoint);
						}
						start.x = plusNormVenturePoint.x;
						start.y = plusNormVenturePoint.y;
					}
				} else
				{
					//Minus norm was longer, so let's assume this is the point to push toward to
					//find a good starting point
					if (minusNormDistance > Config.minMowingLineDistanceMeters)
					{
						//Line length is too long, so let's pull the new starting point back toward the original
						//starting point 'till we get to line length of Config.minMowingLineDistanceMeters
						while (minusNormDistance > Config.minMowingLineDistanceMeters)
						{
							minusNormVenturePoint.x -= minusNormCentimeter.x;
							minusNormVenturePoint.y -= minusNormCentimeter.y;
							minusNormEndPoint = polygonEdgePointByFollowingGivenStartingPointAndHeading(missionBoundary, minusNormVenturePoint, headingRadians);
							minusNormDistance = minusNormVenturePoint.distance(minusNormEndPoint);
						}
						start.x = minusNormVenturePoint.x;
						start.y = minusNormVenturePoint.y;
					} else
					{
						//Line length is too short, so let's push the new starting point back
						//toward the minusNormEndPoint 'till we get to line length
						//of Config.minMowingLineDistanceMeters
						while (minusNormDistance < Config.minMowingLineDistanceMeters)
						{
							minusNormVenturePoint.x += minusNormCentimeter.x;
							minusNormVenturePoint.y += minusNormCentimeter.y;
							minusNormEndPoint = polygonEdgePointByFollowingGivenStartingPointAndHeading(missionBoundary, minusNormVenturePoint, headingRadians);
							minusNormDistance = minusNormVenturePoint.distance(minusNormEndPoint);
						}
						start.x = minusNormVenturePoint.x;
						start.y = minusNormVenturePoint.y;
					}
				}
			//}
			//this is the termination of the mission boundary contains check we did above then
				//commented out for reasons explained above -- it's likely safe
				//to remove this code by the time anyone reads this
			//else
			//{
			//	System.out.println("Mission boundary didn't contain both minus and plus norms, so you may end up with" +
			//			" a bizarre mission path");
			//}
		}
		else //the starting point is within the polygon, so the logic to adjust the starting point is slightly different
		{
			//Add some x,y value that's perpendicular to the start value --
			//find the perpendicular value out by adding then subtracting
			//pi/2 to the headingRadians value and then using that
			//value to get x/y components to add to the start
			//value and seeing if it's within
			//the missionBoundary
			Double perpHeading = headingRadians + Math.PI / 2;

			Double xComponentPerp = Math.cos(perpHeading) * .01;
			Double yComponentPerp = Math.sin(perpHeading) * .01;

			Point2D.Double perpPoint = new Point2D.Double(start.x + xComponentPerp, start.y + yComponentPerp);
			if (missionBoundary.contains(perpPoint))
			{
				start.x = perpPoint.x;
				start.y = perpPoint.y;
			} else
			{
				perpPoint = new Point2D.Double(start.x - xComponentPerp, start.y - yComponentPerp);
				if (missionBoundary.contains(perpPoint))
				{
					start.x = perpPoint.x;
					start.y = perpPoint.y;
				} else
				{
					//Well, the perpendicular points on either side of the start line aren't
					//in the mission polygon so i think something is pretty
					//jacked up
				}
			}

			guide = polygonEdgePointByFollowingGivenStartingPointAndHeading(missionBoundary, start, headingRadians);
			int navigateDirection = directionToNavigateAfterGuideLine(start, guide, missionBoundary);

			while (start.distance(guide) < Config.minMowingLineDistanceMeters)
			{
				Point2D.Double normPerpXY = getPerpendicularNorm(start, guide, .01);
				start.x += normPerpXY.x * navigateDirection;
				start.y += normPerpXY.y * navigateDirection;
				guide = polygonEdgePointByFollowingGivenStartingPointAndHeading(missionBoundary, start, headingRadians);
				Double theDistance = start.distance(guide);
				System.out.println("Hello friends");
			}
		}
	}
	}

	public List<GPSPosition> flattenListOfPoint2DListsThenConvertToGPSPositionList(
			List<List<Point2D.Double>> additionalPointsOnThisHeading,
			GPSCartesianCoordinateSpace space

	)
	{
		List<GPSPosition> gpsPoints = new ArrayList<>();

		if(additionalPointsOnThisHeading != null)
		{
			for(int x = 0; x < additionalPointsOnThisHeading.size(); x++)
			{
				List<Point2D.Double> points = additionalPointsOnThisHeading.get(x);
				if(points != null)
				{
					for(int y = 0; y < points.size(); y++)
					{
						Point2D.Double point = points.get(y);
						gpsPoints.add(space.gpsPositionGivenDistanceFromZeroZero(point.x, point.y));
					}
				}
			}
		}
		return gpsPoints;
	}


	public void circumventObstaclesBetweenTwoPointsAndAddTheGeneratedPointsToTheMission(
								GPSCartesianCoordinateSpace space,
								List<GPSPosition> missionWaypoints,
								GPSPosition startGPSPosition,
								GPSPosition endGPSPosition,
								Path2D.Double missionBoundary,
								List<GPSPositionArea> polyObstaclesGPSPositionAreaList)
	{
		//Get a list of the obstacles intersected by the line formed
		//by startGPSPosition and endGPSPosition
		List<GPSPositionArea> intersectedObstacles = obstaclesThatLineIntersects(
				startGPSPosition, endGPSPosition, polyObstaclesGPSPositionAreaList);
		if(intersectedObstacles.size() > 0)
		{
			//If we get here we know that the line formed by startGPSPosition
			//and endGPSPosition intersects at least 1 obstacle
			Point2D.Double normMeter = getParallelNorm(startGPSPosition, endGPSPosition, 1.0);
			Point2D.Double normCM = getParallelNorm(startGPSPosition, endGPSPosition, 1.0);

			Double startToEndDistance = startGPSPosition.distance(endGPSPosition);

			Point2D.Double flexibleVenturePoint = new Point2D.Double(startGPSPosition.x, startGPSPosition.y);

			//We do this check on the distance between our starting point and the
			//flexibleVenturePoint to see if we've pushed
			//beyond the endGPSPosition
			while(startGPSPosition.distance(flexibleVenturePoint) < startToEndDistance)
			{
				for(int x = 0; x < intersectedObstacles.size(); x++)
				{
					GPSPositionArea obstacle = intersectedObstacles.get(x);
					if(obstacle.contains(flexibleVenturePoint))
					{
						//High level steps of what we're trying to do:
						//
						//if the obstacle contains the venture point we want to:
						//
						//1. Back up the start point of flexibleVenturePoint to the beginning of the obstacle and record that value
						//2. Push flexibleVenturePoint 'till it's outside of obstacle and record the endpoint
						//3. Build the route the start and end point based on the shortest path following the obstacle perimeter
						//4. Record those points we generated in 1-3 above to missionWaypoints
						//5. Somehow flexibleVenturePoint 1cm beyond the end point of the current obstacle so that the next iteration of this while loop is ready

						Point2D.Double obstacleStartPoint = new Point2D.Double(flexibleVenturePoint.x, flexibleVenturePoint.y);
						do
						{
							obstacleStartPoint.x -= normCM.x;
							obstacleStartPoint.y -= normCM.y;
						}
						while(obstacle.contains(obstacleStartPoint));

						obstacleStartPoint.x += normCM.x;
						obstacleStartPoint.y += normCM.y;

						//obstacleStartPoint is the point on the edge of the obstacle where we'll start

						Point2D.Double obstacleEndPoint = new Point2D.Double(flexibleVenturePoint.x, flexibleVenturePoint.y);
						do
						{
							obstacleEndPoint.x += normCM.x;
							obstacleEndPoint.y += normCM.y;
						}
						while(obstacle.contains(obstacleEndPoint));

						obstacleEndPoint.x -= normCM.x;
						obstacleEndPoint.y -= normCM.y;

						//obstacleEndPoint is the point on the edge of the obstacle where we'll end the avoidance dance

						//Now we need to build the shortest path between obstacleStartPoint and obstacleEndPoint
						//by following the perimeter either way and returning
						//the points of the shortest path
						List<Point2D.Double> pointsAroundObstacle = new ArrayList<>();
						pointsAroundObstacle.add(new Point2D.Double(obstacleStartPoint.x, obstacleStartPoint.y));
						tracePathAlongMissionBoundaryFromOnePointToAnotherPointAddingVerticesOfTheShortestPath(
								obstacle.positions,
								obstacleStartPoint,
								obstacleEndPoint,
								pointsAroundObstacle
						);

						//Now convert the pointsAroundObstacle list to GPSPosition objects and
						//append to the missionWaypoints list
						for(int y = 0; y < pointsAroundObstacle.size(); y++ )
						{
							Point2D.Double point = pointsAroundObstacle.get(y);
							missionWaypoints.add(space.gpsPositionGivenDistanceFromZeroZero(point.x, point.y));
						}

						//push flexibleVenturePoint to 1cm beyond obstacleEndPoint
						flexibleVenturePoint.x = obstacleEndPoint.x + normCM.x;
						flexibleVenturePoint.y = obstacleEndPoint.y + normCM.y;

						//baswell after lunch/shop break -- I think we're ready to test this baby
						break;
					}
					else
					{
						if(x == (intersectedObstacles.size() - 1))
						{
							//At this point we know that this point isn't within
							//any obstacles to let's add a meter to
							//the flexibleVenturePoint and keep
							//looping (i.e. looking for
							//points within obstacles)
							flexibleVenturePoint.x += normMeter.x;
							flexibleVenturePoint.y += normMeter.y;
						}
					}
				}
			}
		}
	}

	public List<GPSPositionArea> obstaclesThatLineIntersects(
								GPSPosition startGPSPosition,
								GPSPosition endGPSPosition,
								List<GPSPositionArea> polyObstaclesGPSPositionAreaList
	)
	{
		List<GPSPositionArea> intersectedPolyObstaclesGPSPositionAreaList = new ArrayList<>();

		for (GPSPositionArea obstacleGPSPositionArea : polyObstaclesGPSPositionAreaList)
		{
			for(int x = 0; x < obstacleGPSPositionArea.positions.size(); x++)
			{
				GPSPosition obstacleLineStart = obstacleGPSPositionArea.positions.get(x);
				GPSPosition obstacleLineEnd;
				if(x == (obstacleGPSPositionArea.positions.size() - 1 ))
				{
					//This is the end of the list, so we check if the
					//line intersects the line between this
					//position and the position at the
					//beginning of the list
					obstacleLineEnd = obstacleGPSPositionArea.positions.get(0);
				}
				else
				{
					obstacleLineEnd = obstacleGPSPositionArea.positions.get(x+1);
				}
				Line2D.Double obstacleLine = new Line2D.Double(obstacleLineStart.x, obstacleLineStart.y, obstacleLineEnd.x, obstacleLineEnd.y);
				Line2D.Double pathLine = new Line2D.Double(startGPSPosition.x, startGPSPosition.y, endGPSPosition.x, endGPSPosition.y);
				if(obstacleLine.intersectsLine(pathLine))
				{
					intersectedPolyObstaclesGPSPositionAreaList.add(obstacleGPSPositionArea);
					break;
				}
			}
		}

		return intersectedPolyObstaclesGPSPositionAreaList;
	}


	/**
	 * One of the main methods that orchestrates a lot of the heavy lifting for building out
	 * the mission waypoints
	 *
	 * @param missionBoundaryGPSPositionList
	 * @param mowingPathWidthInMeters
	 * @param headingDegrees
	 * @param startGPSPosition
	 * @return
	 */
    public List<GPSPosition> buildMissionWaypoints(List<GPSPosition> missionBoundaryGPSPositionList,
															   Double mowingPathWidthInMeters,
															   Double headingDegrees,
															   GPSPosition startGPSPosition,
												               List<List<GPSPosition>> polyObstaclesGPSPositionList)
    {
		Double[] minXandMinY = scaleMinXAndMinYToZero(missionBoundaryGPSPositionList, polyObstaclesGPSPositionList);

		//Convert the list of obstacle points into a form we'll use later
		List<GPSPositionArea> polyObstaclesGPSPositionAreaList = new ArrayList<>();
		for(List<GPSPosition> polyObstaclesGPSPositions : polyObstaclesGPSPositionList)
		{
			GPSPositionArea gpsPositionArea = new GPSPositionArea(polyObstaclesGPSPositions);
			polyObstaclesGPSPositionAreaList.add(gpsPositionArea);
		}
		//Just for the record, the scenario where startGPSPosition wouldn't be the
		//first element in the list is where startGPSPosition is where
		//the user has dragged the start marker inside the
		//missionBoundaryGPSPositionList polygon

		startGPSPosition.x -= minXandMinY[0];
		startGPSPosition.y -= minXandMinY[1];

		//Go ahead and pull the min x, max x, min y and max y values --
		//we'll use these to hopefully save a few cpu
		//cycles when building the mission

		Double minX = getMinX(missionBoundaryGPSPositionList); // should always be zero
		Double maxX = getMaxX(missionBoundaryGPSPositionList);

		Double minY = getMinY(missionBoundaryGPSPositionList); // should always be zero
		Double maxY = getMaxY(missionBoundaryGPSPositionList);

		//Not entirely sure we need this space variable --
		//the idea is you're passing in some coordinate
		//other than the home coordinate --
		//before we got rid of "GuidePoint" stuff we were
		//passing in the guidepoint coordinate
		GPSCartesianCoordinateSpace space = new GPSCartesianCoordinateSpace(missionBoundaryGPSPositionList.get(1));

		Double headingRadians = Math.toRadians(headingDegrees);
        Path2D.Double missionBoundary = missionBoundary(missionBoundaryGPSPositionList);

		adjustStartingPointIfFirstLineIsTooShort(startGPSPosition, missionBoundary, missionBoundaryGPSPositionList, headingRadians);
		startGPSPosition = space.gpsPositionGivenDistanceFromZeroZero(startGPSPosition.x, startGPSPosition.y);

		//Build a "guidepoint" to shoe-horn into our old code/logic --
		//the guidepoint is the endpoint of our first mission line --
		//in other words, it's the second waypoint in our mission (i.e.
		//the startGPSPosition is the first waypoint)
		//baswell remove Monday
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
    	normParallel.x *= -1;
    	normParallel.y *= -1;
    	
    	//Convenient way to push/pull the mission line opposite direction of above
    	Point2D.Double normParallelNegative = new Point2D.Double();
    	normParallelNegative.x  = normParallel.x * -1;
    	normParallelNegative.y  = normParallel.y * -1;

		Point2D.Double currentTopPoint = new Point2D.Double(guide.x, guide.y);
		Point2D.Double currentBottomPoint = new Point2D.Double(startGPSPosition.x, startGPSPosition.y);

		Point2D.Double lastBottomPoint = new Point2D.Double(currentBottomPoint.x, currentBottomPoint.y);

		GPSPosition lastBottomGPS = new GPSPosition(startGPSPosition.latitude,
				startGPSPosition.longitude, startGPSPosition.x, startGPSPosition.y);//this.getStartGPS();
		
		Geo geo = new Geo();
		
    	//The first mission point will always be the start position
    	missionWaypoints.add(startGPSPosition);
		GPSPosition adjustedGuideGPS = space.gpsPositionGivenDistanceFromZeroZero(currentTopPoint.x, currentTopPoint.y);
		missionWaypoints.add(adjustedGuideGPS);

		//baswell begin new fancy look-beyond-boundary logic
		List<List<Point2D.Double>> additionalPointsOnThisHeading = additionalValidMissionPointsOnTheGivenPointPathAndHeading(
				missionBoundary,
				missionBoundaryGPSPositionList,
				adjustedGuideGPS,
				headingRadians,
				minX,
				maxX,
				minY,
				maxY
		);

		List<GPSPosition> flatPoints = flattenListOfPoint2DListsThenConvertToGPSPositionList(
				additionalPointsOnThisHeading, space);
		if(flatPoints.size() > 0)
		{
			for(int x = 0; x < flatPoints.size(); x++)
			{
				missionWaypoints.add(flatPoints.get(x));
			}
		}
		//baswell end new fancy look-beyond-boundary logic

		Point2D.Double lastMissionWaypoint = missionWaypoints.get(missionWaypoints.size()-1);

    	//The second mission point will always be the guide point adjusted to ensure next turn is within boundary
		Boolean ADD_ANOTHER_PATH_LINE = adjustTurnInitiationPointSoThatNextOrthogonalPointIsWithinBoundary(currentBottomPoint, lastMissionWaypoint, normPerpXY, missionBoundary);

		//Set up the currentTopPoint/lastTopPoint variables to make the mission-waypoint-adding logic below work as intended
		currentTopPoint = new Point2D.Double(lastMissionWaypoint.x, lastMissionWaypoint.y);
		Point2D.Double lastTopPoint = new Point2D.Double(currentTopPoint.x, currentTopPoint.y);
		GPSPosition lastTopGPS = space.gpsPositionGivenDistanceFromZeroZero(lastTopPoint.x, lastTopPoint.y);
		missionWaypoints.set(missionWaypoints.size()-1, lastTopGPS);//Replace top position with adjusted position

		for(int i = 1; ADD_ANOTHER_PATH_LINE; i++)
		{
			if(i%2 != 0) //we are at the top -- so set the top point to the perpendicular offset and push the bottom point to missionBoundary
			{
				currentTopPoint.x += normPerpXY.x;
				currentTopPoint.y += normPerpXY.y;
				
				currentBottomPoint.x = currentTopPoint.x;
				currentBottomPoint.y = currentTopPoint.y;

				pushLineToBoundary(currentTopPoint, currentBottomPoint, normParallel, missionBoundary);

				List<List<Point2D.Double>> additionalPoints = additionalValidMissionPointsOnTheGivenPointPathAndHeading(
						missionBoundary,
						missionBoundaryGPSPositionList,
						currentBottomPoint,
						headingRadians+Math.PI,
						minX,
						maxX,
						minY,
						maxY
				);

				GPSPosition firstLineStopPoint = space.gpsPositionGivenDistanceFromZeroZero(currentBottomPoint.x, currentBottomPoint.y);

				flatPoints = flattenListOfPoint2DListsThenConvertToGPSPositionList(
						additionalPoints, space);

				if(flatPoints.size()> 0)
				{
					lastMissionWaypoint = flatPoints.get(flatPoints.size()-1);
					currentBottomPoint = new Point2D.Double(lastMissionWaypoint.x, lastMissionWaypoint.y);
				}

				ADD_ANOTHER_PATH_LINE = adjustTurnInitiationPointSoThatNextOrthogonalPointIsWithinBoundary(currentTopPoint, currentBottomPoint, normPerpXY, missionBoundary);

				if(ADD_ANOTHER_PATH_LINE)
				{
					GPSPosition newTopGPS = space.gpsPositionGivenDistanceFromZeroZero(currentTopPoint.x, currentTopPoint.y);

					missionWaypoints.add(newTopGPS);
					System.out.println(i+"Top "+ newTopGPS.latitude + " " + newTopGPS.longitude + " Distance: " + geo.latLongDistance(lastTopGPS.latitude, lastTopGPS.longitude, newTopGPS.latitude, newTopGPS.longitude));

					lastTopGPS = newTopGPS;
					
					if(Math.abs(currentTopPoint.distance(currentBottomPoint)) >= Config.minMowingLineDistanceMeters)
					{
						//baswell -- begin Add check for obstacles between newTopGPS and firstLineStopPoint
						circumventObstaclesBetweenTwoPointsAndAddTheGeneratedPointsToTheMission(
								space,
								missionWaypoints,
								newTopGPS,
								firstLineStopPoint,
								missionBoundary,
								polyObstaclesGPSPositionAreaList);
						//baswell -- end Add check for obstacles between newTopGPS and firstLineStopPoint
						missionWaypoints.add(firstLineStopPoint);


						if(flatPoints.size() > 0)
						{
							for(int x = 0; x < flatPoints.size()-1; x++)
							{
								missionWaypoints.add(flatPoints.get(x));
							}
						}

						//up in the adjustTurnInitiationPoint... method above we do some karate on the last
						//point (i.e. currentBottomPoint) -- so we want add the karateified point
						//to the missionWaypoints list

						missionWaypoints.add(space.gpsPositionGivenDistanceFromZeroZero(currentBottomPoint.x, currentBottomPoint.y));
						lastBottomGPS = space.gpsPositionGivenDistanceFromZeroZero(currentBottomPoint.x, currentBottomPoint.y);
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

				List<List<Point2D.Double>> additionalPoints = additionalValidMissionPointsOnTheGivenPointPathAndHeading(
						missionBoundary,
						missionBoundaryGPSPositionList,
						currentTopPoint,
						headingRadians,
						minX,
						maxX,
						minY,
						maxY
				);

				GPSPosition firstLineStopPoint = space.gpsPositionGivenDistanceFromZeroZero(currentTopPoint.x, currentTopPoint.y);

				flatPoints = flattenListOfPoint2DListsThenConvertToGPSPositionList(
						additionalPoints, space);

				if(flatPoints.size()> 0)
				{
					lastMissionWaypoint = flatPoints.get(flatPoints.size()-1);
					currentTopPoint = new Point2D.Double(lastMissionWaypoint.x, lastMissionWaypoint.y);
				}

				ADD_ANOTHER_PATH_LINE = adjustTurnInitiationPointSoThatNextOrthogonalPointIsWithinBoundary(currentBottomPoint, currentTopPoint, normPerpXY, missionBoundary);
				
				if(ADD_ANOTHER_PATH_LINE)
				{
					GPSPosition newBottomGPS = space.gpsPositionGivenDistanceFromZeroZero(currentBottomPoint.x, currentBottomPoint.y);
					missionWaypoints.add(newBottomGPS);
					System.out.println(i+"Bottom lat:"+ newBottomGPS.latitude + " lon:" + newBottomGPS.longitude + " x: " + newBottomGPS.x + " y:" + newBottomGPS.y +" Distance: " + geo.latLongDistance(lastBottomGPS.latitude, lastBottomGPS.longitude, newBottomGPS.latitude, newBottomGPS.longitude));
					lastBottomGPS = newBottomGPS;
					
					if(Math.abs(currentTopPoint.distance(currentBottomPoint)) >= Config.minMowingLineDistanceMeters)
					{
						GPSPosition newTopGPS = space.gpsPositionGivenDistanceFromZeroZero(currentTopPoint.x, currentTopPoint.y);

						missionWaypoints.add(firstLineStopPoint);

						if(flatPoints.size() > 0)
						{
							for(int x = 0; x < flatPoints.size()-1; x++)
							{
								missionWaypoints.add(flatPoints.get(x));
							}
						}

						//up in the adjustTurnInitiationPoint... method above we do some karate on the last
						//point (i.e. currentTopPoint) -- so we want add the karateified point
						//to the missionWaypoints list
						missionWaypoints.add(space.gpsPositionGivenDistanceFromZeroZero(currentTopPoint.x, currentTopPoint.y));

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
		//Now figure out if the startLatLng is within the mission polygon -- if it is
		//then that will be the mission starting point -- if it's not then
		//find the nearest vertex on the mission polygon to the point
		//and set the point's LatLng to that vertex -- in
		//other words we'll begin building the
		//mission at that vertex

		//Just to have somewhere to start, let's call the (x,y) coordinate of
		//our startGPSPositionUnchecked point (0,0)
		startGPSPositionUnchecked.x = 0;
		startGPSPositionUnchecked.y = 0;

		//In this loop we find the distance from the startGPSPositionUnchecked to every vertex
		//on the mission boundary and we set the (x,y) coordinates of those points
		//as their (x,y) distance from startGPSPositionUnchecked
		for(int x = 0; x < missionBoundaryGPSPositionList.size(); x++)
		{
			setXLatandYLngMetersByDiffingLatAndLonDistanceFromGPSPosition(
					startGPSPositionUnchecked, missionBoundaryGPSPositionList.get(x));
		}

		GPSPosition startGPSPosition = getClosestPolygonVertexIfPointNotWithinPolygon(startGPSPositionUnchecked, missionBoundaryGPSPositionList);

		//If the start point wasn't within the mission boundary polygon (and we thus
		//obtained a startGPSPosition point that's equal to the closest
		//vertex to startGPSPositionUnchecked), then we'll rearrange
		//the mission boundary list to put that vertex at the
		//beginning and then we'll call that vertex (0,0)
		//and set the (x,y) coordinates of the other
		//vertices as their distance from the
		//startGPSPosition vertex -- at
		//some point there was a reason
		//for this logic but I'm really
		//not sure if it's still needed
		if(startGPSPosition.compareTo(startGPSPositionUnchecked) != 0)
		{
			//If startGPSPosition is one of the mission boundary polygon vertices,
			//this moves it to the beginning of the list
			moveStartGPSPositionToBeginningOfList(missionBoundaryGPSPositionList, startGPSPosition);

			startGPSPosition.x = missionBoundaryGPSPositionList.get(0).x = 0;
			startGPSPosition.y = missionBoundaryGPSPositionList.get(0).y = 0;

			for(int x = 1; x < missionBoundaryGPSPositionList.size(); x++)
			{
				setXLatandYLngMetersByDiffingLatAndLonDistanceFromGPSPosition(
						startGPSPosition, missionBoundaryGPSPositionList.get(x));
			}

			for(List<GPSPosition> obstacleGPSPositionList: polyObstaclesGPSPositionList)
			{
				for(int x = 0; x < obstacleGPSPositionList.size(); x++)
				{
					setXLatandYLngMetersByDiffingLatAndLonDistanceFromGPSPosition(
							startGPSPosition, obstacleGPSPositionList.get(x));
				}
			}
		}
		else
		{
			for(List<GPSPosition> obstacleGPSPositionList: polyObstaclesGPSPositionList)
			{
				for(int x = 0; x < obstacleGPSPositionList.size(); x++)
				{
					setXLatandYLngMetersByDiffingLatAndLonDistanceFromGPSPosition(
							startGPSPositionUnchecked, obstacleGPSPositionList.get(x));
				}
			}
		}

		//Note that we've gotta have at least 3 coordinates in order for any of
		//this logic to make sense (i.e. if you've only got 2 points
		//then you've got a line, which doesn't have area and
		//won't contain waypoints by definition)

		List<GPSPosition> waypoints = this.buildMissionWaypoints(
				missionBoundaryGPSPositionList, mowingPathWidthInMeters, heading,
				startGPSPosition, polyObstaclesGPSPositionList);

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
		//Build a missionBoundaryPath2D for easy (x,y) point containment checking
		Path2D missionBoundaryPath2D = missionBoundary(gpsPositionList);
		if(missionBoundaryPath2D.contains(startPointUnchecked.x, startPointUnchecked.y))
		{
			//startPointUnchecked is within the mission boundary, so we can simply send
			//it back to the user as the starting point
			return (GPSPosition)startPointUnchecked.clone();
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
			return (GPSPosition)closestVertex.clone();
		}
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

    /**
     * Returns FALSE if we didn't adjust the line for the next orthogonal point since doing so would have
     * made the current line's value less than Config.minMowingLineDistanceMeters. In other words,
     * if we get back FALSE from this function, it can be assumed that we've reached
     * the end of the mission and should not build any further lines.
	 *
	 * @param lineStartPoint
	 * @param lineTurnPoint
	 * @param normPerp
	 * @param missionBoundary
     */
    private Boolean adjustTurnInitiationPointSoThatNextOrthogonalPointIsWithinBoundary(
    		Point2D.Double lineStartPoint, Point2D.Double lineTurnPoint,
			Point2D.Double normPerp, Path2D missionBoundary)
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
    public Double[] scaleMinXAndMinYToZero(List<GPSPosition> missionBoundaryGPSPositionList, List<List<GPSPosition>> polyObstaclesGPSPositionList)
    {
    	Double minX = getMinX(missionBoundaryGPSPositionList);
    	Double minY = getMinY(missionBoundaryGPSPositionList);
    	
    	for(GPSPosition p : missionBoundaryGPSPositionList)
    	{
			p.x -= minX;
			p.y -= minY;
    	}

		for(List<GPSPosition> obstacleGPSPositionList: polyObstaclesGPSPositionList)
		{
			for(GPSPosition p : obstacleGPSPositionList)
			{
				p.x -= minX;
				p.y -= minY;
			}
		}

    	return new Double[] {minX, minY};
    }
}
