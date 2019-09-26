package com.deepsouthrobotics.api;

import com.deepsouthrobotics.brain.MissionBrain;
import com.deepsouthrobotics.data.GPSPosition;
import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import javax.ws.rs.*;
import javax.ws.rs.core.MediaType;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

/**
|--------------------------------------------------------------------------
| Copyright (C) Wayne Baswell 2019 -- GPL version 3 or later
|
| Servlet API
|--------------------------------------------------------------------------
|
| These methods are the entry point into the application -- handling
| incoming HTTP requests from the outside world
*/
@Path("buildMissionFromLatLngPoints")
public class BuildMissionFromLatLngPoints
{
    /**
     *
     * @param latLonJsonInputStream Input stream that's sent in as the HTTP
     * POST body. The format of the data is this:
     * [{"lat":30.564095635277162,"lng":-87.67660153473776},{"lat":30.56424575559707,"lng":-87.6766632255451} ... ]
     * Note that each lat/long pair represents a vertex on the polygon that
     * we're filling with waypoints.
     * @param mowingPathWidthInMeters The width in meters that will separate
     * the waypoints -- so, for example, if you have a mowing deck that's
     * 50cm wide and you want to allow 10cm of overlap, then you'll want
     * 40cm between your waypoints and you'll pass in 0.4 for this value
     * @return
     * json array of lat/long values that are the mission waypoint
     * coordinates -- data is in the same format as POST body
     * input, i.e.:
     * [{"lng":-87.67663065492417,"lat":30.564106707423818},{"lng":-87.67665345464492,"lat":30.564256827726577} ... ]
     */
    @POST
    @Consumes(MediaType.TEXT_PLAIN)
    @Produces(MediaType.APPLICATION_JSON)
    public String buildMissionFromLatLngPoints(InputStream latLonJsonInputStream, @QueryParam("mowingPathWidthInMeters") Double mowingPathWidthInMeters)
    {
        JSONTokener tokener = new JSONTokener(latLonJsonInputStream);
        JSONObject truckload = new JSONObject(tokener);
        JSONObject startLatLngJsonObj = (JSONObject)truckload.getJSONObject("startMarker");

        GPSPosition startLatLngUnchecked = new GPSPosition(
                startLatLngJsonObj.getDouble("lat"),
                startLatLngJsonObj.getDouble("lng"));

        JSONArray missionPolygonJsonArray = (JSONArray)truckload.get("missionPolygon");
        List<GPSPosition> missionGPSPositionList = buildGPSPositionListFromJsonArrayOfLatLngJsonObj(missionPolygonJsonArray);

        //Note that the client is presently approximating the circles as polygons
        //and just adding them in to the polyObstacles -- hence we're not
        //getting a circleObstacles object from the client -- but
        //I'm not yet sure we'll stick with the polygon
        //approximation approach, so just leaving
        //this unused code sitting around as
        //a reminder 'till some "final"
        //decision is made about
        //circles
        //JSONArray circleObstaclesObj = (JSONArray)truckload.get("circleObstacles");

        //This is a JSONArray of JSONArray where the inner JSONArray's each
        //have {'lat': _____, 'lng':_____} JSONObject's that are
        //the vertices of an obstacle polygon
        JSONArray polyObstaclesJsonArray = (JSONArray)truckload.get("polyObstacles");
        List<List<GPSPosition>> polyObstaclesListOfLists = buildListOfGPSListsFromPolyObstacles(polyObstaclesJsonArray);

        //JSONObject lls = missionPolygonObj;
        //JSONArray circleObstacles = new JSONArray(circleObstaclesObj);
        //JSONArray polyObstacles = new JSONArray(polyObstaclesObj);

        Double heading = Double.parseDouble((String) truckload.get("heading"));

        //JSONArray latLngs = new JSONArray();
        //old code

        MissionBrain brain = new MissionBrain();

        List<GPSPosition> waypoints = brain.buildMissionWaypointsFromLatLngBoundsJSONArray(
                missionGPSPositionList, polyObstaclesListOfLists, mowingPathWidthInMeters,
                startLatLngUnchecked, heading);

        JSONArray jsonArray = new JSONArray();
        for(int x = 0; x < waypoints.size(); x++)
        {
            GPSPosition gpsPosition = waypoints.get(x);
            JSONObject jsonObject = new JSONObject();
            jsonObject.put("lat", gpsPosition.latitude);
            jsonObject.put("lng", gpsPosition.longitude);
            jsonArray.put(x, jsonObject);
        }

        return jsonArray.toString();
    }


    public List<GPSPosition> buildGPSPositionListFromJsonArrayOfLatLngJsonObj(JSONArray missionJsonArray)
    {
        List<GPSPosition> list = new ArrayList<GPSPosition>();

        for(int x=0; x < missionJsonArray.length(); x++)
        {
            JSONObject latLng = (JSONObject) missionJsonArray.get(x);
            Double lat = latLng.getDouble("lat");
            Double lng = latLng.getDouble("lng");
            GPSPosition gpsPosition = new GPSPosition(lat, lng);
            list.add(gpsPosition);
        }

        return list;
    }

    public List<List<GPSPosition>> buildListOfGPSListsFromPolyObstacles(JSONArray polyObstacles)
    {
        List<List<GPSPosition>> listOfGPSPolygonLists = new ArrayList<>();
        for(int x = 0; x < polyObstacles.length(); x++)
        {
            JSONArray polyObstacleJSONList = polyObstacles.getJSONArray(x);
            List<GPSPosition> polyObstacleGPSPositionList = new ArrayList<GPSPosition>();
            for(int y = 0; y < polyObstacleJSONList.length(); y++)
            {
                JSONObject latLngJsonObj = polyObstacleJSONList.getJSONObject(x);
                polyObstacleGPSPositionList.add(new GPSPosition(latLngJsonObj.getDouble("lat"), latLngJsonObj.getDouble("lng")));
            }
            listOfGPSPolygonLists.add(polyObstacleGPSPositionList);
        }
        return listOfGPSPolygonLists;
    }

    @GET
    @Produces(MediaType.TEXT_PLAIN)
    public String getIt() {
        return "You'll want to access this resource via POST";
    }
}
