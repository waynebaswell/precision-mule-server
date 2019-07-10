package org.deepsouthrobotics.api;

import org.deepsouthrobotics.brain.MissionBrain;
import org.deepsouthrobotics.data.GPSPosition;
import org.deepsouthrobotics.data.Position;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import javax.ws.rs.*;
import javax.ws.rs.core.MediaType;

import java.io.InputStream;
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
        JSONArray latLngs = new JSONArray(tokener);
        for (int i = 0; i < latLngs.length(); i++)
        {
            JSONObject latLng = (JSONObject) latLngs.get(i);
            System.out.println(i + " - " + latLng.getDouble("lat") + " " + latLng.getDouble("lng"));
        }

        MissionBrain surgeon = new MissionBrain();

        List<Position> waypoints = surgeon.buildMissionWaypointsFromLatLngBoundsJSONArray(latLngs, mowingPathWidthInMeters);

        JSONArray jsonArray = new JSONArray();
        for(int x = 0; x < waypoints.size(); x++)
        {
            GPSPosition gpsPosition = waypoints.get(x).gps;
            JSONObject jsonObject = new JSONObject();
            jsonObject.put("lat", gpsPosition.latitude);
            jsonObject.put("lng", gpsPosition.longitude);
            jsonArray.put(x, jsonObject);
        }

        return jsonArray.toString();
    }

    @GET
    @Produces(MediaType.TEXT_PLAIN)
    public String getIt() {
        return "You'll want to access this resource via POST";
    }
}
