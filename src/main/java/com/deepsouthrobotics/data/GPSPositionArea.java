package com.deepsouthrobotics.data;

import java.awt.geom.Path2D;
import java.util.List;

/**
 * Associating a list of GPSPosition with a path
 */
public class GPSPositionArea extends Path2D.Double
{
    public List<GPSPosition> positions;

    public GPSPositionArea(List<GPSPosition> positions)
    {
        this.positions = positions;

        Boolean FIRST_LINE = true;

        for(int i = 0; i < positions.size(); i++)
        {
            GPSPosition position = positions.get(i);

            java.lang.Double x = position.x;
            java.lang.Double y = position.y;

            if(FIRST_LINE)
            {
                FIRST_LINE = false;
                this.moveTo(x, y);
            }
            else
            {
                this.lineTo(x, y);
            }
        }
        this.closePath();
    }
}
