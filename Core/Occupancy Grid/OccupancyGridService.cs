using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static MatrixVectorUtilities;
using static PoseMatrix4x4;
using static UnicycleModelUtilities;

public class OccupancyGridService
{
    private float resolution;

    private float lMin = -2.0f;
    private float lMax = 3.5f;

    private float probOcc;  // Se rileva un ostacolo, mi fido al 75%
    private float probFree; // Se non rileva nulla, la probabilità che ci sia un ostacolo scende al 35%
    private float lOcc;
    private float lFree;

    private float zMin;
    private float zMax;

    private Dictionary<(int, int), float> hashMapOnlineoccupancyGrid;
    private (sbyte[], int W, int H, float originX, float originY, float resolution) dataForPublisher;

    private int mapMinX = int.MaxValue;
    private int mapMaxX = int.MinValue;
    private int mapMinY = int.MaxValue;
    private int mapMaxY = int.MinValue;

    private float occBlockThreshold;
    private float elevThrehsold;

    public OccupancyGridService(float resolution, float zMin, float zMax, float probOcc, float probFree, float occBlockThreshold, float elevThrehsold)
    {
        this.resolution = resolution;
        this.hashMapOnlineoccupancyGrid = new Dictionary<(int, int), float>();
        this.zMin = zMin;
        this.zMax = zMax;
        lOcc = Mathf.Log(probOcc / (1.0f - probOcc));
        lFree = Mathf.Log(probFree / (1.0f - probFree));
        this.occBlockThreshold = occBlockThreshold;
        this.elevThrehsold = elevThrehsold;
}

    private (int, int) fromPointToCell(float[] point)
    {
        int cx = (int) Mathf.Floor((point[0]) / resolution);
        int cy = (int) Mathf.Floor((point[1]) / resolution);
        return (cx, cy);
    }

    private float[] cellToPoint((int cx, int cy) cell)
    {
        float vx = (cell.cx + 0.5f) * resolution;
        float vy = (cell.cy + 0.5f) * resolution;
        return new float[3] { vx, vy, 0.0f };
    }

    public void clear()
    {
        hashMapOnlineoccupancyGrid.Clear();
        mapMinX = int.MaxValue; mapMaxX = int.MinValue;
        mapMinY = int.MaxValue; mapMaxY = int.MinValue;
    }

    private void updateCell((int cx, int cy) cell, float delta)
    {
        float v = hashMapOnlineoccupancyGrid.ContainsKey(cell) ? hashMapOnlineoccupancyGrid[cell] : 0.0f;
        hashMapOnlineoccupancyGrid[cell] = Mathf.Clamp(v + delta, lMin, lMax);

        if (cell.cx < mapMinX) mapMinX = cell.cx;
        if (cell.cx > mapMaxX) mapMaxX = cell.cx;
        if (cell.cy < mapMinY) mapMinY = cell.cy;
        if (cell.cy > mapMaxY) mapMaxY = cell.cy;
    }

    public void updateGridMap(float[,] currentT, List<Vector3> scannedPoints, Func<Vector3, Vector3> toWorld)
    {
        (float[,] R, float[] t) decomposedT = decomposeT(currentT);
        float[] t = decomposedT.t;
        Vector3 oW = toWorld(new Vector3(t[0], t[1], t[2]));
        (float x, float y, float z) tRos = UnityToRosPosition(oW[0], oW[1], oW[2]);

        (int cx, int cy) sCell = fromPointToCell(new float[3] { tRos.x, tRos.y, 0 });

        foreach (Vector3 v in scannedPoints)
        {
            Vector3 p_map = toWorld(applyTransformation(currentT, v));
            (float  x, float y, float z) vRos = UnityToRosPosition(p_map.x, p_map.y, p_map.z);
            if (vRos.z <= zMin || vRos.z >= zMax) continue;

            float dxh = vRos.x - tRos.x;
            float dyh = vRos.y - tRos.y;
            if (dxh * dxh + dyh * dyh < 0.30*0.30) continue;

            float dh = Mathf.Sqrt(Mathf.Pow((vRos.x - tRos.x),2) + Mathf.Pow((vRos.y - tRos.y),2));   // distanza orizzontale
            float dz = vRos.z - tRos.z;                                                               // dislivello
            float elev = Mathf.Atan2(dz, dh);                                                         // elevazione del raggio

            (int cx, int cy) eCell = fromPointToCell(new float[3] { vRos.x, vRos.y, vRos.z});
            updateCell(eCell, lOcc);
            if (Mathf.Abs(elev) < elevThrehsold)            // es. ~5-10°
                bresenhamAlgorithm(sCell, eCell);     // free SOLO se il raggio è ~orizzontale
        }
    }

    public void bresenhamAlgorithm((int cx, int cy) p0, (int cx, int cy) p1)
    {
        int x0 = p0.cx;
        int y0 = p0.cy;
        int x1 = p1.cx;
        int y1 = p1.cy;

        int dx = Mathf.Abs(x1 - x0);
        int dy = Mathf.Abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            if (x0 == x1 && y0 == y1) break;

            if (hashMapOnlineoccupancyGrid.TryGetValue((x0, y0), out float l) && l > occBlockThreshold) break;

            updateCell((x0, y0), lFree);

            int e2 = 2 * err;

            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }

            if (e2 < dx) 
            {
                err += dx;
                y0 += sy;
            }
        }
    }


    public Dictionary<(int,int), float> getOccupancyGridMap()
    {
        return hashMapOnlineoccupancyGrid;
    }

    public (int mapMinX, int mapMaxX, int mapMinY, int mapMaxY) getMapExtremas()
    {
        return (mapMinX, mapMaxX, mapMinY, mapMaxY);
    }

    public void  updateDataForPublisher()
    {
        if (hashMapOnlineoccupancyGrid is null || hashMapOnlineoccupancyGrid.Count == 0)
        {
            dataForPublisher = (new sbyte[0], 0, 0, 0, 0, resolution);
            return;
        }
        (int mapMinX, int mapMaxX, int mapMinY, int mapMaxY) mapExtremas = getMapExtremas();

        int W = mapExtremas.mapMaxX - mapExtremas.mapMinX + 1;
        int H = mapExtremas.mapMaxY - mapExtremas.mapMinY + 1;

        float originX = mapExtremas.mapMinX * resolution;
        float originY = mapExtremas.mapMinY * resolution;

        sbyte[] dataForPub = new sbyte[W * H];
        Array.Fill(dataForPub, (sbyte)(-1));
        foreach (var couple in hashMapOnlineoccupancyGrid)
        {
            int cx = couple.Key.Item1;
            int cy = couple.Key.Item2;
            int idx = (cy - mapExtremas.mapMinY) * W + (cx - mapExtremas.mapMinX);
            float p = (float)(1.0 / (1 + Mathf.Pow((float) Math.E, (-1 * couple.Value))));
            dataForPub[idx] = (sbyte)Mathf.Floor(100 * p);
        }
        dataForPublisher = (dataForPub, W, H, originX, originY, resolution);
    }

    public (sbyte[], int, int, float, float, float) getDataForPublisher()
    {
        //Debug.Log($"PUB count={hashMapOnlineoccupancyGrid.Count} extremas=({mapMinX},{mapMaxX},{mapMinY},{mapMaxY})");
        return dataForPublisher;
    }

}
