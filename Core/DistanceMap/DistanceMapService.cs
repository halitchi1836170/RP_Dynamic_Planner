using Assimp;
using System;
using System.Collections.Generic;
using UnityEngine;

public class DistanceMapService
{
    private int W;
    private int H;
    private float originX;
    private float originY;
    private float resolution;
    private sbyte[] occupancyGridMap;
    private float[] distanceCalculatedMap;

    private float obstacleThreshold;

    private float maxDistanceCalculatedInMap;

    public DistanceMapService(float obsThreshold)
    {
        this.obstacleThreshold = obsThreshold;
    }

    public void SetOccupancyGridMap((sbyte[] map, int W, int H, float originX, float originY, float resolution) givenOccupancyGridMapSet)
    {
        this.occupancyGridMap = givenOccupancyGridMapSet.map;
        this.W = givenOccupancyGridMapSet.W;
        this.H = givenOccupancyGridMapSet.H;
        this.originX = givenOccupancyGridMapSet.originX;
        this.originY = givenOccupancyGridMapSet.originY;
        this.resolution = givenOccupancyGridMapSet.resolution;
        this.distanceCalculatedMap = new float[W * H];
        this.maxDistanceCalculatedInMap = float.MinValue;
    }

    public void calculateDistanceMap()
    {
        int totalCells = W * H;
        float[] distanceMap = new float[totalCells];
        Array.Fill(distanceMap, float.MaxValue);

        SimplePriorityQueue<int, float> pq = new SimplePriorityQueue<int, float>();

        for(int i = 0; i < totalCells; i++)
        {
            if (occupancyGridMap[i] >= obstacleThreshold)
            {
                distanceMap[i] = 0f;
                pq.Enqueue(i, 0f);
            } 
        }

        (int dx, int dy, float cost)[] directions = {
            ( 0, -1, 1.0f),    // Nord
            ( 0,  1, 1.0f),    // Sud
            (-1,  0, 1.0f),    // Ovest
            ( 1,  0, 1.0f),    // Est
            (-1, -1, 1.4142f), // Nord-Ovest
            ( 1, -1, 1.4142f), // Nord-Est
            (-1,  1, 1.4142f), // Sud-Ovest
            ( 1,  1, 1.4142f)  // Sud-Est
        };

        while (pq.Count > 0)
        {
            pq.TryDequeue(out int currIndex, out float currDist);

            if (currDist > distanceMap[currIndex])
                continue;

            int cx = currIndex % W;
            int cy = currIndex / W;

            foreach (var dir in directions)
            {
                int nx = cx + dir.dx;
                int ny = cy + dir.dy;

                if (nx >= 0 && nx < W && ny >= 0 && ny < H)
                {
                    int neighborIndex = ny * W + nx;
                    float newDist = currDist + dir.cost;
                    if(newDist > maxDistanceCalculatedInMap)
                    {
                        maxDistanceCalculatedInMap = newDist;
                    }
                    if (newDist < distanceMap[neighborIndex])
                    {
                        distanceMap[neighborIndex] = newDist;

                        pq.Enqueue(neighborIndex, newDist);
                    }
                }
            }
        }

        distanceCalculatedMap = distanceMap;
    }

    public float[] getDistanceMap()
    {
        return distanceCalculatedMap;
    }
    
    public (sbyte[] data, int width, int height, float originX, float originY, float resolution) getDistanceMapForPublisher()
    {
        sbyte[] returnedData = new sbyte[W * H];

        float maxDistanceCalculated = maxDistanceCalculatedInMap;
        if (maxDistanceCalculatedInMap == 0f) maxDistanceCalculated = 1f;

        for(int i=0;i<W*H;i++)
        {
            float currentDist = distanceCalculatedMap[i];

            /*if (distanceCalculatedMap[i] >= maxDistance)
            {
                returnedData[i] = (sbyte)0;
            }else */if(currentDist == 0f)
            {
                returnedData[i] = (sbyte)100;
            }
            else
            {
                float scaledValue = 100f - ((100f / maxDistanceCalculated) * currentDist);
                returnedData[i] = (sbyte)Math.Clamp(Math.Round(scaledValue), 1, 99);
            }
        }
        return (returnedData, W, H, originX, originY, resolution);
    }

}
