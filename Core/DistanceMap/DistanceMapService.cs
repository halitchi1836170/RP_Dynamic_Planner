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
    private float k;
    private float eps;

    private float maxDistanceCalculatedInMap;

    public DistanceMapService(float obsThreshold, float k, float eps)
    {
        this.obstacleThreshold = obsThreshold;
        this.k = k;
        this.eps = eps;
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

    public DistanceMap getDistanceMapInstance()
    {
        return new DistanceMap(distanceCalculatedMap, W, H, originX, originY, resolution, k, eps);
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


public class DistanceMap

{

    private float[] distanceMap;
    private int W;
    private int H;
    private float originX;
    private float originY;
    private float resolution;

    private float k;
    private float eps;

    public DistanceMap(float[] distanceMap, int w, int h, float originX, float originY, float resolution, float k, float eps)
    {
        this.distanceMap = distanceMap;
        this.W = w;
        this.H = h;
        this.originX = originX; 
        this.originY = originY;
        this.resolution = resolution;
        this.k = k;
        this.eps = eps;
    }

    public int getW ()
    {
        return W;
    }

    public int getH()
    {
        return H;
    }

    public float[] getDistanceMap()
    {
        return distanceMap;
    }

    public float getK()
    {
        return k;
    }

    public float getEps()
    {
        return eps;
    }

    public float getResolution()
    {
        return resolution;
    }

    public int getIndexFromWorldPosition((float X, float Y) pos)
    {
        (int cellX, int cellY) cell = getCellFromWorldPosition(pos);
        return getIndexFromCell((cell.cellX, cell.cellY));
    }

    public (int cx, int cy) getCellFromWorldPosition((float X, float Y) pos)
    {
        int cellX = (int)Mathf.Floor((pos.X-originX) / resolution);
        int cellY = (int)Mathf.Floor((pos.Y-originY) / resolution);
        return (cellX, cellY);
    }

    public int getIndexFromCell((int cellX, int cellY) cell)
    {
        return cell.cellY * W + cell.cellX;
    }

    public (int x, int y) getCellFromIndex(int index)
    {
        return ((int)index % W, (int) index/W);
    }

    public List<((int cx, int cy),float dist)> getNeighborsOfWordPosition(int uIndex)
    {
        (int cellX, int cellY) cellPos = getCellFromIndex(uIndex);
        List<((int, int),float)> returnNeighbors = new List<((int, int), float)>();

        float root2 = 1.4142f;
        float d1 = 1f;

        (int, int) uCell = (cellPos.cellX - 1, cellPos.cellY - 1);
        if (cellInMap(uCell)) returnNeighbors.Add((uCell, root2));
        uCell = (cellPos.cellX, cellPos.cellY - 1);
        if (cellInMap(uCell)) returnNeighbors.Add((uCell,d1));
        uCell = (cellPos.cellX + 1, cellPos.cellY - 1);
        if (cellInMap(uCell)) returnNeighbors.Add((uCell,root2));

        uCell = (cellPos.cellX - 1, cellPos.cellY);
        if (cellInMap(uCell)) returnNeighbors.Add((uCell,d1));
        uCell = (cellPos.cellX + 1, cellPos.cellY);
        if (cellInMap(uCell)) returnNeighbors.Add((uCell,d1));

        uCell = (cellPos.cellX - 1, cellPos.cellY + 1);
        if (cellInMap(uCell)) returnNeighbors.Add((uCell,root2));
        uCell = (cellPos.cellX, cellPos.cellY + 1);
        if (cellInMap(uCell)) returnNeighbors.Add((uCell,d1));
        uCell = (cellPos.cellX + 1, cellPos.cellY + 1);
        if (cellInMap(uCell)) returnNeighbors.Add((uCell,root2));

        return returnNeighbors;
    }

    private bool cellInMap((int cellX, int cellY) cell)
    {
        if (cell.cellX >= 0 && cell.cellX < W && cell.cellY >= 0 && cell.cellY < H) return true;
        return false;
    }

    public float getHeuristicDistanceFromGoal( int startIndex, int goalIndex)
    {
        (int startX, int startY) start = getCellFromIndex(startIndex);
        (int goalX, int goalY) goal = getCellFromIndex(goalIndex);

        float squareDistance = Mathf.Sqrt(Mathf.Pow(goal.goalX - start.startX, 2) + Mathf.Pow(goal.goalY - start.startY, 2));
        return squareDistance;
    }


    public (List<int>, List<(float, float)>) ReconstructPath(int[] parentMap, int goalIndex, int startIndex)
    {
        List<int> path = new List<int>();
        List<(float x, float y)> pathWorld = new List<(float x, float y)>();

        int current_index = goalIndex;
        while (current_index != -1)
        {
            int row = (int)current_index / W;
            int col = (int)current_index % W;
            float worldX = (float) (originX + (col + 0.5) * resolution);
            float worldY = (float) (originY + (row + 0.5) * resolution);

            path.Add(current_index);
            pathWorld.Add((worldX, worldY));

            if (current_index == startIndex) break;

            current_index = parentMap[current_index];
        }

        path.Reverse(); // Il percorso ora va da Start a Goal
        pathWorld.Reverse();

        return (path, pathWorld);
    }

}
