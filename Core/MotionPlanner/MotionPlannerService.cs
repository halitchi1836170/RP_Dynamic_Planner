using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public enum Planner { A, Dijkstra }

public class MotionPlannerService
{

    private List<int> trajectoryToBeFollowed;
    private List<(float, float)> trajectoryToBeFollowedWorld;
    private bool trajectoryDetermined;

    public MotionPlannerService()
    {
        trajectoryToBeFollowed = new List<int>();
        trajectoryToBeFollowedWorld = new List<(float, float)>();
        trajectoryDetermined = false;
    }

    public void DetermineTrajectory(DistanceMap distanceMap, (float startXUnity, float startZUnity) start, (float goalXUnity, float goalZUnity) goal, Planner plannerMode)
    {
        if (plannerMode == Planner.A)
        {
            planWithA(distanceMap, start, goal);
        }
    }

    private void planWithA(DistanceMap distanceMap, (float startXUnity, float startZUnity) start, (float goalXUnity, float goalZUnity) goal)
    {
        SimplePriorityQueue<int, float> Q = new SimplePriorityQueue<int, float>();
        float[] distanceMapArr = distanceMap.getDistanceMap();
        int W = distanceMap.getW();
        int H = distanceMap.getH();
        float[] costMap = new float[W * H];
        Array.Fill(costMap, float.MaxValue);

        int[] parentMap = new int[W * H];
        Array.Fill(parentMap, -1);

        int startIndex = distanceMap.getIndexFromWorldPosition(start);
        int goalIndex = distanceMap.getIndexFromWorldPosition(goal);
        costMap[startIndex] = 0f;
        Q.Enqueue(startIndex, costMap[startIndex] + distanceMap.getHeuristicDistanceFromGoal(startIndex, goalIndex));

        float k = distanceMap.getK();
        float eps = distanceMap.getEps();

        while (Q.Count > 0)
        {
            int u_index;
            float u_priority;
            Q.TryDequeue(out u_index, out u_priority);

            //check arrivo a goal
            if (u_index == goalIndex)
            {
                Debug.Log($"ARRIVATI");
                (List<int> path, List<(float, float)> pathWorld) paths = distanceMap.ReconstructPath(parentMap, goalIndex, startIndex);
                setPlannedTrajectory(paths.path, paths.pathWorld);
                return;
            }

            List<((int, int),float)> uNeighbors = distanceMap.getNeighborsOfWordPosition(u_index);
            foreach( ((int nX, int nY), float dist) neighbor in uNeighbors)
            {
                int v_index = distanceMap.getIndexFromCell(neighbor.Item1);
                if (distanceMapArr[v_index] == 0) continue;        //vuol dire che è un ostacolo
                float transitionCost = neighbor.dist + k / (distanceMapArr[v_index]+eps);
                float tentative = costMap[u_index] + transitionCost;
                if(tentative < costMap[v_index])
                {
                    costMap[v_index] = tentative;
                    parentMap[v_index] = u_index;
                    float heuristicDistanceFromVToG = distanceMap.getHeuristicDistanceFromGoal(v_index, goalIndex);
                    Q.Enqueue(v_index, costMap[v_index] + heuristicDistanceFromVToG);
                }
            }
        }
    }

    public bool TrajectoryDetermined()
    {
        return trajectoryDetermined;
    }

    public void setPlannedTrajectory(List<int> path, List<(float, float)> pathWorld)
    {
        this.trajectoryToBeFollowed = path;
        this.trajectoryToBeFollowedWorld = pathWorld;
        this.trajectoryDetermined = true;
    }

    public List<int> getTrajectory()
    {
        return trajectoryToBeFollowed;
    }

    public List<(float, float)> getTrajectoryWorld()
    {
        return trajectoryToBeFollowedWorld;
    }

}
