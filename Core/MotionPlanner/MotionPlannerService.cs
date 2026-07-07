using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public enum Planner { A, Dijkstra }

public class MotionPlannerService
{
    private bool smoothTrajWithLoSPS;
    private float epsilon;
    private DistanceMap distanceMap;
    private List<int> trajectoryToBeFollowed;
    private List<(float, float)> trajectoryToBeFollowedWorld;
    private List<(float, float)> smoothTrajectoryToBeFollowedWorld;
    private bool trajectoryDetermined;

    public MotionPlannerService(bool smoothTrajWithLoSPS, float epsilon)
    {
        trajectoryToBeFollowed = new List<int>();
        trajectoryToBeFollowedWorld = new List<(float, float)>();
        smoothTrajectoryToBeFollowedWorld = new List<(float, float)>();
        trajectoryDetermined = false;
        this.smoothTrajWithLoSPS = smoothTrajWithLoSPS;
        this.epsilon = epsilon;
    }

    public void DetermineTrajectory(DistanceMap distanceMap, (float startXUnity, float startZUnity) start, (float goalXUnity, float goalZUnity) goal, Planner plannerMode)
    {
        this.distanceMap = distanceMap;

        if (plannerMode == Planner.A)
        {
            planWithA(distanceMap, start, goal);
            Debug.Log($"Planned trajectory has {trajectoryToBeFollowedWorld.Count} vertices...");
        }

        if (smoothTrajWithLoSPS)
        {
            smoothTrajectoryWithLOSPS();
            Debug.Log($"Smoothed planned trajectory has {smoothTrajectoryToBeFollowedWorld.Count} vertices...");
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

    // Line-of-Sight Path Smoothing (string-pulling): parte dal path grezzo di A* (spezzata a scaletta,
    // un vertice per cella) e tiene solo il minimo insieme di vertici tali che punti consecutivi si vedano
    // in linea d'aria dentro un tunnel di semi-spessore epsilon (ingombro robot). Elimina gli zig-zag lungo
    // i tratti rettilinei, lasciando i "gomiti" reali imposti dagli ostacoli. Serve come pre-processing:
    // meno vertici e piu' significativi rendono piu' pulito il successivo raffinamento con spline.
    public void smoothTrajectoryWithLOSPS()
    {
        smoothTrajectoryToBeFollowedWorld = new List<(float, float)>();

        int n = trajectoryToBeFollowedWorld.Count;
        if (n == 0) return;
        if (n <= 2)
        {
            smoothTrajectoryToBeFollowedWorld.AddRange(trajectoryToBeFollowedWorld);
            return;
        }

        smoothTrajectoryToBeFollowedWorld.Add(trajectoryToBeFollowedWorld[0]);
        int anchor = 0;
        for (int i = 2; i < n; i++)
        {
            if (!lineOfSightClear(trajectoryToBeFollowedWorld[anchor], trajectoryToBeFollowedWorld[i]))
            {
                // il vertice i non e' piu' visibile dall'anchor: l'ultimo visibile (i-1) e' un gomito
                // necessario e diventa il nuovo anchor da cui ripartire.
                smoothTrajectoryToBeFollowedWorld.Add(trajectoryToBeFollowedWorld[i - 1]);
                anchor = i - 1;
            }
        }
        smoothTrajectoryToBeFollowedWorld.Add(trajectoryToBeFollowedWorld[n - 1]);
    }

    // Il segmento a->b (coordinate mondo ROS) e' percorribile se, campionandolo, ogni campione resta ad almeno
    // epsilon dall'ostacolo piu' vicino. La distance map fornisce la distanza in celle: * resolution -> metri.
    private bool lineOfSightClear((float x, float y) a, (float x, float y) b)
    {
        float[] dmap = distanceMap.getDistanceMap();
        float res = distanceMap.getResolution();
        int W = distanceMap.getW();
        int H = distanceMap.getH();

        float dx = b.x - a.x;
        float dy = b.y - a.y;
        float segLen = Mathf.Sqrt(dx * dx + dy * dy);

        // passo di campionamento ~ mezza cella (Nyquist sulla griglia) per non "saltare" celle occupate
        int nSamples = Mathf.Max(1, Mathf.CeilToInt(segLen / (res * 0.5f)));

        for (int s = 0; s <= nSamples; s++)
        {
            float tt = (float)s / nSamples;
            float px = a.x + tt * dx;
            float py = a.y + tt * dy;

            (int cx, int cy) = distanceMap.getCellFromWorldPosition((px, py));
            if (cx < 0 || cx >= W || cy < 0 || cy >= H) return false;   // esce dalla mappa -> non percorribile

            int idx = distanceMap.getIndexFromCell((cx, cy));
            float clearanceMeters = dmap[idx] * res;                    // distanza dall'ostacolo piu' vicino [m]
            if (clearanceMeters < epsilon) return false;                // il tunnel toccherebbe un ostacolo
        }
        return true;
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

    public List<(float, float)> getSmoothTrajectoryWorld()
    {
        return smoothTrajectoryToBeFollowedWorld;
    }

}
