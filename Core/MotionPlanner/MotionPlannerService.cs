using System;
using System.Collections.Generic;
using System.IO;
using Unity.VisualScripting;
using UnityEngine;
using static MatrixVectorUtilities;
using static TridiagonalSolver;

public enum Planner { A, Dijkstra }

public class MotionPlannerService
{
    private bool smoothTrajWithLoSPS;
    private float epsilon;
    private DistanceMap distanceMap;
    private List<int> geometricIndexesTrajectoryToBeFollowed;
    private List<(float, float)> geometricTrajectoryToBeFollowedWorld;
    private List<(float, float)> smoothGeometricTrajectoryToBeFollowedWorld;
    private bool geometricTrajectoryDetermined;
    private float linearMeanVelocity;
    private (double vel_qi, double acc_qi) qiCouple;
    private (double vel_qf, double acc_qf) qfCouple;

    public MotionPlannerService(bool smoothTrajWithLoSPS, float epsilon, float linearMeanVelocity, (double vel_qi, double acc_qi) qiCouple, (double vel_qf, double acc_qf) qfCouple)
    {
        geometricIndexesTrajectoryToBeFollowed = new List<int>();
        geometricTrajectoryToBeFollowedWorld = new List<(float, float)>();
        smoothGeometricTrajectoryToBeFollowedWorld = new List<(float, float)>();
        geometricTrajectoryDetermined = false;
        this.smoothTrajWithLoSPS = smoothTrajWithLoSPS;
        this.epsilon = epsilon;
        this.linearMeanVelocity = linearMeanVelocity;
        this.qiCouple = qiCouple;
        this.qfCouple = qfCouple;
    }

    public void DetermineGeometricTrajectory(DistanceMap distanceMap, (float startXUnity, float startZUnity) start, (float goalXUnity, float goalZUnity) goal, Planner plannerMode)
    {
        this.distanceMap = distanceMap;

        if (plannerMode == Planner.A)
        {
            planWithA(distanceMap, start, goal);
            Debug.Log($"Planned trajectory has {geometricTrajectoryToBeFollowedWorld.Count} vertices...");
        }

        if (smoothTrajWithLoSPS)
        {
            smoothGeometricTrajectoryWithLOSPS();
            Debug.Log($"Smoothed planned trajectory has {smoothGeometricTrajectoryToBeFollowedWorld.Count} vertices...");
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
                setPlannedGeometricTrajectory(paths.path, paths.pathWorld);
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
    public void smoothGeometricTrajectoryWithLOSPS()
    {
        smoothGeometricTrajectoryToBeFollowedWorld = new List<(float, float)>();

        int n = geometricTrajectoryToBeFollowedWorld.Count;
        if (n == 0) return;
        if (n <= 2)
        {
            smoothGeometricTrajectoryToBeFollowedWorld.AddRange(geometricTrajectoryToBeFollowedWorld);
            return;
        }

        smoothGeometricTrajectoryToBeFollowedWorld.Add(geometricTrajectoryToBeFollowedWorld[0]);
        int anchor = 0;
        for (int i = 2; i < n; i++)
        {
            if (!lineOfSightClear(geometricTrajectoryToBeFollowedWorld[anchor], geometricTrajectoryToBeFollowedWorld[i]))
            {
                // il vertice i non e' piu' visibile dall'anchor: l'ultimo visibile (i-1) e' un gomito
                // necessario e diventa il nuovo anchor da cui ripartire.
                smoothGeometricTrajectoryToBeFollowedWorld.Add(geometricTrajectoryToBeFollowedWorld[i - 1]);
                anchor = i - 1;
            }
        }
        smoothGeometricTrajectoryToBeFollowedWorld.Add(geometricTrajectoryToBeFollowedWorld[n - 1]);
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

    public bool GeometricTrajectoryDetermined()
    {
        return geometricTrajectoryDetermined;
    }

    public void setPlannedGeometricTrajectory(List<int> path, List<(float, float)> pathWorld)
    {
        this.geometricIndexesTrajectoryToBeFollowed = path;
        this.geometricTrajectoryToBeFollowedWorld = pathWorld;
        this.geometricTrajectoryDetermined = true;
    }

    public List<(float,float)> getGeometricTrajectoryToBeUsed()
    {
        if (smoothTrajWithLoSPS)
        {
            return smoothGeometricTrajectoryToBeFollowedWorld;
        }
        else
        { 
            return geometricTrajectoryToBeFollowedWorld; 
        }
    }

    public List<(float, float)> getGeometricTrajectoryWorld()
    {
        return geometricTrajectoryToBeFollowedWorld;
    }

    public List<(float, float)> getGeometricSmoothTrajectoryWorld()
    {
        return smoothGeometricTrajectoryToBeFollowedWorld;
    }

    public void LetsSplineGeometricTrajectory()
    {
        List<(float, float)> geomtricTrajectoryTBU = getGeometricTrajectoryToBeUsed();
        int N = geomtricTrajectoryTBU.Count;

        double v = linearMeanVelocity;
        
        (double[] xd, double[] yd, List<double> t) res = getXYTFromGeometricTrajectory(geomtricTrajectoryTBU,v);

        double[] xd = res.xd;
        double[] yd = res.yd;
        List<double> t = res.t;

        double[] dt = new double[t.Count-1];
        for(int i=0; i+1<t.Count; i++)
        {
            dt[i] = t[i + 1] - t[i];
        }

        double qxi = xd[0];
        double qxf = xd[N - 1];
        double vel_qxi = qiCouple.vel_qi;
        double vel_qxf = qfCouple.vel_qf;
        double acc_qxi = qiCouple.acc_qi;
        double acc_qxf = qfCouple.acc_qf;
        double qVirtualStartX0 = qxi + dt[0] * vel_qxi + (dt[0] * dt[0]) / 3 * acc_qxi ;
        double qVirtualFinalX0 = qxf - dt[N] * vel_qxf + (dt[N] * dt[N]) / 3 * acc_qxf;
        
        double qyi = yd[0];
        double qyf = yd[N - 1];
        double vel_qyi = qiCouple.vel_qi;
        double vel_qyf = qfCouple.vel_qf;
        double acc_qyi = qiCouple.acc_qi;
        double acc_qyf = qfCouple.acc_qf;
        double qVirtualStartY0 = qyi + dt[0] * vel_qyi + (dt[0] * dt[0]) / 3 * acc_qyi;
        double qVirtualFinalY0 = qyf - dt[N] * vel_qyf + (dt[N] * dt[N]) / 3 * acc_qyf;

        //VETTORE q PER LA COORDINATA DESIDERATA X
        double[] qx = getQVectorForDesiredCoordinate(xd, qVirtualStartX0, qVirtualFinalX0);

        //VETTORE q PER LA COORDINATA DESIDERATA Y
        double[] qy = getQVectorForDesiredCoordinate(yd, qVirtualStartY0, qVirtualFinalY0);

        //VETTORI a, b E c
        (double[] a, double[] b, double[] c) abcVectors= getABCVectors(dt);
        double[] a = abcVectors.a;
        double[] b = abcVectors.b;
        double[] c = abcVectors.c;

        //VETTORI DEI TERMINI NOTI, UNO PER COORDINATA
        double[] dx = new double[N];
        double[] dy = new double[N];
        dx = getDVectorForDesiredCoordinate(qx, dt, acc_qxi, acc_qxf);
        dy = getDVectorForDesiredCoordinate(qy, dt, acc_qyi, acc_qyf);

        double[] xAcc = SolveTridiagonalMatrix(a, b, c, dx);
        double[] yAcc = SolveTridiagonalMatrix(a, b, c, dy);

    }

    private double[] getQVectorForDesiredCoordinate(double[] xd, double startingVirtual, double finalVirtual)
    {
        int N = xd.Length;
        double[] qx = new double[N + 2];
        qx[0] = xd[0];                                                           // q_1
        qx[1] = startingVirtual;                                                 // q_2  (virtuale, parte nota)
        for (int j = 2; j <= N - 1; j++) qx[j] = xd[j - 1];                      // q_3 ... q_N
        qx[N] = finalVirtual;                                                    // q_{N+1} (virtuale, parte nota)
        qx[N + 1] = xd[N - 1];
        return qx;
    }

    private double[] getDVectorForDesiredCoordinate(double[] qDesired, double[] dt, double desiredInitialAcc, double desiredFinalAcc)
    {
        int N = qDesired.Length - 2;
        double[] dx = new double[N];
        dx[0] = 6.0 * qDesired[0] / dt[0] + 6.0 * qDesired[2] / dt[1] - 6.0 * qDesired[1] * (dt[0] + dt[1]) / (dt[0] * dt[1]) - dt[0] * desiredInitialAcc;
        dx[1] = 6.0 * (qDesired[1] - qDesired[2]) / dt[1] + 6.0 * (qDesired[3] - qDesired[2]) / dt[2];
        for (int k = 2; k <= N - 3; k++)
        {
            dx[k] = 6.0 * (qDesired[k] - qDesired[k + 1]) / dt[k] + 6.0 * (qDesired[k + 2] - qDesired[k + 1]) / dt[k + 1];
        }
        dx[N - 2] = 6.0 * (qDesired[N - 2] - qDesired[N - 1]) / dt[N - 2] + 6.0 * (qDesired[N] - qDesired[N - 1]) / dt[N - 1];
        dx[N - 1] = 6.0 * qDesired[N - 1] / dt[N - 1] + 6.0 * qDesired[N + 1] / dt[N] - 6.0 * qDesired[N] * (dt[N - 1] + dt[N]) / (dt[N - 1] * dt[N]) - dt[N] * desiredFinalAcc;
        return dx;
    }

    private (double[] a, double[] b, double[] c) getABCVectors(double[] dt)
    {
        int N = dt.Length - 1;

        double[] a = new double[N];
        double[] b = new double[N];
        double[] c = new double[N];

        b[0] = (dt[0] + dt[1]) * (2.0 + dt[0] / dt[1]);
        c[0] = dt[1];

        a[1] = dt[1] - dt[0] * dt[0] / dt[1];
        b[1] = 2.0 * (dt[1] + dt[2]);
        c[1] = dt[2];

        for (int k = 2; k <= N - 3; k++)
        {
            a[k] = dt[k];
            b[k] = 2.0 * (dt[k] + dt[k + 1]);
            c[k] = dt[k + 1];
        }

        a[N - 2] = dt[N - 2];
        b[N - 2] = 2.0 * (dt[N - 2] + dt[N - 1]);
        c[N - 2] = dt[N - 1] - dt[N] * dt[N] / dt[N - 1];

        a[N - 1] = dt[N - 1];
        b[N - 1] = (dt[N - 1] + dt[N]) * (2.0 + dt[N] / dt[N - 1]);

        return (a, b, c);
    }

    private (double[] xd, double[] yd, List<double> t) getXYTFromGeometricTrajectory(List<(float, float)> geomtricTrajectoryTBU,double velocity)
    {
        int N = geomtricTrajectoryTBU.Count;

        double[] xd = new double[N];
        double[] yd = new double[N];

        List<double> t = new List<double>();

        int i = 0;
        xd[i] = geomtricTrajectoryTBU[i].Item1;
        yd[i] = geomtricTrajectoryTBU[i].Item2;
        t.Insert(i, 0.0f);

        double dx = 0;
        double dy = 0;

        while (i + 1 <= N - 1)
        {
            xd[i + 1] = geomtricTrajectoryTBU[i + 1].Item1;
            yd[i + 1] = geomtricTrajectoryTBU[i + 1].Item2;
            dx = xd[i + 1] - xd[i];
            dy = yd[i + 1] - yd[i];
            t.Insert(i + 1, t[i] + (Math.Sqrt(dx * dx + dy * dy) / velocity));
            i++;
        }

        return (xd, yd, t);

    }

}
