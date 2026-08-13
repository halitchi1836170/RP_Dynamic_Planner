using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UIElements;
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
    private HashSet<int> forcedWaypointIndices = new HashSet<int>();   // indici (nel path) dei nodi waypoint da NON semplificare col LOS-PS
    private float linearMeanVelocity;
    private (double vel_qi, double acc_qi) qiCouple;
    private (double vel_qf, double acc_qf) qfCouple;
    private List<(double t, double dt, double ak, double bk, double ck, double dk)> coefficients_xSpline;
    private List<(double t, double dt, double ak, double bk, double ck, double dk)> coefficients_ySpline;
    private List<(float x, float y)> geomtricTrajecotryFromSplinesToBeFollowed;
    private List<(float t, float x, float y, float xd, float yd, float xdd, float ydd)> geometricTrajectoryTableForController;
    private bool splinedGeometricTrajectoryDetermined;
    private bool controllerGeometricTrajectoryDetermined;
    private int subSamplesPerSpline;
    private int secondsControlFrequency;
    private float wMax;   // cap di velocita' angolare per il rallentamento in curva del profilo (kappa*v <= wMax)
    private float aMax;   // accelerazione tangenziale per le rampe accel/decel del profilo
    private float vMax;   // tetto di velocita' lineare: la crociera del profilo NON deve superarlo (== clamp del controller)

    public MotionPlannerService(bool smoothTrajWithLoSPS, float epsilon, float linearMeanVelocity, (double vel_qi, double acc_qi) qiCouple, (double vel_qf, double acc_qf) qfCouple, int subSamplesPerSpline, int secondsControlFrequency, float wMax, float aMax, float vMax)
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
        this.coefficients_xSpline = new List<(double, double, double, double, double, double)>();
        this.coefficients_ySpline = new List<(double, double, double, double, double, double)>();
        this.geomtricTrajecotryFromSplinesToBeFollowed = new List<(float, float)>();
        this.geometricTrajectoryTableForController = new List<(float t, float x, float y, float xd, float yd, float xdd, float ydd)>();
        this.splinedGeometricTrajectoryDetermined = false;
        this.controllerGeometricTrajectoryDetermined = false;
        this.subSamplesPerSpline = subSamplesPerSpline;
        this.secondsControlFrequency = secondsControlFrequency;
        this.wMax = wMax;
        this.aMax = aMax;
        this.vMax = vMax;
    }

    public void DetermineGeometricTrajectory(DistanceMap distanceMap, (float startXUnity, float startZUnity) start, (float goalXUnity, float goalZUnity) goal, Planner plannerMode)
    {
        this.distanceMap = distanceMap;
        forcedWaypointIndices.Clear();

        if (plannerMode == Planner.A)
        {
            (List<int> path, List<(float, float)> pathWorld) seg = planSegmentWithA(distanceMap, start, goal);
            setPlannedGeometricTrajectory(seg.path, seg.pathWorld);
        }

        if (smoothTrajWithLoSPS)
        {
            smoothGeometricTrajectoryWithLOSPS();
        }
    }

    // Trajectory attraverso una lista ORDINATA di waypoint (start, nodi intermedi, goal): A* su OGNI intervallo,
    // concatenato in un'unica trajectory geometrica (senza duplicare il nodo condiviso), poi LOS-PS come al solito.
    public void DetermineGeometricTrajectoryFromWaypoints(DistanceMap distanceMap, List<(float, float)> waypoints, Planner plannerMode)
    {
        this.distanceMap = distanceMap;

        List<int> fullPath = new List<int>();
        List<(float, float)> fullPathWorld = new List<(float, float)>();
        forcedWaypointIndices.Clear();

        for (int i = 0; i + 1 < waypoints.Count; i++)
        {
            (List<int> path, List<(float, float)> pathWorld) seg = (new List<int>(), new List<(float, float)>());
            if (plannerMode == Planner.A) seg = planSegmentWithA(distanceMap, waypoints[i], waypoints[i + 1]);

            int startIdx = (i == 0) ? 0 : 1;   // salto il primo punto (== ultimo del segmento precedente) per non duplicare il nodo
            for (int j = startIdx; j < seg.pathWorld.Count; j++)
            {
                fullPath.Add(seg.path[j]);
                fullPathWorld.Add(seg.pathWorld[j]);
            }

            // l'ultimo punto appena aggiunto e' il nodo waypoint[i+1]: lo forzo se INTERMEDIO (start = indice 0
            // e goal = ultimo punto sono comunque sempre tenuti dal LOS-PS). Traccio l'INDICE, non le coordinate.
            if (i + 1 < waypoints.Count - 1) forcedWaypointIndices.Add(fullPathWorld.Count - 1);
        }

        setPlannedGeometricTrajectory(fullPath, fullPathWorld);

        if (smoothTrajWithLoSPS) smoothGeometricTrajectoryWithLOSPS();
    }

    private (List<int> path, List<(float, float)> pathWorld) planSegmentWithA(DistanceMap distanceMap, (float startXUnity, float startZUnity) start, (float goalXUnity, float goalZUnity) goal)
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
                return distanceMap.ReconstructPath(parentMap, goalIndex, startIndex);
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
        return (new List<int>(), new List<(float, float)>());   // nessun percorso trovato
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

            // forzo i nodi waypoint: devono restare vertici (la spline ci passera' sopra) e diventano anchor.
            if (forcedWaypointIndices.Contains(i))
            {
                smoothGeometricTrajectoryToBeFollowedWorld.Add(geometricTrajectoryToBeFollowedWorld[i]);
                anchor = i;
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
        List<(float, float)> listTBU = new List<(float, float)>();
        if (smoothTrajWithLoSPS)
        {
            if (splinedGeometricTrajectoryDetermined)
            {
                listTBU = geomtricTrajecotryFromSplinesToBeFollowed;
            }
            else
            {
                listTBU = smoothGeometricTrajectoryToBeFollowedWorld;
            }
        }
        else
        {
            listTBU = geometricTrajectoryToBeFollowedWorld; 
        }
        return listTBU;
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
        //Debug.Log($"geometric trajectory TBU has {N} points...");
        double v = linearMeanVelocity;
        
        (double[] xd, double[] yd, List<double> t) res = getXYTFromGeometricTrajectory(geomtricTrajectoryTBU,v);
        //Debug.Log($"xd.count: {res.xd.Length}, yd.count: {res.yd.Length}, t.count: {res.t.Count}");

        double[] xd = res.xd;
        double[] yd = res.yd;
        List<double> t = res.t;

        double tN1 = (t[N - 1] + t[N - 2]) / 2.0;
        double t2 = (t[1] + t[0]) / 2.0;
        t.Insert(N - 1, tN1);       //inserimento tN+1 come media
        t.Insert(1, t2);          //inserimento t2 come media

        int N_intervals_augmented = t.Count-1;
        double[] dt = new double[N_intervals_augmented];      //bisogna aggiungere anche i due nuovi istanti di tempo (t2 e qN+1)
        for (int i = 0; i < N_intervals_augmented; i++)
        {
            dt[i] = t[i + 1] - t[i];
        }
        //Debug.Log($"dt array intervals: {dt.Length}");

        double qxi = xd[0];
        double qxf = xd[N - 1];
        double vel_qxi = qiCouple.vel_qi;
        double vel_qxf = qfCouple.vel_qf;
        double acc_qxi = qiCouple.acc_qi;
        double acc_qxf = qfCouple.acc_qf;
        double qVirtualStartX0 = qxi + dt[0] * vel_qxi + (dt[0] * dt[0]) / 3 * acc_qxi ;
        double qVirtualFinalX0 = qxf - dt[N-2] * vel_qxf + (dt[N-2] * dt[N-2]) / 3 * acc_qxf;
        
        double qyi = yd[0];
        double qyf = yd[N - 1];
        double vel_qyi = qiCouple.vel_qi;
        double vel_qyf = qfCouple.vel_qf;
        double acc_qyi = qiCouple.acc_qi;
        double acc_qyf = qfCouple.acc_qf;
        double qVirtualStartY0 = qyi + dt[0] * vel_qyi + (dt[0] * dt[0]) / 3 * acc_qyi;
        double qVirtualFinalY0 = qyf - dt[N-2] * vel_qyf + (dt[N-2] * dt[N-2]) / 3 * acc_qyf;

        //VETTORE q PER LA COORDINATA DESIDERATA X
        double[] qx = getQVectorForDesiredCoordinateIncomplete(xd, qVirtualStartX0, qVirtualFinalX0);
        //Debug.Log($"qVector for desired coordinate has dimension: {qx.Length}");

        //VETTORE q PER LA COORDINATA DESIDERATA Y
        double[] qy = getQVectorForDesiredCoordinateIncomplete(yd, qVirtualStartY0, qVirtualFinalY0);

        //VETTORI a, b E c
        (double[] a, double[] b, double[] c) abcVectors= getABCVectors(dt);
        double[] a = abcVectors.a;
        double[] b = abcVectors.b;
        double[] c = abcVectors.c;

        //Debug.Log($"Some elements of vector a (length: {a.Length}) before Tridiagonal Solver: {a[0]}-{a[1]}-{a[2]}-{a[3]}");
        //Debug.Log($"Some elements of vector b (length: {b.Length}) before Tridiagonal Solver: {b[0]}-{b[1]}-{b[2]}-{b[3]}");
        //Debug.Log($"Some elements of vector c (length: {c.Length}) before Tridiagonal Solver: {c[0]}-{c[1]}-{c[2]}-{c[3]}");

        //VETTORI DEI TERMINI NOTI, UNO PER COORDINATA
        double[] dx = getDVectorForDesiredCoordinate(qx, dt, acc_qxi, acc_qxf);
        //Debug.Log($"Some elements of vector dx (length: {dx.Length}) before Tridiagonal Solver: {dx[0]}-{dx[1]}-{dx[2]}-{dx[3]}");

        double[] dy = getDVectorForDesiredCoordinate(qy, dt, acc_qyi, acc_qyf);

        //Debug.Log("Calling tridiagonal matrix solver utility...");

        double[] qx_Acc = augmentAccelerationsWithExtremas(SolveTridiagonalMatrix(a, b, c, dx),acc_qxi, acc_qxf);
        //Debug.Log($"Some elements of vector qx_Acc (length: {qx_Acc.Length}) after Tridiagonal Solver: {qx_Acc[0]}-{qx_Acc[1]}-{qx_Acc[2]}-{qx_Acc[3]}");

        double[] qy_Acc = augmentAccelerationsWithExtremas(SolveTridiagonalMatrix(a, b, c, dy),acc_qyi, acc_qyf);

        qx = completeQVectorTerms(qx, (dt[0] * dt[0] / 6.0) * qx_Acc[1], (dt[N - 3] * dt[N - 3] / 6.0) * qx_Acc[N - 2]);
        qy = completeQVectorTerms(qy, (dt[0] * dt[0] / 6.0) * qy_Acc[1], (dt[N - 3] * dt[N - 3] / 6.0) * qy_Acc[N - 2]);

        //Debug.Log($"completed q vector dim: {qx.Length}, q_Acc vector dim: {qx_Acc.Length}");

        List<(double t, double dt, double ak, double bk, double ck, double dk)> listCoefficients_xSpline = getSplineCoefficients(qx, qx_Acc, t, dt); 
        List<(double t, double dt, double ak, double bk, double ck, double dk)> listCoefficients_ySpline = getSplineCoefficients(qy, qy_Acc, t, dt);

        //Debug.Log(listCoefficients_xSpline[0]);
        //Debug.Log(listCoefficients_xSpline[1]);
        //Debug.Log(listCoefficients_xSpline[2]);

        this.coefficients_xSpline = listCoefficients_xSpline;
        this.coefficients_ySpline = listCoefficients_ySpline;
    }

    private List<(double t, double dt, double ak, double bk, double ck, double dk)> getSplineCoefficients(double[] q, double[] q_Acc, List<double> t, double[] dt)
    {
        List<(double t, double dt, double ak, double bk, double ck, double dk)> resultCoefficients = new List<(double, double, double, double, double, double)>();
        int Nintervals = t.Count - 1;
        for(int k = 0; k+1 <= Nintervals; k++)
        {
            double tk = t[k];
            double dtk = dt[k];

            double Deltak = q[k + 1] - q[k];
            double dd_Deltak = q_Acc[k + 1] - q_Acc[k];

            double dk = q[k];
            double ck = ((Deltak / dtk) - ((dtk / 6.0) * (dd_Deltak + 3 * q_Acc[k])));
            double bk = q_Acc[k] / 2.0;
            double ak = dd_Deltak / (6 * dtk);

            resultCoefficients.Add((tk, dtk, ak, bk, ck, dk));
        }
        return resultCoefficients;
    }

    private double[] augmentAccelerationsWithExtremas(double[] acc, double acc_qi, double acc_qf)
    {
        int Nresult = acc.Length + 2;
        double[] result = new double[Nresult];
        result[0] = acc_qi;
        result[Nresult - 1] = acc_qf;
        for(int i = 1; i <= Nresult - 2; i++)
        {
            result[i]= acc[i-1]; 
        }
        return result;
    }

    private double[] getQVectorForDesiredCoordinateIncomplete(double[] xd, double startingVirtual, double finalVirtual)
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

    private double[] completeQVectorTerms(double[] q, double acc_q2_term, double acc_qN1_term)
    {
        int N = q.Length;
        double[] qNew = q;
        qNew[1] = qNew[1] + acc_q2_term;
        qNew[N-2] = qNew[N-2] + acc_qN1_term;
        return qNew;
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

    public void DetermineGeomtricTrajectoryFromSplines()
    {
        int nInstants = coefficients_xSpline.Count;

        for (int currentK = 0; currentK < nInstants; currentK++)
        {
            double dtk = coefficients_xSpline[currentK].dt;

            for (int s = 0; s < subSamplesPerSpline; s++)
            {
                double tau = ((double)s / subSamplesPerSpline) * dtk;   // <-- cast a double: niente divisione intera

                float xt = calculateSplineValue(coefficients_xSpline[currentK].ak,
                                                coefficients_xSpline[currentK].bk,
                                                coefficients_xSpline[currentK].ck,
                                                coefficients_xSpline[currentK].dk, tau);

                float yt = calculateSplineValue(coefficients_ySpline[currentK].ak,
                                                coefficients_ySpline[currentK].bk,
                                                coefficients_ySpline[currentK].ck,
                                                coefficients_ySpline[currentK].dk, tau);

                geomtricTrajecotryFromSplinesToBeFollowed.Add((xt, yt));
            }
        }
        splinedGeometricTrajectoryDetermined = true;
    }

    private float calculateSplineValue(double axt, double bxt, double cxt, double dxt, double t)
    {
        float result = (float)(axt * (t*t*t) + bxt * (t*t) + cxt * (t) + dxt);
        return result;
    }

    private float calculateFirstDerivateSplineValue(double axt, double bxt, double cxt, double t)
    {
        float result = (float)(3*axt * (t * t) + 2*bxt * (t) + cxt);
        return result;
    }

    private float calculateSecondDerivateSplineValue(double axt, double bxt, double t)
    {
        float result = (float)(6 * axt * (t) + 2 * bxt);   // d2/dt2 (a t^3 + b t^2 + c t + d) = 6a t + 2b
        return result;
    }

    public List<(float, float)> getSplinedGeomtricTrajectory()
    {
        return this.geomtricTrajecotryFromSplinesToBeFollowed;
    }

    public List<(float t, float x, float y, float xd, float yd, float xdd, float ydd)> getGeometricTrajectoryForController()
    {
        if (controllerGeometricTrajectoryDetermined == false)
        {
            DetermineGeometricTrajectoryTableForController();
            controllerGeometricTrajectoryDetermined = true;
        }
        return geometricTrajectoryTableForController;
    }

    // Costruisce la tabella-riferimento per il controller DISACCOPPIANDO geometria e velocita':
    //  1) campiona fitto la GEOMETRIA della spline -> per ogni campione: arco cumulato s, tangente theta,
    //     curvatura kappa (grandezze geometriche, indipendenti dalla velocita' di parametrizzazione);
    //  2) ricampiona a passo di controllo dt avanzando l'arco col PROFILO DI VELOCITA': crociera limitata
    //     dalla curvatura (rallenta in curva, wMax/|kappa|) e rampe accel/decel (sqrt(2 a s)).
    // Le derivate salvate (xd,yd,xdd,ydd) sono ricostruite da (theta,kappa,v) cosi' il controller resta
    // INVARIATO: ne ricava v_des=v, theta_des=theta, w_des=kappa*v.
    public void DetermineGeometricTrajectoryTableForController()
    {
        int nInstants = coefficients_xSpline.Count;
        if (nInstants == 0) return;

        // --- 1) campionamento fitto della geometria ---
        List<double> sArr = new List<double>(), xArr = new List<double>(), yArr = new List<double>();
        List<double> thArr = new List<double>(), kArr = new List<double>();
        double sAcc = 0.0, prevX = 0.0, prevY = 0.0;
        bool first = true;
        for (int k = 0; k < nInstants; k++)
        {
            double dtk = coefficients_xSpline[k].dt;
            double xa = coefficients_xSpline[k].ak, xb = coefficients_xSpline[k].bk, xc = coefficients_xSpline[k].ck, xd0 = coefficients_xSpline[k].dk;
            double ya = coefficients_ySpline[k].ak, yb = coefficients_ySpline[k].bk, yc = coefficients_ySpline[k].ck, yd0 = coefficients_ySpline[k].dk;

            for (int sub = first ? 0 : 1; sub <= subSamplesPerSpline; sub++)   // sub=0 saltato (== nodo precedente) tranne il primo
            {
                double tau = ((double)sub / subSamplesPerSpline) * dtk;
                double x = calculateSplineValue(xa, xb, xc, xd0, tau);
                double y = calculateSplineValue(ya, yb, yc, yd0, tau);
                double dx = calculateFirstDerivateSplineValue(xa, xb, xc, tau);
                double dy = calculateFirstDerivateSplineValue(ya, yb, yc, tau);
                double ddx = calculateSecondDerivateSplineValue(xa, xb, tau);
                double ddy = calculateSecondDerivateSplineValue(ya, yb, tau);

                double sp2 = dx * dx + dy * dy;
                double theta = Math.Atan2(dy, dx);
                double kappa = sp2 > 1e-9 ? (dx * ddy - dy * ddx) / (sp2 * Math.Sqrt(sp2)) : 0.0;   // (x'y''-y'x'')/|p'|^3

                if (!first) sAcc += Math.Sqrt((x - prevX) * (x - prevX) + (y - prevY) * (y - prevY));
                sArr.Add(sAcc); xArr.Add(x); yArr.Add(y); thArr.Add(theta); kArr.Add(kappa);
                prevX = x; prevY = y; first = false;
            }
        }
        double L = sArr[sArr.Count - 1];
        int n = sArr.Count;

        // --- 2) PROFILO DI VELOCITA' con passata forward-backward (TOPP-lite) ---
        double vCruise = Math.Min(linearMeanVelocity, vMax);    // crociera (disaccoppiata dalla spline), MAI oltre il clamp vMax
        const double vMin = 0.05;                               // floor: evita ds=0 (stallo) dove il profilo -> 0

        // cap statico: crociera + rallentamento in curva (kappa*v <= wMax)
        double[] vprof = new double[n];
        for (int i = 0; i < n; i++)
            vprof[i] = Math.Abs(kArr[i]) > 1e-4 ? Math.Min(vCruise, wMax / Math.Abs(kArr[i])) : vCruise;

        // forward pass (accelerazione, parte da fermo): v <= sqrt(v_prev^2 + 2 a ds)
        vprof[0] = 0.0;
        for (int i = 1; i < n; i++)
        {
            double ds = sArr[i] - sArr[i - 1];
            vprof[i] = Math.Min(vprof[i], Math.Sqrt(vprof[i - 1] * vprof[i - 1] + 2 * aMax * ds));
        }
        // backward pass (PRE-FRENATA in curva + arrivo a fermo): v <= sqrt(v_next^2 + 2 a ds)
        vprof[n - 1] = 0.0;
        for (int i = n - 2; i >= 0; i--)
        {
            double ds = sArr[i + 1] - sArr[i];
            vprof[i] = Math.Min(vprof[i], Math.Sqrt(vprof[i + 1] * vprof[i + 1] + 2 * aMax * ds));
        }
        for (int i = 0; i < n; i++) vprof[i] = Math.Max(vprof[i], vMin);   // floor per lo stepping

        // --- 3) ricampionamento a passo di controllo, avanzando l'arco col profilo v(s) ---
        double dt = 1.0 / secondsControlFrequency;
        double s = 0.0, t = 0.0;
        int j = 0;
        while (s < L)
        {
            while (j < n - 2 && sArr[j + 1] < s) j++;
            double seg = sArr[j + 1] - sArr[j];
            double a = seg > 1e-9 ? (s - sArr[j]) / seg : 0.0;
            double x = xArr[j] + a * (xArr[j + 1] - xArr[j]);   // posizione interpolata per arco (no stallo)
            double y = yArr[j] + a * (yArr[j + 1] - yArr[j]);
            double theta = thArr[j], kappa = kArr[j];           // theta/kappa: campione piu' vicino (variano piano)
            double v = vprof[j] + a * (vprof[j + 1] - vprof[j]);  // velocita' dal profilo forward-backward

            double xd = v * Math.Cos(theta), yd = v * Math.Sin(theta);
            double xdd = -v * v * kappa * Math.Sin(theta), ydd = v * v * kappa * Math.Cos(theta);   // v_dot ~ 0 (w_des=kappa*v resta esatto)
            geometricTrajectoryTableForController.Add(((float)t, (float)x, (float)y, (float)xd, (float)yd, (float)xdd, (float)ydd));

            s += v * dt;
            t += dt;
        }
        int last = sArr.Count - 1;
        geometricTrajectoryTableForController.Add(((float)t, (float)xArr[last], (float)yArr[last], 0f, 0f, 0f, 0f));   // goal a riposo
    }
}
