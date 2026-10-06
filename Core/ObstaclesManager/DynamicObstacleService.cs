using NUnit.Framework;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Diagnostics;
using static ICPUtils;
using static PoseMatrix4x4;
using static UnicycleModelUtilities;

public class DynamicObstacleService
{

    private float voxelSize;                 // dedicato alla detection: piu' fine di quello dell'ICP
    private Transform laserTransform;

    private float selfHitRadius;             // usato solo se il filtro a footprint e' disattivato
    private bool useChassisFootprint;        // scarta gli auto-hit col rettangolo VERO della scocca
    private float chassisFront;              // estensione della scocca davanti al laser [m]
    private float chassisRear;               // ...e dietro
    private float chassisHalfWidth;
    private float chassisTopRelLaser;        // quota del tetto scocca rispetto al laser (negativa)
    private float chassisMargin;
    private float tol;              // tolleranza a range 0: tol(d) = tol + tolPerMeter * d
    private float tolPerMeter;      // l'errore sul punto cresce col range (heading della localizzazione, rumore, chamfer)
    private float maxDetectionRange;// oltre questo range i punti sono ignorati (falsi positivi lontani, zone mai mappate)
    private float clusteringRadius;
    private int minClusterPoints;   // cluster con meno punti = rumore (dopo il voxel a voxelSize)
    private int minClusterPointsNear;        // soglia ridotta entro nearClusterRange
    private float nearClusterRange;
    private float clusterMargin;             // margine aggiunto al raggio del cerchio di ingombro
    private float maxUnexplainedFraction;    // oltre questa frazione di punti non spiegati la scansione e scartata
    private float minSuspiciousSpread;       // ...ma solo se sono SPARSI: concentrati sono un ostacolo vero
    private float obstacleZMin;              // banda di quota DEDICATA alla detection, piu' stretta di quella di mapping
    private float obstacleZMax;
    private float maxElevationDeg;           // solo verso l'ALTO: un ostacolo basso si vede con elevazione negativa

    private List<(float x, float y, float z)> lastUnexplainedPoints = new List<(float, float, float)>();
    private (int candidates, int unexplained, float spread, bool discarded, int clusters, int rejectedSmall, int rejectedSelf, int biggestCluster) lastDiagnostics;


    public DynamicObstacleService(float voxelSize, Transform marrtionLaserLinkTransform, float selfHitRadius, float obsTol, float obsTolPerMeter, float maxDetectionRange, float clusteringRadius, int minClusterPoints, int minClusterPointsNear, float nearClusterRange, float clusterMargin, float maxUnexplainedFraction, float minSuspiciousSpread, float obstacleZMin, float obstacleZMax, float maxElevationDeg,
        bool useChassisFootprint, float chassisFront, float chassisRear, float chassisHalfWidth, float chassisTopRelLaser, float chassisMargin)
    {
        this.voxelSize = voxelSize;
        this.laserTransform = marrtionLaserLinkTransform;
        this.selfHitRadius = selfHitRadius;
        this.tol = obsTol;
        this.tolPerMeter = obsTolPerMeter;
        this.maxDetectionRange = maxDetectionRange;
        this.clusteringRadius = clusteringRadius;
        this.minClusterPoints = minClusterPoints;
        this.minClusterPointsNear = minClusterPointsNear;
        this.nearClusterRange = nearClusterRange;
        this.clusterMargin = clusterMargin;
        this.maxUnexplainedFraction = maxUnexplainedFraction;
        this.minSuspiciousSpread = minSuspiciousSpread;
        this.obstacleZMin = obstacleZMin;
        this.obstacleZMax = obstacleZMax;
        this.maxElevationDeg = maxElevationDeg;
        this.useChassisFootprint = useChassisFootprint;
        this.chassisFront = chassisFront;
        this.chassisRear = chassisRear;
        this.chassisHalfWidth = chassisHalfWidth;
        this.chassisTopRelLaser = chassisTopRelLaser;
        this.chassisMargin = chassisMargin;
    }

    // Il laser e' montato 10 cm AVANTI al centro: una zona cieca circolare attorno ad esso scarta
    // anche punti che stanno davanti alla scocca, cioe' proprio dove si urta. Qui si scarta solo
    // cio' che cade dentro il rettangolo reale del telaio, espresso nel frame del laser
    // (Unity locale: +z avanti, +x destra, +y alto).
    private bool isChassisHit(Vector3 localPoint)
    {
        return Mathf.Abs(localPoint.x) <= chassisHalfWidth + chassisMargin
            && localPoint.z <= chassisFront + chassisMargin
            && localPoint.z >= -(chassisRear + chassisMargin)
            && localPoint.y <= chassisTopRelLaser + chassisMargin;
    }

    // Perche' una scansione non ha prodotto l'ostacolo atteso: e' l'unico modo per distinguere
    // "nessun punto candidato" da "scartati dalla tolleranza" da "cluster troppo piccolo".
    public (int candidates, int unexplained, float spread, bool discarded, int clusters, int rejectedSmall, int rejectedSelf, int biggestCluster) getLastDetectionDiagnostics()
    {
        return lastDiagnostics;
    }

    // Punti non spiegati con la LORO QUOTA: serve a capire in RViz a che altezza nascono i falsi positivi.
    public List<(float x, float y, float z)> getLastUnexplainedPoints3D()
    {
        return lastUnexplainedPoints;
    }

    public List<(float cx, float cy, float r, int n)> getROSObstacleCentroids(List<Vector3> scannedPoints, float[,] T_Map_laser, DistanceMap distanceMap)
    {

        List<(float cx, float cy, float r, int n)> result = new List<(float, float, float, int)>();

        // Posizione del laser in frame mappa ROS, dalla STESSA T con cui riproietto i punti (convenzione
        // toRosPose di LocalizationService: x = T[2,3], y = -T[0,3]). Serve per range e filtro auto-hit.
        (float x, float y, float z) laserROS = (T_Map_laser[2, 3], -T_Map_laser[0, 3], T_Map_laser[1, 3]);
        lastUnexplainedPoints = new List<(float, float, float)>();
        lastDiagnostics = (0, 0, 0f, false, 0, 0, 0, 0);
        float maxElevationRad = maxElevationDeg * Mathf.Deg2Rad;

        VoxelGrid vg = new VoxelGrid();
        List<Vector3> downsampledPoints = vg.Downsample(scannedPoints, voxelSize);
        List<Vector3> local_pts = ToLocalFrame(laserTransform, downsampledPoints);

        // candidato = punto 2D + range orizzontale dal laser (il range decide la tolleranza nel test successivo)
        List<(float x, float y, float z, float range)> candidates = new List<(float x, float y, float z, float range)>();
        foreach(Vector3 p in local_pts)
        {
            Vector3 pMap = applyTransformation(T_Map_laser, p);
            (float x, float y, float z) pMap_ROS = UnityToRosPosition(pMap.x, pMap.y, pMap.z);

            // quota ASSOLUTA: solo cio' che il robot puo' urtare (soffitti e pavimento fuori)
            if (pMap_ROS.z <= obstacleZMin || pMap_ROS.z >= obstacleZMax) continue;

            float dxl = pMap_ROS.x - laserROS.x;
            float dyl = pMap_ROS.y - laserROS.y;
            float range = Mathf.Sqrt(dxl * dxl + dyl * dyl);
            if (useChassisFootprint ? isChassisHit(p) : range < selfHitRadius) continue;   // auto-hit del corpo
            if (range > maxDetectionRange) continue;        // troppo lontano: inaffidabile e inutile per la CBF

            // Solo i raggi verso l'ALTO sono soffitto. Verso il BASSO ci sono gli ostacoli bassi
            // (un oggetto piu' basso del laser si vede per forza con elevazione negativa): li' il
            // filtro di quota obstacleZMin basta, e un gate simmetrico li cancellerebbe.
            if (Mathf.Atan2(pMap_ROS.z - laserROS.z, range) > maxElevationRad) continue;

            candidates.Add((pMap_ROS.x, pMap_ROS.y, pMap_ROS.z, range));
        }

        List<(float x, float y)> unexplained = new List<(float x, float y)>();
        DistanceMap dMapInstance = distanceMap;

        foreach ((float px, float py, float pz, float range) p in candidates)
        {
            (float px, float py) p2 = (p.px, p.py);
            if (!(dMapInstance.cellInMap(dMapInstance.getCellFromWorldPosition(p2)))) continue;
            int pidx = dMapInstance.getIndexFromWorldPosition(p2);
            float clearence = dMapInstance.getDistanceMap()[pidx] * dMapInstance.getResolution();
            float tolAtRange = tol + tolPerMeter * p.range;  // severo da vicino, permissivo da lontano
            if(clearence > tolAtRange)
            {
                unexplained.Add(p2);
                lastUnexplainedPoints.Add((p.px, p.py, p.pz));
            }
        }

        // Posa sbagliata vs ostacolo vicino: in entrambi i casi la frazione di punti non spiegati e' alta,
        // ma una posa sbagliata li sparpaglia su TUTTI i muri, mentre un ostacolo li concentra in un punto.
        // Senza il test di dispersione, avvicinandosi a un ostacolo grande si scartava la scansione buona.
        lastDiagnostics = (candidates.Count, unexplained.Count, getSpread(unexplained), false, 0, 0, 0, 0);

        if (candidates.Count > 0 && (float)unexplained.Count / candidates.Count > maxUnexplainedFraction
            && getSpread(unexplained) > minSuspiciousSpread)
        {
            lastDiagnostics.discarded = true;
            Debug.LogWarning($"Detection scartata: {unexplained.Count}/{candidates.Count} punti non spiegati e sparsi su {getSpread(unexplained):F1} m, localizzazione sospetta");
            return result;
        }

        Dictionary<(int cx, int cy), List<(float x, float y)>> buckets = new Dictionary<(int, int), List<(float, float)>>();
        foreach((float px, float py) p in unexplained)
        {
            (int cx, int cy) cellOfp = ((int)Mathf.Floor(p.px / clusteringRadius), (int)Mathf.Floor(p.py / clusteringRadius));
            if (!buckets.ContainsKey(cellOfp)) buckets[cellOfp] = new List<(float, float)>();
            buckets[cellOfp].Add(p);
        }

        // BFS sui bucket 8-connessi non ancora visitati: ogni componente connessa = un cluster.
        HashSet<(int cx, int cy)> visited = new HashSet<(int, int)>();
        foreach((int cx, int cy) seed in buckets.Keys)
        {
            if (visited.Contains(seed)) continue;

            List<(float x, float y)> cluster = new List<(float, float)>();
            Queue<(int cx, int cy)> frontier = new Queue<(int, int)>();
            frontier.Enqueue(seed);
            visited.Add(seed);

            while (frontier.Count > 0)
            {
                (int cx, int cy) cell = frontier.Dequeue();
                cluster.AddRange(buckets[cell]);

                for (int dx = -1; dx <= 1; dx++)
                {
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        if (dx == 0 && dy == 0) continue;
                        (int cx, int cy) neighbor = (cell.cx + dx, cell.cy + dy);
                        if (buckets.ContainsKey(neighbor) && !visited.Contains(neighbor))
                        {
                            visited.Add(neighbor);
                            frontier.Enqueue(neighbor);
                        }
                    }
                }
            }

            lastDiagnostics.clusters += 1;
            if (cluster.Count > lastDiagnostics.biggestCluster) lastDiagnostics.biggestCluster = cluster.Count;

            (float cx, float cy, float r) circle = clusterToCircle(cluster);

            // Il clustering a BFS puo' unire punti presi da direzioni opposte (p.es. la parete alle spalle
            // e qualcosa di lato): il cerchio risultante e' centrato ADDOSSO al robot. Non puo' essere un
            // ostacolo vero - il robot e' li' - e manderebbe il QP in h<<0, quindi si scarta.
            if ((circle.cx - laserROS.x) * (circle.cx - laserROS.x) + (circle.cy - laserROS.y) * (circle.cy - laserROS.y) < circle.r * circle.r)
            {
                lastDiagnostics.rejectedSelf += 1;
                continue;
            }

            // Avvicinandosi, la finestra di quota utile si restringe (FOV verticale + gate di elevazione)
            // e i voxel disponibili CALANO: una soglia fissa cancella proprio gli ostacoli piu' pericolosi.
            float distanceToCluster = Mathf.Sqrt((circle.cx - laserROS.x) * (circle.cx - laserROS.x) + (circle.cy - laserROS.y) * (circle.cy - laserROS.y));
            int required = distanceToCluster < nearClusterRange ? minClusterPointsNear : minClusterPoints;

            if (cluster.Count < required)
            {
                lastDiagnostics.rejectedSmall += 1;
                continue;                                     // rumore
            }

            result.Add((circle.cx, circle.cy, circle.r, cluster.Count));
        }

        return result;
    }

    // Lato maggiore del bounding box dei punti: distingue "sparso ovunque" da "concentrato su un oggetto".
    private float getSpread(List<(float x, float y)> points)
    {
        if (points.Count == 0) return 0f;

        float minX = float.MaxValue, maxX = float.MinValue, minY = float.MaxValue, maxY = float.MinValue;
        foreach ((float x, float y) p in points)
        {
            if (p.x < minX) minX = p.x;
            if (p.x > maxX) maxX = p.x;
            if (p.y < minY) minY = p.y;
            if (p.y > maxY) maxY = p.y;
        }
        return Mathf.Max(maxX - minX, maxY - minY);
    }

    // Cerchio di ingombro del cluster: centroide + raggio = distanza massima dal centroide + margine.
    private (float cx, float cy, float r) clusterToCircle(List<(float x, float y)> cluster)
    {
        float sx = 0f, sy = 0f;
        foreach ((float x, float y) p in cluster) { sx += p.x; sy += p.y; }
        float cx = sx / cluster.Count;
        float cy = sy / cluster.Count;

        float maxSqDist = 0f;
        foreach ((float x, float y) p in cluster)
        {
            float sq = (p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy);
            if (sq > maxSqDist) maxSqDist = sq;
        }
        return (cx, cy, Mathf.Sqrt(maxSqDist) + clusterMargin);
    }

}
