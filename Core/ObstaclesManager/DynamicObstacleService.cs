using NUnit.Framework;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Diagnostics;
using static ICPUtils;
using static PoseMatrix4x4;
using static UnicycleModelUtilities;

public class DynamicObstacleService
{

    private float voxelSize;
    private Transform laserTransform;

    private float zMin;
    private float zMax;
    private float bodyRadius;
    private float tol;              // tolleranza a range 0: tol(d) = tol + tolPerMeter * d
    private float tolPerMeter;      // l'errore sul punto cresce col range (heading della localizzazione, rumore, chamfer)
    private float maxDetectionRange;// oltre questo range i punti sono ignorati (falsi positivi lontani, zone mai mappate)
    private float clusteringRadius;
    private int minClusterPoints;   // cluster con meno punti = rumore (dopo il voxel a voxelSize)
    private float clusterMargin;    // margine aggiunto al raggio del cerchio di ingombro


    public DynamicObstacleService(float voxelSize, Transform marrtionLaserLinkTransform, float zMin, float zMax, float bodyRadius, float obsTol, float obsTolPerMeter, float maxDetectionRange, float clusteringRadius, int minClusterPoints, float clusterMargin)
    {
        this.voxelSize = voxelSize;
        this.laserTransform = marrtionLaserLinkTransform;
        this.zMin = zMin;
        this.zMax = zMax;
        this.bodyRadius = bodyRadius;
        this.tol = obsTol;
        this.tolPerMeter = obsTolPerMeter;
        this.maxDetectionRange = maxDetectionRange;
        this.clusteringRadius = clusteringRadius;
        this.minClusterPoints = minClusterPoints;
        this.clusterMargin = clusterMargin;
    }

    public List<(float cx, float cy, float r)> getROSObstacleCentroids(List<Vector3> scannedPoints, float[,] T_Map_laser, DistanceMap distanceMap)
    {

        List<(float cx, float cy, float r)> result = new List<(float, float, float)>();

        // Posizione del laser in frame mappa ROS, dalla STESSA T con cui riproietto i punti (convenzione
        // toRosPose di LocalizationService: x = T[2,3], y = -T[0,3]). Serve per range e filtro auto-hit.
        (float x, float y) laserROS = (T_Map_laser[2, 3], -T_Map_laser[0, 3]);

        VoxelGrid vg = new VoxelGrid();
        List<Vector3> downsampledPoints = vg.Downsample(scannedPoints, voxelSize);
        List<Vector3> local_pts = ToLocalFrame(laserTransform, downsampledPoints);

        // candidato = punto 2D + range orizzontale dal laser (il range decide la tolleranza nel test successivo)
        List<(float x, float y, float range)> candidates = new List<(float x, float y, float range)>();
        foreach(Vector3 p in local_pts)
        {
            Vector3 pMap = applyTransformation(T_Map_laser, p);
            (float x, float y, float z) pMap_ROS = UnityToRosPosition(pMap.x, pMap.y, pMap.z);
            if (pMap_ROS.z <= zMin || pMap_ROS.z >= zMax) continue;

            float dxl = pMap_ROS.x - laserROS.x;
            float dyl = pMap_ROS.y - laserROS.y;
            float range = Mathf.Sqrt(dxl * dxl + dyl * dyl);
            if (range < bodyRadius) continue;               // auto-hit del corpo
            if (range > maxDetectionRange) continue;        // troppo lontano: inaffidabile e inutile per la CBF

            candidates.Add((pMap_ROS.x, pMap_ROS.y, range));
        }

        List<(float x, float y)> unexplained = new List<(float x, float y)>();
        DistanceMap dMapInstance = distanceMap;

        foreach ((float px, float py, float range) p in candidates)
        {
            (float px, float py) p2 = (p.px, p.py);
            if (!(dMapInstance.cellInMap(dMapInstance.getCellFromWorldPosition(p2)))) continue;
            int pidx = dMapInstance.getIndexFromWorldPosition(p2);
            float clearence = dMapInstance.getDistanceMap()[pidx] * dMapInstance.getResolution();
            float tolAtRange = tol + tolPerMeter * p.range;  // severo da vicino, permissivo da lontano
            if(clearence > tolAtRange)
            {
                unexplained.Add(p2);
            }
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

            if (cluster.Count < minClusterPoints) continue;   // rumore

            result.Add(clusterToCircle(cluster));
        }

        return result;
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
