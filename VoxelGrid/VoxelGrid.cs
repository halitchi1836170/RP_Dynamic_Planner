using System.Collections.Generic;
using System.Linq;
using System.Reflection.Emit;
using Unity.VisualScripting;
using UnityEngine;

public class VoxelGrid
{
    private Dictionary<(int, int, int), (Vector3, int)> localBuckets;
    private Dictionary<(int, int, int), Vector3> localCentroids;

    public VoxelGrid()
    {
        localBuckets = new Dictionary<(int, int, int), (Vector3, int)>();
        localCentroids = new Dictionary<(int, int, int), Vector3>();
    }

    public void setLocalBuckets(Dictionary<(int, int, int), (Vector3, int)> buckets)
    {
        localBuckets = buckets;
    }

    public void setLocalCentroids(Dictionary<(int, int, int), Vector3> centroids)
    {
        localCentroids = centroids;
    }

    public List<Vector3> Downsample(List<Vector3> cloudPoints, float voxelSize = 0.1f)
    {
        Dictionary<(int, int, int), (Vector3, int)> buckets = new Dictionary<(int, int, int), (Vector3, int)>();
        Dictionary<(int, int, int), Vector3> centroids = new Dictionary<(int, int, int), Vector3>();

        for (int i = 0; i < cloudPoints.Count; i++)
        {
            Vector3 point = cloudPoints[i];
            int ix = (int)Mathf.Floor(point.x / voxelSize);
            int iy = (int)Mathf.Floor(point.y / voxelSize);
            int iz = (int)Mathf.Floor(point.z / voxelSize);
            (int, int, int) key = (ix, iy, iz);
            if (!buckets.ContainsKey(key))
            {
                buckets[key] = (Vector3.zero,0);
            }
            (Vector3 sum, int count) = buckets[key];
            buckets[key] = (sum + point, count + 1);
        }
        foreach(var kvp in buckets)
        {
            Vector3 centroid = kvp.Value.Item1 / kvp.Value.Item2;
            centroids[kvp.Key] = centroid;
        }

        setLocalBuckets(buckets);
        setLocalCentroids(centroids);

        return centroids.Values.ToList();
    }



}
