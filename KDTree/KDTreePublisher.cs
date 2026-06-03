using System.Collections.Generic;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Visualization;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class KDTreePublisher : MonoBehaviour
{
    public int maxDepth = 15;
    public float lineWidth = 0.05f;
    public bool publishKDTree = false;

    private LiDAR3D mountedLidar;
    private KDTree kdTree;

    void Start()
    {
        mountedLidar = GetComponent<LiDAR3D>();
        kdTree = new KDTree();
        mountedLidar.OnScanComplete += PublishTree;
        ROSConnection.GetOrCreateInstance().RegisterPublisher<MarkerArrayMsg>("/kdtree_viz");
    }

    private void PublishTree()
    {
        if (publishKDTree)
        {
            kdTree.BuildTree(mountedLidar.ScannedPoints, 0);
            List<(Vector3 from, Vector3 to, int depth)> edges = kdTree.GetEdges(maxDepth);

            float currentTime = Time.time;
            int sec = (int)(uint)currentTime;
            uint nanosec = (uint)((currentTime - sec) * 1e9f);

            MarkerMsg marker = new MarkerMsg();
            marker.header = new HeaderMsg { stamp = new TimeMsg(sec, nanosec), frame_id = "odom" };
            marker.ns = "kdtree";
            marker.id = 0;
            marker.type = 5;   // LINE_LIST
            marker.action = 0; // ADD
            marker.scale = new Vector3Msg { x = lineWidth, y = lineWidth, z = lineWidth };
            marker.color = new ColorRGBAMsg { r = 1f, g = 1f, b = 1f, a = 1f };
            marker.pose = new PoseMsg
            {
                position = new PointMsg(0, 0, 0),
                orientation = new QuaternionMsg(0, 0, 0, 1)
            };

            var points = new List<PointMsg>(edges.Count * 2);

            foreach (var (from, to, depth) in edges)
            {
                points.Add(new PointMsg(from.z, -from.x, from.y));
                points.Add(new PointMsg(to.z, -to.x, to.y));
            }

            marker.points = points.ToArray();
            marker.colors = new ColorRGBAMsg[0];

            //Debug.Log($"KDTree edges: {edges.Count}, points: {points.Count}");

            ROSConnection.GetOrCreateInstance().Publish("/kdtree_viz", new MarkerArrayMsg
            {
                markers = new MarkerMsg[] { marker }
            });
        }

    }
}
