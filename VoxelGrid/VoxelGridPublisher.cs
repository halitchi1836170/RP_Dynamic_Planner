using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using static UnicycleModelUtilities;

public class VoxelGridPublisher : MonoBehaviour
{
    public float voxelSize = 0.1f;
    public bool publishVoxelGrid = false;

    private LiDAR3D mountedLidar;
    private VoxelGrid voxelGrid;

    void Start()
    {
        mountedLidar = GetComponent<LiDAR3D>();
        voxelGrid = new VoxelGrid();
        mountedLidar.OnScanComplete += PublishCentroids;
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>("/voxelgrid_centroids");
    }

    void Update() { }

    private void PublishCentroids()
    {
        if (publishVoxelGrid)
        {
            List<Vector3> centroids = voxelGrid.Downsample(mountedLidar.ScannedPoints, voxelSize);

            float currentTime = Time.time;
            int sec = (int)(uint)currentTime;
            uint nanosec = (uint)((currentTime - sec) * 1e9f);

            uint pointStep = 12;
            uint width = (uint)centroids.Count;
            byte[] data = new byte[pointStep * width];

            for (int i = 0; i < centroids.Count; i++)
            {
                var (rx, ry, rz) = UnityToRosPosition(centroids[i].x, centroids[i].y, centroids[i].z);
                int offset = i * (int)pointStep;
                Array.Copy(BitConverter.GetBytes(rx), 0, data, offset, 4);
                Array.Copy(BitConverter.GetBytes(ry), 0, data, offset + 4, 4);
                Array.Copy(BitConverter.GetBytes(rz), 0, data, offset + 8, 4);
            }

            PointCloud2Msg msg = new PointCloud2Msg();
            msg.header = new HeaderMsg { stamp = new TimeMsg(sec, nanosec), frame_id = "odom" };
            msg.height = 1;
            msg.width = width;
            msg.fields = new PointFieldMsg[]
            {
            new PointFieldMsg("x", 0, PointFieldMsg.FLOAT32, 1),
            new PointFieldMsg("y", 4, PointFieldMsg.FLOAT32, 1),
            new PointFieldMsg("z", 8, PointFieldMsg.FLOAT32, 1),
            };
            msg.is_bigendian = false;
            msg.point_step = pointStep;
            msg.row_step = pointStep * width;
            msg.data = data;
            msg.is_dense = true;

            ROSConnection.GetOrCreateInstance().Publish("/voxelgrid_centroids", msg);
        }

    }
}
