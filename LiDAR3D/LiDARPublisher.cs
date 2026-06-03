using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using System;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using System.Collections.Generic;

public class LiDARPublisher : MonoBehaviour
{
    public bool publishLIDAR = false;

    private LiDAR3D mountedLidar;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        mountedLidar = this.GetComponent<LiDAR3D>();
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>("/point_cloud");
        mountedLidar.OnScanComplete += Publish;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void Publish()
    {
        if (publishLIDAR)
        {
            //Debug.Log($"Publishing {mountedLidar.ScannedPoints.Count} points");

            //header properties
            HeaderMsg header = getPointCloud2MsgHeader();

            uint height = 1;
            uint width = (uint)mountedLidar.ScannedPoints.Count;

            PointFieldMsg x_fields = new PointFieldMsg("x", 0, PointFieldMsg.FLOAT32, 1);
            PointFieldMsg y_fields = new PointFieldMsg("y", 4, PointFieldMsg.FLOAT32, 1);
            PointFieldMsg z_fields = new PointFieldMsg("z", 8, PointFieldMsg.FLOAT32, 1);
            PointFieldMsg[] fields = new PointFieldMsg[] { x_fields, y_fields, z_fields };

            uint point_step = 3 * 4; //3 perché x y e z da 4 byte ciascuno
            uint row_step = (uint)(point_step * width);

            byte[] data = getScannedPointInBytesWithRosConvention(point_step, width);

            //assegnazione proprietà a msg
            PointCloud2Msg msg = new PointCloud2Msg();
            msg.header = header;
            msg.height = height;
            msg.width = width;
            msg.fields = fields;
            msg.is_bigendian = false;
            msg.point_step = point_step;
            msg.row_step = row_step;
            msg.data = data;
            msg.is_dense = true;

            ROSConnection.GetOrCreateInstance().Publish("/point_cloud", msg);
        }

    }

    private HeaderMsg getPointCloud2MsgHeader()
    {
        float currentTime = Time.time;
        int sec = (int)(uint)currentTime;
        uint nanosec = (uint)((currentTime - sec) * 1e9f);
        TimeMsg headerTimeMsg = new TimeMsg(sec, nanosec);

        string coordinateFramID = "odom";       //momentaneo, corretto: marrtino_laser_link
        HeaderMsg header = new HeaderMsg();
        header.stamp = headerTimeMsg;
        header.frame_id = coordinateFramID;
        return header;
    }

    private byte[] getScannedPointInBytesWithRosConvention(uint point_step, uint width)
    {
        byte[] data = new byte[point_step * width];
        for (int i = 0; i < mountedLidar.ScannedPoints.Count; i++)
        {
            Vector3 point = mountedLidar.ScannedPoints[i];
            int offset = i * (int)point_step;
            float rosX = point.z;       //causa differente convenzione tra ROS e Unity
            float rosY = -point.x;
            float rosZ = point.y;
            Array.Copy(BitConverter.GetBytes(rosX), 0, data, offset, 4);
            Array.Copy(BitConverter.GetBytes(rosY), 0, data, offset + 4, 4);
            Array.Copy(BitConverter.GetBytes(rosZ), 0, data, offset + 8, 4);
        }
        return data;
    }

}
