using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using RosMessageTypes.Visualization;
using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using Unity.VisualScripting;
using UnityEngine;
using static UnicycleModelUtilities;

public class PublishingService
{   

    public PublishingService()
    {

    }

    public void PublishICPPath(Queue<Vector3> icpWorldPositions, string topic)
    {
        PathMsg icpPathMsg = new PathMsg();

        float currentTime = Time.time;
        int sec = (int)(uint)currentTime;
        uint nanosec = (uint)((currentTime - sec) * 1e9f);
        HeaderMsg header = new HeaderMsg { stamp = new TimeMsg(sec, nanosec), frame_id = "odom" };

        icpPathMsg.header = header;

        List<PoseStampedMsg> poses = new List<PoseStampedMsg>(icpWorldPositions.Count);
        foreach (Vector3 point in icpWorldPositions)
        {
            PoseStampedMsg poseStamped = new PoseStampedMsg();
            poseStamped.header = header;
            poseStamped.pose = new PoseMsg
            {
                position = new PointMsg { x = point.z, y = -point.x, z = 0.0f },
                orientation = new QuaternionMsg { x = 0, y = 0, z = 0, w = 1 }
            };
            poses.Add(poseStamped);
        }
        icpPathMsg.poses = poses.ToArray();

        ROSConnection.GetOrCreateInstance().Publish(topic, icpPathMsg);
    }

    public void PublishICPMap(List<Vector3> transformedWorldPointsList, string topic)
    {
        float currentTime = Time.time;
        int sec = (int)(uint)currentTime;
        uint nanosec = (uint)((currentTime - sec) * 1e9f);

        uint pointStep = 12;
        uint width = (uint)transformedWorldPointsList.Count;
        byte[] data = new byte[pointStep * width];

        for (int i = 0; i < transformedWorldPointsList.Count; i++)
        {
            var (rx, ry, rz) = UnityToRosPosition(transformedWorldPointsList[i].x, transformedWorldPointsList[i].y, transformedWorldPointsList[i].z);
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

        ROSConnection.GetOrCreateInstance().Publish(topic, msg);
    }


    public void PublishLastOdometry((float, float, float) currentPosition, string topic)
    {
        float current_xt = currentPosition.Item1;
        float current_yt = currentPosition.Item2;
        float current_thetat = currentPosition.Item3;

        OdometryMsg msg = new OdometryMsg();
        msg.header = getOdometryMsgHeader();
        msg.child_frame_id = "base_link";

        PointMsg pose = new PointMsg();
        pose.x = current_xt;
        pose.y = current_yt;
        pose.z = 0.0f;
        (float, float, float, float) quaterion = GetQuaternionFromEuler(current_thetat, 0.0f, 0.0f);
        QuaternionMsg orientation = new QuaternionMsg();
        orientation.x = quaterion.Item1;
        orientation.y = quaterion.Item2;
        orientation.z = quaterion.Item3;
        orientation.w = quaterion.Item4;
        PoseWithCovarianceMsg poseWithCovarianceMsg = new PoseWithCovarianceMsg();
        poseWithCovarianceMsg.pose.position = pose;
        poseWithCovarianceMsg.pose.orientation = orientation;
        TwistWithCovarianceMsg twistWithCovarianceMsg = new TwistWithCovarianceMsg();

        msg.pose = poseWithCovarianceMsg;
        msg.twist = twistWithCovarianceMsg;

        ROSConnection.GetOrCreateInstance().Publish(topic, msg);
    }

    private HeaderMsg getOdometryMsgHeader()
    {
        var now = DateTimeOffset.UtcNow;
        int sec = (int)now.ToUnixTimeSeconds();
        uint nanosec = (uint)((now.ToUnixTimeMilliseconds() % 1000) * 1e6f);
        TimeMsg headerTimeMsg = new TimeMsg(sec, nanosec);

        string coordinateFramID = "odom";
        HeaderMsg header = new HeaderMsg();
        header.stamp = headerTimeMsg;
        header.frame_id = coordinateFramID;
        return header;
    }

    internal void PublishOdometryPath( Queue<Vector3> lastNPoses, string odometryPathRosTopic)
    {
        PathMsg pathMsg = new PathMsg();
        pathMsg.header = getPathMsgHeader();

        List<PoseStampedMsg> poses = new List<PoseStampedMsg>();
        foreach (Vector3 point in lastNPoses)
        {
            PoseStampedMsg poseStampedMsg = new PoseStampedMsg();
            poseStampedMsg.header = pathMsg.header;
            PoseMsg newPose = new PoseMsg();
            PointMsg newPosePoint = new PointMsg();
            newPosePoint.x = point.z;
            newPosePoint.y = -point.x;
            newPosePoint.z = 0.0f;
            newPose.position = newPosePoint;
            poseStampedMsg.pose = newPose;
            poses.Add(poseStampedMsg);
        }
        pathMsg.poses = poses.ToArray();
        ROSConnection.GetOrCreateInstance().Publish(odometryPathRosTopic, pathMsg);
    }

    private HeaderMsg getPathMsgHeader()
    {
        float currentTime = Time.time;
        int sec = (int)(uint)currentTime;
        uint nanosec = (uint)((currentTime - sec) * 1e9f);
        TimeMsg headerTimeMsg = new TimeMsg(sec, nanosec);

        string coordinateFramID = "odom";
        HeaderMsg header = new HeaderMsg();
        header.stamp = headerTimeMsg;
        header.frame_id = coordinateFramID;
        return header;
    }

    public void PublishTF((float x, float y, float z) currentPosition, Vector3 localPosition, Quaternion localRotation, string tfTopic)
    {
        TFMessageMsg tfMessage = new TFMessageMsg();

        TransformStampedMsg odom_base = new TransformStampedMsg();
        TransformStampedMsg base_laser = new TransformStampedMsg();

        odom_base.header = getTransformStampedMsgHeader("odom");
        base_laser.header = getTransformStampedMsgHeader("base_link");

        odom_base.child_frame_id = "base_link";
        base_laser.child_frame_id = "marrtino_laser_link";

        TransformMsg odom_base_Transform = new TransformMsg();
        Vector3Msg odom_base_v3 = new Vector3Msg();
        (float, float, float) current_pos = currentPosition;
        odom_base_v3.x = current_pos.Item1;
        odom_base_v3.y = current_pos.Item2;
        odom_base_v3.z = 0.0f;
        QuaternionMsg odom_base_quat = new QuaternionMsg();
        (float, float, float, float) current_theta_quaternion = GetQuaternionFromEuler(current_pos.Item3, 0.0f, 0.0f);
        odom_base_quat.x = current_theta_quaternion.Item1;
        odom_base_quat.y = current_theta_quaternion.Item2;
        odom_base_quat.z = current_theta_quaternion.Item3;
        odom_base_quat.w = current_theta_quaternion.Item4;
        odom_base_Transform.translation = odom_base_v3;
        odom_base_Transform.rotation = odom_base_quat;

        TransformMsg base_laser_Transform = new TransformMsg();
        Vector3 localPos = localPosition;
        (float, float, float) rosTransformedPosition = UnityToRosPosition(localPos.x, localPos.y, localPos.z);
        base_laser_Transform.translation = new Vector3Msg(rosTransformedPosition.Item1, rosTransformedPosition.Item2, rosTransformedPosition.Item3);
        Quaternion localRot = localRotation;
        (float, float, float, float) rosTransformedRotation = UnityToRosRotation(localRot.x, localRot.y, localRot.z, localRot.w);
        base_laser_Transform.rotation = new QuaternionMsg(rosTransformedRotation.Item1, rosTransformedRotation.Item2, rosTransformedRotation.Item3, rosTransformedRotation.Item4);

        odom_base.transform = odom_base_Transform;
        base_laser.transform = base_laser_Transform;

        tfMessage.transforms = new TransformStampedMsg[2] { odom_base, base_laser };

        ROSConnection.GetOrCreateInstance().Publish(tfTopic, tfMessage);
    
    }

    private HeaderMsg getTransformStampedMsgHeader(string frameID)
    {
        var now = DateTimeOffset.UtcNow;
        int sec = (int)now.ToUnixTimeSeconds();
        uint nanosec = (uint)((now.ToUnixTimeMilliseconds() % 1000) * 1e6f);
        TimeMsg headerTimeMsg = new TimeMsg(sec, nanosec);

        string coordinateFramID = frameID;
        HeaderMsg header = new HeaderMsg();
        header.stamp = headerTimeMsg;
        header.frame_id = coordinateFramID;
        return header;
    }

    internal void PublishUpdatedGlobalPointCloudMap(List<Vector3> globalPointCloud, string graphSlamGlobalpointCloudTopic)
    {
        HeaderMsg header = getPointCloud2MsgHeader();

        uint height = 1;
        uint width = (uint)globalPointCloud.Count;

        PointFieldMsg x_fields = new PointFieldMsg("x", 0, PointFieldMsg.FLOAT32, 1);
        PointFieldMsg y_fields = new PointFieldMsg("y", 4, PointFieldMsg.FLOAT32, 1);
        PointFieldMsg z_fields = new PointFieldMsg("z", 8, PointFieldMsg.FLOAT32, 1);
        PointFieldMsg[] fields = new PointFieldMsg[] { x_fields, y_fields, z_fields };

        uint point_step = 3 * 4; //3 perché x y e z da 4 byte ciascuno
        uint row_step = (uint)(point_step * width);

        byte[] data = getScannedPointInBytesWithRosConvention(globalPointCloud, point_step, width);

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

        ROSConnection.GetOrCreateInstance().Publish(graphSlamGlobalpointCloudTopic, msg);
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

    private byte[] getScannedPointInBytesWithRosConvention(List<Vector3> points, uint point_step, uint width)
    {
        byte[] data = new byte[point_step * width];
        for (int i = 0; i < points.Count; i++)
        {
            Vector3 point = points[i];
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

    public void PublishGraphNodes(List<Vector3> nodePositions, string topic)
    {
        if (nodePositions.Count == 0) return;

        uint pointStep = 12;
        List<byte> dataList = new List<byte>(nodePositions.Count * (int)pointStep);
        int validCount = 0;
        foreach (Vector3 p in nodePositions)
        {
            var (rx, ry, rz) = UnityToRosPosition(p.x, p.y, p.z);
            if (float.IsNaN(rx) || float.IsNaN(ry) || float.IsNaN(rz)) continue;
            if (Mathf.Abs(rx) > 1e4f || Mathf.Abs(ry) > 1e4f || Mathf.Abs(rz) > 1e4f) continue;
            dataList.AddRange(BitConverter.GetBytes(rx));
            dataList.AddRange(BitConverter.GetBytes(ry));
            dataList.AddRange(BitConverter.GetBytes(rz));
            validCount++;
        }
        if (validCount == 0) return;

        uint width = (uint)validCount;
        byte[] data = dataList.ToArray();

        PointCloud2Msg msg = new PointCloud2Msg();
        msg.header = getPointCloud2MsgHeader();
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

        ROSConnection.GetOrCreateInstance().Publish(topic, msg);
    }

    public void PublishLoopClosureCircle(Vector3 vCenter, float radius, string topic, int nLat = 10, int nLon = 20)
    {
        int totalPoints = nLat * nLon;
        uint pointStep = 12;
        byte[] data = new byte[pointStep * totalPoints];

        int idx = 0;
        for (int iLat = 0; iLat < nLat; iLat++)
        {
            float lat = (float)Math.PI * (-0.5f + (float)iLat / (nLat - 1));
            for (int iLon = 0; iLon < nLon; iLon++)
            {
                float lon = 2.0f * (float)Math.PI * (float)iLon / nLon;
                float ux = vCenter.x + radius * (float)Math.Cos(lat) * (float)Math.Cos(lon);
                float uy = vCenter.y + radius * (float)Math.Sin(lat);
                float uz = vCenter.z + radius * (float)Math.Cos(lat) * (float)Math.Sin(lon);
                var (rx, ry, rz) = UnityToRosPosition(ux, uy, uz);
                int offset = idx * (int)pointStep;
                Array.Copy(BitConverter.GetBytes(rx), 0, data, offset, 4);
                Array.Copy(BitConverter.GetBytes(ry), 0, data, offset + 4, 4);
                Array.Copy(BitConverter.GetBytes(rz), 0, data, offset + 8, 4);
                idx++;
            }
        }

        PointCloud2Msg msg = new PointCloud2Msg();
        msg.header = getPointCloud2MsgHeader();
        msg.height = 1;
        msg.width = (uint)totalPoints;
        msg.fields = new PointFieldMsg[]
        {
            new PointFieldMsg("x", 0, PointFieldMsg.FLOAT32, 1),
            new PointFieldMsg("y", 4, PointFieldMsg.FLOAT32, 1),
            new PointFieldMsg("z", 8, PointFieldMsg.FLOAT32, 1),
        };
        msg.is_bigendian = false;
        msg.point_step = pointStep;
        msg.row_step = pointStep * (uint)totalPoints;
        msg.data = data;
        msg.is_dense = true;

        ROSConnection.GetOrCreateInstance().Publish(topic, msg);
    }

    public void PublishEmptyPointCloud(string topic)
    {
        PointCloud2Msg msg = new PointCloud2Msg();
        msg.header = getPointCloud2MsgHeader();
        msg.height = 1;
        msg.width = 0;
        msg.fields = new PointFieldMsg[]
        {
            new PointFieldMsg("x", 0, PointFieldMsg.FLOAT32, 1),
            new PointFieldMsg("y", 4, PointFieldMsg.FLOAT32, 1),
            new PointFieldMsg("z", 8, PointFieldMsg.FLOAT32, 1),
        };
        msg.is_bigendian = false;
        msg.point_step = 12;
        msg.row_step = 0;
        msg.data = new byte[0];
        msg.is_dense = true;
        ROSConnection.GetOrCreateInstance().Publish(topic, msg);
    }

    public void PublishLoopClosureEdges(List<(Vector3 from, Vector3 to)> edges, string topic, int pointsPerEdge = 12)
    {
        if (edges.Count == 0) return;

        int totalPoints = edges.Count * pointsPerEdge;
        uint pointStep = 12;
        byte[] data = new byte[pointStep * totalPoints];

        int idx = 0;
        foreach (var (from, to) in edges)
        {
            for (int i = 0; i < pointsPerEdge; i++)
            {
                float t = (float)i / (pointsPerEdge - 1);
                float ux = from.x + t * (to.x - from.x);
                float uy = from.y + t * (to.y - from.y);
                float uz = from.z + t * (to.z - from.z);
                var (rx, ry, rz) = UnityToRosPosition(ux, uy, uz);
                int offset = idx * (int)pointStep;
                Array.Copy(BitConverter.GetBytes(rx), 0, data, offset, 4);
                Array.Copy(BitConverter.GetBytes(ry), 0, data, offset + 4, 4);
                Array.Copy(BitConverter.GetBytes(rz), 0, data, offset + 8, 4);
                idx++;
            }
        }

        PointCloud2Msg msg = new PointCloud2Msg();
        msg.header = getPointCloud2MsgHeader();
        msg.height = 1;
        msg.width = (uint)totalPoints;
        msg.fields = new PointFieldMsg[]
        {
            new PointFieldMsg("x", 0, PointFieldMsg.FLOAT32, 1),
            new PointFieldMsg("y", 4, PointFieldMsg.FLOAT32, 1),
            new PointFieldMsg("z", 8, PointFieldMsg.FLOAT32, 1),
        };
        msg.is_bigendian = false;
        msg.point_step = pointStep;
        msg.row_step = pointStep * (uint)totalPoints;
        msg.data = data;
        msg.is_dense = true;

        ROSConnection.GetOrCreateInstance().Publish(topic, msg);
    }

    // Ventaglio/forbice del cono di ricerca loop closure, centrato sul robot (Unity world).
    // Disegna i due raggi laterali a ±halfAngle e l'arco al raggio, sul piano orizzontale X-Z
    // (il cono di ricerca è 2D, Y = verticale). apex/forward sono in Unity world.
    public void PublishConeFan(Vector3 apex, Vector3 forwardWorld, float halfAngleDeg, float minRadius, float maxRadius, string topic, int arcSegments = 24, int pointsPerEdge = 20)
    {
        Vector3 fwd = new Vector3(forwardWorld.x, 0f, forwardWorld.z);
        if (fwd.sqrMagnitude < 1e-12f) return;
        fwd.Normalize();

        Vector3 leftDir = Quaternion.AngleAxis(halfAngleDeg, Vector3.up) * fwd;
        Vector3 rightDir = Quaternion.AngleAxis(-halfAngleDeg, Vector3.up) * fwd;

        List<Vector3> pts = new List<Vector3>();
        // due raggi laterali (dal vertice fino al raggio)
        for (int i = 0; i <= pointsPerEdge; i++)
        {
            float t = maxRadius * i / pointsPerEdge;
            pts.Add(apex + leftDir * t);
            pts.Add(apex + rightDir * t);
        }
        // arco che chiude il ventaglio
        for (int i = 0; i <= arcSegments; i++)
        {
            float a = Mathf.Lerp(-halfAngleDeg, halfAngleDeg, (float)i / arcSegments);
            Vector3 dir = Quaternion.AngleAxis(a, Vector3.up) * fwd;
            pts.Add(apex + dir * maxRadius);
            pts.Add(apex + dir * minRadius);
        }

        uint pointStep = 12;
        uint width = (uint)pts.Count;
        byte[] data = new byte[pointStep * width];
        for (int i = 0; i < pts.Count; i++)
        {
            var (rx, ry, rz) = UnityToRosPosition(pts[i].x, pts[i].y, pts[i].z);
            int offset = i * (int)pointStep;
            Array.Copy(BitConverter.GetBytes(rx), 0, data, offset, 4);
            Array.Copy(BitConverter.GetBytes(ry), 0, data, offset + 4, 4);
            Array.Copy(BitConverter.GetBytes(rz), 0, data, offset + 8, 4);
        }

        PointCloud2Msg msg = new PointCloud2Msg();
        msg.header = getPointCloud2MsgHeader();
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

        ROSConnection.GetOrCreateInstance().Publish(topic, msg);
    }

    internal void PublishOccupancyGridMap((sbyte[] data, int width, int height, float originX, float originY, float resolution) dataReturned, string occupancyRosTopic)
    {
        //Debug.Log($"OCC received: len={dataReturned.data.Length} W={dataReturned.width} H={dataReturned.height}");

        if ( dataReturned.data is null || dataReturned.data.Length == 0 || dataReturned.width <= 0 || dataReturned.height <= 0) return;

        OccupancyGridMsg msg = new OccupancyGridMsg();
        msg.header = getPointCloud2MsgHeader();   // frame_id "odom", stesso degli altri topic

        msg.info = new MapMetaDataMsg();
        msg.info.resolution = dataReturned.resolution;
        msg.info.width = (uint)dataReturned.width;
        msg.info.height = (uint)dataReturned.height;
        msg.info.origin = new PoseMsg
        {
            position = new PointMsg(dataReturned.originX, dataReturned.originY, 0.0),       // angolo in basso-sinistra (metri)
            orientation = new QuaternionMsg(0, 0, 0, 1)            // nessuna rotazione della griglia
        };

        msg.data = dataReturned.data;   // row-major, valori 0..100 e -1 = sconosciuto

        ROSConnection.GetOrCreateInstance().Publish(occupancyRosTopic, msg);
    }


    internal void PublishDistanceMap((sbyte[] data, int width, int height, float originX, float originY, float resolution) dataReturned, string distanceMapRosTopic)
    {
        //Debug.Log($"OCC received: len={dataReturned.data.Length} W={dataReturned.width} H={dataReturned.height}");

        if (dataReturned.data is null || dataReturned.data.Length == 0 || dataReturned.width <= 0 || dataReturned.height <= 0) return;

        OccupancyGridMsg msg = new OccupancyGridMsg();
        msg.header = getPointCloud2MsgHeader();   // frame_id "odom", stesso degli altri topic

        msg.info = new MapMetaDataMsg();
        msg.info.resolution = dataReturned.resolution;
        msg.info.width = (uint)dataReturned.width;
        msg.info.height = (uint)dataReturned.height;
        msg.info.origin = new PoseMsg
        {
            position = new PointMsg(dataReturned.originX, dataReturned.originY, 0.0),       // angolo in basso-sinistra (metri)
            orientation = new QuaternionMsg(0, 0, 0, 1)            // nessuna rotazione della griglia
        };

        msg.data = dataReturned.data;   // row-major, valori 0..100 e -1 = sconosciuto

        ROSConnection.GetOrCreateInstance().Publish(distanceMapRosTopic, msg);
    }

    // Traiettoria pianificata come nav_msgs/Path. I punti arrivano GIA' in coordinate ROS
    // (frame della griglia, X-Y di "odom"): originX/originY + (col/row)*resolution -> NIENTE UnityToRos.
    public void PublishPlannedTrajectory(List<(float x, float y)> pathWorld, string topic)
    {
        if (pathWorld == null) return;

        PathMsg pathMsg = new PathMsg();
        HeaderMsg header = getPointCloud2MsgHeader();   // frame_id "odom", stesso di occupancy/distance map
        pathMsg.header = header;

        List<PoseStampedMsg> poses = new List<PoseStampedMsg>(pathWorld.Count);
        foreach ((float x, float y) p in pathWorld)
        {
            PoseStampedMsg ps = new PoseStampedMsg();
            ps.header = header;
            ps.pose = new PoseMsg
            {
                position = new PointMsg(p.x, p.y, 0.0),       // già ROS, nessuna conversione
                orientation = new QuaternionMsg(0, 0, 0, 1)
            };
            poses.Add(ps);
        }
        pathMsg.poses = poses.ToArray();

        ROSConnection.GetOrCreateInstance().Publish(topic, pathMsg);
    }

    // Pubblica un singolo punto (debug) come PointCloud2. Il punto è GIA' in coordinate ROS
    // (frame griglia, X-Y di "odom") -> nessuna conversione. Riusabile per start, goal, ecc.
    // In RViz: display PointCloud2 con Size grande (es. 0.15) per vederlo.
    public void PublishDebugPoint((float x, float y) rosPoint, string topic)
    {
        uint pointStep = 12;
        uint width = 1;
        byte[] data = new byte[pointStep * width];
        Array.Copy(BitConverter.GetBytes(rosPoint.x), 0, data, 0, 4);
        Array.Copy(BitConverter.GetBytes(rosPoint.y), 0, data, 4, 4);
        Array.Copy(BitConverter.GetBytes(0.0f), 0, data, 8, 4);

        PointCloud2Msg msg = new PointCloud2Msg();
        msg.header = getPointCloud2MsgHeader();   // frame_id "odom", stesso di griglia/path
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

        ROSConnection.GetOrCreateInstance().Publish(topic, msg);
    }

}
