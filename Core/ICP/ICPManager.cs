using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using static ICPSolver;
using static MatrixVectorUtilities;
using static PoseMatrix4x4;
using static UnicycleModelUtilities;
using static ICPUtils;

public class ICPManager : MonoBehaviour
{
    public float deltaHuber = 0.2f;
    public ICPMode icpMode = ICPMode.P2C;
    public int maxIteration = 20;
    public float maxDistance = 1.0f;
    public float convergenceThreshold = 1e-5f;
    public float voxelSize = 0.1f;


    public float minDeltaTranslation = 0.1f;
    public float minDeltaRotation = 0.5f;
    int k = 15;
    float loopClosureRadius = 0.2f;
    float thresholdLoopClosure = 0.1f;

    private ArticulationBodyRefs articulationBodyRefs;
    private LiDAR3D mountedLidar;
    private Transform marrtionLaserLinkTransform;
    private Vector3 firstMarrtinoLocation;
    private Quaternion firstMarrtinoRotation;
    private KDTree kdTargetTree;
    private List<Vector3> targetLocalTreeListOfPoints;       //downsampled
    private VoxelGrid targetVoxelGrid;
    private KDTree kdSourceTree;
    private List<Vector3> sourceLocalTreeListOfPoints;       //downsampled
    private VoxelGrid sourceVoxelGrid;

    private int nodeCounter;
    private int edgeCounter;
    private PoseGraph poseGraph;
    private PoseNode newNode;
    private PoseNode lastNode;
    private PoseEdge newEdge;


    private float[,] TWorldICP;
    private float[,] TRelativeICP;
    List<Vector3> transformedWorldPointsList;

    public int nPosesPath = 500;

    private Vector3 icpEstimatedPos;
    private Queue<Vector3> icpPathPositions = new Queue<Vector3>();
    private PathMsg icpPathMsg = new PathMsg();


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        articulationBodyRefs = GetComponent<ArticulationBodyRefs>();
        mountedLidar = articulationBodyRefs.getMarrtinoLaserLinkArticulationBodyReference().GetComponent<LiDAR3D>();
        mountedLidar.OnScanComplete += ScanCompletedLetsICP;
        marrtionLaserLinkTransform = articulationBodyRefs.getMarrtinoLaserLinkTransformReference();
        TWorldICP = Identity4();
        TRelativeICP = Identity4();
        transformedWorldPointsList = new List<Vector3>();
        targetVoxelGrid = new VoxelGrid();
        sourceVoxelGrid = new VoxelGrid();
        poseGraph = new PoseGraph();
        nodeCounter = 0;
        edgeCounter = 0;
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PathMsg>("/icp/path");
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>("/icp/map");
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void ScanCompletedLetsICP()
    {
        if (kdTargetTree == null)
        {
            kdTargetTree = new KDTree();
            targetLocalTreeListOfPoints = ToLocalFrame(marrtionLaserLinkTransform, targetVoxelGrid.Downsample(mountedLidar.ScannedPoints, voxelSize));
            kdTargetTree.BuildTree(targetLocalTreeListOfPoints, 0);
            icpEstimatedPos = marrtionLaserLinkTransform.position;
            marrtionLaserLinkTransform.GetPositionAndRotation(out firstMarrtinoLocation,out firstMarrtinoRotation);
            lastNode = new PoseNode(Identity4(), targetLocalTreeListOfPoints, kdTargetTree);
            poseGraph.AddNode(lastNode);
            nodeCounter++;
            return;
        }

        kdSourceTree = new KDTree();
        sourceLocalTreeListOfPoints = ToLocalFrame(marrtionLaserLinkTransform, sourceVoxelGrid.Downsample(mountedLidar.ScannedPoints, voxelSize));
        kdSourceTree.BuildTree(sourceLocalTreeListOfPoints, 0);

        UpdateTWorldICP();

        float[] se3Vector = LogMap(TRelativeICP);
        float[] measuredTraslation = new float[3] { se3Vector[0], se3Vector[1], se3Vector[2]  };
        float[] measuredRotation = new float[3] {se3Vector[3], se3Vector[4], se3Vector[5] };

        if(getNormV3(measuredTraslation)> minDeltaTranslation || getNormV3(measuredRotation) > minDeltaRotation)
        {
            newNode = new PoseNode(TWorldICP, sourceLocalTreeListOfPoints, kdSourceTree);
            newEdge = new PoseEdge(lastNode.PoseID(), newNode.PoseID(), TRelativeICP, Identity6());
            poseGraph.AddNode(newNode);
            nodeCounter++;
            poseGraph.AddEdge(newEdge);
            edgeCounter++;
            
            //bool da studiare per far sì che SearchLoopClosure venga chiamato solo quando effettivamente il liad dopo K iterazioni ha ritrovato punti già noti, magari presenti in un qualche dizionario
            bool someConditionToBeFiguredOut = true;

            if (someConditionToBeFiguredOut)
            {
                List<PoseNode> candidates = poseGraph.SearchLoopClosureCandidates(newNode,k,loopClosureRadius);

                foreach(PoseNode candidate in candidates)
                {
                    float[,] deltaTCandidate = Solve(productSquareMatrix4(InverseT(candidate.PoseT()), newNode.PoseT()), candidate.PoseScannedPointsKDTree(), candidate.PoseScannedPoints(), newNode.PoseScannedPoints(), deltaHuber, maxIteration, maxDistance, convergenceThreshold, ICPMode.P2C);
                    float[] logError = LogMap(deltaTCandidate);
                    if (getNorm(logError)< thresholdLoopClosure)
                    {
                        PoseEdge closureEdge = new PoseEdge(newNode.PoseID(), candidate.PoseID(), deltaTCandidate, Identity6());
                        poseGraph.AddEdge(closureEdge);
                        edgeCounter++;
                    }
                }

            }
            
            lastNode = newNode;

        }

        TrasnformPointsForICPMap();
        PublishICPMap();

        PublishICPPath();

        targetLocalTreeListOfPoints = sourceLocalTreeListOfPoints;
        kdTargetTree = kdSourceTree;
    }

    private void TrasnformPointsForICPMap()
    {
        transformedWorldPointsList.Clear();
        Vector3 transformedP;
        foreach (Vector3 p in sourceLocalTreeListOfPoints)
        {
            transformedP = applyTransformation(TWorldICP, p);
            transformedWorldPointsList.Add(firstMarrtinoRotation*transformedP+firstMarrtinoLocation);
        }
    }

    private void PublishICPMap()
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

        ROSConnection.GetOrCreateInstance().Publish("/icp/map", msg);
    }

    private void UpdateTWorldICP()
    {
        float[,] deltaT = Solve(TRelativeICP, kdTargetTree, targetLocalTreeListOfPoints, sourceLocalTreeListOfPoints, deltaHuber, maxIteration, maxDistance, convergenceThreshold, icpMode);
        TRelativeICP = deltaT;
        TWorldICP = productSquareMatrix4(TWorldICP, TRelativeICP);
        // ICP path (cyan): accumula la traslazione stimata
        Vector3 localDelta = new Vector3(TRelativeICP[0, 3], TRelativeICP[1, 3], TRelativeICP[2, 3]);
        Vector3 worldDelta = marrtionLaserLinkTransform.TransformVector(localDelta);
        icpEstimatedPos += worldDelta;
        if (icpPathPositions.Count == nPosesPath)
            icpPathPositions.Dequeue();
        icpPathPositions.Enqueue(icpEstimatedPos);
    }

    private void PublishICPPath()
    {
        float currentTime = Time.time;
        int sec = (int)(uint)currentTime;
        uint nanosec = (uint)((currentTime - sec) * 1e9f);
        HeaderMsg header = new HeaderMsg { stamp = new TimeMsg(sec, nanosec), frame_id = "odom" };

        icpPathMsg.header = header;

        List<PoseStampedMsg> poses = new List<PoseStampedMsg>(icpPathPositions.Count);
        foreach (Vector3 point in icpPathPositions)
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

        ROSConnection.GetOrCreateInstance().Publish("/icp/path", icpPathMsg);
    }
}
