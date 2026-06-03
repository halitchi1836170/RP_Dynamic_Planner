using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Tf2;
using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using static MatrixVectorUtilities;
using static PoseMatrix4x4;

public class Orchestrator : MonoBehaviour
{
    //--------------------------------------------------------------------------------------------------------------------------------------------------------
    //                                                                   PUBLIC FIELDS
    //--------------------------------------------------------------------------------------------------------------------------------------------------------
    public string ICPFields = "FOLLOWING ICP PUBLIC FIELDS";
    public ICPMode icpMode = ICPMode.P2C;
    public float deltaHuber = 0.2f;
    public int maxIteration = 20;
    public float maxDistance = 1.0f;
    public float convergenceThreshold = 1e-5f;
    public float voxelSize = 0.1f;
    public int nPosesPath = 250;

    public string ROSTOPICS = "FOLLOWING ROS TOPICS";
    public string icpMapRosTopic = "/icp/map";
    public string icpPathRosTopic = "/icp/path";
    public string odometryRosTopic = "/odometry";
    public string odometryPathRosTopic = "/odometry/path";
    public string tfRosTopic = "/tf";
    public string graphSlamGlobalpointCloudTopic = "/icp/map_global";
    public string graphSlamNodesTopic = "/graph_slam/nodes";
    public string loopClosureCircleTopic = "/graph_slam/loop_closure_radius";
    public string loopClosureEdgesTopic = "/graph_slam/loop_closure_edges";

    public string GRASLAMFIELDS = "FOLLOWING GRAPH SLAM FIELDS";
    public float minDeltaTranslation = 0.1f;
    public float minDeltaRotation = 0.5f;
    public int k_midDeltaIDBetweenCandidates = 15;
    public int k_tryLoopClosure = 10;
    public float loopClosureRadius = 1.0f;
    public float thresholdLoopClosure = 0.5f;
    public int maxIterations = 50;
    public int maxCGIterations = 100;
    public float graphSlamOptimizerConvergenceThreshold = 1e-6f;
    public float CGConvergenceThreshold = 1e-6f;

    public string ODOMETRYFILEDS = "FOLLOWING ODOMETRY FILEDS";
    public float angularVelocityThreshold = 0.001f;
    public float wheelVelocityThreshold = 0.05f;
    public float odometryFrequency = 55.0f; //Hz
    public bool publishLastNOdometryPoses = true;
    public int NOdometryLastPoses = 250;

    //--------------------------------------------------------------------------------------------------------------------------------------------------------
    //                                                                    HARDWARE FIELDS
    //--------------------------------------------------------------------------------------------------------------------------------------------------------
    ArticulationBodyRefs articulationBodiesRefs;
    private ArticulationBody leftWheel;
    private ArticulationBody rightWheel;
    private LiDAR3D lidar;
    private Transform marrtionLaserLinkTransform;




    //--------------------------------------------------------------------------------------------------------------------------------------------------------
    //                                                                       SERVICES
    //--------------------------------------------------------------------------------------------------------------------------------------------------------
    private ICPService icpService;
    private GraphSlamService graphSlamService;
    private PublishingService publisherService;
    private OdometryService odometryService;
    private List<(Vector3 from, Vector3 to)> closureEdgesWorldPositions = new List<(Vector3, Vector3)>();



    //--------------------------------------------------------------------------------------------------------------------------------------------------------
    //                                                                       ROBOT STATE
    //--------------------------------------------------------------------------------------------------------------------------------------------------------



    private void Awake()
    {
        articulationBodiesRefs = this.GetComponent<ArticulationBodyRefs>();
        (ArticulationBody lw, ArticulationBody rw) wheels = articulationBodiesRefs.getWheelArticulationBodyReference();
        leftWheel = wheels.lw;
        rightWheel = wheels.rw;
        lidar = articulationBodiesRefs.getMarrtinoLaserLinkArticulationBodyReference().GetComponent<LiDAR3D>();
        marrtionLaserLinkTransform = articulationBodiesRefs.getMarrtinoLaserLinkTransformReference();

        //Debug.Log($"left wheel body var test name: {leftWheel.name}");
    }




    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        //------------------INIZIALIZZAZIONE STATO




        //------------------INIZIALIZZAZIONE SERVIZI
        graphSlamService = new GraphSlamService(maxIterations, maxCGIterations, graphSlamOptimizerConvergenceThreshold, CGConvergenceThreshold, minDeltaTranslation, minDeltaRotation);
        icpService = new ICPService(icpMode, deltaHuber, maxIteration, maxDistance, convergenceThreshold, voxelSize, nPosesPath, marrtionLaserLinkTransform, lidar, graphSlamService);
        publisherService = new PublishingService();
        odometryService = new OdometryService(angularVelocityThreshold, wheelVelocityThreshold, odometryFrequency, articulationBodiesRefs, publishLastNOdometryPoses, NOdometryLastPoses);



        //------------------REGISTRAZIONE EVENTI
        lidar.OnScanComplete += ScanCompletedLetsWork;
        Invoke("ClearVisualizationTopics", 1.5f);






        //------------------REGISTRAZIONE PUBLISHER
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PathMsg>(icpPathRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(icpMapRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<OdometryMsg>(odometryRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PathMsg>(odometryPathRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<TFMessageMsg>(tfRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(graphSlamGlobalpointCloudTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(graphSlamNodesTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(loopClosureCircleTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(loopClosureEdgesTopic);
    }

    // Update is called once per frame
    void Update()
    {
        if (odometryService.isTimeToLocalize())
        {
            odometryService.letsLocalizeUsingOdometry();
            (float xt, float yt, float thetat) currentConfig= odometryService.getUpdatedConfiguration();

            publisherService.PublishTF(currentConfig, marrtionLaserLinkTransform.localPosition, marrtionLaserLinkTransform.localRotation, tfRosTopic);

            publisherService.PublishLastOdometry(currentConfig, odometryRosTopic);

            if (publishLastNOdometryPoses)
            {
                publisherService.PublishOdometryPath(odometryService.getUpdatedLastOdometryPoses(), odometryPathRosTopic);
            }

            

        }
    }

    void ClearVisualizationTopics()
    {
        publisherService.PublishEmptyPointCloud(graphSlamNodesTopic);
        publisherService.PublishEmptyPointCloud(loopClosureCircleTopic);
        publisherService.PublishEmptyPointCloud(loopClosureEdgesTopic);
        publisherService.PublishEmptyPointCloud(graphSlamGlobalpointCloudTopic);
    }

    void ScanCompletedLetsWork()
    {
        //Debug.Log("LiDAR scan completed, orchestrator called ...");

        (Vector3 worldDelta, Queue<Vector3> icpWorldPositions, List<Vector3> transformedWorldPointsList) updatedWorld = icpService.ScanCompletedRunOneICP();

        float[,] TWorldICP = icpService.getTWorldICP();
        float[,] TRelativeICP = icpService.getTRelativeICP();

        List<Vector3> sourceLocalTreeListOfPoints = icpService.getSourceLocalTreeListOfPoints();
        KDTree kdSourceTree = icpService.getSourceKDTree();

        bool hasMoovedSinceLastNode = graphSlamService.checkIfMoovedSinceLastNode(TRelativeICP);

        if (hasMoovedSinceLastNode)
        {
            graphSlamService.updateGraphSLAMWithNewNode(TWorldICP, TRelativeICP, sourceLocalTreeListOfPoints, kdSourceTree);
            //Debug.Log("Inserted new node in the Graph SLAM");
            publisherService.PublishGraphNodes( icpService.ICPToWorldPositionEnumerableNodes(graphSlamService.getGraphNodes()), graphSlamNodesTopic);

            int nodeCounter = graphSlamService.getNodeCounter();
            //bool da studiare per far sì che SearchLoopClosure venga chiamato solo quando effettivamente il lidar dopo K iterazioni ha ritrovato punti già noti, magari presenti in un qualche dizionario
            //bool someConditionToBeFiguredOut = true;
            bool someConditionToBeFiguredOut = nodeCounter % k_tryLoopClosure == 0;

            if (someConditionToBeFiguredOut)
            {
                Debug.Log($"Trying to find a new closure...");
                //publisherService.PublishLoopClosureCircle( icpService.ICPToWorldPosition(TWorldICP[0, 3], TWorldICP[1, 3], TWorldICP[2, 3]), loopClosureRadius, loopClosureCircleTopic);

                List<PoseNode> candidates = graphSlamService.getLoopClosureCandidates(k_midDeltaIDBetweenCandidates, loopClosureRadius);
                List<Vector3> sourceScannedPoints = graphSlamService.getNewNodeScannedPoints();

                int closureEdges = 0;
                Vector3 currentWorldPos = icpService.ICPToWorldPosition(TWorldICP[0, 3], TWorldICP[1, 3], TWorldICP[2, 3]);
                foreach (PoseNode candidate in candidates)
                {
                    float[,] initialGuess = graphSlamService.GetInitialGuessForCandidate(candidate);
                    float[,] deltaTCandidate = icpService.SolveInverseICPProblem(initialGuess, candidate, sourceScannedPoints);
                    float[] logError = LogMap(deltaTCandidate);

                    if (getNorm(logError) < thresholdLoopClosure)
                    {
                        Debug.Log($"Loop closure found, adding it to the graph slam...");
                        graphSlamService.updateGraphSLAMWithClosureEdge(candidate, deltaTCandidate);
                        Vector3 candidateWorldPos = icpService.ICPToWorldPosition(candidate.PoseT()[0, 3], candidate.PoseT()[1, 3], candidate.PoseT()[2, 3]);
                        closureEdgesWorldPositions.Add((currentWorldPos, candidateWorldPos));
                        closureEdges++;
                    }
                }

                if (closureEdges > 0)
                {
                    Debug.Log($"Added {closureEdges} loop closures, optimizing...");
                    graphSlamService.OptimizeGraph();
                    // Re-anchor dell'accumulatore ICP alla posa ottimizzata del nodo più recente:
                    // senza questo, i nodi successivi nascono sulla vecchia catena raw e l'edge di
                    // odometria diventa incoerente con le pose ottimizzate → la prossima ottimizzazione
                    // parte da un dx enorme e distorce il grafo. TRelativeICP resta intatto (è relativo).
                    icpService.setTWorldICP(graphSlamService.getNewNode().PoseT());
                    publisherService.PublishGraphNodes(icpService.ICPToWorldPositionEnumerableNodes(graphSlamService.getGraphNodes()), graphSlamNodesTopic);
                    graphSlamService.updateGlobalScannedPointsAfterOptimization();
                    publisherService.PublishUpdatedGlobalPointCloudMap( icpService.ICPToWorldPositionListVectors(graphSlamService.getUpdatedGlobalMapPointCloud()) , graphSlamGlobalpointCloudTopic);
                    publisherService.PublishLoopClosureEdges(closureEdgesWorldPositions, loopClosureEdgesTopic);
                }

            }

            graphSlamService.updateLastNode();

        }

        publisherService.PublishICPPath(updatedWorld.icpWorldPositions, icpPathRosTopic);

        publisherService.PublishICPMap(updatedWorld.transformedWorldPointsList, icpMapRosTopic);

    }

}
