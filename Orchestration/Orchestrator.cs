using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Tf2;
using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using static MatrixVectorUtilities;
using static PoseMatrix4x4;
using static UnicycleModelUtilities;

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
    public string coneFanTopic = "/graph_slam/cone_fan";
    public string occupancyRosTopic = "/occupancy_grid";
    public string distanceMapRosTopic = "/distanceMap";
    public string plannedTrajectoryRosTopic = "/planned_path";
    public string smoothedTrajectoryRosTopic = "/smoothed_path";
    public string startDebugRosTopic = "/debug/start_pose";
    public string goalDebugRosTopic = "/debug/goal_pose";

    public string GRASLAMFIELDS = "FOLLOWING GRAPH SLAM FIELDS";
    public float minDeltaTranslation = 0.1f;
    public float minDeltaRotation = 0.5f;
    public int k_midDeltaIDBetweenCandidates = 15;
    public int k_tryLoopClosure = 10;
    public float thresholdLoopClosure = 0.5f;
    public int maxIterations = 50;
    public int maxCGIterations = 100;
    public float graphSlamOptimizerConvergenceThreshold = 1e-6f;
    public float CGConvergenceThreshold = 1e-6f;
    public LoopClosureFinder loopClosureMode = LoopClosureFinder.TimeAndConeBased;
    public float maxRadiusLoopClosureFinder = 0.5f;
    public float halfConeAngleLoopClosureFinder = 30.0f;
    public int secondsFrequencyLoopClosureFinder = 15;
    public bool planarConstraintFlag = true;
    public float huberDeltaGraphSlamOptimization = 0.5f;

    public string ODOMETRYFILEDS = "FOLLOWING ODOMETRY FIELDS";
    public float angularVelocityThreshold = 0.001f;
    public float wheelVelocityThreshold = 0.05f;
    public float odometryFrequency = 55.0f; //Hz
    public bool publishLastNOdometryPoses = true;
    public int NOdometryLastPoses = 250;

    public string OCCUPANCYGRIDFIELDS = "FOOLLOWING OCCUPANCY GRID FIELDS";
    public float resolution = 0.02f;
    public float zMin = 0.2f;
    public float zMax = 1.0f;
    public float probOcc = 0.75f;  // Se rileva un ostacolo, mi fido al 75%
    public float probFree = 0.35f; // Se non rileva nulla, la probabilità che ci sia un ostacolo scende al 35%
    public float occBlockThreshold = 1.8f;
    public float elevThrehsold = 6;
    public bool calculateAndOverrideOccupancyMapFlag = false;
    public string occupancyGridMapFileName = "occupancyGridMap";

    public string DISTANCEMAPFIELDS = "FOLLOWING DISTANCE MAP FIELDS";
    public float obstacleThreshold = 50f;
    public float distanceMapRepublishPeriod = 1.0f;
    private bool distanceMapComputed = false;
    private float lastDistanceMapPublish = 0f;

    public string MOTIONPLANNINGFILED = "FOLLOWING MOTION PLANNING FIELDS";
    public bool goalSettedLetsPlan = true;
    public bool controlTrajectory = false;
    public float goalXUnity = -4f;
    public float goalZUnity = 1.5f;
    public Planner plannerMode = Planner.A;
    public float k = 50f;
    public float eps = 0.01f;
    private bool trajectoryPlanned = false;
    private float lastTrajectoryPublish = 0f;
    public float plannedTrajectoryRepublishPeriod = 1.0f;
    private (float x, float y) startDebugPoint;
    private (float x, float y) goalDebugPoint;
    public bool smoothTrajWithLoSPS = true;
    public float epsilonTunnelLoSPS = 0.25f; // Dall'URDF: box base 0.28x0.37 m -> raggio circoscritto ~0.23 m; 0.25 m aggiunge un piccolo margine.


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
    private LoopClosureFinderService loopClosureFinderService;
    private OccupancyGridService occupancyGridService;
    private IOFileOperationService ioService;
    private DistanceMapService distanceMapService;
    private MotionPlannerService motionPlannerService;
    private ControllerService controllerService;

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
        graphSlamService = new GraphSlamService(maxIterations, maxCGIterations, graphSlamOptimizerConvergenceThreshold, CGConvergenceThreshold, minDeltaTranslation, minDeltaRotation, huberDeltaGraphSlamOptimization);
        icpService = new ICPService(icpMode, deltaHuber, maxIteration, maxDistance, convergenceThreshold, voxelSize, nPosesPath, marrtionLaserLinkTransform, lidar, graphSlamService, planarConstraintFlag);
        publisherService = new PublishingService();
        odometryService = new OdometryService(angularVelocityThreshold, wheelVelocityThreshold, odometryFrequency, articulationBodiesRefs, publishLastNOdometryPoses, NOdometryLastPoses);
        loopClosureFinderService = new LoopClosureFinderService(halfConeAngleLoopClosureFinder, k_midDeltaIDBetweenCandidates, secondsFrequencyLoopClosureFinder, maxRadiusLoopClosureFinder, thresholdLoopClosure);
        occupancyGridService = new OccupancyGridService(resolution, zMin, zMax, probOcc, probFree, occBlockThreshold, elevThrehsold);
        ioService = new IOFileOperationService(occupancyGridMapFileName);
        distanceMapService = new DistanceMapService(obstacleThreshold, k, eps);
        motionPlannerService = new MotionPlannerService(smoothTrajWithLoSPS, epsilonTunnelLoSPS);
        controllerService = new ControllerService();

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
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(coneFanTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<OccupancyGridMsg>(occupancyRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<OccupancyGridMsg>(distanceMapRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PathMsg>(plannedTrajectoryRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PathMsg>(smoothedTrajectoryRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(startDebugRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(goalDebugRosTopic);

        //------------------ONE TIME ACTIONS
        if (calculateAndOverrideOccupancyMapFlag == false)
        {
            distanceMapService.SetOccupancyGridMap(ioService.ReadOccupancyGrid());
            distanceMapService.calculateDistanceMap();
            publisherService.PublishDistanceMap(distanceMapService.getDistanceMapForPublisher(), distanceMapRosTopic);
            distanceMapComputed = true;

            if (goalSettedLetsPlan)
            {
                (float sx, float sy, float sz) start = UnityToRosPosition(marrtionLaserLinkTransform.position.x, marrtionLaserLinkTransform.position.y, marrtionLaserLinkTransform.position.z);
                (float gx, float gy, float gz) goal = UnityToRosPosition(goalXUnity, 0, goalZUnity);
                startDebugPoint = (start.sx, start.sy);
                goalDebugPoint = (goal.gx, goal.gy);
                motionPlannerService.DetermineTrajectory(distanceMapService.getDistanceMapInstance(), startDebugPoint, goalDebugPoint, plannerMode);
                trajectoryPlanned = motionPlannerService.TrajectoryDetermined();
            }

            if(motionPlannerService.TrajectoryDetermined() && controlTrajectory)
            {
                controllerService.FollowTrajectory();
            }

        }
    }

    // Update is called once per frame
    void Update()
    {
        if (distanceMapComputed && Time.time - lastDistanceMapPublish > distanceMapRepublishPeriod)
        {
            publisherService.PublishDistanceMap(distanceMapService.getDistanceMapForPublisher(), distanceMapRosTopic);
            lastDistanceMapPublish = Time.time;
        }

        if(trajectoryPlanned && Time.time - lastTrajectoryPublish > plannedTrajectoryRepublishPeriod)
        {
            publisherService.PublishPlannedTrajectory(motionPlannerService.getTrajectoryWorld(), plannedTrajectoryRosTopic);
            if (smoothTrajWithLoSPS) publisherService.PublishPlannedTrajectory(motionPlannerService.getSmoothTrajectoryWorld(), smoothedTrajectoryRosTopic);
            publisherService.PublishDebugPoint(startDebugPoint, startDebugRosTopic);
            publisherService.PublishDebugPoint(goalDebugPoint, goalDebugRosTopic);
            lastTrajectoryPublish = Time.time;
        }

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

            //Debug.Log($"occupancy cells = {occupancyGridService.getOccupancyGridMap().Count}");

        }

        // Ventaglio del cono di ricerca, centrato sul robot live, pubblicato ogni frame (solo in TimeAndCone).
        if (loopClosureMode == LoopClosureFinder.TimeAndConeBased)
        {
            publisherService.PublishConeFan(marrtionLaserLinkTransform.position, marrtionLaserLinkTransform.forward, halfConeAngleLoopClosureFinder, thresholdLoopClosure, maxRadiusLoopClosureFinder, coneFanTopic);
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

        // Gate sul movimento REALE (ruote): se il robot è fermo non chiamiamo checkIfMoovedSinceLastNode,
        // così l'accumulatore NON somma il drift dell'ICP da fermo -> niente nodi spuri -> niente muri
        // fantasma paralleli nella occupancy grid. Lo short-circuit && garantisce il "non accumulo".
        bool hasMoovedSinceLastNode = odometryService.isRobotMoving() && graphSlamService.checkIfMoovedSinceLastNode(TRelativeICP);

        if (hasMoovedSinceLastNode)
        {
            occupancyGridService.updateGridMap(TWorldICP, sourceLocalTreeListOfPoints, v => icpService.ICPToWorldPosition(v.x, v.y, v.z));
            occupancyGridService.updateDataForPublisher();

            graphSlamService.updateGraphSLAMWithNewNode(TWorldICP, sourceLocalTreeListOfPoints, kdSourceTree);
            //Debug.Log("Inserted new node in the Graph SLAM");
            publisherService.PublishGraphNodes( icpService.ICPToWorldPositionEnumerableNodes(graphSlamService.getGraphNodes()), graphSlamNodesTopic);

            // Modalità KNodeGap: il trigger è legato alla creazione di un nuovo nodo (ogni k nodi).
            if (loopClosureMode == LoopClosureFinder.KNodeGapBased && graphSlamService.getNodeCounter() % k_tryLoopClosure == 0)
            {
                TryLoopClosure();
            }

            graphSlamService.updateLastNode();
        }

        // Modalità TimeAndCone: trigger temporale, controllato ad OGNI scan (anche da fermo, quando
        // non si crea alcun nodo) -> permette di puntare i nodi già creati e aspettare l'aggancio.
        // La guardia getNewNode() != null evita l'NPE finché esiste solo il nodo 0 (che non setta newNode);
        // è messa prima del timer così, se non possiamo agire, non consumiamo/resettiamo il timer.
        if (loopClosureMode == LoopClosureFinder.TimeAndConeBased
            && graphSlamService.getNewNode() != null
            && loopClosureFinderService.itsTimeToFindLoopClosure())
        {
            TryLoopClosure();
        }

        publisherService.PublishICPPath(updatedWorld.icpWorldPositions, icpPathRosTopic);

        publisherService.PublishICPMap(updatedWorld.transformedWorldPointsList, icpMapRosTopic);

        if (calculateAndOverrideOccupancyMapFlag) {
            publisherService.PublishOccupancyGridMap(occupancyGridService.getDataForPublisher(), occupancyRosTopic);
        }
        else
        {
            publisherService.PublishOccupancyGridMap(ioService.ReadOccupancyGrid(), occupancyRosTopic);
        }

    }

    // Ricerca candidati -> ICP inverso -> aggiunta closure edges -> ottimizzazione + publish.
    // Usa sempre il nodo più recente (getNewNode) come query, indipendentemente dal fatto che in
    // questo scan sia stato creato un nuovo nodo: così funziona anche da robot fermo.
    void TryLoopClosure()
    {
        int nodeCounter = graphSlamService.getNodeCounter();
        float[,] TWorldICP = icpService.getTWorldICP();

        List<PoseNode> candidates = loopClosureFinderService.getLoopClosureCandidates(
            loopClosureMode,
            graphSlamService.getNewNode(),
            graphSlamService.getGraphNodesWithIDUpperBound(nodeCounter - k_midDeltaIDBetweenCandidates),
            marrtionLaserLinkTransform.position,
            marrtionLaserLinkTransform.forward,
            node => icpService.ICPToWorldPosition(node.PoseT()[0, 3], node.PoseT()[1, 3], node.PoseT()[2, 3]));

        Debug.Log($"Trying to find a new closure, found {candidates.Count} candidates...");

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
                Vector3 candidateWorldPos = icpService.ICPToWorldPosition(candidate.PoseT()[0, 3], candidate.PoseT()[1, 3], candidate.PoseT()[2, 3]);
                // Incrementa solo se l'edge è stato davvero aggiunto (coppia non già chiusa):
                // sui trigger ripetuti da fermo evita di ri-ottimizzare a vuoto.
                if (graphSlamService.updateGraphSLAMWithClosureEdge(candidate, deltaTCandidate))
                {
                    closureEdges++;
                }
            }
        }

        if (closureEdges > 0)
        {
            Debug.Log($"Added {closureEdges} loop closures, optimizing...");
            graphSlamService.OptimizeGraph();
            // Re-anchor dell'accumulatore ICP alla posa ottimizzata del nodo più recente.
            // Va moltiplicato per il moto accumulato dall'ultimo nodo: se il trigger scatta mentre
            // il robot si è già mosso oltre l'ultimo nodo (tipico in TimeAndCone), TWorldICP =
            // T_newNode_opt * accumulatore preserva la posa corrente del robot. In KNodeGap
            // l'accumulatore è Identity, quindi si riduce a getNewNode().PoseT().
            icpService.setTWorldICP(productSquareMatrix4(graphSlamService.getNewNode().PoseT(), graphSlamService.getAccumulatedRelativeSinceLastNode()));
            publisherService.PublishGraphNodes(icpService.ICPToWorldPositionEnumerableNodes(graphSlamService.getGraphNodes()), graphSlamNodesTopic);
            
            graphSlamService.updateGlobalScannedPointsAfterOptimization();

            Debug.Log($"Updating occupancy grid map after optimization...");
            occupancyGridService.clear();
            foreach (PoseNode node in graphSlamService.getGraphNodes())
            {
                occupancyGridService.updateGridMap(node.PoseT(), node.PoseScannedPoints(), v => icpService.ICPToWorldPosition(v.x, v.y, v.z));
            }
            occupancyGridService.updateDataForPublisher();

            if (calculateAndOverrideOccupancyMapFlag)
            {
                var d = occupancyGridService.getDataForPublisher();   // (data, W, H, originX, originY, resolution)
                ioService.WriteOccupancyGrid(d.Item1, d.Item2, d.Item3, d.Item6, d.Item4, d.Item5);
            }

            publisherService.PublishUpdatedGlobalPointCloudMap( icpService.ICPToWorldPositionListVectors(graphSlamService.getUpdatedGlobalMapPointCloud()) , graphSlamGlobalpointCloudTopic);
            publisherService.PublishLoopClosureEdges(icpService.ICPToWorldPositionPairs(graphSlamService.getClosureEdgeVector3Couples()), loopClosureEdgesTopic);
        }
    }

}
