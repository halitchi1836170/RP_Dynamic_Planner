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
    public string coneFanTopic = "/graph_slam/cone_fan";

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
    private float huberDeltaGraphSlamOptimization = 0.5f;

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
    private LoopClosureFinderService loopClosureFinderService;




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

        bool hasMoovedSinceLastNode = graphSlamService.checkIfMoovedSinceLastNode(TRelativeICP);

        if (hasMoovedSinceLastNode)
        {
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
            publisherService.PublishUpdatedGlobalPointCloudMap( icpService.ICPToWorldPositionListVectors(graphSlamService.getUpdatedGlobalMapPointCloud()) , graphSlamGlobalpointCloudTopic);
            publisherService.PublishLoopClosureEdges(icpService.ICPToWorldPositionPairs(graphSlamService.getClosureEdgeVector3Couples()), loopClosureEdgesTopic);
        }
    }

}
