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
    public string splinedTrajectoryRosTopic = "/splined_path";
    public string startDebugRosTopic = "/debug/start_pose";
    public string waypointsDebugRosTopic = "/debug/waypoints";                 // missione originale
    public string activeWaypointsDebugRosTopic = "/debug/active_waypoints";    // quelli usati dal piano corrente
    public string goalDebugRosTopic = "/debug/goal_pose";
    public string currentPoseDebugRosTopic = "/debug/current_pose";   // posa corrente (odometria di ruota) in frame ROS
    public string truePoseDebugRosTopic = "/debug/true_pose";         // posa VERA (transform live -> ROS), solo debug
    public string icpPoseLocalizationDebugRosTopic = "/debug/localization/icp_pose";
    public string dynamicObstaclesRosTopic = "/debug/dynamic_obstacles";
    public string trackedObstaclesRosTopic = "/debug/tracked_obstacles";
    public string unexplainedPointsRosTopic = "/debug/unexplained_points";

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
    private List<Vector3> updatedGlobalPointCloudCached;
    private bool updatedGlobalPointCloudReady = false;
    private float lastGlobalPointCloudPublish = 0f;
    private float globalPointCloudPeriod = 5.0f;
    private KDTree cachedGlobalPointCloudKDTree;
    private VoxelGrid cachedGlobalCloudVoxelGrid;

    public string ODOMETRYFILEDS = "FOLLOWING ODOMETRY FIELDS";
    public float angularVelocityThreshold = 0.001f;
    public float wheelVelocityThreshold = 0.05f;
    public float odometryFrequency = 55.0f; //Hz
    public bool publishLastNOdometryPoses = true;
    public int NOdometryLastPoses = 250;
    // La localizzazione integra a odometryFrequency (55 Hz), ma la PATH (250 pose) va PUBBLICATA a rate basso:
    // a 55 Hz satura la coda TCP (Queue full -> messaggi droppati) e genera allocazioni GC ad ogni frame.
    public float odometryPathPublishPeriod = 0.2f;   // 5 Hz
    private float lastOdometryPathPublish = 0f;

    public string OCCUPANCYGRIDFIELDS = "FOOLLOWING OCCUPANCY GRID FIELDS";
    public float resolution = 0.02f;
    public float zMin = 0.2f;
    public float zMax = 1.0f;
    public float bodyRadius = 0.45f;
    public float probOcc = 0.75f;  // Se rileva un ostacolo, mi fido al 75%
    public float probFree = 0.35f; // Se non rileva nulla, la probabilità che ci sia un ostacolo scende al 35%
    public float occBlockThreshold = 1.8f;
    public float elevThrehsold = 6;
    public bool calculateAndOverrideOccupancyMapFlag = false;
    public string occupancyGridMapFileName = "occupancyGridMap";
    // In navigazione (mappa gia' costruita) la occupancy si legge UNA volta e si ripubblica throttlata,
    // invece di rileggerla da disco ad ogni scan.
    public float occupancyRepublishPeriod = 5.0f;
    private (sbyte[], int, int, float, float, float) occupancyGridCached;
    private bool occupancyGridCachedReady = false;
    private float lastOccupancyPublish = 0f;

    public string DISTANCEMAPFIELDS = "FOLLOWING DISTANCE MAP FIELDS";
    public float obstacleThreshold = 50f;
    public float distanceMapRepublishPeriod = 5.0f;
    private bool distanceMapComputed = false;
    private float lastDistanceMapPublish = 0f;

    public string MOTIONPLANNINGFIELDS = "FOLLOWING MOTION PLANNING FIELDS";
    public bool goalSettedLetsPlan = true;
    public float goalXUnity = -1f;
    public float goalZUnity = -2.5f;
    public string goalStateObjectName = "GOAL STATE";   // waypoint da scena: oggetti "1_NODE","2_NODE",... poi questo
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
    public float linearMeanVelocity = 0.08f;
    public (double vel_qi, double acc_qi) qiCouple = (0.0, 0.0);            //unico vettore perché suppongo gli stessi valori sia per x che y 
    public (double vel_qf, double acc_qf) qfCouple = (0.0, 0.0);
    public int subSamplesPerSpline = 30;
    public float collinearToleranceDeg = 10f;     // vertici quasi allineati: rimossi, danno solo nodi spline
    public float minVertexSpacing = 0.12f;        // distanza minima fra vertici della spezzata [m]
    public float waypointRelaxClearance = 0.20f;  // un waypoint dentro un ostacolo viene proiettato sulla
    public float waypointRelaxRadius = 1.2f;      // cella libera piu' vicina entro questo raggio
    public float startEscapeRadius = 0.5f;        // se il robot e' dentro un disco, puo' uscirne
    private List<(float, float)> waypoints;

    public string CONTROLSTRAtEGIESFIELDS = "FOLLOWING CONTROL SERVICE FIELDS";
    public int secondsControlFrequency = 20;
    public bool boolControlMarrtino = true;
    public ControlStrategy controlStrategy = ControlStrategy.NonlinearControl;
    public bool controlOnGroundTruth = false;
    public bool flipControlOmega = true;
    private (float x, float y, float theta) currentConfig;
    public float b = 4.0f;
    public float zeta = 0.9f;
    public float vMax = 0.2f;       // circa 1.5 volte linearMeanVelocity
    public float wMax = 0.8f;        // cap di CURVATURA del profilo (rallenta in curva): tienilo fisico ~2
    public float wMaxClamp = 1.2f;   // clamp di sicurezza su omega del controller: > wMax, da' margine al feedback
    public float aMax = 0.08f;
    public bool useICPLocalization = true;
    public float minInlierRatio = 0.6f;      // frazione minima di scansione spiegata dalla mappa
    public float maxResidual = 0.25f;        // errore medio max [m]; ~2-3x voxelSize
    public float localizationPeriod = 0.35f;  // l'ICP e' costoso: correggi a ~10 Hz, non ogni frame
    public float maxLocalizationJump = 0.4f;  // salto max [m] corrected-vs-guess: oltre -> convergenza ICP errata, scarto
    private float lastLocalization = 0f;
    private (float x, float y, float theta) icpLocalizedConfig;
    public int maxIterationLocalization = 7;

    public string REPLANNINGFIELDS = "FOLLOWING REPLANNING FIELDS";
    public bool replanningEnabled = true;
    public float persistentObstacleAge = 3.0f;    // eta' minima di un track per giustificare un replan [s]
    public float replanLookAheadDistance = 1.5f;  // quanto riferimento residuo controllo [m]
    public float minCBFEngagement = 1.5f;         // secondi di filtro CBF attivo prima di considerare un replan
    public float engagementMarginThreshold = 0.4f;// ...e solo se un ostacolo dinamico e' entro questo margine
    public float engagementDeviationThreshold = 0.04f; // ...e se il filtro corregge almeno di tanto
    public float minReplanInterval = 10f;         // intervallo MINIMO garantito fra due piani [s]
    public float obstacleInflationMargin = 0.25f; // margine sul raggio quando dipingo l'ostacolo nella griglia [m]
    public float reducedInflationFactor = 0.5f;   // secondo tentativo se il primo non trova percorso
    public float stallWindow = 4.0f;              // finestra per il rilevamento di stallo [s]
    public float stallMinProgress = 0.15f;        // avanzamento sotto il quale sono fermo [m]
    public float replanMaxDeviation = 0.6f;       // scostamento dal piano oltre il quale ripianifico [m]
    public float deviationWindow = 1.5f;          // per quanto deve persistere lo scostamento [s]
    public int maxInfeasibleSteps = 5;
    public float replanCooldown = 4.0f;
    public float replanRetryPeriod = 5.0f;
    public int maxReplanFailures = 3;
    public float waypointReachedRadius = 0.4f;    // entro questo raggio il waypoint e' considerato raggiunto
    public float waypointPassedRadius = 1.5f;     // ...e solo entro questo si valuta se lo si e' oltrepassato
    public float replanCheckPeriod = 0.2f;        // i trigger scorrono la tabella: inutile valutarli ad ogni frame
    private float lastReplanCheck = 0f;
    private int nextWaypointIdx = 1;              // 0 e' lo start: i residui partono da 1
    private string lastPlanningFailure = "none";
    private ReplanState lastLoggedState = ReplanState.Following;

    public string PLOTTINGFIELDS = "FOLLOWING PLOTTING FIELDS";
    public bool recordPlotData = true;
    public float plotSamplePeriod = 0.05f;   // campionamento delle pose per i grafici
    private bool plotDataDumped = false;

    public string CBFFIELDS = "FOLLOWING CBF SAFETY FILTER FIELDS";
    public bool cbfEnabled = true;
    public float bLookAhead = 0.15f;          // punto avanzato: da autorita di sterzata al QP
    public float rSafeDynamic = 0.20f;        // >= raggio robot + bLookAhead
    public float rSafeStatic = 0.18f;         // < epsilonTunnelLoSPS, altrimenti litiga col percorso nominale
    public float alphaDynamic = 0.8f;
    public float alphaStatic = 0.8f;
    public float gammaCLF = 0.5f;
    public float slackPenalty = 100f;
    public float cbfSlackPenalty = 10000f;    // barriere soft ad altissimo costo: il QP non e' mai infeasible
    public float commandSmoothingWeight = 0.5f;  // smorza l'alternanza destra/sinistra del QP; 0 = disattivato
    public float vDeviationWeight = 1.0f;     // costo di deviare dalla v nominale
    public float wDeviationWeight = 0.2f;     // < vDeviationWeight -> il QP sterza invece di frenare
    public float staticActivationDistance = 0.6f;
    public float obstacleActivationRange = 3.0f;
    public float gradientStepCells = 2f;
    public float cbfActivationTolerance = 1e-3f;
    public float robotBodyRadius = 0.23f;     // raggio circoscritto della scocca: seconda barriera sul CORPO
    public float referenceMaxLag = 0.20f;     // il riferimento aspetta solo oltre questo scarto [m]
    public float wMaxCBF = 0.9f;              // limite di omega DEL QP: molto sotto wMaxClamp, altrimenti strattona
    public float maxLinearAccelCommand = 0.3f;    // slew rate del comando v [m/s^2]
    public float maxAngularAccelCommand = 1.5f;   // slew rate del comando omega [rad/s^2]

    public string OBSTACLESMANAGERFILEDS = "FOLLOWING DYNAMIC OBSTACLES MANAGER FIELDS";
    private List<(float cx, float cy, float r, int n)> obstaclesCentroidsSensed;
    public float obsTol = 0.12f;            // tolleranza "spiegato dalla mappa" a range 0 [m]: tol(d) = obsTol + obsTolPerMeter*d
                                            // NB: un ostacolo addossato a un muro ha poca clearance: se la
                                            // tolleranza e' alta i suoi punti sembrano "gia' nella mappa"
    public float obsTolPerMeter = 0.02f;    // crescita della tolleranza col range (errore di heading + rumore + chamfer)
    public float maxDetectionRange = 3.5f;  // oltre: punti ignorati (alla CBF servono solo gli ostacoli vicini)
    public float clusteringRadius = 0.3f;
    public int minClusterPoints = 4;       // sotto = rumore (punti gia' downsampled a detectionVoxelSize)
    public int minClusterPointsNear = 2;   // soglia entro nearClusterRange: da vicino i voxel sono pochi
    public float nearClusterRange = 1.5f;
    public float detectionVoxelSize = 0.05f;  // piu' fine di voxelSize (ICP): quadruplica i punti su una superficie
    public float clusterMargin = 0.03f;    // margine sul raggio del cerchio di ingombro [m]
    public float maxUnexplainedFraction = 0.5f;   // oltre: scansione scartata (posa sospetta)...
    public float minSuspiciousSpread = 2.0f;      // ...ma solo se i punti sono sparsi su piu' di tanto [m]
    public float selfHitRadius = 0.28f;           // usato solo se useChassisFootprint = false
    public bool useChassisFootprint = true;       // scarta gli auto-hit col rettangolo VERO della scocca
    public float chassisFront = 0.04f;            // URDF: box 0.28x0.37, laser a +0.10 -> davanti restano 4 cm
    public float chassisRear = 0.24f;
    public float chassisHalfWidth = 0.19f;
    public float chassisTopRelLaser = -0.125f;    // tetto scocca 0.21 - quota laser 0.335
    public float chassisMargin = 0.04f;
    public float obstacleZMin = 0.15f;            // banda di quota per la detection (indipendente da quella di mapping)
    public float obstacleZMax = 1.00f;            // il soffitto lo scarta il gate di elevazione, non serve stringere qui
    public float maxElevationDeg = 12f;           // scarta i raggi troppo inclinati: soffitti e pavimento
    public float trackGate = 0.5f;          // associazione detection-track: distanza max [m]
    public float trackAlphaLowpass = 0.3f;  // filtro sulla velocita' stimata
    public float trackVDeadzone = 0.05f;    // sotto: velocita' = 0 (jitter del centroide) [m/s]
    public int trackMinHits = 3;            // scan consecutivi per confermare un track
    public float trackForgetTime = 1.0f;    // track non visto da piu' di tanto: rimosso [s]
    public float blindZoneRadius = 0.7f;    // oltre al raggio del track: entro qui il sensore non vede
    public float blindZoneForgetFactor = 6f;// quanto piu' a lungo si ricorda un track nella zona cieca
    public int minPointsForUpdate = 4;      // sotto: cluster troppo povero per spostare centro e raggio
    public float radiusDecayPerUpdate = 0.98f;  // decadimento LENTO, e solo su osservazioni buone
    public float blindZoneGrowthRate = 0.05f;   // [m/s] il raggio cresce finche' il track non si rivede
    public float maxBlindGrowth = 0.20f;        // tetto al gonfiaggio: oltre, il track e' solo rumore
    public float closeRangeMergeGate = 0.15f;  // quanto vicino deve stare un track per considerare la detection gia' coperta
    public float closeRangeOverride = 1.3f; // sotto questa distanza le detection GREZZE vanno alla CBF
                                            // senza passare da conferma e tracking: ultima rete di sicurezza

    //--------------------------------------------------------------------------------------------------------------------------------------------------------
    //                                                                    HARDWARE FIELDS
    //--------------------------------------------------------------------------------------------------------------------------------------------------------
    ArticulationBodyRefs articulationBodiesRefs;
    private ArticulationBody leftWheel;
    private ArticulationBody rightWheel;
    private float wheelRadius;
    private float wheelSeparation;
    private LiDAR3D lidar;
    private Transform marrtionLaserLinkTransform;
    private DifferentialDriveController differentialDriveController;




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
    private LocalizationService localizationService;
    private DynamicObstacleService dynamicObstacleService;
    private ObstacleTrackerService obstacleTrackerService;
    private CBFService cbfService;
    private DistanceMap pristineDistanceMap;
    private PlotDataService plotDataService;
    private ReplanningService replanningService;

    //--------------------------------------------------------------------------------------------------------------------------------------------------------
    //                                                                       ROBOT STATE
    //--------------------------------------------------------------------------------------------------------------------------------------------------------



    private void Awake()
    {
        articulationBodiesRefs = this.GetComponent<ArticulationBodyRefs>();
        (ArticulationBody lw, ArticulationBody rw) wheels = articulationBodiesRefs.getWheelArticulationBodyReference();
        this.leftWheel = wheels.lw;
        this.rightWheel = wheels.rw;
        this.wheelRadius = articulationBodiesRefs.wheelRadius;
        this.wheelSeparation = articulationBodiesRefs.wheelSeparation;
        lidar = articulationBodiesRefs.getMarrtinoLaserLinkArticulationBodyReference().GetComponent<LiDAR3D>();
        marrtionLaserLinkTransform = articulationBodiesRefs.getMarrtinoLaserLinkTransformReference();
        differentialDriveController = this.GetComponent<DifferentialDriveController>();

        //Debug.Log($"left wheel body var test name: {leftWheel.name}");
    }




    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        //------------------INIZIALIZZAZIONE STATO




        //------------------INIZIALIZZAZIONE SERVIZI
        graphSlamService = new GraphSlamService(maxIterations, maxCGIterations, graphSlamOptimizerConvergenceThreshold, CGConvergenceThreshold, minDeltaTranslation, minDeltaRotation, huberDeltaGraphSlamOptimization);
        icpService = new ICPService(icpMode, deltaHuber, maxIteration, maxIterationLocalization, maxDistance, convergenceThreshold, voxelSize, nPosesPath, marrtionLaserLinkTransform, lidar, graphSlamService, planarConstraintFlag);
        publisherService = new PublishingService();
        odometryService = new OdometryService(angularVelocityThreshold, wheelVelocityThreshold, odometryFrequency, articulationBodiesRefs, publishLastNOdometryPoses, NOdometryLastPoses);
        loopClosureFinderService = new LoopClosureFinderService(halfConeAngleLoopClosureFinder, k_midDeltaIDBetweenCandidates, secondsFrequencyLoopClosureFinder, maxRadiusLoopClosureFinder, thresholdLoopClosure);
        occupancyGridService = new OccupancyGridService(resolution, zMin, zMax, bodyRadius, probOcc, probFree, occBlockThreshold, elevThrehsold);
        ioService = new IOFileOperationService(occupancyGridMapFileName);
        distanceMapService = new DistanceMapService(obstacleThreshold, k, eps);
        motionPlannerService = new MotionPlannerService(smoothTrajWithLoSPS, epsilonTunnelLoSPS, linearMeanVelocity, qiCouple, qfCouple, subSamplesPerSpline, secondsControlFrequency, wMax, aMax, vMax, waypointRelaxClearance, waypointRelaxRadius, startEscapeRadius, collinearToleranceDeg, minVertexSpacing);
        plotDataService = new PlotDataService(plotSamplePeriod);
        replanningService = new ReplanningService(persistentObstacleAge, replanLookAheadDistance, obstacleInflationMargin, stallWindow, stallMinProgress, replanMaxDeviation, deviationWindow, maxInfeasibleSteps, minCBFEngagement, engagementMarginThreshold, engagementDeviationThreshold, minReplanInterval, replanCooldown, replanRetryPeriod, maxReplanFailures);
        cbfService = new CBFService(bLookAhead, rSafeDynamic, rSafeStatic, alphaDynamic, alphaStatic, gammaCLF, slackPenalty, cbfSlackPenalty, commandSmoothingWeight, vDeviationWeight, wDeviationWeight, staticActivationDistance, obstacleActivationRange, gradientStepCells, cbfActivationTolerance, wMaxCBF, robotBodyRadius);
        controllerService = new ControllerService(secondsControlFrequency, controlStrategy, (leftWheel, rightWheel), wheelRadius, wheelSeparation, b, zeta, vMax, wMaxClamp, referenceMaxLag, maxLinearAccelCommand, maxAngularAccelCommand, cbfEnabled, cbfService);
        localizationService = new LocalizationService(icpService, marrtionLaserLinkTransform, minInlierRatio, maxResidual, maxLocalizationJump);
        dynamicObstacleService = new DynamicObstacleService(detectionVoxelSize, marrtionLaserLinkTransform, selfHitRadius, obsTol, obsTolPerMeter, maxDetectionRange, clusteringRadius, minClusterPoints, minClusterPointsNear, nearClusterRange, clusterMargin, maxUnexplainedFraction, minSuspiciousSpread, obstacleZMin, obstacleZMax, maxElevationDeg,
            useChassisFootprint, chassisFront, chassisRear, chassisHalfWidth, chassisTopRelLaser, chassisMargin);
        obstacleTrackerService = new ObstacleTrackerService(trackGate, trackAlphaLowpass, trackVDeadzone, trackMinHits, trackForgetTime, blindZoneRadius, blindZoneForgetFactor,
            minPointsForUpdate, radiusDecayPerUpdate, blindZoneGrowthRate, maxBlindGrowth);


        //------------------REGISTRAZIONE EVENTI
        if (calculateAndOverrideOccupancyMapFlag)
            lidar.OnScanComplete += ScanCompletedLetsWork;
        else
            lidar.OnScanComplete += ScanCompletedNavigation;
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
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PathMsg>(splinedTrajectoryRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(startDebugRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(waypointsDebugRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(activeWaypointsDebugRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(goalDebugRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(currentPoseDebugRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(truePoseDebugRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(icpPoseLocalizationDebugRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(dynamicObstaclesRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(trackedObstaclesRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(unexplainedPointsRosTopic);

        //------------------ONE TIME ACTIONS
        if (calculateAndOverrideOccupancyMapFlag == false)
        {
            List<Vector3> globalStoredUpdatedMap = ioService.ReadGlobalPointCloud();
            updatedGlobalPointCloudCached = globalStoredUpdatedMap;
            updatedGlobalPointCloudReady = true;
            graphSlamService.setUpdatedGlobalMapPointCloud(updatedGlobalPointCloudCached);
            publisherService.PublishUpdatedGlobalPointCloudMap(updatedGlobalPointCloudCached, graphSlamGlobalpointCloudTopic);
            icpService.SetLocalizationMap(updatedGlobalPointCloudCached);
            localizationService.Initialize();
            icpLocalizedConfig = localizationService.GetRosPose();   // posa iniziale nota, prima del primo Step

            (sbyte[], int, int, float, float, float) occGrid = ioService.ReadOccupancyGrid();
            occupancyGridCached = occGrid;               // cache per la ripubblicazione throttlata in Update
            occupancyGridCachedReady = true;
            distanceMapService.SetOccupancyGridMap(occGrid);
            distanceMapService.calculateDistanceMap();
            // ESPLICITO: la CBF usa per sempre la EDT della mappa PRISTINA. Il replanning ricalcola la
            // distance map sulla griglia gonfiata, e se la CBF la seguisse conterebbe gli ostacoli due volte
            // (una nella barriera statica, una in quelle dinamiche).
            pristineDistanceMap = distanceMapService.getDistanceMapInstance();
            cbfService.SetDistanceMap(pristineDistanceMap);
            publisherService.PublishDistanceMap(distanceMapService.getDistanceMapForPublisher(), distanceMapRosTopic);
            distanceMapComputed = true;

            if (goalSettedLetsPlan)
            {
                waypoints = readWaypointNodes();   // [start (robot), 1_NODE, 2_NODE, ..., GOAL]
                startDebugPoint = waypoints[0];
                goalDebugPoint = waypoints[waypoints.Count - 1];
                nextWaypointIdx = 1;
                trajectoryPlanned = RunPlanningPipeline(waypoints, distanceMapService.getDistanceMapInstance());
            }

            if(motionPlannerService.GeometricTrajectoryDetermined() && boolControlMarrtino)
            {
                if (differentialDriveController != null) differentialDriveController.autonomousControlActive = true;   // la tastiera si fa da parte
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

        if (occupancyGridCachedReady && Time.time - lastOccupancyPublish > occupancyRepublishPeriod)
        {
            publisherService.PublishOccupancyGridMap(occupancyGridCached, occupancyRosTopic);
            lastOccupancyPublish = Time.time;
        }

        if( updatedGlobalPointCloudReady && Time.time - lastGlobalPointCloudPublish > globalPointCloudPeriod)
        {
            publisherService.PublishUpdatedGlobalPointCloudMap(updatedGlobalPointCloudCached, graphSlamGlobalpointCloudTopic);
            lastGlobalPointCloudPublish = Time.time;
        }

        if( (!calculateAndOverrideOccupancyMapFlag) && boolControlMarrtino && trajectoryPlanned && Time.time - lastTrajectoryPublish > plannedTrajectoryRepublishPeriod)
        {
            if (motionPlannerService.getGeometricTrajectoryWorld().Count > 0)
            {
                publisherService.PublishPlannedTrajectory(motionPlannerService.getGeometricTrajectoryWorld(), plannedTrajectoryRosTopic);
                if (smoothTrajWithLoSPS) publisherService.PublishPlannedTrajectory(motionPlannerService.getGeometricSmoothTrajectoryWorld(), smoothedTrajectoryRosTopic);
                publisherService.PublishLSplinedGeometricTrajectory(motionPlannerService.getSplinedGeomtricTrajectory(), splinedTrajectoryRosTopic);
            }
            publisherService.PublishDebugPoint(startDebugPoint, startDebugRosTopic);
            if(waypoints.Count > 0)
            {
                int i = 0;
                List<(float x, float y)> listWayPoints = new List<(float, float)>();
                while (i < waypoints.Count - 1)
                {
                    listWayPoints.Add((waypoints[i]));
                    i += 1;
                }
                publisherService.PublishListOfROSPointsAsPointCloud(listWayPoints, waypointsDebugRosTopic);
            }
            publisherService.PublishDebugPoint(goalDebugPoint, goalDebugRosTopic);

            // I tre path sopra vengono gia' dal piano CORRENTE (il planner viene azzerato e ricostruito
            // a ogni replan): qui si pubblicano i waypoint che quel piano insegue davvero, che dopo un
            // rilassamento NON coincidono piu' con quelli della missione.
            publisherService.PublishListOfROSPointsAsPointCloud(motionPlannerService.getEffectiveWaypoints(), activeWaypointsDebugRosTopic);

            lastTrajectoryPublish = Time.time;
        }

        if (recordPlotData && plotDataService.isTimeToSamplePose())
        {
            (float trx, float trty, float trz) gtPose = UnityToRosPosition(marrtionLaserLinkTransform.position.x, marrtionLaserLinkTransform.position.y, marrtionLaserLinkTransform.position.z);
            float gtThetaPose = Mathf.Atan2(-marrtionLaserLinkTransform.forward.x, marrtionLaserLinkTransform.forward.z);
            (float xo, float yo, float tho) = odometryService.getUpdatedConfiguration();
            plotDataService.RecordPose(Time.time, (gtPose.trx, gtPose.trty, gtThetaPose), icpLocalizedConfig, (xo, -yo, -tho));
        }

        if (recordPlotData && !plotDataDumped && trajectoryPlanned && boolControlMarrtino && controllerService.isTrajectoryCompleted())
        {
            DumpPlotData();
        }

        if (replanningEnabled && (!calculateAndOverrideOccupancyMapFlag) && boolControlMarrtino && trajectoryPlanned
            && Time.time - lastReplanCheck > replanCheckPeriod)
        {
            lastReplanCheck = Time.time;
            (float x, float y, float theta) pose = getControlPose();
            updateNextWaypointIndex(pose);

            ReplanTrigger trigger = replanningService.EvaluateTriggers(Time.time, pose, obstacleTrackerService.GetConfirmedTracks(),
                controllerService.getTrajectoryTable(), controllerService.getReferenceIndex(), getPlanDeviation(pose));

            if (trigger != ReplanTrigger.None)
            {
                Debug.Log($"Trigger {trigger} (CBF attiva da {replanningService.getCBFEngagementTime():F1}s, scarto {getPlanDeviation(pose):F2} m)");
                ExecuteReplanning(pose, trigger);
            }

            if (replanningService.getState() != lastLoggedState)
            {
                lastLoggedState = replanningService.getState();
                Debug.Log($"Stato replanning: {lastLoggedState}");
            }
        }

        if (odometryService.isTimeToLocalize())
        {
            odometryService.letsLocalizeUsingOdometry();
            currentConfig= odometryService.getUpdatedConfiguration();

            publisherService.PublishTF(currentConfig, marrtionLaserLinkTransform.localPosition, marrtionLaserLinkTransform.localRotation, tfRosTopic);

            publisherService.PublishLastOdometry(currentConfig, odometryRosTopic);

            publisherService.PublishDebugPoint((currentConfig.x, -currentConfig.y), currentPoseDebugRosTopic);
            (float trx, float trY, float trz) truePose = UnityToRosPosition(marrtionLaserLinkTransform.position.x, marrtionLaserLinkTransform.position.y, marrtionLaserLinkTransform.position.z);
            publisherService.PublishDebugPoint((truePose.trx, truePose.trY), truePoseDebugRosTopic);

            if (publishLastNOdometryPoses && Time.time - lastOdometryPathPublish > odometryPathPublishPeriod)
            {
                publisherService.PublishOdometryPath(odometryService.getUpdatedLastOdometryPoses(), odometryPathRosTopic);
                lastOdometryPathPublish = Time.time;

                // DIAGNOSTICO odometria: confronto posa+heading odometria vs VERITA' (frame ROS).
                // thetaOdo dovrebbe seguire eulerAngles.y. Se diverge l'ANGOLO -> bug heading; se diverge solo
                // la POSIZIONE con angolo giusto -> scala/slip su v.
                //Debug.Log($"ODO pos=({currentConfig.x:F3},{-currentConfig.y:F3}) thetaOdo={currentConfig.theta * Mathf.Rad2Deg:F1}  |  TRUE pos=({truePose.trx:F3},{truePose.trY:F3}) eulerY={marrtionLaserLinkTransform.eulerAngles.y:F1}");
            }

        }

        // Localizzazione scan-to-map: predict->correct->gate a ~localizationPeriod; posa pubblicata per RViz.
        if (useICPLocalization && odometryService.isRobotMoving() && Time.time - lastLocalization > localizationPeriod)
        {
            (float lx, float ly, float lth, bool accepted) = localizationService.Step(odometryService.getUpdatedConfiguration());
            icpLocalizedConfig = (lx, ly, lth);
            publisherService.PublishDebugPoint((lx, ly), icpPoseLocalizationDebugRosTopic);
            lastLocalization = Time.time;
        }

        if( (!calculateAndOverrideOccupancyMapFlag) && boolControlMarrtino && controllerService.isTimeToControl())
        {
            (float x, float y, float theta) currentConfigRos = getControlPose();

            (double v, double w) feedbackControl = controllerService.ControlStep(currentConfigRos);
            replanningService.NotifyControlStep(controllerService.isLastCBFFeasible(), controllerService.getLastCBFDeviation(), cbfService.getLastDiagnostics().minMargin, Time.time);

            if (recordPlotData)
            {
                plotDataService.RecordControl(Time.time, feedbackControl, controllerService.getLastNominalControl(), controllerService.isCBFActive(), controllerService.isLastCBFFeasible(), cbfService.getLastDiagnostics(), controllerService.getLastTrackingTerms());
            }

            if (flipControlOmega) feedbackControl.w = -feedbackControl.w;
            controllerService.applyToWheels(feedbackControl);

            if (recordPlotData)
            {
                plotDataService.RecordWheels(Time.time, controllerService.getLastWheelCommand(), leftWheel.jointVelocity[0], rightWheel.jointVelocity[0],
                    leftWheel.xDrive.targetVelocity * Mathf.Deg2Rad, rightWheel.xDrive.targetVelocity * Mathf.Deg2Rad);
            }
        }

     
        if (calculateAndOverrideOccupancyMapFlag && loopClosureMode == LoopClosureFinder.TimeAndConeBased)
        {
            publisherService.PublishConeFan(marrtionLaserLinkTransform.position, marrtionLaserLinkTransform.forward, halfConeAngleLoopClosureFinder, thresholdLoopClosure, maxRadiusLoopClosureFinder, coneFanTopic);
        }
    }

    // Un ostacolo marginale (pochi punti, conferme intermittenti) puo' non diventare mai un track:
    // da vicino e' inaccettabile perderlo, quindi le detection grezze ravvicinate entrano comunque
    // nel QP. Velocita' nulla: su una singola scansione non e' stimabile.
    private List<(float cx, float cy, float r, float vx, float vy)> mergeCloseRangeDetections(List<(float cx, float cy, float r, float vx, float vy)> tracked, List<(float cx, float cy, float r, int n)> detections, (float x, float y, float theta) pose)
    {
        if (detections == null) return tracked;

        foreach ((float cx, float cy, float r, int n) detection in detections)
        {
            float dx = detection.cx - pose.x;
            float dy = detection.cy - pose.y;
            if (Mathf.Sqrt(dx * dx + dy * dy) > closeRangeOverride) continue;

            bool alreadyTracked = false;
            foreach ((float cx, float cy, float r, float vx, float vy) t in tracked)
            {
                float ex = t.cx - detection.cx;
                float ey = t.cy - detection.cy;
                // gate STRETTO: con 0.5 m un track sbagliato sopprimeva la detection fresca che
                // avrebbe dovuto scavalcarlo, disattivando proprio la rete di sicurezza
                if (ex * ex + ey * ey < closeRangeMergeGate * closeRangeMergeGate) { alreadyTracked = true; break; }
            }
            if (!alreadyTracked) tracked.Add((detection.cx, detection.cy, detection.r, 0f, 0f));
        }
        return tracked;
    }

    private List<(float cx, float cy, float r)> toCircles(List<(float cx, float cy, float r, int n)> detections)
    {
        List<(float cx, float cy, float r)> circles = new List<(float, float, float)>();
        foreach ((float cx, float cy, float r, int n) detection in detections) circles.Add((detection.cx, detection.cy, detection.r));
        return circles;
    }

    // Posa su cui si chiude l'anello di controllo, nella convenzione ROS.
    private (float x, float y, float theta) getControlPose()
    {
        if (controlOnGroundTruth)
        {
            (float grx, float gry, float grz) gt = UnityToRosPosition(marrtionLaserLinkTransform.position.x, marrtionLaserLinkTransform.position.y, marrtionLaserLinkTransform.position.z);
            return (gt.grx, gt.gry, Mathf.Atan2(-marrtionLaserLinkTransform.forward.x, marrtionLaserLinkTransform.forward.z));
        }
        if (useICPLocalization) return icpLocalizedConfig;

        (float xo, float yo, float tho) = odometryService.getUpdatedConfiguration();
        return (xo, -yo, -tho);
    }

    // Pipeline completa A* -> LOS-PS -> spline -> profilo -> Arm. Usata sia all'avvio sia dal replanning.
    private bool RunPlanningPipeline(List<(float, float)> wps, DistanceMap dmap)
    {
        motionPlannerService.Reset();
        lastPlanningFailure = "none";

        if (!motionPlannerService.DetermineGeometricTrajectoryFromWaypoints(dmap, wps, plannerMode))
        {
            lastPlanningFailure = "noPath";
            Debug.LogWarning("Pianificazione fallita: nessun percorso per i waypoint richiesti");
            return false;
        }

        motionPlannerService.LetsSplineGeometricTrajectory();
        motionPlannerService.DetermineGeomtricTrajectoryFromSplines();

        List<(float t, float x, float y, float xd, float yd, float xdd, float ydd)> table = motionPlannerService.getGeometricTrajectoryForController();
        if (!motionPlannerService.ControllerTrajectoryIsFinite())
        {
            lastPlanningFailure = "badTable";
            Debug.LogWarning($"Pianificazione fallita: tabella non valida ({table.Count} righe). "
                + $"A*={motionPlannerService.getGeometricTrajectoryWorld().Count} punti, "
                + $"LOS-PS={motionPlannerService.getGeometricSmoothTrajectoryWorld().Count} punti, "
                + $"spacing minimo={motionPlannerService.getMinConsecutiveSpacing():F4} m");
            return false;
        }

        if (boolControlMarrtino) controllerService.Arm(table);
        if (recordPlotData) plotDataService.RecordPlan(Time.time, table);
        return true;
    }

    // Waypoint non ancora raggiunti, preceduti dalla posa corrente: e' da li' che riparte il nuovo piano.
    private List<(float, float)> getRemainingWaypoints((float x, float y, float theta) fromPose)
    {
        List<(float, float)> remaining = new List<(float, float)>();
        remaining.Add((fromPose.x, fromPose.y));
        for (int i = nextWaypointIdx; i < waypoints.Count; i++) remaining.Add(waypoints[i]);
        return remaining;
    }

    private void updateNextWaypointIndex((float x, float y, float theta) pose)
    {
        if (waypoints == null || nextWaypointIdx >= waypoints.Count - 1) return;   // il goal non si consuma

        // Il solo criterio di prossimita' non basta: evitando un ostacolo il robot puo' girare al largo
        // e non passare MAI entro waypointReachedRadius, lasciando l'indice bloccato su un waypoint che
        // si e' di fatto lasciato alle spalle. Si considera superato anche chi e' finito dietro il piano
        // perpendicolare al segmento verso il waypoint successivo.
        while (nextWaypointIdx < waypoints.Count - 1)
        {
            float dx = waypoints[nextWaypointIdx].Item1 - pose.x;
            float dy = waypoints[nextWaypointIdx].Item2 - pose.y;
            bool nearEnough = dx * dx + dy * dy < waypointReachedRadius * waypointReachedRadius;

            float sx = waypoints[nextWaypointIdx + 1].Item1 - waypoints[nextWaypointIdx].Item1;
            float sy = waypoints[nextWaypointIdx + 1].Item2 - waypoints[nextWaypointIdx].Item2;

            // Il criterio del semipiano vale SOLO da vicino: su un percorso che si richiude, un
            // waypoint lontano risulta facilmente "oltrepassato" e verrebbe saltato, violando
            // l'ordine della missione.
            bool closeEnoughToJudge = dx * dx + dy * dy < waypointPassedRadius * waypointPassedRadius;
            bool passed = closeEnoughToJudge
                && (pose.x - waypoints[nextWaypointIdx].Item1) * sx + (pose.y - waypoints[nextWaypointIdx].Item2) * sy > 0f;

            if (!nearEnough && !passed) break;
            nextWaypointIdx += 1;
            Debug.Log($"Waypoint {nextWaypointIdx - 1} superato (vicino={nearEnough}, oltrepassato={passed}) -> prossimo {nextWaypointIdx}");
        }
    }

    // Distanza dal punto piu' vicino del piano attivo: misura lo scostamento per il trigger.
    private float getPlanDeviation((float x, float y, float theta) pose)
    {
        List<(float t, float x, float y, float xd, float yd, float xdd, float ydd)> table = controllerService.getTrajectoryTable();
        if (table.Count == 0) return 0f;

        float best = float.MaxValue;
        foreach ((float t, float x, float y, float xd, float yd, float xdd, float ydd) row in table)
        {
            float dx = row.x - pose.x;
            float dy = row.y - pose.y;
            float sq = dx * dx + dy * dy;
            if (sq < best) best = sq;
        }
        return Mathf.Sqrt(best);
    }

    private bool tryPlanWithInflation((float x, float y, float theta) currentPose, List<ObstacleTrack> tracks, float margin)
    {
        (sbyte[] data, int W, int H, float originX, float originY, float resolution) inflated = replanningService.InflateOccupancyGrid(occupancyGridCached, tracks, Time.time, margin);

        distanceMapService.SetOccupancyGridMap(inflated);
        distanceMapService.calculateDistanceMap();
        publisherService.PublishDistanceMap(distanceMapService.getDistanceMapForPublisher(), distanceMapRosTopic);

        return RunPlanningPipeline(getRemainingWaypoints(currentPose), distanceMapService.getDistanceMapInstance());
    }

    // Il servizio resta con la griglia gonfiata dopo un replan: va riportato alla mappa vera, altrimenti
    // cio' che viene pubblicato (e chiunque legga la mappa viva) continua a mostrare ostacoli inventati.
    private void restorePristineDistanceMap()
    {
        distanceMapService.SetOccupancyGridMap(occupancyGridCached);
        distanceMapService.calculateDistanceMap();
        publisherService.PublishDistanceMap(distanceMapService.getDistanceMapForPublisher(), distanceMapRosTopic);
    }

    // Ferma il robot, gonfia la griglia PRISTINA con gli ostacoli persistenti, ricalcola la EDT e ripianifica.
    private void ExecuteReplanning((float x, float y, float theta) currentPose, ReplanTrigger trigger)
    {
        Debug.Log($"Replanning (trigger: {trigger}) dalla posa ({currentPose.x:F2}, {currentPose.y:F2})");

        controllerService.Disarm();
        controllerService.applyToWheels((0.0, 0.0));

        List<ObstacleTrack> tracks = obstacleTrackerService.GetConfirmedTracks();

        // In corridoio stretto il disco gonfiato puo' essere piu' largo della clearance disponibile e
        // chiudere il passaggio: meglio un percorso rasente che nessun percorso. Si riprova a margine ridotto.
        bool success = tryPlanWithInflation(currentPose, tracks, replanningService.getInflationMargin());
        float usedMargin = replanningService.getInflationMargin();
        if (!success)
        {
            usedMargin = replanningService.getInflationMargin() * reducedInflationFactor;
            Debug.LogWarning($"Replanning: nessun percorso con margine {replanningService.getInflationMargin():F2} m, riprovo con {usedMargin:F2} m");
            success = tryPlanWithInflation(currentPose, tracks, usedMargin);
        }
        if (!success)
        {
            usedMargin = 0f;   // solo l'ingombro misurato: resta comunque la CBF a tenere le distanze
            Debug.LogWarning("Replanning: riprovo senza margine di inflazione");
            success = tryPlanWithInflation(currentPose, tracks, usedMargin);
        }
        replanningService.NotifyReplanResult(success, Time.time);
        if (recordPlotData) plotDataService.RecordReplan(Time.time, currentPose, trigger.ToString(), success, usedMargin, tracks.Count, replanningService.getFailureCount(), lastPlanningFailure);

        if (success)
        {
            lastTrajectoryPublish = 0f;   // forza la ripubblicazione del nuovo percorso
        }
        else
        {
            // Riprendere il vecchio piano va bene per un fallimento isolato, NON quando il planner
            // ha concluso ripetutamente che un percorso non esiste: li' significa andare addosso
            // all'ostacolo. In stato Blocked si resta fermi, ma i trigger continuano a girare,
            // quindi si riparte da soli appena la situazione si sblocca.
            if (replanningService.isBlocked()) controllerService.applyToWheels((0.0, 0.0));
            else controllerService.ReArm();
            restorePristineDistanceMap();
            Debug.LogWarning($"Replanning fallito: riprendo il piano precedente (stato {replanningService.getState()})");
        }
    }

    // Legge i waypoint dalla scena: start (posa laser) + oggetti "1_NODE","2_NODE",... (numerazione contigua,
    // mi fermo al primo mancante) + "GOAL STATE" (fallback ai goalXUnity/goalZUnity se assente). Tutto in frame ROS.
    private List<(float, float)> readWaypointNodes()
    {
        List<(float, float)> waypoints = new List<(float, float)>();

        (float sx, float sy, float sz) start = UnityToRosPosition(marrtionLaserLinkTransform.position.x, marrtionLaserLinkTransform.position.y, marrtionLaserLinkTransform.position.z);
        waypoints.Add((start.sx, start.sy));

        int i = 1;
        GameObject node;
        while ((node = GameObject.Find($"{i}_NODE")) != null)
        {
            (float nx, float ny, float nz) n = UnityToRosPosition(node.transform.position.x, node.transform.position.y, node.transform.position.z);
            waypoints.Add((n.nx, n.ny));
            i++;
        }

        GameObject goalObj = GameObject.Find(goalStateObjectName);
        if (goalObj != null)
        {
            (float gx, float gy, float gz) g = UnityToRosPosition(goalObj.transform.position.x, goalObj.transform.position.y, goalObj.transform.position.z);
            waypoints.Add((g.gx, g.gy));
        }
        else
        {
            (float gx, float gy, float gz) g = UnityToRosPosition(goalXUnity, 0, goalZUnity);   // fallback hard-coded
            waypoints.Add((g.gx, g.gy));
        }

        return waypoints;
    }

    void DumpPlotData()
    {
        plotDataDumped = true;
        ioService.WriteCsv("control.csv", PlotDataService.CONTROL_HEADER, plotDataService.getControlRows());
        ioService.WriteCsv("poses.csv", PlotDataService.POSE_HEADER, plotDataService.getPoseRows());
        ioService.WriteCsv("plans.csv", PlotDataService.PLAN_HEADER, plotDataService.getPlanRows());
        ioService.WriteCsv("obstacles.csv", PlotDataService.OBSTACLE_HEADER, plotDataService.getObstacleRows());
        ioService.WriteCsv("detection.csv", PlotDataService.DETECTION_HEADER, plotDataService.getDetectionRows());
        ioService.WriteCsv("replans.csv", PlotDataService.REPLAN_HEADER, plotDataService.getReplanRows());
        ioService.WriteCsv("wheels.csv", PlotDataService.WHEEL_HEADER, plotDataService.getWheelRows());
        ioService.WriteCsv("limits.csv", "name,value", getPlotLimits());
        if (distanceMapComputed)
        {
            (int W, int H, float originX, float originY, float resolution) meta = distanceMapService.getDistanceMapMetadata();
            ioService.WriteDistanceMapBinary(distanceMapService.getDistanceMap(), meta.W, meta.H, meta.originX, meta.originY, meta.resolution);
        }
        Debug.Log($"Dati per i grafici in: {ioService.GetRunDirectory()}");
    }

    private List<string> getPlotLimits()
    
    {
        System.Globalization.CultureInfo inv = System.Globalization.CultureInfo.InvariantCulture;
        return new List<string>() {
            "vMax," + vMax.ToString(inv),
            "linearMeanVelocity," + linearMeanVelocity.ToString(inv),
            "wMax," + wMax.ToString(inv),
            "wMaxClamp," + wMaxClamp.ToString(inv),
            "wMaxCBF," + wMaxCBF.ToString(inv),
            "aMax," + aMax.ToString(inv),
            "maxLinearAccelCommand," + maxLinearAccelCommand.ToString(inv),
            "maxAngularAccelCommand," + maxAngularAccelCommand.ToString(inv),
            "rSafeStatic," + rSafeStatic.ToString(inv),
            "commandSmoothingWeight," + commandSmoothingWeight.ToString(inv),
            "selfHitRadius," + selfHitRadius.ToString(inv),
            "closeRangeOverride," + closeRangeOverride.ToString(inv),
            "obsTol," + obsTol.ToString(inv),
            "obsTolPerMeter," + obsTolPerMeter.ToString(inv),
            "maxDetectionRange," + maxDetectionRange.ToString(inv),
            "obstacleZMin," + obstacleZMin.ToString(inv),
            "obstacleZMax," + obstacleZMax.ToString(inv),
            "maxElevationDeg," + maxElevationDeg.ToString(inv),
            "clusteringRadius," + clusteringRadius.ToString(inv),
            "minClusterPoints," + minClusterPoints.ToString(inv),
            "clusterMargin," + clusterMargin.ToString(inv),
            "maxUnexplainedFraction," + maxUnexplainedFraction.ToString(inv),
            "minSuspiciousSpread," + minSuspiciousSpread.ToString(inv),
            "trackMinHits," + trackMinHits.ToString(inv),
            "trackForgetTime," + trackForgetTime.ToString(inv),
            "trackGate," + trackGate.ToString(inv),
            "blindZoneRadius," + blindZoneRadius.ToString(inv),
            "wheelRadius," + wheelRadius.ToString(inv),
            "wheelSeparation," + wheelSeparation.ToString(inv),
            "useChassisFootprint," + (useChassisFootprint ? "1" : "0"),
            "robotBodyRadius," + robotBodyRadius.ToString(inv),
            "minPointsForUpdate," + minPointsForUpdate.ToString(inv),
            "radiusDecayPerUpdate," + radiusDecayPerUpdate.ToString(inv),
            "blindZoneGrowthRate," + blindZoneGrowthRate.ToString(inv),
            "maxBlindGrowth," + maxBlindGrowth.ToString(inv),
            "engagementMarginThreshold," + engagementMarginThreshold.ToString(inv),
            "engagementDeviationThreshold," + engagementDeviationThreshold.ToString(inv),
            "minReplanInterval," + minReplanInterval.ToString(inv),
            "waypointPassedRadius," + waypointPassedRadius.ToString(inv),
            "collinearToleranceDeg," + collinearToleranceDeg.ToString(inv),
            "minVertexSpacing," + minVertexSpacing.ToString(inv),
            "waypointRelaxClearance," + waypointRelaxClearance.ToString(inv),
            "waypointRelaxRadius," + waypointRelaxRadius.ToString(inv),
            "startEscapeRadius," + startEscapeRadius.ToString(inv),
            "closeRangeMergeGate," + closeRangeMergeGate.ToString(inv),
            "voxelSize," + voxelSize.ToString(inv),
            "detectionVoxelSize," + detectionVoxelSize.ToString(inv),
            "minClusterPointsNear," + minClusterPointsNear.ToString(inv),
            "nearClusterRange," + nearClusterRange.ToString(inv),
            "rSafeDynamic," + rSafeDynamic.ToString(inv),
            "bLookAhead," + bLookAhead.ToString(inv),
        };
    }

    void OnApplicationQuit()
    {
        if (recordPlotData && !plotDataDumped) DumpPlotData();
    }

    void ClearVisualizationTopics()
    {
        publisherService.PublishEmptyPointCloud(graphSlamNodesTopic);
        publisherService.PublishEmptyPointCloud(loopClosureCircleTopic);
        publisherService.PublishEmptyPointCloud(loopClosureEdgesTopic);
        publisherService.PublishEmptyPointCloud(coneFanTopic);                       // in navigazione non si aggiorna piu'
        publisherService.PublishICPPath(new Queue<Vector3>(), icpPathRosTopic);      // path vuota -> ripulisce /icp/path
        publisherService.PublishEmptyPointCloud(startDebugRosTopic);
        publisherService.PublishEmptyPointCloud(goalDebugRosTopic);

        if (!boolControlMarrtino)
        {
            publisherService.PublishEmptyPointCloud(graphSlamGlobalpointCloudTopic);
            publisherService.PublishEmptyPointCloud(plannedTrajectoryRosTopic);
            publisherService.PublishEmptyPointCloud(smoothedTrajectoryRosTopic);
            publisherService.PublishEmptyPointCloud(splinedTrajectoryRosTopic);
        }

    }

    void ScanCompletedNavigation()
    {
        obstaclesCentroidsSensed = dynamicObstacleService.getROSObstacleCentroids(lidar.ScannedPoints, localizationService.GetTMapLaser(), pristineDistanceMap);
        //Debug.Log($"Sensed and publishing {obstaclesCentroidsSensed.Count} obstacles centroids...");
        publisherService.PublishListOfObstacleCentroids(toCircles(obstaclesCentroidsSensed), dynamicObstaclesRosTopic);

        obstacleTrackerService.Update(obstaclesCentroidsSensed, Time.time, getControlPose());
        publisherService.PublishListOfObstacleCentroids(obstacleTrackerService.GetConfirmedCircles(), trackedObstaclesRosTopic);
        cbfService.SetObstacles(mergeCloseRangeDetections(obstacleTrackerService.GetConfirmedCirclesWithVelocity(), obstaclesCentroidsSensed, getControlPose()));
        publisherService.PublishListOfROSPoints3D(dynamicObstacleService.getLastUnexplainedPoints3D(), unexplainedPointsRosTopic);
        if (recordPlotData)
        {
            plotDataService.RecordObstacles(Time.time, obstacleTrackerService.GetConfirmedTracks());
            plotDataService.RecordDetection(Time.time, getControlPose(), dynamicObstacleService.getLastDetectionDiagnostics(), obstacleTrackerService.GetConfirmedTracks().Count);
        }
        //foreach (ObstacleTrack t in obstacleTrackerService.GetConfirmedTracks())
        //    Debug.Log($"track {t.id}: pos=({t.cx:F2},{t.cy:F2}) r={t.r:F2} v=({t.vx:F2},{t.vy:F2}) age={t.age(Time.time):F1}s hits={t.hits}");
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

        publisherService.PublishOccupancyGridMap(occupancyGridService.getDataForPublisher(), occupancyRosTopic);

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
                ioService.WriteGlobalPointCloud(icpService.ICPToWorldPositionListVectors(graphSlamService.getUpdatedGlobalMapPointCloud()));
            }

            publisherService.PublishUpdatedGlobalPointCloudMap( icpService.ICPToWorldPositionListVectors(graphSlamService.getUpdatedGlobalMapPointCloud()) , graphSlamGlobalpointCloudTopic);
            publisherService.PublishLoopClosureEdges(icpService.ICPToWorldPositionPairs(graphSlamService.getClosureEdgeVector3Couples()), loopClosureEdgesTopic);
        }
    }

}
