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
    public string waypointsDebugRosTopic = "/debug/waypoints";
    public string goalDebugRosTopic = "/debug/goal_pose";
    public string currentPoseDebugRosTopic = "/debug/current_pose";   // posa corrente (odometria di ruota) in frame ROS
    public string truePoseDebugRosTopic = "/debug/true_pose";         // posa VERA (transform live -> ROS), solo debug
    public string icpPoseLocalizationDebugRosTopic = "/debug/localization/icp_pose";

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
    public float linearMeanVelocity = 0.3f;
    public (double vel_qi, double acc_qi) qiCouple = (0.0, 0.0);            //unico vettore perché suppongo gli stessi valori sia per x che y 
    public (double vel_qf, double acc_qf) qfCouple = (0.0, 0.0);
    public int subSamplesPerSpline = 30;
    private List<(float, float)> waypoints;

    public string CONTROLSTRAtEGIESFIELDS = "FOLLOWING CONTROL SERVICE FIELDS";
    public int secondsControlFrequency = 20;
    public bool boolControlMarrtino = true;
    public ControlStrategy controlStrategy = ControlStrategy.NonlinearControl;
    public bool controlOnGroundTruth = false;
    public bool flipControlOmega = true;
    private (float x, float y, float theta) currentConfig;
    public float b = 10.0f;
    public float zeta = 0.8f;
    public float vMax = 0.5f;       // circa 1.5 volte linearMeanVelocity
    public float wMax = 2.0f;        // cap di CURVATURA del profilo (rallenta in curva): tienilo fisico ~2
    public float wMaxClamp = 4.0f;   // clamp di sicurezza su omega del controller: > wMax, da' margine al feedback
    public float aMax = 0.2f;
    public bool useICPLocalization = true;
    public float minInlierRatio = 0.6f;      // frazione minima di scansione spiegata dalla mappa
    public float maxResidual = 0.25f;        // errore medio max [m]; ~2-3x voxelSize
    public float localizationPeriod = 0.35f;  // l'ICP e' costoso: correggi a ~10 Hz, non ogni frame
    public float maxLocalizationJump = 0.4f;  // salto max [m] corrected-vs-guess: oltre -> convergenza ICP errata, scarto
    private float lastLocalization = 0f;
    private (float x, float y, float theta) icpLocalizedConfig;
    public int maxIterationLocalization = 7;

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
        occupancyGridService = new OccupancyGridService(resolution, zMin, zMax, probOcc, probFree, occBlockThreshold, elevThrehsold);
        ioService = new IOFileOperationService(occupancyGridMapFileName);
        distanceMapService = new DistanceMapService(obstacleThreshold, k, eps);
        motionPlannerService = new MotionPlannerService(smoothTrajWithLoSPS, epsilonTunnelLoSPS, linearMeanVelocity, qiCouple, qfCouple, subSamplesPerSpline, secondsControlFrequency, wMax, aMax, vMax);
        controllerService = new ControllerService(secondsControlFrequency, controlStrategy, (leftWheel, rightWheel), wheelRadius, wheelSeparation, b, zeta, vMax, wMaxClamp);
        localizationService = new LocalizationService(icpService, marrtionLaserLinkTransform, minInlierRatio, maxResidual, maxLocalizationJump);

        //------------------REGISTRAZIONE EVENTI
        if (calculateAndOverrideOccupancyMapFlag)
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
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PathMsg>(splinedTrajectoryRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(startDebugRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(waypointsDebugRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(goalDebugRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(currentPoseDebugRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(truePoseDebugRosTopic);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<PointCloud2Msg>(icpPoseLocalizationDebugRosTopic);

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
            publisherService.PublishDistanceMap(distanceMapService.getDistanceMapForPublisher(), distanceMapRosTopic);
            distanceMapComputed = true;

            if (goalSettedLetsPlan)
            {
                waypoints = readWaypointNodes();   // [start (robot), 1_NODE, 2_NODE, ..., GOAL]
                startDebugPoint = waypoints[0];
                goalDebugPoint = waypoints[waypoints.Count - 1];
                motionPlannerService.DetermineGeometricTrajectoryFromWaypoints(distanceMapService.getDistanceMapInstance(), waypoints, plannerMode);
                trajectoryPlanned = motionPlannerService.GeometricTrajectoryDetermined();
                motionPlannerService.LetsSplineGeometricTrajectory();
                motionPlannerService.DetermineGeomtricTrajectoryFromSplines();
            }

            if(motionPlannerService.GeometricTrajectoryDetermined() && boolControlMarrtino)
            {
                controllerService.Arm(motionPlannerService.getGeometricTrajectoryForController());
                if (differentialDriveController != null) differentialDriveController.autonomousControlActive = true;   // la tastiera si fa da parte
                //controllerService.FollowTrajectory(motionPlannerService.getGeometricTrajectoryForController());
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
            publisherService.PublishPlannedTrajectory(motionPlannerService.getGeometricTrajectoryWorld(), plannedTrajectoryRosTopic);
            if (smoothTrajWithLoSPS) publisherService.PublishPlannedTrajectory(motionPlannerService.getGeometricSmoothTrajectoryWorld(), smoothedTrajectoryRosTopic);
            publisherService.PublishLSplinedGeometricTrajectory(motionPlannerService.getSplinedGeomtricTrajectory(), splinedTrajectoryRosTopic);
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
                publisherService.PublishListOfROSPoints(listWayPoints, waypointsDebugRosTopic);
            }
            publisherService.PublishDebugPoint(goalDebugPoint, goalDebugRosTopic);
            lastTrajectoryPublish = Time.time;
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
            (float x, float y, float theta) currentConfigRos;
            if (controlOnGroundTruth)
            {
                (float grx, float gry, float grz) gt = UnityToRosPosition(marrtionLaserLinkTransform.position.x, marrtionLaserLinkTransform.position.y, marrtionLaserLinkTransform.position.z);
                float gtTheta = Mathf.Atan2(-marrtionLaserLinkTransform.forward.x, marrtionLaserLinkTransform.forward.z);
                currentConfigRos = (gt.grx, gt.gry, gtTheta);
            }
            else if (useICPLocalization)
            {
                currentConfigRos = icpLocalizedConfig;
            }
            else
            {
                (float xo, float yo, float tho) = odometryService.getUpdatedConfiguration();
                currentConfigRos = (xo, -yo, -tho);
            }

            (double v, double w) feedbackControl = controllerService.ControlStep(currentConfigRos);

            if (flipControlOmega) feedbackControl.w = -feedbackControl.w;
            controllerService.applyToWheels(feedbackControl);
        }

     
        if (calculateAndOverrideOccupancyMapFlag && loopClosureMode == LoopClosureFinder.TimeAndConeBased)
        {
            publisherService.PublishConeFan(marrtionLaserLinkTransform.position, marrtionLaserLinkTransform.forward, halfConeAngleLoopClosureFinder, thresholdLoopClosure, maxRadiusLoopClosureFinder, coneFanTopic);
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
