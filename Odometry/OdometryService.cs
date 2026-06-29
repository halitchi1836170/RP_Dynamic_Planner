using System;
using System.Collections.Generic;
using UnityEngine;

public class OdometryService
{
    private float odometryFrequency;
    private float lastTimeOdometry = 0.0f;

    private float wheelVelocityThreshold;

    private ArticulationBodyRefs articulationBodyRefs;
    private ArticulationBody leftWheel;
    private ArticulationBody rightWheel;

    private OdometryModel odometryModel;

    private (float, float, float) currentConfiguration;

    private bool publishLastNOdometryPoses;
    private Queue<Vector3> lastNOdometryPoses;
    private int nOdometryLastPoses;

    public OdometryService(float angularVelocityThreshold, float wheelVelocityThreshold, float odometryFrequency, ArticulationBodyRefs articulationBodiesRefs, bool publishLastNOdometryPoses, int NOdometryLastPoses = 0)
    {
        this.odometryFrequency = odometryFrequency;
        this.articulationBodyRefs = articulationBodiesRefs;
        this.wheelVelocityThreshold = wheelVelocityThreshold;

        (ArticulationBody, ArticulationBody) wheels = articulationBodyRefs.getWheelArticulationBodyReference();
        leftWheel = wheels.Item1;
        rightWheel = wheels.Item2;

        (float currentX, float currentY, float currentTheta) currentConfig = articulationBodyRefs.getTransformCurrentConfiguration();
        this.currentConfiguration = currentConfig;
        
        this.odometryModel = new OdometryModel(wheelVelocityThreshold, angularVelocityThreshold, currentConfig, articulationBodyRefs.wheelSeparation, articulationBodyRefs.wheelRadius);

        if (publishLastNOdometryPoses)
        {
            this.lastNOdometryPoses = new Queue<Vector3>();
            this.nOdometryLastPoses = NOdometryLastPoses;
        }

    }

    public bool isTimeToLocalize()
    {
        return lastTimeOdometry == 0.0f || (Time.time - lastTimeOdometry) > (1.0f / odometryFrequency);
    }

    private (float, float) GetWheelJointVelocities()
    {
        return (leftWheel.jointVelocity[0], rightWheel.jointVelocity[0]);
    }

    // Il robot si sta muovendo davvero (traslazione o rotazione) se almeno una ruota gira sopra soglia.
    // Serve a NON creare nodi/aggiornare la griglia quando il robot è fermo: così il drift dell'ICP
    // da fermo non genera nodi spuri (e quindi muri "fantasma" paralleli nella occupancy grid).
    public bool isRobotMoving()
    {
        (float wL, float wR) = GetWheelJointVelocities();
        return Mathf.Abs(wL) > wheelVelocityThreshold || Mathf.Abs(wR) > wheelVelocityThreshold;
    }

    public void letsLocalizeUsingOdometry()
    {
        (float, float) wheelAngularVelocities = GetWheelJointVelocities();
        //dDebug.Log($"wL={leftWheel.jointVelocity[0]:F6} wR={rightWheel.jointVelocity[0]:F6}");

        float wL = wheelAngularVelocities.Item1;
        float wR = wheelAngularVelocities.Item2;

        odometryModel.ComputeOdometryLocalization(wL, wR);
        lastTimeOdometry = Time.time;

        this.currentConfiguration = odometryModel.getCurrentUpdatedPosition();

        if (this.publishLastNOdometryPoses)
        {
            UpdateLastNOdometryPosesQueue(this.currentConfiguration);
        }

    }

    private void UpdateLastNOdometryPosesQueue((float, float, float) currentConfiguration)
    {
        Vector3 newPosition = new Vector3(currentConfiguration.Item2, 0.1f, currentConfiguration.Item1);
        if (this.lastNOdometryPoses.Count == this.nOdometryLastPoses)
        {
            this.lastNOdometryPoses.Dequeue();
        }
        this.lastNOdometryPoses.Enqueue(newPosition);
    }

    public (float, float, float) getUpdatedConfiguration()
    {
        return this.currentConfiguration;
    }

    public Queue<Vector3>  getUpdatedLastOdometryPoses()
    {
        return this.lastNOdometryPoses;
    }
}
