using System;
using UnityEngine;
using static UnicycleModelUtilities;

public class OdometryModel
{
    private float wheelVelocityThreshold;
    private float angularVelocityThreshold;

    private float current_xt;
    private float current_yt;
    private float current_thetat;

    private float wheelSeparation;
    private float wheelRadius;

    public OdometryModel(float wheelVelocityThreshold, float angularVelocityThreshold, (float currentX, float currentY, float currentTheta) currentConfig, float wheelSeparation, float wheelRadius)
    {
        this.wheelVelocityThreshold = wheelVelocityThreshold;
        this.angularVelocityThreshold = angularVelocityThreshold;

        current_xt = currentConfig.currentX; // z perché Unity usa XZ come piano orizzontale
        current_yt = currentConfig.currentY;
        current_thetat = currentConfig.currentTheta;

        this.wheelSeparation = wheelSeparation;
        this.wheelRadius = wheelRadius;
    }

    public void ComputeOdometryLocalization(float wL, float wR)
    {
        if (Mathf.Abs(wL) < wheelVelocityThreshold && Mathf.Abs(wR) < wheelVelocityThreshold)
        {
            return;
        }

        float dt = Time.deltaTime;
        (float, float) wheelDirectVelocities = GetDirectVelocities(wheelSeparation, wheelRadius, wL, wR);
        float vt = wheelDirectVelocities.Item1;
        float wt = wheelDirectVelocities.Item2;

        (float, float, float) newCoordinates;
        if (Mathf.Abs(wt) <= angularVelocityThreshold)
        {
            newCoordinates = GetRungeKuttaOdometry(vt, wt, dt);
        }
        else
        {
            newCoordinates = GetExactOdometry(vt, wt, dt);
        }

        //Debug.Log($"vt={vt:F3} wt={wt:F3} dx={newCoordinates.Item1 - current_xt:F4} dy={newCoordinates.Item2 - current_yt:F4}");

        current_xt = newCoordinates.Item1;
        current_yt = newCoordinates.Item2;
        current_thetat = newCoordinates.Item3;

    }

    private (float, float, float) GetRungeKuttaOdometry(float vt, float wt, float dt)
    {
        float new_xt = current_xt + vt * dt * Mathf.Cos(current_thetat + 0.5f * (wt * dt));
        float new_yt = current_yt + vt * dt * Mathf.Sin(current_thetat + 0.5f * (wt * dt));
        float new_thetat = current_thetat + wt * dt;
        return (new_xt, new_yt, new_thetat);
    }

    private (float, float, float) GetExactOdometry(float vt, float wt, float dt)
    {
        float new_thetat = current_thetat + wt * dt;
        float new_xt = current_xt + (vt / wt) * (Mathf.Sin(new_thetat) - Mathf.Sin(current_thetat));
        float new_yt = current_yt - (vt / wt) * (Mathf.Cos(new_thetat) - Mathf.Cos(current_thetat));
        return (new_xt, new_yt, new_thetat);
    }

    internal (float, float, float) getCurrentUpdatedPosition()
    {
        return (current_xt, current_yt, current_thetat);
    }
}
