using System;
using System.Collections.Generic;
using UnityEngine;
using static UnicycleModelUtilities;

public enum ControlStrategy { ApproximateLinearization, NonlinearControl, IOLinearization };

public class ControllerService
{
    private int secondsControlFrequency;
    private ControlStrategy controlStrategy;
    private List<(float tl, float x, float y, float dx, float dy, float ddx, float ddyt)> geometricTrajectoryTable;
    private int iter;
    private bool controllerActive;
    private float lastControl;
    private (ArticulationBody leftWheel, ArticulationBody rightWheel) wheelBodies;
    private float wheelRadius;
    private float wheelSeparation;
    private float b;
    private float zeta;
    private float vMax;   // clamp di sicurezza sulla velocita' lineare comandata
    private float wMax;   // clamp di sicurezza sulla velocita' angolare comandata

    public ControllerService(int secondsControlFrequency, ControlStrategy controlStrategy, (ArticulationBody leftWheel, ArticulationBody rightWheel) wheelBodies, float wheelRadius, float wheelSeparation, float b, float zeta, float vMax, float wMax)
    {
        this.secondsControlFrequency = secondsControlFrequency;
        this.controlStrategy = controlStrategy;
        this.geometricTrajectoryTable = new List<(float tl, float x, float y, float dx, float dy, float ddx, float ddyt)>();
        this.iter = 0;
        this.controllerActive = false;
        this.lastControl = Time.time;
        this.wheelBodies = wheelBodies;
        this.wheelRadius = wheelRadius;
        this.wheelSeparation = wheelSeparation;
        this.b = b;
        this.zeta = zeta;
        this.vMax = vMax;
        this.wMax = wMax;
    }

    private (double v, double w) getControlInput(int iter, (float x, float y, float theta) currentConfig)
    {
        (double v, double w) returnControlInput = (0.0, 0.0);
        if (controlStrategy == ControlStrategy.NonlinearControl)
        {
            returnControlInput = getNonLinearFeedbackControl(iter, currentConfig);
        }
        else if (controlStrategy == ControlStrategy.IOLinearization)
        {
            //TODO
        }
        else if(controlStrategy == ControlStrategy.ApproximateLinearization)
        {
            //TODO
        }
        return returnControlInput;
    }

    private (double v, double w) getNonLinearFeedbackControl(int iter, (float x, float y, float theta) currentConfig)
    {
        (float tl, float x, float y, float dx, float dy, float ddx, float ddy) rowTabelIter = geometricTrajectoryTable[iter];
        double v_des = getDesiredV(rowTabelIter.dx, rowTabelIter.dy);
        double w_des = getDesiredW(rowTabelIter.dx, rowTabelIter.dy, rowTabelIter.ddx, rowTabelIter.ddy);
        float theta_des = getDesiredTheta(rowTabelIter.dx, rowTabelIter.dy);

        (float x_des, float y_des, float theta_des) desConfig = (rowTabelIter.x, rowTabelIter.y, theta_des);

        float[] err = getErrorVector(desConfig, currentConfig);
        float e1 = err[0];
        float e2 = err[1];
        float e3 = err[2];

        (float k1, float k2, float k3) ks = getKsControllerComponents(v_des, w_des);

        float u1 = getU1FeedbackComponent(e1, ks.k1);
        float u2 = getU2FeedbackComponent(v_des, e2, e3, ks.k2, ks.k3); 

        double v_feed = v_des * Mathf.Cos(e3) - u1;
        double w_feed = w_des - u2;

        v_feed = Math.Clamp(v_feed, -vMax, vMax);
        w_feed = Math.Clamp(w_feed, -wMax, wMax);

        return (v_feed, w_feed);    
    }

    private float getU2FeedbackComponent(double v_des, float e2, float e3, float k2, float k3)
    {
        double sinc = (Math.Abs(e3) < 1e-4) ? 1.0 : Math.Sin(e3) / e3;
        float t1 = (float)(-1.0f * k2 * v_des * sinc * e2);
        float t2 = -1 * k3 * e3;
        return t1 + t2;
    }

    private float getU1FeedbackComponent(float e1, float k1)
    {
        return -1 * k1 * e1;
    }

    private (float k1, float k2, float k3) getKsControllerComponents(double v_des, double w_des)
    {
        float a = (float)Math.Sqrt(b * v_des * v_des + w_des * w_des);
        float k1 = 2 * zeta * a;
        float k3 = 2 * zeta * a;
        float k2 = b;
        return (k1, k2, k3);
    }

    private float[] getErrorVector((float x_des, float y_des, float theta_des) desConfig, (float x, float y, float theta) currentConfig)
    {
        float ex = desConfig.x_des - currentConfig.x; 
        float ey = desConfig.y_des - currentConfig.y;
        float e1 = (float)(Math.Cos(currentConfig.theta) * ex + Math.Sin(currentConfig.theta) * ey);        // longitudinale
        float e2 = (float)(-1 * Math.Sin(currentConfig.theta) * ex + Math.Cos(currentConfig.theta) * ey);   // laterale
        float e3 = wrapToPi(desConfig.theta_des - currentConfig.theta);                                     // ESSENZIALE: riporta in (-pi, pi]
        return new float[3] { e1, e2, e3 };
    }

    // Riporta un angolo (in RADIANTI) nell'intervallo (-pi, pi]. atan2(sin,cos) e' la forma robusta.
    private float wrapToPi(float theta)
    {
        return Mathf.Atan2(Mathf.Sin(theta), Mathf.Cos(theta));
    }

    private float getDesiredTheta(float dx, float dy)
    {
        return Mathf.Atan2(dy, dx);
    }

    private double getDesiredW(float dx, float dy, float ddx, float ddy)
    {
        double den = dx * dx + dy * dy;
        if (den < 1e-9) return 0.0;      // v_des ~ 0 (es. riga finale = goal a riposo): evita 0/0 -> NaN
        return ((ddy * dx - ddx * dy) / den);
    }

    private double getDesiredV(float dx, float dy)
    {
        return Math.Sqrt(dx * dx + dy * dy);
    }

    public (double v, double w) ControlStep((float x, float y, float theta) currentConfig)
    {
        if (!controllerActive) return (0, 0);
        if(iter >= geometricTrajectoryTable.Count)
        {
            Debug.Log("Traiettoria completata");
            controllerActive = false;
            return (0,0);
        }
        (double v, double w) controlInput = getControlInput(iter, currentConfig);
        iter += 1;
        lastControl = Time.time;
        return controlInput;
    }

    public void Arm(List<(float t, float x, float y, float xd, float yd, float xdd, float ydd)> list)
    {
        this.geometricTrajectoryTable = list;
        this.iter = 0;
        this.controllerActive = true;
    }

    public bool isTimeToControl()
    {
        return (Time.time - lastControl) > 1.0f / secondsControlFrequency;
    }

    internal void applyToWheels((double v, double w) feedbackControl)
    {
        // Rete di sicurezza: un NaN/Inf passa indenne da Math.Clamp (i confronti con NaN sono falsi) e
        // finirebbe sul drive ("non-finite values"). Meglio non attuare nulla in quel frame.
        if (double.IsNaN(feedbackControl.v) || double.IsInfinity(feedbackControl.v) ||
            double.IsNaN(feedbackControl.w) || double.IsInfinity(feedbackControl.w)) return;

        (float wl, float wr) wheelsCommand = GetInverseAngularVelocities(wheelSeparation, wheelRadius, (float) feedbackControl.v, (float) feedbackControl.w);
        SetDriveTargetVelocity(wheelBodies.leftWheel, wheelsCommand.wl);
        SetDriveTargetVelocity(wheelBodies.rightWheel, wheelsCommand.wr);
    }

    private void SetDriveTargetVelocity(ArticulationBody wheel, float velocity)
    {
        // 'velocity' e' la velocita' angolare di ruota in rad/s (da GetInverseAngularVelocities), ma in Unity
        // ArticulationDrive.targetVelocity di un Velocity drive e' in GRADI/s -> converto, altrimenti il robot
        // gira ~57x piu' lento di v_des e il riferimento (time-indexed) lo supera fermandolo a meta' percorso.
        ArticulationDrive wheelXDrive = wheel.xDrive;
        wheelXDrive.targetVelocity = velocity * Mathf.Rad2Deg;
        wheel.xDrive = wheelXDrive;
        //Debug.Log($"{wheel.name} driveType={wheel.xDrive.driveType} targetVel={wheel.xDrive.targetVelocity} damping={wheel.xDrive.damping}");
    }
}
