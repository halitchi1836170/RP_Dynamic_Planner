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
    private float referenceTime;   // tempo percorso sulla traiettoria, indipendente dal rate del loop
    private bool controllerActive;
    private bool trajectoryCompleted;   // vero SOLO a riferimento esaurito: Disarm non e' una conclusione
    private float lastControl;
    private (ArticulationBody leftWheel, ArticulationBody rightWheel) wheelBodies;
    private float wheelRadius;
    private float wheelSeparation;
    private float b;
    private float zeta;
    private float vMax;   // clamp di sicurezza sulla velocita' lineare comandata
    private float wMax;   // clamp di sicurezza sulla velocita' angolare comandata
    private bool cbfEnabled;
    private CBFService cbfService;
    private float referenceMaxLag;
    private float maxLinearAccelCommand;
    private float maxAngularAccelCommand;
    private (float wl, float wr) lastWheelCommand;
    private double lastCommandedV;
    private double lastCommandedW;
    private (double v, double w) lastNominalControl;
    private bool lastCBFFeasible;
    private (float e1, float e2, float e3, float k2, double v_des, double w_des) lastTrackingTerms;
    private bool cbfActive;
    private float cbfActiveSince;

    public ControllerService(int secondsControlFrequency, ControlStrategy controlStrategy, (ArticulationBody leftWheel, ArticulationBody rightWheel) wheelBodies, float wheelRadius, float wheelSeparation, float b, float zeta, float vMax, float wMax, float referenceMaxLag, float maxLinearAccelCommand, float maxAngularAccelCommand, bool cbfEnabled, CBFService cbfService)
    {
        this.secondsControlFrequency = secondsControlFrequency;
        this.controlStrategy = controlStrategy;
        this.geometricTrajectoryTable = new List<(float tl, float x, float y, float dx, float dy, float ddx, float ddyt)>();
        this.iter = 0;
        this.referenceTime = 0f;
        this.controllerActive = false;
        this.lastControl = Time.time;
        this.wheelBodies = wheelBodies;
        this.wheelRadius = wheelRadius;
        this.wheelSeparation = wheelSeparation;
        this.b = b;
        this.zeta = zeta;
        this.vMax = vMax;
        this.wMax = wMax;
        this.referenceMaxLag = referenceMaxLag;
        this.maxLinearAccelCommand = maxLinearAccelCommand;
        this.maxAngularAccelCommand = maxAngularAccelCommand;
        this.lastCommandedV = 0.0;
        this.lastCommandedW = 0.0;
        this.cbfEnabled = cbfEnabled;
        this.cbfService = cbfService;
        this.cbfActive = false;
        this.cbfActiveSince = -1f;
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

        cbfActive = false;
        lastCBFFeasible = true;
        lastNominalControl = returnControlInput;
        if (iter < geometricTrajectoryTable.Count) lastTrackingTerms = getTrackingTerms(iter, currentConfig);

        if (cbfEnabled && cbfService != null && iter < geometricTrajectoryTable.Count)
        {
            (float e1, float e2, float e3, float k2, double v_des, double w_des) terms = lastTrackingTerms;
            (double v, double w, bool modified, bool feasible) filtered = cbfService.FilterControlInput(returnControlInput, currentConfig, terms, vMax);
            lastCBFFeasible = filtered.feasible;
            if (!filtered.feasible) Debug.LogWarning("CBF QP infeasible: robot fermo in questo step");
            cbfActive = filtered.modified;
            returnControlInput = (filtered.v, filtered.w);
        }
        return returnControlInput;
    }

    // Termini del riferimento corrente usati dal vincolo CLF (V e la sua derivata lungo la dinamica).
    private (float e1, float e2, float e3, float k2, double v_des, double w_des) getTrackingTerms(int iter, (float x, float y, float theta) currentConfig)
    {
        (float tl, float x, float y, float dx, float dy, float ddx, float ddy) rowTabelIter = geometricTrajectoryTable[iter];
        double v_des = getDesiredV(rowTabelIter.dx, rowTabelIter.dy);
        double w_des = getDesiredW(rowTabelIter.dx, rowTabelIter.dy, rowTabelIter.ddx, rowTabelIter.ddy);
        float theta_des = getDesiredTheta(rowTabelIter.dx, rowTabelIter.dy);

        float[] err = getErrorVector((rowTabelIter.x, rowTabelIter.y, theta_des), currentConfig);
        (float k1, float k2, float k3) ks = getKsControllerComponents(v_des, w_des);
        return (err[0], err[1], err[2], ks.k2, v_des, w_des);
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
            trajectoryCompleted = true;
            controllerActive = false;
            return (0,0);
        }
        // Tempo REALE fra due passi di controllo: il loop gira al rate di Update (spesso 5-10 Hz con
        // LiDAR e ICP attivi), non ai secondsControlFrequency nominali.
        float dt = Mathf.Clamp(Time.time - lastControl, 0f, 0.5f);

        (double v, double w) controlInput = applyRateLimit(getControlInput(iter, currentConfig), dt);
        if (getReferenceLag(iter, currentConfig) < referenceMaxLag) advanceReference(dt);   // il riferimento aspetta solo se il robot e' rimasto indietro
        updateCBFActiveTime();
        lastControl = Time.time;
        return controlInput;
    }

    // Il riferimento scorre col TEMPO, non di una riga per chiamata: la tabella e' campionata a
    // 1/secondsControlFrequency, quindi avanzare di una riga a ogni passo con il loop piu' lento
    // faceva scorrere la traiettoria a una frazione della sua velocita' e il robot la superava.
    private void advanceReference(float dt)
    {
        referenceTime += dt;

        while (iter + 1 < geometricTrajectoryTable.Count && geometricTrajectoryTable[iter + 1].tl <= referenceTime) iter += 1;

        if (iter == geometricTrajectoryTable.Count - 1 && referenceTime > geometricTrajectoryTable[iter].tl)
            iter = geometricTrajectoryTable.Count;
    }

    public void Arm(List<(float t, float x, float y, float xd, float yd, float xdd, float ydd)> list)
    {
        this.geometricTrajectoryTable = list;
        this.iter = 0;
        this.referenceTime = 0f;
        this.trajectoryCompleted = false;
        this.controllerActive = true;
        this.lastControl = Time.time;
        this.lastCommandedV = 0.0;
        this.lastCommandedW = 0.0;
    }

    // Il QP puo cambiare il punto di lavoro di colpo quando cambia l'insieme dei vincoli attivi: senza
    // questo limite il comando e discontinuo e la rotazione brusca fa divergere l'ICP di localizzazione.
    private (double v, double w) applyRateLimit((double v, double w) controlInput, float elapsed)
    {
        // Math.Clamp propaga NaN: senza questa guardia lastCommanded resterebbe NaN per sempre.
        if (double.IsNaN(controlInput.v) || double.IsInfinity(controlInput.v)) controlInput.v = 0.0;
        if (double.IsNaN(controlInput.w) || double.IsInfinity(controlInput.w)) controlInput.w = 0.0;

        double dt = elapsed > 1e-4f ? elapsed : 1.0 / secondsControlFrequency;
        double maxDeltaV = maxLinearAccelCommand * dt;
        double maxDeltaW = maxAngularAccelCommand * dt;

        lastCommandedV += Math.Clamp(controlInput.v - lastCommandedV, -maxDeltaV, maxDeltaV);
        lastCommandedW += Math.Clamp(controlInput.w - lastCommandedW, -maxDeltaW, maxDeltaW);
        return (lastCommandedV, lastCommandedW);
    }

    private float getReferenceLag(int iter, (float x, float y, float theta) currentConfig)
    {
        (float tl, float x, float y, float dx, float dy, float ddx, float ddy) rowTabelIter = geometricTrajectoryTable[iter];
        float ex = rowTabelIter.x - currentConfig.x;
        float ey = rowTabelIter.y - currentConfig.y;
        return Mathf.Sqrt(ex * ex + ey * ey);
    }

    private void updateCBFActiveTime()
    {
        if (!cbfActive) cbfActiveSince = -1f;
        else if (cbfActiveSince < 0f) cbfActiveSince = Time.time;
    }

    // Comando di ruota in rad/s: confrontato con jointVelocity dice se il drive insegue davvero il target.
    public (float wl, float wr) getLastWheelCommand()
    {
        return lastWheelCommand;
    }

    public (double v, double w) getLastNominalControl()
    {
        return lastNominalControl;
    }

    public bool isLastCBFFeasible()
    {
        return lastCBFFeasible;
    }

    public (float e1, float e2, float e3, float k2, double v_des, double w_des) getLastTrackingTerms()
    {
        return lastTrackingTerms;
    }

    public bool isTrajectoryCompleted()
    {
        return trajectoryCompleted;
    }

    public bool isControllerActive()
    {
        return controllerActive;
    }

    // Quanto il filtro sta correggendo, non solo SE sta correggendo: `cbfActive` scatta gia' a 1e-3.
    public double getLastCBFDeviation()
    {
        return cbfService != null ? cbfService.getLastDeviation() : 0.0;
    }

    public bool isCBFActive()
    {
        return cbfActive;
    }

    public float getCBFActiveTime()
    {
        return cbfActiveSince < 0f ? 0f : Time.time - cbfActiveSince;
    }

    // Riprende il piano precedente dal punto in cui era: Disarm non tocca tabella e iter.
    public void ReArm()
    {
        if (geometricTrajectoryTable.Count > 0 && iter < geometricTrajectoryTable.Count) this.controllerActive = true;
    }

    public void Disarm()
    {
        this.controllerActive = false;
        this.lastCommandedV = 0.0;
        this.lastCommandedW = 0.0;
        this.cbfActive = false;
        this.cbfActiveSince = -1f;
    }

    // Avanzamento lungo il riferimento: serve al rilevamento di stallo del replanner.
    public int getReferenceIndex()
    {
        return iter;
    }

    public (float x, float y) getReferencePoint()
    {
        if (iter >= geometricTrajectoryTable.Count) return (0f, 0f);
        return (geometricTrajectoryTable[iter].x, geometricTrajectoryTable[iter].y);
    }

    public List<(float t, float x, float y, float xd, float yd, float xdd, float ydd)> getTrajectoryTable()
    {
        return geometricTrajectoryTable;
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
        lastWheelCommand = wheelsCommand;
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
