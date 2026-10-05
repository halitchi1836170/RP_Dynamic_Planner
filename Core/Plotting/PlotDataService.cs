using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

// Raccolta delle serie temporali per i grafici di analisi. I dati vengono scritti come CSV
// (IOFileOperationService) e il rendering della figura e' fatto da tools/plot_run.py.
// NB: formattazione InvariantCulture, altrimenti la virgola decimale italiana rompe il CSV.
public class PlotDataService
{
    public const string CONTROL_HEADER = "t,v_cmd,w_cmd,v_nom,w_nom,cbf_active,cbf_feasible,min_barrier,min_margin,static_margin,e1,e2,e3,V";
    public const string POSE_HEADER = "t,gt_x,gt_y,gt_theta,icp_x,icp_y,icp_theta,odo_x,odo_y,odo_theta";
    public const string PLAN_HEADER = "plan_id,t_arm,x,y";
    public const string OBSTACLE_HEADER = "t,id,cx,cy,r";

    private float samplePeriod;
    private float lastPoseSample;
    private int planCounter;

    private List<string> controlRows;
    private List<string> poseRows;
    private List<string> planRows;
    private List<string> obstacleRows;

    public PlotDataService(float samplePeriod)
    {
        this.samplePeriod = samplePeriod;
        this.lastPoseSample = -1f;
        this.planCounter = 0;
        this.controlRows = new List<string>();
        this.poseRows = new List<string>();
        this.planRows = new List<string>();
        this.obstacleRows = new List<string>();
    }

    private static string F(double value)
    {
        return value.ToString("G9", CultureInfo.InvariantCulture);
    }

    public void RecordControl(float t, (double v, double w) command, (double v, double w) nominal, bool cbfActive, bool cbfFeasible, (float minBarrier, float minMargin, float staticMargin, int constraints) diagnostics, (float e1, float e2, float e3, float k2, double v_des, double w_des) terms)
    {
        float V = 0.5f * terms.k2 * (terms.e1 * terms.e1 + terms.e2 * terms.e2) + 0.5f * terms.e3 * terms.e3;

        controlRows.Add(string.Join(",", new string[] {
            F(t), F(command.v), F(command.w), F(nominal.v), F(nominal.w),
            cbfActive ? "1" : "0", cbfFeasible ? "1" : "0",
            NaNable(diagnostics.minBarrier), NaNable(diagnostics.minMargin), NaNable(diagnostics.staticMargin),
            F(terms.e1), F(terms.e2), F(terms.e3), F(V) }));
    }

    private static string NaNable(float value)
    {
        return value == float.MaxValue ? "nan" : F(value);
    }

    public bool isTimeToSamplePose()
    {
        return lastPoseSample < 0f || (Time.time - lastPoseSample) > samplePeriod;
    }

    public void RecordPose(float t, (float x, float y, float theta) groundTruth, (float x, float y, float theta) icp, (float x, float y, float theta) odometry)
    {
        lastPoseSample = Time.time;
        poseRows.Add(string.Join(",", new string[] {
            F(t), F(groundTruth.x), F(groundTruth.y), F(groundTruth.theta),
            F(icp.x), F(icp.y), F(icp.theta),
            F(odometry.x), F(odometry.y), F(odometry.theta) }));
    }

    // Un blocco per ogni Arm del controller: cosi' i replanning restano tratti distinti nel grafico.
    public void RecordPlan(float t, List<(float t, float x, float y, float xd, float yd, float xdd, float ydd)> table)
    {
        foreach ((float t, float x, float y, float xd, float yd, float xdd, float ydd) row in table)
        {
            planRows.Add(string.Join(",", new string[] { planCounter.ToString(), F(t), F(row.x), F(row.y) }));
        }
        planCounter += 1;
    }

    public void RecordObstacles(float t, List<ObstacleTrack> tracks)
    {
        foreach (ObstacleTrack track in tracks)
        {
            obstacleRows.Add(string.Join(",", new string[] { F(t), track.id.ToString(), F(track.cx), F(track.cy), F(track.r) }));
        }
    }

    public List<string> getControlRows()
    {
        return controlRows;
    }

    public List<string> getPoseRows()
    {
        return poseRows;
    }

    public List<string> getPlanRows()
    {
        return planRows;
    }

    public List<string> getObstacleRows()
    {
        return obstacleRows;
    }

    public int getPlanCount()
    {
        return planCounter;
    }
}
