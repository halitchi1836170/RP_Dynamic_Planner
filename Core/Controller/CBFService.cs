using System;
using System.Collections.Generic;
using Accord.Math.Optimization;
using UnityEngine;

// Filtro di sicurezza CBF-QP sul controllo nominale. Variabili del QP: x = (v, w, delta).
// Accord risolve   min 0.5 x^T Q x + d^T x   s.t.   A x >= b   (una riga di A per vincolo).
public class CBFService
{
    private float bLookAhead;
    private float rSafeDynamic;
    private float rSafeStatic;
    private float alphaDynamic;
    private float alphaStatic;
    private float gammaCLF;
    private float slackPenalty;
    private float vDeviationWeight;
    private float wDeviationWeight;
    private float staticActivationDistance;
    private float obstacleActivationRange;
    private float gradientStepCells;
    private float activationTolerance;
    private float wMaxCBF;

    private float lastMinBarrier;     // h = |p_b - p_o|^2 - R^2  [m^2]
    private float lastMinMargin;      // dist - R  [m], stessa cosa ma leggibile
    private float lastStaticMargin;   // d(p_b) - rSafeStatic  [m]
    private int lastConstraintCount;

    private DistanceMap distanceMap;
    private List<(float cx, float cy, float r, float vx, float vy)> obstacles;

    public CBFService(float bLookAhead, float rSafeDynamic, float rSafeStatic, float alphaDynamic, float alphaStatic, float gammaCLF, float slackPenalty, float vDeviationWeight, float wDeviationWeight, float staticActivationDistance, float obstacleActivationRange, float gradientStepCells, float activationTolerance, float wMaxCBF)
    {
        this.bLookAhead = bLookAhead;
        this.rSafeDynamic = rSafeDynamic;
        this.rSafeStatic = rSafeStatic;
        this.alphaDynamic = alphaDynamic;
        this.alphaStatic = alphaStatic;
        this.gammaCLF = gammaCLF;
        this.slackPenalty = slackPenalty;
        this.vDeviationWeight = vDeviationWeight;
        this.wDeviationWeight = wDeviationWeight;
        this.staticActivationDistance = staticActivationDistance;
        this.obstacleActivationRange = obstacleActivationRange;
        this.gradientStepCells = gradientStepCells;
        this.activationTolerance = activationTolerance;
        this.wMaxCBF = wMaxCBF;
        this.obstacles = new List<(float, float, float, float, float)>();
    }

    public void SetDistanceMap(DistanceMap distanceMap)
    {
        this.distanceMap = distanceMap;
    }

    public void SetObstacles(List<(float cx, float cy, float r, float vx, float vy)> obstacles)
    {
        this.obstacles = obstacles ?? new List<(float, float, float, float, float)>();
    }

    public (double v, double w, bool modified, bool feasible) FilterControlInput((double v, double w) uNom, (float x, float y, float theta) currentConfig, (float e1, float e2, float e3, float k2, double v_des, double w_des) terms, float vMax)
    {
        float cosT = Mathf.Cos(currentConfig.theta);
        float sinT = Mathf.Sin(currentConfig.theta);

        // punto avanzato: h dipende anche da theta -> il vincolo agisce su v E su w
        float pbx = currentConfig.x + bLookAhead * cosT;
        float pby = currentConfig.y + bLookAhead * sinT;

        List<double[]> rows = new List<double[]>();
        List<double> values = new List<double>();

        lastMinBarrier = float.MaxValue;
        lastMinMargin = float.MaxValue;
        lastStaticMargin = float.MaxValue;

        addBoxConstraints(rows, values, vMax, wMaxCBF);
        addCLFConstraint(rows, values, terms, uNom);
        addDynamicConstraints(rows, values, pbx, pby, cosT, sinT);
        addStaticConstraint(rows, values, pbx, pby, cosT, sinT);

        // pesi diversi su v e w: con wDeviationWeight < vDeviationWeight il QP preferisce sterzare che frenare
        double[,] Q = new double[3, 3] { { vDeviationWeight, 0.0, 0.0 }, { 0.0, wDeviationWeight, 0.0 }, { 0.0, 0.0, slackPenalty } };
        double[] d = new double[3] { -vDeviationWeight * uNom.v, -wDeviationWeight * uNom.w, 0.0 };

        double[,] A = new double[rows.Count, 3];
        double[] bVector = new double[rows.Count];
        for (int i = 0; i < rows.Count; i++)
        {
            A[i, 0] = rows[i][0];
            A[i, 1] = rows[i][1];
            A[i, 2] = rows[i][2];
            bVector[i] = values[i];
        }

        lastConstraintCount = rows.Count;

        GoldfarbIdnani solver = new GoldfarbIdnani(Q, d, A, bVector, 0);
        if (!solver.Minimize()) return (0.0, 0.0, true, false);

        double vSolution = solver.Solution[0];
        double wSolution = solver.Solution[1];
        if (double.IsNaN(vSolution) || double.IsInfinity(vSolution) || double.IsNaN(wSolution) || double.IsInfinity(wSolution))
            return (0.0, 0.0, true, false);

        double deviation = Math.Sqrt((vSolution - uNom.v) * (vSolution - uNom.v) + (wSolution - uNom.w) * (wSolution - uNom.w));

        return (vSolution, wSolution, deviation > activationTolerance, true);
    }

    // diagnostica: h minimo fra gli ostacoli considerati e numero di righe del QP
    public (float minBarrier, float minMargin, float staticMargin, int constraints) getLastDiagnostics()
    {
        return (lastMinBarrier, lastMinMargin, lastStaticMargin, lastConstraintCount);
    }

    private void addBoxConstraints(List<double[]> rows, List<double> values, float vMax, float wMax)
    {
        rows.Add(new double[3] { 1.0, 0.0, 0.0 }); values.Add(-vMax);
        rows.Add(new double[3] { -1.0, 0.0, 0.0 }); values.Add(-vMax);
        rows.Add(new double[3] { 0.0, 1.0, 0.0 }); values.Add(-wMax);
        rows.Add(new double[3] { 0.0, -1.0, 0.0 }); values.Add(-wMax);
        rows.Add(new double[3] { 0.0, 0.0, 1.0 }); values.Add(0.0);
    }

    // V = k2/2 (e1^2 + e2^2) + e3^2/2,  Vdot = a0 - k2 e1 v - e3 w <= -gamma V + delta
    // delta entra con coefficiente +1: il vincolo CLF cede sempre, non puo rendere infeasible il QP.
    // Il termine noto e il MENO esigente fra il vincolo classico e il decadimento che il nominale gia'
    // realizza: cosi' senza ostacoli la soluzione e' esattamente u_nom (il filtro non tocca il tracking).
    private void addCLFConstraint(List<double[]> rows, List<double> values, (float e1, float e2, float e3, float k2, double v_des, double w_des) terms, (double v, double w) uNom)
    {
        float V = 0.5f * terms.k2 * (terms.e1 * terms.e1 + terms.e2 * terms.e2) + 0.5f * terms.e3 * terms.e3;
        double a0 = terms.k2 * terms.v_des * (terms.e1 * Mathf.Cos(terms.e3) + terms.e2 * Mathf.Sin(terms.e3)) + terms.e3 * terms.w_des;
        double nominalDecay = terms.k2 * terms.e1 * uNom.v + terms.e3 * uNom.w;

        rows.Add(new double[3] { terms.k2 * terms.e1, terms.e3, 1.0 });
        values.Add(Math.Min(gammaCLF * V + a0, nominalDecay));
    }

    // h = |p_b - p_o|^2 - R^2,  hdot = a_v v + a_w w - 2 (p_b - p_o) . v_o >= -alpha h
    private void addDynamicConstraints(List<double[]> rows, List<double> values, float pbx, float pby, float cosT, float sinT)
    {
        foreach ((float cx, float cy, float r, float vx, float vy) o in obstacles)
        {
            float dx = pbx - o.cx;
            float dy = pby - o.cy;
            float R = o.r + rSafeDynamic;
            float margin = Mathf.Sqrt(dx * dx + dy * dy) - R;
            if (margin > obstacleActivationRange) continue;

            float h = dx * dx + dy * dy - R * R;
            if (h < lastMinBarrier) lastMinBarrier = h;
            if (margin < lastMinMargin) lastMinMargin = margin;
            double av = 2.0 * (dx * cosT + dy * sinT);
            double aw = 2.0 * bLookAhead * (-dx * sinT + dy * cosT);

            rows.Add(new double[3] { av, aw, 0.0 });
            values.Add(-alphaDynamic * h + 2.0 * (dx * o.vx + dy * o.vy));
        }
    }

    // h = d(p_b) - rSafeStatic,  hdot = grad(d) . G(theta) u >= -alpha h
    private void addStaticConstraint(List<double[]> rows, List<double> values, float pbx, float pby, float cosT, float sinT)
    {
        if (distanceMap == null) return;

        (float d, float gx, float gy, bool valid) field = sampleDistanceField(pbx, pby);
        if (!field.valid) return;

        float h = field.d - rSafeStatic;
        lastStaticMargin = h;
        if (h > staticActivationDistance) return;   // lontano dai muri il gradiente sulle creste non e affidabile

        double av = field.gx * cosT + field.gy * sinT;
        double aw = bLookAhead * (-field.gx * sinT + field.gy * cosT);

        rows.Add(new double[3] { av, aw, 0.0 });
        values.Add(-alphaStatic * h);
    }

    private (float d, float gx, float gy, bool valid) sampleDistanceField(float x, float y)
    {
        float step = gradientStepCells * distanceMap.getResolution();

        (float d, bool valid) center = distanceAt(x, y);
        (float d, bool valid) xPlus = distanceAt(x + step, y);
        (float d, bool valid) xMinus = distanceAt(x - step, y);
        (float d, bool valid) yPlus = distanceAt(x, y + step);
        (float d, bool valid) yMinus = distanceAt(x, y - step);
        if (!center.valid || !xPlus.valid || !xMinus.valid || !yPlus.valid || !yMinus.valid) return (0f, 0f, 0f, false);

        float gx = (xPlus.d - xMinus.d) / (2f * step);
        float gy = (yPlus.d - yMinus.d) / (2f * step);
        return (center.d, gx, gy, true);
    }

    private (float d, bool valid) distanceAt(float x, float y)
    {
        (int cx, int cy) cell = distanceMap.getCellFromWorldPosition((x, y));
        if (!distanceMap.cellInMap(cell)) return (0f, false);
        return (distanceMap.getDistanceMap()[distanceMap.getIndexFromCell(cell)] * distanceMap.getResolution(), true);
    }
}
