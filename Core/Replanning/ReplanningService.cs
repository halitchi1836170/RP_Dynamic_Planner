using System.Collections.Generic;
using UnityEngine;

public enum ReplanState { Following, Replanning, Blocked };
public enum ReplanTrigger { None, PersistentObstacle, Stall, Deviation, Infeasible, Retry };

// Decide QUANDO ripianificare: la CBF copre il transitorio, qui si gestisce il persistente.
// La pipeline vera e' eseguita dall'Orchestrator, che ha in mano tutti i servizi.
public class ReplanningService
{
    private float persistentAge;
    private float lookAheadDistance;
    private float inflationMargin;
    private float stallWindow;
    private float stallMinProgress;
    private float maxDeviation;
    private float deviationWindow;
    private int maxInfeasibleSteps;
    private float minCBFEngagement;
    private float engagementMarginThreshold;   // sotto questo margine l'ostacolo e' davvero in gioco
    private float engagementDeviationThreshold;// ...e il filtro deve correggere in modo SIGNIFICATIVO
    private float minReplanInterval;           // tempo minimo garantito fra due piani, qualunque sia il trigger
    private float cooldown;
    private float retryPeriod;
    private int maxFailures;

    private ReplanState state;
    private float lastReplanTime;
    private int failureCount;
    private int infeasibleCounter;
    private float cbfEngagementTime;      // secondi "netti" di filtro attivo: sale quando interviene, scende quando no
    private float lastControlStepTime;
    private float deviationSince;
    private float stallWindowStart;
    private (float x, float y) stallWindowPosition;
    private ReplanTrigger lastTrigger;

    public ReplanningService(float persistentAge, float lookAheadDistance, float inflationMargin, float stallWindow, float stallMinProgress, float maxDeviation, float deviationWindow, int maxInfeasibleSteps, float minCBFEngagement, float engagementMarginThreshold, float engagementDeviationThreshold, float minReplanInterval, float cooldown, float retryPeriod, int maxFailures)
    {
        this.persistentAge = persistentAge;
        this.lookAheadDistance = lookAheadDistance;
        this.inflationMargin = inflationMargin;
        this.stallWindow = stallWindow;
        this.stallMinProgress = stallMinProgress;
        this.maxDeviation = maxDeviation;
        this.deviationWindow = deviationWindow;
        this.maxInfeasibleSteps = maxInfeasibleSteps;
        this.minCBFEngagement = minCBFEngagement;
        this.engagementMarginThreshold = engagementMarginThreshold;
        this.engagementDeviationThreshold = engagementDeviationThreshold;
        this.minReplanInterval = minReplanInterval;
        this.cbfEngagementTime = 0f;
        this.lastControlStepTime = -1f;
        this.cooldown = cooldown;
        this.retryPeriod = retryPeriod;
        this.maxFailures = maxFailures;
        this.state = ReplanState.Following;
        this.lastReplanTime = -999f;
        this.failureCount = 0;
        this.infeasibleCounter = 0;
        this.deviationSince = -1f;
        this.stallWindowStart = -1f;
        this.lastTrigger = ReplanTrigger.None;
    }

    public float getInflationMargin()
    {
        return inflationMargin;
    }

    public ReplanState getState()
    {
        return state;
    }

    public ReplanTrigger getLastTrigger()
    {
        return lastTrigger;
    }

    // Accumula il tempo in cui il filtro CBF sta davvero correggendo il nominale. Integrare invece di
    // guardare l'istante corrente rende la misura insensibile allo sfarfallio delle detection.
    // `dynamicMargin` = margine della barriera DINAMICA (NaN/float.MaxValue se nessun ostacolo in raggio).
    // Conta solo l'ingaggio dovuto a un ostacolo VICINO: in corridoio stretto la barriera statica e'
    // quasi sempre attiva e farebbe scattare il replan anche con il tracking perfetto.
    public void NotifyControlStep(bool cbfFeasible, double deviation, float dynamicMargin, float now)
    {
        infeasibleCounter = cbfFeasible ? 0 : infeasibleCounter + 1;

        // Non basta che il filtro "sia attivo" (scatta gia' per 1e-3): deve correggere in modo
        // significativo E per colpa di un ostacolo vicino. Altrimenti il gate "il tracking e' in
        // difficolta'" risulta soddisfatto anche quando il robot sta seguendo benissimo la traiettoria.
        bool engagedByObstacle = deviation > engagementDeviationThreshold && dynamicMargin < engagementMarginThreshold;

        float dt = lastControlStepTime < 0f ? 0f : Mathf.Max(now - lastControlStepTime, 0f);
        lastControlStepTime = now;
        cbfEngagementTime = Mathf.Clamp(cbfEngagementTime + (engagedByObstacle ? dt : -dt), 0f, 2f * minCBFEngagement + 1f);
    }

    public float getCBFEngagementTime()
    {
        return cbfEngagementTime;
    }

    // Da chiamare dopo ogni tentativo di pipeline: azzera le finestre, cosi' i trigger ripartono puliti.
    public void NotifyReplanResult(bool success, float now)
    {
        lastReplanTime = now;
        failureCount = success ? 0 : failureCount + 1;
        state = success ? ReplanState.Following : (failureCount >= maxFailures ? ReplanState.Blocked : ReplanState.Following);
        infeasibleCounter = 0;
        cbfEngagementTime = 0f;
        deviationSince = -1f;
        stallWindowStart = -1f;
    }

    // Blocked NON e' piu' terminale: restare fermi per sempre e' lo scenario peggiore. Dopo maxFailures
    // si continua a riprovare, solo molto piu' di rado (le condizioni cambiano: gli ostacoli scadono).
    public bool isBlocked()
    {
        return state == ReplanState.Blocked;
    }

    public int getFailureCount()
    {
        return failureCount;
    }

    public ReplanTrigger EvaluateTriggers(float now, (float x, float y, float theta) robotPose, List<ObstacleTrack> tracks, List<(float t, float x, float y, float xd, float yd, float xdd, float ydd)> table, int iter, float planDeviation)
    {
        lastTrigger = ReplanTrigger.None;

        // Dopo un fallimento si aspetta di piu' prima di riprovare, ma la condizione va comunque
        // RIVERIFICATA: un retry incondizionato ripianificava anche quando non serviva piu' nulla
        // (tipicamente l'ostacolo era nel frattempo scaduto) producendo un piano identico all'originale.
        // Un piano nuovo ogni pochi secondi destabilizza il tracking piu' dell'ostacolo che evita:
        // meglio restare sul piano corrente, al limite fermarsi, che ripianificare di continuo.
        float waitTime = Mathf.Max(cooldown, minReplanInterval);
        if (failureCount > 0) waitTime = retryPeriod;
        if (failureCount >= maxFailures) waitTime = retryPeriod * 3f;
        if (now - lastReplanTime < waitTime) return ReplanTrigger.None;

        // Un ostacolo sul percorso NON basta: finche' il robot segue bene la traiettoria la CBF se la cava
        // da sola e ripianificare sarebbe prematuro (la stima dell'ostacolo e' anche peggiore da lontano).
        // Si ripianifica solo se il filtro sta gia' lavorando contro il nominale da abbastanza tempo.
        if (cbfEngagementTime >= minCBFEngagement && isReferenceBlocked(tracks, table, iter, now))
            lastTrigger = ReplanTrigger.PersistentObstacle;
        else if (isStalled(now, robotPose)) lastTrigger = ReplanTrigger.Stall;
        else if (isDeviating(now, planDeviation)) lastTrigger = ReplanTrigger.Deviation;
        else if (infeasibleCounter >= maxInfeasibleSteps) lastTrigger = ReplanTrigger.Infeasible;

        return lastTrigger;
    }

    // Un ostacolo conta solo se persistente E se il suo disco interseca il riferimento ANCORA DA PERCORRERE
    // entro lookAheadDistance: quello che mi sono lasciato alle spalle non giustifica un replan.
    private bool isReferenceBlocked(List<ObstacleTrack> tracks, List<(float t, float x, float y, float xd, float yd, float xdd, float ydd)> table, int iter, float now)
    {
        if (tracks == null || table == null) return false;

        float arc = 0f;
        for (int i = Mathf.Max(iter, 1); i < table.Count; i++)
        {
            float dx = table[i].x - table[i - 1].x;
            float dy = table[i].y - table[i - 1].y;
            arc += Mathf.Sqrt(dx * dx + dy * dy);
            if (arc > lookAheadDistance) break;

            foreach (ObstacleTrack track in tracks)
            {
                if (track.age(now) < persistentAge) continue;
                float ex = table[i].x - track.cx;
                float ey = table[i].y - track.cy;
                float blockRadius = track.r + inflationMargin;
                if (ex * ex + ey * ey < blockRadius * blockRadius) return true;
            }
        }
        return false;
    }

    private bool isStalled(float now, (float x, float y, float theta) robotPose)
    {
        if (stallWindowStart < 0f)
        {
            stallWindowStart = now;
            stallWindowPosition = (robotPose.x, robotPose.y);
            return false;
        }

        float dx = robotPose.x - stallWindowPosition.x;
        float dy = robotPose.y - stallWindowPosition.y;
        if (Mathf.Sqrt(dx * dx + dy * dy) > stallMinProgress)
        {
            stallWindowStart = now;                                  // mi sto muovendo: finestra riaperta
            stallWindowPosition = (robotPose.x, robotPose.y);
            return false;
        }
        return now - stallWindowStart > stallWindow;
    }

    private bool isDeviating(float now, float planDeviation)
    {
        if (planDeviation <= maxDeviation)
        {
            deviationSince = -1f;
            return false;
        }
        if (deviationSince < 0f) deviationSince = now;
        return now - deviationSince > deviationWindow;
    }

    // Copia della occupancy PRISTINA con i soli ostacoli persistenti dipinti come occupati.
    // Ripartire sempre dalla mappa originale e' cio' che fa ritrovare il percorso corto quando spariscono.
    public (sbyte[] data, int W, int H, float originX, float originY, float resolution) InflateOccupancyGrid((sbyte[] data, int W, int H, float originX, float originY, float resolution) pristine, List<ObstacleTrack> tracks, float now, float marginOverride)
    {
        sbyte[] inflated = (sbyte[])pristine.data.Clone();

        foreach (ObstacleTrack track in tracks)
        {
            if (track.age(now) < persistentAge) continue;

            float radius = track.r + marginOverride;
            int cellRadius = Mathf.CeilToInt(radius / pristine.resolution);
            int cx = Mathf.FloorToInt((track.cx - pristine.originX) / pristine.resolution);
            int cy = Mathf.FloorToInt((track.cy - pristine.originY) / pristine.resolution);

            for (int dy = -cellRadius; dy <= cellRadius; dy++)
            {
                for (int dx = -cellRadius; dx <= cellRadius; dx++)
                {
                    if (dx * dx + dy * dy > cellRadius * cellRadius) continue;
                    int x = cx + dx;
                    int y = cy + dy;
                    if (x < 0 || x >= pristine.W || y < 0 || y >= pristine.H) continue;
                    inflated[y * pristine.W + x] = 100;
                }
            }
        }
        return (inflated, pristine.W, pristine.H, pristine.originX, pristine.originY, pristine.resolution);
    }
}
