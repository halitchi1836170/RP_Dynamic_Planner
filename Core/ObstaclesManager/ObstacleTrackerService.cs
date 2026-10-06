using System.Collections.Generic;
using UnityEngine;

public class ObstacleTrack
{
    public int id;
    public float cx;
    public float cy;
    public float r;
    public float vx;
    public float vy;
    public float rMax;          // raggio massimo mai OSSERVATO: un track confermato non scende sotto
    public float firstSeen;
    public float lastSeen;
    public float lastCoast;
    public int hits;

    public ObstacleTrack(int id, (float cx, float cy, float r, int n) detection, float now)
    {
        this.id = id;
        this.cx = detection.cx;
        this.cy = detection.cy;
        this.r = detection.r;
        this.rMax = detection.r;
        this.vx = 0f;
        this.vy = 0f;
        this.firstSeen = now;
        this.lastSeen = now;
        this.lastCoast = now;
        this.hits = 1;
    }

    public float age(float now)
    {
        return now - firstSeen;
    }
}

public class ObstacleTrackerService
{
    private float gate;
    private float alphaLowpass;
    private float vDeadzone;
    private int minHits;
    private float forgetTime;
    private float blindZoneRadius;        // entro questo raggio il sensore non vede: non e' una prova di assenza
    private float blindZoneForgetFactor;
    private int minPointsForUpdate;      // sotto questo numero il cluster e' troppo povero per spostare il track
    private float radiusDecayPerUpdate;  // decadimento lento, applicato SOLO su osservazioni buone
    private float blindZoneGrowthRate;   // in zona cieca il raggio CRESCE: non vedere non e' sapere
    private float maxBlindGrowth;        // ...ma di quanto al massimo: l'incertezza e' limitata, non infinita

    private List<ObstacleTrack> tracks;
    private int nextId;

    public ObstacleTrackerService(float gate, float alphaLowpass, float vDeadzone, int minHits, float forgetTime, float blindZoneRadius, float blindZoneForgetFactor, int minPointsForUpdate, float radiusDecayPerUpdate, float blindZoneGrowthRate, float maxBlindGrowth)
    {
        this.gate = gate;
        this.alphaLowpass = alphaLowpass;
        this.vDeadzone = vDeadzone;
        this.minHits = minHits;
        this.forgetTime = forgetTime;
        this.blindZoneRadius = blindZoneRadius;
        this.blindZoneForgetFactor = blindZoneForgetFactor;
        this.minPointsForUpdate = minPointsForUpdate;
        this.radiusDecayPerUpdate = radiusDecayPerUpdate;
        this.blindZoneGrowthRate = blindZoneGrowthRate;
        this.maxBlindGrowth = maxBlindGrowth;
        this.tracks = new List<ObstacleTrack>();
        this.nextId = 0;
    }

    public void Update(List<(float cx, float cy, float r, int n)> detections, float now, (float x, float y, float theta) robotPose)
    {
        HashSet<int> assigned = new HashSet<int>();
        HashSet<int> assignedTracks = new HashSet<int>();

        // i track piu' vecchi scelgono per primi
        tracks.Sort((a, b) => a.firstSeen.CompareTo(b.firstSeen));

        foreach (ObstacleTrack t in tracks)
        {
            int best = -1;
            float trackGateEffective = Mathf.Min(gate, t.r + 0.15f);   // 0.5 m e' troppo per un oggetto da 0.25
            float bestSqDist = trackGateEffective * trackGateEffective;
            for (int i = 0; i < detections.Count; i++)
            {
                if (assigned.Contains(i)) continue;
                float dx = detections[i].cx - t.cx;
                float dy = detections[i].cy - t.cy;
                float sq = dx * dx + dy * dy;
                if (sq < bestSqDist)
                {
                    bestSqDist = sq;
                    best = i;
                }
            }
            if (best == -1) continue;

            updateTrack(t, detections[best], now);
            assigned.Add(best);
            assignedTracks.Add(t.id);
        }

        for (int i = 0; i < detections.Count; i++)
        {
            if (assigned.Contains(i)) continue;
            tracks.Add(new ObstacleTrack(nextId++, detections[i], now));
        }

        // Un ostacolo che sparisce perche' sono troppo vicino per vederlo va tenuto piu' a lungo:
        // e' esattamente il caso in cui dimenticarlo porta a urtarlo.
        // Un track non aggiornato NON e' un track valido congelato: l'incertezza cresce. Gonfiare il
        // raggio durante il coasting e' l'unico modo onesto di rappresentarlo, altrimenti il cerchio
        // resta dov'era mentre la stima di posa deriva e finisce per NON coprire piu' l'ostacolo.
        foreach (ObstacleTrack t in tracks)
        {
            if (assignedTracks.Contains(t.id)) continue;

            t.vx = 0f;      // senza osservazioni la velocita' e' una finzione, e una finzione che
            t.vy = 0f;      // nel vincolo CBF ALLENTA la barriera se "sembra" che si allontani

            // Il gonfiaggio vale SOLO se il sensore davvero non puo' vedere (zona cieca) ed e' LIMITATO:
            // senza queste due condizioni ogni detection intermittente faceva crescere il raggio a
            // cricchetto, perche' un'osservazione buona lo riassorbiva solo del 2%.
            if (!isInBlindZone(t, robotPose)) continue;
            t.r = Mathf.Min(t.r + blindZoneGrowthRate * Mathf.Max(now - t.lastCoast, 0f), t.rMax + maxBlindGrowth);
            t.lastCoast = now;
        }

        tracks.RemoveAll(t => now - t.lastSeen > forgetTime * (isInBlindZone(t, robotPose) ? blindZoneForgetFactor : 1f));
    }

    private bool isInBlindZone(ObstacleTrack track, (float x, float y, float theta) robotPose)
    {
        float dx = track.cx - robotPose.x;
        float dy = track.cy - robotPose.y;
        return Mathf.Sqrt(dx * dx + dy * dy) < track.r + blindZoneRadius;
    }

    private void updateTrack(ObstacleTrack t, (float cx, float cy, float r, int n) d, float now)
    {
        float dt = now - t.lastSeen;
        bool poorObservation = d.n < minPointsForUpdate;
        if (dt > 1e-3f)
        {
            float vxRaw = (d.cx - t.cx) / dt;
            float vyRaw = (d.cy - t.cy) / dt;
            t.vx = (1f - alphaLowpass) * t.vx + alphaLowpass * vxRaw;
            t.vy = (1f - alphaLowpass) * t.vy + alphaLowpass * vyRaw;
            if (t.vx * t.vx + t.vy * t.vy < vDeadzone * vDeadzone)
            {
                t.vx = 0f;
                t.vy = 0f;
            }
        }

        // Un cluster degenere (pochi punti) non deve ne' spostare il centro ne' ridurre il raggio:
        // con 2 punti il centroide finisce dove capita e il raggio e' una semilarghezza inventata.
        if (!poorObservation)
        {
            t.cx = d.cx;
            t.cy = d.cy;
            // high-water mark delle misure VERE, che dimentica lentamente una sovrastima isolata
            t.rMax = Mathf.Max(d.r, t.rMax * radiusDecayPerUpdate);
            t.r = t.rMax;                   // tornare alla misura scarta subito il gonfiaggio del coasting
        }
        else
        {
            t.r = Mathf.Max(t.r, t.rMax);   // osservazione povera: si conserva il meglio gia' misurato
        }

        t.lastSeen = now;
        t.lastCoast = now;
        t.hits += 1;
    }

    public List<ObstacleTrack> GetConfirmedTracks()
    {
        List<ObstacleTrack> confirmed = new List<ObstacleTrack>();
        foreach (ObstacleTrack t in tracks)
        {
            if (t.hits >= minHits) confirmed.Add(t);
        }
        return confirmed;
    }

    public List<ObstacleTrack> GetAllTracks()
    {
        return tracks;
    }

    public List<(float cx, float cy, float r, float vx, float vy)> GetConfirmedCirclesWithVelocity()
    {
        List<(float cx, float cy, float r, float vx, float vy)> circles = new List<(float, float, float, float, float)>();
        foreach (ObstacleTrack t in GetConfirmedTracks())
        {
            circles.Add((t.cx, t.cy, t.r, t.vx, t.vy));
        }
        return circles;
    }

    public List<(float cx, float cy, float r)> GetConfirmedCircles()
    {
        List<(float cx, float cy, float r)> circles = new List<(float, float, float)>();
        foreach (ObstacleTrack t in GetConfirmedTracks())
        {
            circles.Add((t.cx, t.cy, t.r));
        }
        return circles;
    }
}
