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
    public float firstSeen;
    public float lastSeen;
    public int hits;

    public ObstacleTrack(int id, (float cx, float cy, float r) detection, float now)
    {
        this.id = id;
        this.cx = detection.cx;
        this.cy = detection.cy;
        this.r = detection.r;
        this.vx = 0f;
        this.vy = 0f;
        this.firstSeen = now;
        this.lastSeen = now;
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

    private List<ObstacleTrack> tracks;
    private int nextId;

    public ObstacleTrackerService(float gate, float alphaLowpass, float vDeadzone, int minHits, float forgetTime)
    {
        this.gate = gate;
        this.alphaLowpass = alphaLowpass;
        this.vDeadzone = vDeadzone;
        this.minHits = minHits;
        this.forgetTime = forgetTime;
        this.tracks = new List<ObstacleTrack>();
        this.nextId = 0;
    }

    public void Update(List<(float cx, float cy, float r)> detections, float now)
    {
        HashSet<int> assigned = new HashSet<int>();

        // i track piu' vecchi scelgono per primi
        tracks.Sort((a, b) => a.firstSeen.CompareTo(b.firstSeen));

        foreach (ObstacleTrack t in tracks)
        {
            int best = -1;
            float bestSqDist = gate * gate;
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
        }

        for (int i = 0; i < detections.Count; i++)
        {
            if (assigned.Contains(i)) continue;
            tracks.Add(new ObstacleTrack(nextId++, detections[i], now));
        }

        tracks.RemoveAll(t => now - t.lastSeen > forgetTime);
    }

    private void updateTrack(ObstacleTrack t, (float cx, float cy, float r) d, float now)
    {
        float dt = now - t.lastSeen;
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

        t.cx = d.cx;
        t.cy = d.cy;
        t.r = Mathf.Max(d.r, t.r * 0.9f);   // il raggio non deve collassare su un frame parziale
        t.lastSeen = now;
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
