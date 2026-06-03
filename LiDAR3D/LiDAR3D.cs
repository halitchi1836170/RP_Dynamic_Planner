using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;
using System;

public class LiDAR3D : MonoBehaviour
{
    //LiDAR3D hardware parameters
    public int channels = 32;
    public float vAngleMin = -15.0f;
    public float vAngleMax = 15.0f;
    public int pointsPerChannel = 720;
    public float maxRange = 25.0f;
    public float scanFrequenzy = 10.0f; //Hz
    public bool displayLaserScan = false;

    //public properties for publisher
    public List<Vector3> ScannedPoints => scannedPoints;
    public event Action OnScanComplete;

    //mesearued lenght noise parameters
    public float noiseMean = 0.0f;
    public float noiseStd = 0.02f;

    //auxialiry variables
    private float lastTimeScan = 0.0f;
    private float deltaChannel;
    private float deltaVAngles;
    private List<Vector3> scannedPoints = new List<Vector3>();

    //constant variables
    private const float giroCompleto = 360.0f;
    private const float twoPI = math.PI2;
    Unity.Mathematics.Random rnd;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        //inizializzo il delta di avanzamento per canale
        deltaChannel = getDeltaChannels();
        deltaVAngles = getDeltaVAngles();
        rnd = new Unity.Mathematics.Random((uint)System.Environment.TickCount);
    }

    // Update is called once per frame
    void Update()
    {
        if(lastTimeScan == 0.0f || (Time.time - lastTimeScan) > (1.0f / scanFrequenzy))
        {
            Scan();
        }
    }

    private void Scan()
    {
        lastTimeScan = Time.time;
        scannedPoints.Clear();

        for(float currentHorizontalAngle=0.0f; currentHorizontalAngle < giroCompleto; currentHorizontalAngle = currentHorizontalAngle + deltaChannel)
        {
            for(float currentVerticalAngle=vAngleMin; currentVerticalAngle<=vAngleMax;currentVerticalAngle = currentVerticalAngle + deltaVAngles)
            {
                float theta_t = currentHorizontalAngle * Mathf.Deg2Rad;
                float psi_t = currentVerticalAngle * Mathf.Deg2Rad;
                (float, float, float) actualDirection = getDirectionFromAngles(1.0f, theta_t, psi_t);
                Vector3 v3Direction = new Vector3(actualDirection.Item1, actualDirection.Item2, actualDirection.Item3);
                Vector3 vOrigin = transform.position;
                
                Vector3 calculated3DPoint = new Vector3();
                RaycastHit hit;
                float noise = getGaussianNoise(noiseMean, noiseStd);
                if (Physics.Raycast(vOrigin, v3Direction, out hit, maxRange))
                {
                    float noisyDistance = hit.distance + noise;
                    calculated3DPoint = vOrigin + v3Direction * noisyDistance;
                    if (displayLaserScan) Debug.DrawLine(vOrigin, calculated3DPoint, Color.lightGreen, 1.0f / scanFrequenzy);
                    scannedPoints.Add(calculated3DPoint);
                }
            }
        }
        OnScanComplete?.Invoke();
    }

    
    private float getGaussianNoise(float mean, float std) //formula box-muller
    {
        float u1; float u2;
        do
        {
           u1 = rnd.NextFloat(0.0f, 1.0f);
        } while (u1 == 0.0f);
        u2 = rnd.NextFloat(0.0f, 1.0f);

        float mag = std * math.sqrt(-2.0f * math.log(u1));
        float z0 = mag * math.cos(twoPI * u2) + mean;
        return z0;
    }

    private float getDeltaChannels()
    {
        return giroCompleto / pointsPerChannel;
    }

    private float getDeltaVAngles()
    {
        return (vAngleMax - vAngleMin) / (channels-1);
    }

    private (float, float, float) getDirectionFromAngles(float lenght, float theta_t, float psi_t)
    {
        float xd = lenght * math.cos(psi_t) * math.sin(theta_t);
        float yd = lenght * math.sin(psi_t);
        float zd = lenght * math.cos(psi_t) * math.cos(theta_t);
        return (xd,yd,zd);
    }
}
