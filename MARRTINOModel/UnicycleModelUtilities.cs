using Unity.Mathematics;
using UnityEngine;

public class UnicycleModelUtilities
{

    public static (float, float) GetInverseAngularVelocities(float wheelSeparation, float wheelRadius, float v, float w)
    {
        float d = wheelSeparation;
        float r = wheelRadius;

        float wL = 1 / r * (v + w * (d / 2));
        float wR = 1 / r * (v - w * (d / 2));
        return (wL, wR);
    }

    public static (float, float) GetDirectVelocities(float wheelSeparation, float wheelRadius,float wL, float wR)
    {
        float d = wheelSeparation;
        float r = wheelRadius;

        float v = r / 2 * (wL + wR);
        float w = r / d * (wL - wR);
        return (v, w);
    }

    public static (float, float, float, float) GetQuaternionFromEuler(float yaw, float pitch, float roll)
    {
        float qx = Mathf.Sin(roll / 2) * Mathf.Cos(pitch / 2) * Mathf.Cos(yaw / 2) - Mathf.Cos(roll / 2) * Mathf.Sin(pitch / 2) * Mathf.Sin(yaw / 2);
        float qy = Mathf.Cos(roll / 2) * Mathf.Sin(pitch / 2) * Mathf.Cos(yaw / 2) + Mathf.Sin(roll / 2) * Mathf.Cos(pitch / 2) * Mathf.Sin(yaw / 2);
        float qz = Mathf.Cos(roll / 2) * Mathf.Cos(pitch / 2) * Mathf.Sin(yaw / 2) - Mathf.Sin(roll / 2) * Mathf.Sin(pitch / 2) * Mathf.Cos(yaw / 2);
        float qw = Mathf.Cos(roll / 2) * Mathf.Cos(pitch / 2) * Mathf.Cos(yaw / 2) + Mathf.Sin(roll / 2) * Mathf.Sin(pitch / 2) * Mathf.Sin(yaw / 2);
        return (qx,qy,qz,qw);
    }

    public static (float, float, float) GetEulerFromQuaternion(float qx, float qy, float qz, float qw)
    {
        float t0 = 2.0f * (qw * qx + qy * qz);
        float t1 = 1.0f - 2.0f * (qx * qx + qy * qy);
        float roll = math.atan2(t0, t1);
        float t2 = 2.0f * (qw * qy - qz * qx);
        t2 = t2 > 1.0f ? 1.0f : t2;
        t2 = t2 < -1.0f ? -1.0f : t2;
        float pitch = math.asin(t2);
        float t3 = 2.0f * (qw * qz + qx * qy);
        float t4 = 1.0f - 2.0f * (qy * qy + qz * qz);
        float yaw = math.atan2(t3, t4);
        return (yaw, pitch, roll);
    }


    public static (float, float, float) UnityToRosPosition(float xt, float yt, float zt)
    {
        return (zt, -xt, yt);
    }

    public static (float, float, float, float) UnityToRosRotation(float xt, float yt, float zt, float wt)
    {
        return (zt, -xt, yt, wt);
    }


    public static (float, float, float) RosToUnityPosition(float xt, float yt, float zt)
    {
        return (-yt, zt, xt);
    }

}
