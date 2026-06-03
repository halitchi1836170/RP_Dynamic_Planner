using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class CmdVelSubscriber : MonoBehaviour
{

    private DifferentialDriveController driveController;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        // TODO: trovare DifferentialDriveController nella scena

        driveController = this.GetComponent<DifferentialDriveController>();

        // TODO: registrare il subscriber su /cmd_vel
        // usa ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>(...)
        ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>("/cmd_vel", OnCmdVel);
    }

    void OnCmdVel(TwistMsg msg)
    {
        // TODO: estrarre v da msg.linear.x
        // TODO: estrarre w da msg.angular.z
        // TODO: chiamare driveController.SetVelocity(v, w)
        // Debug.Log($"Received cmd_vel: v={msg.linear.x}, w={msg.angular.z}");
        float receivedV = (float)msg.linear.x;
        float receivedW = (float)msg.angular.z;
        driveController.SetVelocity(receivedV, receivedW);

    }
}
