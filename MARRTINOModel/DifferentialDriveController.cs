using System.Collections.Generic;
using UnityEngine.InputSystem;
using UnityEngine;
using static UnicycleModelUtilities;

public class DifferentialDriveController : MonoBehaviour
{
    public float stiffness = 0.0f;
    public float damping = 100.0f;
    public float forceLimit = 1000.0f;

    public float linearSpeed = 20.0f;
    public float angularSpeed = 100.0f;

    private float lastROSVelocityCommandTime = 0.0f;
    public float thresholdBeforeKeybordControl = 0.5f;

    private ArticulationBodyRefs articulationBodyRefs;
    private ArticulationBody leftWheel;
    private ArticulationBody rightWheel;

    void Start()
    {
        // TODO: trovare i GameObject delle ruote nella gerarchia
        // e assegnare leftWheel e rightWheel

        //Debug.Log("Starting...");
        //Debug.Log($"Setted left wheel name is: {leftWheelName}");
        articulationBodyRefs = this.GetComponent<ArticulationBodyRefs>();
        
        (ArticulationBody, ArticulationBody) wheels = articulationBodyRefs.getWheelArticulationBodyReference();
        leftWheel = wheels.Item1;
        rightWheel = wheels.Item2;
        //Debug.Log($"left wheel body var test name: {leftWheel.name}");

        // TODO: configurare l'ArticulationDrive di entrambe le ruote
        // (DriveType, stiffness, damping, forceLimit)
        //Debug.Log("Configurating articulation drive of both wheels...");
        SetWheelDriveConfiguration(leftWheel, stiffness, damping, forceLimit);
        SetWheelDriveConfiguration(rightWheel, stiffness, damping, forceLimit);

    }

    void Update()
    {
        // TODO: leggere input frecce (su/giù = lineare, sinistra/destra = angolare)
        // e ricavare v (velocità lineare) e omega (velocità angolare del robot)

        bool rosIsActive = (Time.time - lastROSVelocityCommandTime) < thresholdBeforeKeybordControl;

        if (!rosIsActive)
        {
            bool SelectionUpInput = Keyboard.current.upArrowKey.isPressed;
            bool SelectionDownInput = Keyboard.current.downArrowKey.isPressed;
            bool SelectionRightInput = Keyboard.current.rightArrowKey.isPressed;
            bool SelectionLeftInput = Keyboard.current.leftArrowKey.isPressed;

            float newV = 0.0f;
            float newW = 0.0f;

            if (SelectionUpInput || SelectionDownInput)
            {
                newV = linearSpeed;
                if (SelectionDownInput)
                {
                    newV = -1 * newV;
                }
            }

            if (SelectionLeftInput || SelectionRightInput)
            {
                newW = angularSpeed;
                if (SelectionLeftInput)
                {
                    newW = -1 * newW;
                }
            }

            //Debug.Log($"New v and w are: v={newV}, w={newW}");

            // TODO: calcolare ωL e ωR dalla cinematica differenziale inversa
            // (usa wheelRadius e wheelSeparation)

            var returnAngularV = GetInverseAngularVelocities(articulationBodyRefs.wheelSeparation, articulationBodyRefs.wheelRadius, newV, newW);
            float wL = returnAngularV.Item1;
            float wR = returnAngularV.Item2;

            //Debug.Log($"New wL and wR are: wL={wL}, wR={wR}");

            // TODO: applicare ωL e ωR alle ruote tramite SetDriveTargetVelocity
            SetDriveTargetVelocity(leftWheel, wL);
            SetDriveTargetVelocity(rightWheel, wR);
        }

    }

    public void SetVelocity(float v, float w)
    {
        lastROSVelocityCommandTime = Time.time;
        //Debug.Log($"SetVelocity called: v={v}, w={w}");
        var returnAngularV = GetInverseAngularVelocities(articulationBodyRefs.wheelSeparation, articulationBodyRefs.wheelRadius, v, w);
        float wL = returnAngularV.Item1;
        float wR = returnAngularV.Item2;
        SetDriveTargetVelocity(leftWheel, wL);
        SetDriveTargetVelocity(rightWheel, wR);
        //Debug.Log($"Setted wL={wL}, wR={wR}");
    }

    private void SetDriveTargetVelocity(ArticulationBody wheel, float velocity)
    {
        // TODO: leggere xDrive, settare targetVelocity, riscrivere xDrive
        ArticulationDrive wheelXDrive = wheel.xDrive;
        wheelXDrive.targetVelocity = velocity;
        wheel.xDrive = wheelXDrive;
        //Debug.Log($"{wheel.name} driveType={wheel.xDrive.driveType} targetVel={wheel.xDrive.targetVelocity} damping={wheel.xDrive.damping}");
    }

    private void SetWheelDriveConfiguration(ArticulationBody wheel, float stiffness, float damping, float forceLimit)
    {
        //Debug.Log($"Setting configuration for body name: {wheel.name}");
        ArticulationDrive wheelDrive = wheel.xDrive;
        wheelDrive.driveType = ArticulationDriveType.Velocity;
        wheelDrive.damping = damping;
        wheelDrive.forceLimit = forceLimit;
        wheelDrive.stiffness = stiffness;
        wheel.xDrive = wheelDrive;
    }

}