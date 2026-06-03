using System.Collections.Generic;
using UnityEngine;

public class ArticulationBodyRefs : MonoBehaviour
{
    public float wheelRadius = 0.07F;
    public float wheelSeparation = 0.42F;

    public string leftWheelName = "marrtino_left_wheel_link";
    public string rightWheelName = "marrtino_right_wheel_link";
    public string marrtinoLaserLinkName = "marrtino_laser_link";

    private ArticulationBody[] articulationBodies;
    private Dictionary<string, ArticulationBody> dictArticulationBody = new Dictionary<string, ArticulationBody>();

    private ArticulationBody leftWheel;
    private ArticulationBody rightWheel;

    private ArticulationBody marrtinoLaserLink;
    private Transform marrtionLaserLinkTransform;

    private void Awake()
    {
        articulationBodies = this.GetComponentsInChildren<ArticulationBody>();
        foreach (ArticulationBody body in articulationBodies)
        {
            //Debug.Log($"Adding to dictionary body with name: {body.name}...");
            dictArticulationBody[body.name] = body;
        }

        leftWheel = dictArticulationBody[leftWheelName];
        rightWheel = dictArticulationBody[rightWheelName];
        marrtinoLaserLink = dictArticulationBody[marrtinoLaserLinkName];
        marrtionLaserLinkTransform = marrtinoLaserLink.transform;
        //Debug.Log($"left wheel body var test name: {leftWheel.name}");
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    public (ArticulationBody, ArticulationBody) getWheelArticulationBodyReference()
    {
        return (leftWheel, rightWheel);
    }

    public ArticulationBody getMarrtinoLaserLinkArticulationBodyReference()
    {
        return marrtinoLaserLink;
    }

    public Transform getMarrtinoLaserLinkTransformReference()
    {
        return marrtionLaserLinkTransform;
    }

    public (float, float, float) getTransformCurrentConfiguration()
    {
        float current_xt = transform.position.z; // z perché Unity usa XZ come piano orizzontale
        float current_yt = transform.position.x;
        float current_thetat = transform.eulerAngles.y * Mathf.Deg2Rad;
        return (current_xt, current_yt, current_thetat);
    }

}
