using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarController : MonoBehaviour
{
    [Header("References")]
    [SerializeField] WheelCollider[] wheels_c = new WheelCollider[4];
    [SerializeField] Transform[] wheels_m = new Transform[4];
    [SerializeField] Transform centerOfMass;
    private Rigidbody rb;


    [Header("Car Settings")]
    [SerializeField] float wheelBase = 2.55f; // in metters
    [SerializeField] float rearTrack = 1.5f; // in metters
    

    [Header("Drive Settings")]
    [SerializeField] float motorForce = 1000f;
    [SerializeField] float brakeForce = 1000f;
    [SerializeField] float maxForwardsSpeed = 180;
    [SerializeField] float maxBackwardsSpeed = 60;
    private float drive_input;
    private float movingDirection;


    [Header("Steer Settings")]
    [SerializeField] float turnRadius = 7.5f;
    private float currentTurnRadius;
    private float turn_input;
   

    [Header("Drift")]
    [SerializeField] float driftFriction = 2f; // the lower it is the stronger the dirft is
    private WheelFrictionCurve forwardFriction, sidewaysFriction;
    private bool drift_input;
    private float driftFactor;


    [Header("Gravity and In Air Movement")]
    [SerializeField] float downForce = 2f;


    private float KPH;
    

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = centerOfMass.localPosition;
    }

   
    void FixedUpdate()
    {
        Inputs();
        MoveCar();
        GravityAndInAirMovement();
        Drift();

        AckermanSteering();
        for (int i = 0; i < wheels_c.Length; i++)
        {
            AnimateWheels(wheels_c[i], wheels_m[i]);
        }

    }

    

    private void MoveCar()
    {
        
        movingDirection = transform.InverseTransformDirection(rb.velocity).z; // inversetransformdirection

        KPH = rb.velocity.magnitude * 3.6f;

        if ((movingDirection < -0.2f && drive_input > 0.1f) || (movingDirection > 0.2f && drive_input < -0.1f)) // If the player drives the other way than the car is currently in, do a fast brake.
            FastBrake();
        else if (drive_input != 0)
            Drive();
        else
            NormalBrake();
    }

    private void Drive()
    {
        
        if (drive_input > 0 && KPH<=maxForwardsSpeed) // Driving forwards
        {
            foreach (WheelCollider wc in wheels_c)
            {
                wc.motorTorque = (motorForce / 4) * 5 * drive_input;
                wc.brakeTorque = 0;
            }
        }

        else if (drive_input < 0 && KPH <= maxBackwardsSpeed) // Driving backwards
        {
            foreach (WheelCollider wc in wheels_c)
            {
                wc.motorTorque = (motorForce / 4) * 3 * drive_input;
                wc.brakeTorque = 0;
            }
        }
        else
        {
            float currentMaxSpeed = drive_input >= 0 ? maxForwardsSpeed : maxBackwardsSpeed;
            Vector3 direction = rb.velocity.normalized;

            rb.velocity = direction * (currentMaxSpeed / 3.6f);
        }
        
        
    }

    private void FastBrake()
    {
        float fastBrakeMultiplier = 2f;
        foreach (WheelCollider wc in wheels_c)
        {
            wc.motorTorque = 0;
            wc.brakeTorque = brakeForce * 5 * fastBrakeMultiplier;
        }
        rb.AddForce(rb.mass/2 * drive_input * transform.forward, ForceMode.Impulse);

    }

    private void NormalBrake()
    {
        foreach (WheelCollider wc in wheels_c)
        {
            wc.brakeTorque = (brakeForce / 4);
            wc.motorTorque = 0;
        }
    }


    void AckermanSteering()
    {
        currentTurnRadius = IncreaseTurnRadius();

        //float disableSnapping;

        //disableSnapping = Mathf.Lerp(0,turn_input, Time.deltaTime / (KPH+1));
        //Debug.Log(disableSnapping);


        if (turn_input > 0) // maybe to add disable snapping tool
        {
            wheels_c[0].steerAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (currentTurnRadius + (rearTrack / 2))) * turn_input;
            wheels_c[1].steerAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (currentTurnRadius - (rearTrack / 2))) * turn_input;
        }
        else if (turn_input < 0)
        {
            wheels_c[0].steerAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (currentTurnRadius - (rearTrack / 2))) * turn_input;
            wheels_c[1].steerAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (currentTurnRadius + (rearTrack / 2))) * turn_input;
        }
        else
        {
            wheels_c[0].steerAngle = 0;
            wheels_c[1].steerAngle = 0;
        }

    }

    void Drift()
    {
        //the time it takes to go from normal drive to drift 
        float driftSmothFactor = .7f * Time.deltaTime;

        if (drift_input && turn_input != 0)
        {
            sidewaysFriction = wheels_c[0].sidewaysFriction;
            forwardFriction = wheels_c[0].forwardFriction;

            float velocity = 0;
            sidewaysFriction.extremumValue = sidewaysFriction.asymptoteValue = forwardFriction.extremumValue = forwardFriction.asymptoteValue =
            Mathf.SmoothDamp(forwardFriction.asymptoteValue, driftFactor * driftFriction, ref velocity, driftSmothFactor);

            for (int i = 0; i < 4; i++)
            {
                wheels_c[i].sidewaysFriction = sidewaysFriction;
                wheels_c[i].forwardFriction = forwardFriction;
            }

            sidewaysFriction.extremumValue = sidewaysFriction.asymptoteValue = forwardFriction.extremumValue = forwardFriction.asymptoteValue = 1.1f;
            //extra grip for the front wheels
            for (int i = 0; i < 2; i++)
            {
                wheels_c[i].sidewaysFriction = sidewaysFriction;
                wheels_c[i].forwardFriction = forwardFriction;
            }

        }

        else
        {

            forwardFriction = wheels_c[0].forwardFriction;
            sidewaysFriction = wheels_c[0].sidewaysFriction;

            forwardFriction.extremumValue = forwardFriction.asymptoteValue = sidewaysFriction.extremumValue = sidewaysFriction.asymptoteValue =
            ((KPH * driftFriction) / 300) + 1;

            for (int i = 0; i < 4; i++)
            {
                wheels_c[i].forwardFriction = forwardFriction;
                wheels_c[i].sidewaysFriction = sidewaysFriction;

            }
        }

        //checks the amount of slip to control the drift
        for (int i = 2; i < 4; i++)
        {
            WheelHit wheelHit;

            wheels_c[i].GetGroundHit(out wheelHit);

            if (wheelHit.sidewaysSlip < 0) driftFactor = (1 + -turn_input) * Mathf.Abs(wheelHit.sidewaysSlip);

            if (wheelHit.sidewaysSlip > 0) driftFactor = (1 + turn_input) * Mathf.Abs(wheelHit.sidewaysSlip);
        }

        if (drift_input)//barke for the rear wheels 
        {
            for (int i = 2; i < 4; i++)
            {
                wheels_c[i].brakeTorque = 50 * brakeForce;
            }
        }
        else
        {
            for (int i = 2; i < 4; i++)
            {
                wheels_c[i].brakeTorque = 0;
            }
        }


    }

    private float IncreaseTurnRadius()
    {
        return turnRadius + (KPH / 20);
    }

    void AnimateWheels(WheelCollider wheelCollider, Transform wheelTransform)//Makes the wheels transform and position to follow the wheel collider;
    {
        Vector3 position;
        Quaternion rotation;

        wheelCollider.GetWorldPose(out position, out rotation);
        wheelTransform.rotation = rotation;
        wheelTransform.position = position;

    }

    private void GravityAndInAirMovement()
    {
        rb.AddForce(Vector3.down * downForce * rb.velocity.magnitude); // this makes the car more stable and doesnt let the car to flip on its back;
    }


    private void Inputs()
    {
        drive_input = Input.GetAxis("Vertical");
        turn_input = Input.GetAxis("Horizontal");
        drift_input = Input.GetKey(KeyCode.Space) ? true : false;
    }
}
