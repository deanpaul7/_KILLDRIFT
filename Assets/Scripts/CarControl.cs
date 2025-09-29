using UnityEngine;
using UnityEngine.InputSystem;

public class CarControl : MonoBehaviour
{
    [Header("Car Properties")]
    public float motorTorque = 2000f;
    public float brakeTorque = 2000f;
    public float handbrakeTorque = 4000f;
    public float maxSpeed = 20f;
    public float steeringRange = 30f;
    public float steeringRangeAtMaxSpeed = 10f;
    public float centreOfGravityOffset = -1f;
    [Header("Drift Properties")]
    public float driftFrictionFactor = 0.6f; // Increased for better grip
    public float driftSteerMultiplier = 1.5f; // Boost steering during drift

    private WheelControl[] wheels;
    private Rigidbody rigidBody;
    private PlayerInput playerInput;
    private InputAction movementAction;
    private InputAction handbrakeAction;

    // Start is called before the first frame update
    void Start()
    {
        rigidBody = GetComponent<Rigidbody>();

        // Adjust center of mass to improve stability and prevent rolling
        Vector3 centerOfMass = rigidBody.centerOfMass;
        centerOfMass.y += centreOfGravityOffset;
        rigidBody.centerOfMass = centerOfMass;

        // Get all wheel components attached to the car
        wheels = GetComponentsInChildren<WheelControl>();

        // Initialize Input System
        playerInput = GetComponent<PlayerInput>();
        if (playerInput != null)
        {
            movementAction = playerInput.actions.FindAction("CarControl/Movement");
            handbrakeAction = playerInput.actions.FindAction("CarControl/Handbrake");
            if (movementAction == null || handbrakeAction == null)
            {
                Debug.LogError("Movement or Handbrake action not found in Input Actions. Please check the Action Map.");
            }
        }
        else
        {
            Debug.LogError("PlayerInput component not found on this GameObject. Please add a PlayerInput component and assign the Input Actions asset.");
        }
    }

    // FixedUpdate is called at a fixed time interval 
    void FixedUpdate()
    {
        // Get player input for acceleration, steering, and handbrake
        float vInput = movementAction != null ? movementAction.ReadValue<Vector2>().y : Input.GetAxis("Vertical"); // Vertical input
        float hInput = movementAction != null ? movementAction.ReadValue<Vector2>().x : Input.GetAxis("Horizontal"); // Horizontal input
        bool isHandbrake = handbrakeAction != null && handbrakeAction.IsPressed();

        // Calculate current speed along the car's forward axis
        float forwardSpeed = Vector3.Dot(transform.forward, rigidBody.linearVelocity);
        float speedFactor = Mathf.InverseLerp(0, maxSpeed, Mathf.Abs(forwardSpeed));

        // Reduce motor torque and steering at high speeds for better handling
        float currentMotorTorque = Mathf.Lerp(motorTorque, 0, speedFactor);
        float currentSteerRange = Mathf.Lerp(steeringRange, steeringRangeAtMaxSpeed, speedFactor);

        // Determine if the player is accelerating or trying to reverse
        bool isAccelerating = Mathf.Sign(vInput) == Mathf.Sign(forwardSpeed) || forwardSpeed == 0;

        foreach (var wheel in wheels)
        {
            if (wheel.WheelCollider != null) // Safety check
            {
                // Apply steering to wheels that support steering
                if (wheel.steerable)
                {
                    float steerAngle = hInput * currentSteerRange;
                    wheel.WheelCollider.steerAngle = isHandbrake ? steerAngle * driftSteerMultiplier : steerAngle;
                    Debug.Log($"Steer Angle for {wheel.name}: {wheel.WheelCollider.steerAngle}"); // Debug steering
                }

                WheelFrictionCurve sidewaysFriction = wheel.WheelCollider.sidewaysFriction;
                if (isHandbrake && wheel.motorized) // Apply handbrake only to motorized (rear) wheels
                {
                    // Reduce sideways friction for drifting on rear wheels
                    sidewaysFriction.stiffness = driftFrictionFactor;
                    wheel.WheelCollider.sidewaysFriction = sidewaysFriction;

                    // Minimal brake torque to allow sliding and steering
                    wheel.WheelCollider.motorTorque = 0f;
                    wheel.WheelCollider.brakeTorque = handbrakeTorque * 0.05f;
                }
                else
                {
                    // Restore normal friction for all wheels
                    sidewaysFriction.stiffness = 1f;
                    wheel.WheelCollider.sidewaysFriction = sidewaysFriction;

                    if (isAccelerating)
                    {
                        // Apply torque to motorized wheels
                        if (wheel.motorized)
                        {
                            wheel.WheelCollider.motorTorque = vInput * currentMotorTorque;
                        }
                        // Release brakes when accelerating
                        wheel.WheelCollider.brakeTorque = 0f;
                    }
                    else
                    {
                        // Apply brakes when reversing direction
                        wheel.WheelCollider.motorTorque = 0f;
                        wheel.WheelCollider.brakeTorque = Mathf.Abs(vInput) * brakeTorque;
                    }
                }
            }
        }
    }
}