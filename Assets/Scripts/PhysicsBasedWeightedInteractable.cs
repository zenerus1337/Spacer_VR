using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;

[RequireComponent(typeof(Rigidbody))]
public class PhysicsBasedWeightedInteractable : UnityEngine.XR.Interaction.Toolkit.Interactables.XRGrabInteractable
{
    [Header("Weight Settings")]
    [Tooltip("Mass in kg - affects physics directly")]
    [SerializeField] private float weight = 1.0f;

    [Tooltip("How much to scale the physics forces (tweak per project)")]
    [SerializeField] private float strengthMultiplier = 1000f;

    [Tooltip("Maximum distance before teleporting (prevents stretching)")]
    [SerializeField] private float maxDistanceBeforeTeleport = 0.5f;

    private Rigidbody rb;
    private Transform attachPoint;
    private Vector3 previousPosition;
    private float previousTime;

    protected override void Awake()
    {
        base.Awake();
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("PhysicsBasedWeightedInteractable requires a Rigidbody component!", this);
            return;
        }

        rb.mass = weight;

        // Critical settings for physics behavior
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        movementType = MovementType.VelocityTracking;
        smoothPosition = false;
        smoothRotation = false;

        // Disable default XR direct movement
        trackPosition = false;
        trackRotation = false;
    }

    protected override void OnSelectEntered(SelectEnterEventArgs args)
    {
        base.OnSelectEntered(args);

        if (args.interactorObject == null)
        {
            Debug.LogWarning("SelectEntered with null interactor", this);
            return;
        }

        attachPoint = args.interactorObject.GetAttachTransform(this);
        if (attachPoint == null)
        {
            Debug.LogWarning("No attach transform found", this);
            return;
        }

        previousPosition = attachPoint.position;
        previousTime = Time.time;

        // Apply proper physics settings when grabbed
        rb.useGravity = false;
        rb.linearDamping = 0f;
        rb.angularDamping = 0.05f;

        // Haptic feedback with proper null checks
        if (args.interactorObject is UnityEngine.XR.Interaction.Toolkit.Interactors.XRBaseInputInteractor controllerInteractor)
        {
            var controller = controllerInteractor.xrController;
            if (controller != null)
            {
                float amplitude = Mathf.Clamp01(weight / 50f); // Scale 0-1 for 0-50kg
                controller.SendHapticImpulse(amplitude, 0.3f);
            }
        }
    }

    protected override void OnSelectExited(SelectExitEventArgs args)
    {
        base.OnSelectExited(args);

        if (rb != null)
        {
            rb.useGravity = true;
        }
        attachPoint = null;
    }

    private void FixedUpdate()
    {
        if (!isSelected || attachPoint == null || rb == null) return;

        // Calculate how much the controller moved this frame
        float deltaTime = Time.time - previousTime;
        if (deltaTime <= 0) return;

        Vector3 controllerVelocity = (attachPoint.position - previousPosition) / deltaTime;

        // Check if we need to teleport (object got stuck)
        float currentDistance = Vector3.Distance(transform.position, attachPoint.position);
        if (currentDistance > maxDistanceBeforeTeleport)
        {
            rb.MovePosition(attachPoint.position);
            rb.linearVelocity = Vector3.zero;
        }
        else
        {
            // Apply proper physics forces based on weight
            ApplyMovementForces(controllerVelocity);
        }

        previousPosition = attachPoint.position;
        previousTime = Time.time;
    }

    private void ApplyMovementForces(Vector3 targetVelocity)
    {
        if (rb == null) return;

        // Calculate required force using F=ma (adjusted with our multiplier)
        Vector3 velocityDifference = targetVelocity - rb.linearVelocity;
        Vector3 force = velocityDifference * (rb.mass * strengthMultiplier * Time.fixedDeltaTime);

        // Clamp force to prevent extreme values
        float maxForce = rb.mass * 100f;
        if (force.magnitude > maxForce)
        {
            force = force.normalized * maxForce;
        }

        rb.AddForce(force);

        // Add some drag when not moving to prevent oscillation
        if (targetVelocity.magnitude < 0.1f)
        {
            rb.AddForce(-rb.linearVelocity * rb.mass * 5f * Time.fixedDeltaTime);
        }
    }

    public override void ProcessInteractable(XRInteractionUpdateOrder.UpdatePhase updatePhase)
    {
        if (updatePhase == XRInteractionUpdateOrder.UpdatePhase.Dynamic && isSelected)
        {
            // Only handle rotation here
            if (attachPoint != null && rb != null)
            {
                Quaternion rotationDelta = attachPoint.rotation * Quaternion.Inverse(rb.rotation);
                rotationDelta.ToAngleAxis(out float angle, out Vector3 axis);

                if (angle > 180f) angle -= 360f;

                // Apply torque based on weight
                Vector3 torque = axis * (angle * Mathf.Deg2Rad * rb.mass * 50f * Time.deltaTime);
                rb.AddTorque(torque);
            }
        }
    }
}