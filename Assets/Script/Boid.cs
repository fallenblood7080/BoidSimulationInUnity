using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Boid : MonoBehaviour
{
    [SerializeField] private float fov;
    [SerializeField] private float maxSpeed;
    [SerializeField] private float maxForce;
    [SerializeField] private float flockDetecctionRadius;

    [SerializeField] private float seekInterval;
    private float nextTimeToSeek;

    private Vector3 bound;
    public Vector3 Velocity { get; private set; }

    private Vector3 currentTargetSteeringVector = Vector3.zero;

    private void Start()
    {
        nextTimeToSeek = seekInterval;
        bound = FindFirstObjectByType<BoidSimulation>().Bounds / 2;
        Vector3 point = Random.onUnitSphere;
        Vector3 dir = (point - transform.position).normalized;
        transform.forward = dir;
    }

    private void Update()
    {
        //transform.position += transform.forward * MaxSpeed * Time.deltaTime;

        Velocity += CalcuateSteering() * Time.deltaTime;
        Velocity = Vector3.ClampMagnitude(Velocity, maxSpeed);

        Vector3 boundaryRepulsion = BoundSteer();
        Velocity += boundaryRepulsion * Time.deltaTime;

        transform.position += Velocity * Time.deltaTime;
        transform.forward = Velocity.normalized;
    }

    private Vector3 BoundSteer()
    {
        Vector3 boundaryRepulsion = Vector3.zero;

        if (transform.position.x < -bound.x)
        {
            boundaryRepulsion.x = 1f;
        }
        else if (transform.position.x > bound.x)
        {
            boundaryRepulsion.x = -1f;
        }

        if (transform.position.y < -bound.y)
        {
            boundaryRepulsion.y = 1f;
        }
        else if (transform.position.y > bound.y)
        {
            boundaryRepulsion.y = -1f;
        }

        if (transform.position.z < -bound.z)
        {
            boundaryRepulsion.z = 1f;
        }
        else if (transform.position.z > bound.z)
        {
            boundaryRepulsion.z = -1f;
        }

        return boundaryRepulsion.normalized * maxForce;
    }

    private Vector3 Seek()
    {
        Vector3 steeringVector = currentTargetSteeringVector;
        if (nextTimeToSeek <= 0)
        {
            nextTimeToSeek = seekInterval;
            Vector3 point = Random.insideUnitSphere * bound.x / 2;
            Vector3 desiredVelocity = (point - transform.position).normalized * maxSpeed;
            steeringVector = desiredVelocity - Velocity;

            steeringVector = Vector3.ClampMagnitude(steeringVector, maxForce);
            currentTargetSteeringVector = steeringVector;
        }
        else
        {
            nextTimeToSeek -= Time.deltaTime;
        }

        return steeringVector;
    }

    private Vector3 Separation(Collider[] flocks)
    {
        Vector3 sepForce = Vector3.zero;
        for (int i = 0; i < flocks.Length; i++)
        {
            if (flocks[i].TryGetComponent<Boid>(out var neighbourBoid))
            {
                if (neighbourBoid != this)
                {
                    Vector3 dir = neighbourBoid.transform.position - transform.position;
                    float distance = dir.magnitude;
                    dir /= distance;
                    float repulsion = 1 / Mathf.Pow(distance, 2);
                    Vector3 repForce = dir * repulsion;
                    sepForce += repForce;
                }
            }
        }
        return sepForce;
    }

    private Vector3 Align(Collider[] flocks)
    {
        Vector3 avgVector = Vector3.zero;
        int neighbourCount = 0;
        for (int i = 0; i < flocks.Length; i++)
        {
            if (flocks[i].TryGetComponent<Boid>(out var neighbourboid))
            {
                if (neighbourboid != this)
                {
                    avgVector += neighbourboid.Velocity;
                    neighbourCount++;
                }
            }
        }
        if (neighbourCount > 0)
        {
            avgVector /= neighbourCount;
            Vector3 desiredVector = avgVector.normalized * maxSpeed;
            Vector3 streeingVector = desiredVector - Velocity;

            streeingVector = Vector3.ClampMagnitude(streeingVector, maxForce);
            return streeingVector;
        }
        else
        {
            return Vector3.zero;
        }
    }

    private Vector3 Cohesion(Collider[] flocks)
    {
        Vector3 avgPos = Vector3.zero;
        int neighbourCount = 0;
        for (int i = 0; i < flocks.Length; i++)
        {
            if (flocks[i].TryGetComponent<Boid>(out var neighbourboid))
            {
                if (neighbourboid != this)
                {
                    avgPos += neighbourboid.transform.position;
                    neighbourCount++;
                }
            }
        }
        if (neighbourCount > 0)
        {
            avgPos /= neighbourCount;
            Vector3 desiredVector = (avgPos - transform.position).normalized * maxSpeed;
            Vector3 streeingVector = desiredVector - Velocity;

            streeingVector = Vector3.ClampMagnitude(streeingVector, maxForce);
            return streeingVector;
        }
        else
        {
            return Vector3.zero;
        }
    }

    private Vector3 CalcuateSteering()
    {
        Collider[] flocks = Physics.OverlapSphere(transform.position, flockDetecctionRadius);
        if (flocks == null || flocks.Length == 0) return Vector3.zero;

        Vector3 totalForce;
        Vector3 seekForce = Seek();
        Vector3 separationForce = Separation(flocks);
        Vector3 alignForce = Align(flocks);
        Vector3 cohensionFOrce = Cohesion(flocks);

        totalForce = seekForce + separationForce + alignForce + cohensionFOrce;
        totalForce = Vector3.ClampMagnitude(totalForce, maxForce);

        return totalForce;
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, flockDetecctionRadius);
    }
}