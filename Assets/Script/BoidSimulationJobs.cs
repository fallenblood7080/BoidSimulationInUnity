using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

using Random = UnityEngine.Random;

public class BoidSimulationJobs : MonoBehaviour
{
    [field: SerializeField] public Vector3 Bounds { get; private set; }

    [SerializeField] private GameObject boidPrefab;
    [SerializeField] private int boidSize;
    [SerializeField] private float flocksDetectionRadius;
    [SerializeField] private float maxSpeed;
    [SerializeField] private float maxForce;
    [SerializeField] private float boundRepMultiplier;

    private BoidsData[] boids;

    private NativeArray<float3> boidsPosArray;
    private NativeArray<float3> boidsVel;
    private NativeArray<float3> boidsForce;
    private NativeArray<float3> boidsSeekTargets;

    private void Start()
    {
        boids = new BoidsData[boidSize];
        boidsPosArray = new(boidSize, Allocator.Persistent);
        boidsForce = new(boidSize, Allocator.Persistent);
        boidsVel = new(boidSize, Allocator.Persistent);
        boidsSeekTargets = new(boidSize, Allocator.Persistent);

        for (int i = 0; i < boidSize; i++)
        {
            Vector3 randomPoint = new Vector3(Random.Range(-Bounds.x / 2, Bounds.x / 2), Random.Range(-Bounds.y / 2, Bounds.y / 2), Random.Range(-Bounds.z / 2, Bounds.z / 2));
            boids[i] = Instantiate(boidPrefab, randomPoint, Random.rotationUniform).GetComponent<BoidsData>();
        }
    }

    private void OnDestroy()
    {
        boidsPosArray.Dispose();
        boidsVel.Dispose();
        boidsForce.Dispose();
        boidsSeekTargets.Dispose();
    }

    private void Update()
    {
        for (int i = 0; i < boidSize; i++)
        {
            boidsPosArray[i] = boids[i].transform.position;
            boids[i].Velocity = boids[i].transform.forward;
            boidsVel[i] = boids[i].Velocity;
        }

        DoBoidsSimulation sumulation = new DoBoidsSimulation
        {
            boidsPosArray = boidsPosArray,
            boidsVel = boidsVel,
            boidsForce = boidsForce,
            detectionRadius = flocksDetectionRadius,
            boidsTargets = boidsSeekTargets,
            bounds = Bounds,
            maxSpeed = maxSpeed,
            maxForce = maxForce,
        };

        JobHandle jobHandle = sumulation.Schedule(boidSize, 100);
        jobHandle.Complete();

        for (int i = 0; i < boidSize; i++)
        {
            Vector3 vel = boidsVel[i];
            vel += (Vector3)boidsForce[i] * Time.deltaTime;
            vel = Vector3.ClampMagnitude(vel, maxSpeed);
            boidsVel[i] = vel;

            Vector3 boundaryRepulsion = BoundRepel(i);
            Vector3 boundRep = boundaryRepulsion.normalized * maxForce;
            vel += boundRep * Time.deltaTime * boundRepMultiplier;
            boids[i].transform.position += vel * Time.deltaTime;
            boids[i].transform.forward = vel.normalized;
        }
    }

    private Vector3 BoundRepel(int i)
    {
        Vector3 boundaryRepulsion = Vector3.zero;

        if (boids[i].transform.position.x < -Bounds.x)
        {
            boundaryRepulsion.x = 1f;
        }
        else if (boids[i].transform.position.x > Bounds.x)
        {
            boundaryRepulsion.x = -1f;
        }

        if (boids[i].transform.position.y < -Bounds.y)
        {
            boundaryRepulsion.y = 1f;
        }
        else if (boids[i].transform.position.y > Bounds.y)
        {
            boundaryRepulsion.y = -1f;
        }

        if (boids[i].transform.position.z < -Bounds.z)
        {
            boundaryRepulsion.z = 1f;
        }
        else if (boids[i].transform.position.z > Bounds.z)
        {
            boundaryRepulsion.z = -1f;
        }

        return boundaryRepulsion;
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.white;
        Gizmos.DrawWireCube(Vector3.zero, Bounds);
    }
}

[BurstCompile]
public struct DoBoidsSimulation : IJobParallelFor
{
    [ReadOnly] public NativeArray<float3> boidsPosArray;
    [ReadOnly] public NativeArray<float3> boidsVel;
    [ReadOnly] public float3 bounds;
    public NativeArray<float3> boidsForce;
    public NativeArray<float3> boidsTargets;

    public float detectionRadius;
    public float maxSpeed;
    public float maxForce;

    public void Execute(int index)
    {
        float3 currentBoid = boidsPosArray[index];

        float3 alignForce = float3.zero;
        float3 separationForce = float3.zero;
        float3 cohesionForce = float3.zero;
        float3 totalForce = float3.zero;

        int flockCount = 0;

        float3 avgVelocity = float3.zero;
        float3 avgPosition = float3.zero;

        Unity.Mathematics.Random random = new Unity.Mathematics.Random(123);

        if (math.distance(currentBoid, boidsTargets[index]) < 0.1)
        {
            boidsTargets[index] = random.NextFloat3(float3.zero, bounds);
        }

        for (int i = 0; i < boidsPosArray.Length; i++)
        {
            if (i != index) // Avoid checking against self
            {
                float dist = math.distance(currentBoid, boidsPosArray[i]);
                if (dist <= detectionRadius)
                {
                    avgVelocity += boidsVel[i];
                    avgPosition += boidsPosArray[i];

                    // Calculate separation force
                    float3 dir = currentBoid - boidsPosArray[i];
                    float distance = math.length(dir);
                    if (distance > 0)
                    {
                        float repulsion = 1 / (distance * distance);
                        separationForce += math.normalize(dir) * repulsion;
                    }

                    flockCount++;
                }
            }
        }

        if (flockCount > 0)
        {
            // Calculate average velocity and position
            avgVelocity /= flockCount;
            avgPosition /= flockCount;

            // Calculate alignment force
            alignForce = math.normalize(avgVelocity) * maxSpeed - boidsVel[index];

            // Calculate cohesion force
            cohesionForce = math.normalize(avgPosition - currentBoid) * maxSpeed - boidsVel[index];
        }

        // Apply forces to the boid
        totalForce = separationForce + alignForce + cohesionForce;
        if (math.lengthsq(totalForce) > math.pow(maxForce, 2))
        {
            totalForce = math.normalize(totalForce) * maxForce;
        }
        boidsForce[index] = totalForce;
    }
}