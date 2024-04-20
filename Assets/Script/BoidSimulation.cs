using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BoidSimulation : MonoBehaviour
{
    [field: SerializeField] public Vector3 Bounds { get; private set; }

    [SerializeField] private GameObject boidPrefab;
    [SerializeField] private int boidSize;

    private void Start()
    {
        for (int i = 0; i < boidSize; i++)
        {
            Vector3 randomPoint = new Vector3(Random.Range(-Bounds.x / 2, Bounds.x / 2), Random.Range(-Bounds.y / 2, Bounds.y / 2), Random.Range(-Bounds.z / 2, Bounds.z / 2));
            Instantiate(boidPrefab, randomPoint, Quaternion.identity);
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.white;
        Gizmos.DrawWireCube(Vector3.zero, Bounds);
    }
}