using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HealthController : MonoBehaviour
{
    private AckermannController car;
    private float lastHitTime = 0f;
    private float iFrameTime = 0.2f; // Can't get hit until this time (seconds) has passed

    void Start()
    {
        car = GetComponent<AckermannController>();
    }

    private void OnCollisionEnter(Collision other) {
        if (other.gameObject.CompareTag("Obstacle") && Time.realtimeSinceStartup - lastHitTime >= iFrameTime) {
            lastHitTime = Time.realtimeSinceStartup;
            GameManager.instance.TakeDamage(Mathf.Max(5f, car.linear_vel.magnitude * 7f));
        }
    }
}
