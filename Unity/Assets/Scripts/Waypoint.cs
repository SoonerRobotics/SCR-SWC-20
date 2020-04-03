using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Waypoint : MonoBehaviour
{
    public int index;
    public float triggerDist = 1f;
    private bool reached = false;

    private void FixedUpdate() {
        Transform robotTf = GameManager.instance.robotTf;

        if (index == 0) {
            Destroy(this);
            return;
        }

        if (!reached && (robotTf.position - transform.position).sqrMagnitude < triggerDist * triggerDist) {
            reached = true;
            GameManager.instance.ReachWaypoint(index);
            Destroy(this);
        }
    }
}
