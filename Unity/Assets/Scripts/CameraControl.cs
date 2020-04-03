using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraControl : MonoBehaviour
{
    public float speed = 0.1f;

    void Update()
    {
        this.transform.Translate(Input.mouseScrollDelta.y * this.transform.forward * speed, Space.World);
    }
}
