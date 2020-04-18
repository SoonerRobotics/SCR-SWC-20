using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KillOnServer : MonoBehaviour
{
    void Awake() {
        if (SystemInfo.graphicsDeviceName == null) {
            Destroy(this.gameObject);
        }
    }
}
