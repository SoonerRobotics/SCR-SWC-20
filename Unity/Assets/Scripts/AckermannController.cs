using System.Collections;
using System.Collections.Generic;
using UnityEngine;


/*
 * Uses equations from
 * http://datagenetics.com/blog/december12016/index.html
 * and
 * https://www.xarg.org/book/kinematics/ackerman-steering/
 */
public class AckermannController : MonoBehaviour
{

    public GameObject leftWheel;
    public GameObject rightWheel;

    [Range(0.0F, 1.0F)]
    public float drag = 0.85f;

    public float L = 0.7f;
    public float T = 0.6f;

    [Range(0.0F, 10.0F)]
    public float ManualTopSpeed = 1.0f;

    public float Angle { get; private set; }
    public float Power { get; private set; }

    [Range(-30.0F, 30.0F)]
    public float CntrlAngle = 0;
    public float CntrlPower = 0;

    public Vector3 linear_vel { get; private set; } = new Vector3();
    public Vector3 angular_vel { get; private set; } = new Vector3();
    public Vector3 accel { get; private set; } = new Vector3();

    private float Radius;

    // Start is called before the first frame update
    void Start()
    {
        ManualTopSpeed = ConfigLoader.simulator.ManualTopSpeed;
    }

    // Update is called once per frame
    void Update()
    {
        if (!ConfigLoader.simulator.ManualControl)
            return;

        CntrlPower = Input.GetAxis("Speed") * ManualTopSpeed;
        CntrlAngle = Input.GetAxis("Angle") * 30;
    }

    private void FixedUpdate()
    {
        Angle += (1.0f - drag) * (CntrlAngle - Angle);
        Power += (1.0f - drag) * (CntrlPower - Power);

        Angle = Mathf.Clamp(Angle, -30, 30);
        Power = Mathf.Clamp(Power, -8, 8);

        float radAngle = Angle * Mathf.Deg2Rad;

        float leftAngle = 0;
        float rightAngle = 0;

        if (Mathf.Abs(radAngle) > 0.1f)
        {
            Radius = L / Mathf.Tan(radAngle);

            leftAngle = Mathf.Atan2(L, Radius - T / 2);
            rightAngle = Mathf.Atan2(L, Radius + T / 2);

        }

        leftWheel.transform.localRotation = Quaternion.Euler(new Vector3(90, leftAngle * Mathf.Rad2Deg, 0));
        rightWheel.transform.localRotation = Quaternion.Euler(new Vector3(90, rightAngle * Mathf.Rad2Deg, 0));

        float heading = -this.transform.rotation.eulerAngles.y * Mathf.Deg2Rad;

        Vector3 new_linear_vel = new Vector3(Power * Mathf.Cos(heading), 0, Power * Mathf.Sin(heading));
        accel = (new_linear_vel - linear_vel) / Time.fixedDeltaTime;

        linear_vel = new_linear_vel;
        transform.Translate(linear_vel * Time.fixedDeltaTime, Space.World);

        angular_vel = new Vector3(0, Power / L * Mathf.Tan(radAngle), 0);
        transform.Rotate(angular_vel * Mathf.Rad2Deg * Time.fixedDeltaTime, Space.World);
    }
}
