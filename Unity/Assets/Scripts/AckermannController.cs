using System.Collections;
using System.Collections.Generic;
using System.Threading;
using RosSharp.RosBridgeClient;
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
    public GameObject frontAxleTf;
    public GameObject robotCamera;

    private float turnDrag = 0.8f;
    private float maxAccel = 20.0f;

    private float L = 0.3f;
    private float T = 0.26f;

    [Range(0.0F, 10.0F)]
    public float ManualTopSpeed = 1.0f;

    public float Angle { get; private set; }
    public float Power { get; private set; }

    public float CntrlAngle { get; private set; } = 0;
    public float CntrlPower { get; private set; } = 0;

    public Vector3 linear_vel { get; private set; } = new Vector3();
    public Vector3 angular_vel { get; private set; } = new Vector3();
    public Vector3 accel { get; private set; } = new Vector3();

    private float Radius;

    private Rigidbody rb;

    // Start is called before the first frame update
    void Start()
    {
        ManualTopSpeed = ConfigLoader.simulator.ManualTopSpeed;

        GameManager.instance.robotTf = frontAxleTf.transform;

        if (ConfigLoader.simulator.EnableCamera) {
            robotCamera.SetActive(true);
            GetComponent<ImagePublisher>().enabled = true;
        }

        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = new Vector3(0,0,0);
        rb.inertiaTensor = new Vector3(100,100,100);
        rb.inertiaTensorRotation = Quaternion.identity;
        rb.maxAngularVelocity = 100f;

        maxAccel *= Time.fixedDeltaTime;
    }

    // Update is called once per frame
    void Update()
    {
        if (ConfigLoader.simulator.ManualControl) {
            CntrlPower = Input.GetAxis("Speed") * ManualTopSpeed;
            CntrlAngle = Input.GetAxis("Angle") * 20;
        }
    }

    public void SetControl(float power, float angle) {
        CntrlPower = power;
        CntrlAngle = angle;
    }

    private void FixedUpdate()
    {

        Angle += (1.0f - turnDrag) * (CntrlAngle - Angle);
        Power += Mathf.Clamp(CntrlPower - Power, -maxAccel, maxAccel);

        Angle = Mathf.Clamp(Angle, -30, 30);
        Power = Mathf.Clamp(Power, -8, 8);

        float radAngle = Angle * Mathf.Deg2Rad;

        float heading = -this.transform.rotation.eulerAngles.y * Mathf.Deg2Rad;

        float leftAngle = 0;
        float rightAngle = 0;

        leftAngle = Mathf.Atan2(2 * L * Mathf.Sin(radAngle), 2 * L * Mathf.Cos(radAngle) - T * Mathf.Sin(radAngle));
        rightAngle = Mathf.Atan2(2 * L * Mathf.Sin(radAngle), 2 * L * Mathf.Cos(radAngle) + T * Mathf.Sin(radAngle));

        leftWheel.transform.localRotation = Quaternion.Euler(new Vector3(90, leftAngle * Mathf.Rad2Deg, 0));
        rightWheel.transform.localRotation = Quaternion.Euler(new Vector3(90, rightAngle * Mathf.Rad2Deg, 0));

        Vector3 new_linear_vel = new Vector3(Power * Mathf.Cos(heading), 0, Power * Mathf.Sin(heading));
        accel = (new_linear_vel - linear_vel) / Time.fixedDeltaTime;

        linear_vel = new_linear_vel;
        //transform.Translate(linear_vel * Time.fixedDeltaTime, Space.World);
        rb.velocity = linear_vel;
        //rb.MovePosition(rb.position + linear_vel * Time.fixedDeltaTime);

        angular_vel = new Vector3(0, Power / L * Mathf.Tan(radAngle), 0);
        // transform.Rotate(angular_vel * Mathf.Rad2Deg * Time.fixedDeltaTime, Space.World);
        rb.angularVelocity = angular_vel;
    }
}
