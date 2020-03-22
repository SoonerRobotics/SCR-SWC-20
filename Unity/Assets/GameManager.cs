using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class GameManager : MonoBehaviour
{
    public static GameManager instance;
    public float maxTime = 300f;
    public float minDistance = 0.1f;
    public GameObject robot;

    private float startTime = 10000f;

    private List<ScoreAdjust> adjustments = new List<ScoreAdjust>();
    private List<Vector2> waypoints = new List<Vector2>();

    public class ScoreAdjust
    {
        public float adjustment;
        public string reason;
    }

    // Start is called before the first frame update
    void Start()
    {
        instance = this;

        waypoints.Add(new Vector2(1, 0));
    }

    // Update is called once per frame
    void Update()
    {
        Vector2 robotPos = new Vector2();
        robotPos.x = robot.transform.position.x;
        robotPos.y = robot.transform.position.z;

        // Reached final waypoint
        if ((robotPos - waypoints[waypoints.Count - 1]).magnitude < minDistance)
        {
            StopSim();
        }

        // Time limit reached
        if (Time.time - startTime >= maxTime)
        {
            StopSim();
        }
    }

    public void StopSim()
    {
        float finalTime = Time.time - startTime;

        Debug.Log("Job's done!");

        StreamWriter writer = new StreamWriter("results.txt");

        writer.WriteLine(finalTime);

        foreach (ScoreAdjust score in adjustments)
        {
            writer.WriteLine(score.adjustment);
            writer.WriteLine(score.reason);
        }

        writer.Close();

        Application.Quit();
    }

    public void StartSim()
    {
        startTime = Time.time;
    }
}
