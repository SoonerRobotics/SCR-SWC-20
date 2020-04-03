using RosSharp.RosBridgeClient;
using System.Collections.Generic;
using System.IO;
using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;

[RequireComponent(typeof(RosConnector))]
public class GameManager : MonoBehaviour
{
    public static GameManager instance;

    public TextMeshProUGUI loadingText;
    public float maxTime = 300f;

    public Transform robotTf;

    private float startTime = 100000f;

    private List<ScoreAdjust> adjustments = new List<ScoreAdjust>();
    private List<Vector2> waypoints = new List<Vector2>();

    private RosConnector rosConnector;

    public enum GameState
    {
        INIT,
        WAITING_FOR_ROS,
        PLAYING,
        QUITTING
    }

    public GameState State { get; private set; } = GameState.INIT;

    // Start is called before the first frame update
    void Start()
    {
        if (instance)
            Destroy(instance.gameObject);

        instance = this;

        DontDestroyOnLoad(this);

        Initalize();
    }

    void Initalize()
    {
        try
        {
            ConfigLoader.LoadConfig();
        }
        catch (ConfigLoader.ConfigLoadException e)
        {
            loadingText.text = e.Message;
            return;
        }

        // Get rosConnector
        rosConnector = GetComponent<RosConnector>();
        rosConnector.RosBridgeServerUrl = ConfigLoader.simulator.RosBridgeServerUrl;
        rosConnector.enabled = true;

        waypoints.Add(new Vector2(0, -37));

        if (ConfigLoader.simulator.Seed != -1)
            Random.InitState(ConfigLoader.simulator.Seed);

        for (int i=0; i<3; i++) {
            waypoints.Add(new Vector2(Random.Range(-30, 30), Random.Range(-20, 20)));
        }

        waypoints.Add(new Vector2(0, 37));

        loadingText.text = "Waiting for Ros Bridge at '" + rosConnector.RosBridgeServerUrl + "'...";

        State = GameState.WAITING_FOR_ROS;
    }
    private void FixedUpdate() {

        if (State == GameState.PLAYING && !rosConnector.Connected)
        {
            if (ConfigLoader.simulator.CompetitionMode) {
                StopSim("Connection to RosBridge lost!");
                State = GameState.QUITTING;
            } else {
                ReloadSim();
            }
        }

        if (State == GameState.WAITING_FOR_ROS && rosConnector.Connected)
        {
            State = GameState.PLAYING;
            SceneManager.LoadScene(1);
            AddBonusAdjustments();
        }

        // Time limit reached
        if (State == GameState.PLAYING && ConfigLoader.simulator.CompetitionMode && Time.time - startTime >= maxTime)
        {
            StopSim(null);
            State = GameState.QUITTING;
        }
    }

    public Vector2[] GetWaypoints() {
        return waypoints.ToArray();
    }

    public void ReachWaypoint(int index)
    {
        Debug.Log("reached " + index);

        if (index == waypoints.Count - 1 && ConfigLoader.simulator.CompetitionMode)
        {
            StopSim(null);
            return;
        }

        if (index > 0) {
            ScoreAdjust waypointScore = new ScoreAdjust();
            waypointScore.adjustment = 0.8f;
            waypointScore.reason = "Waypoint " + index + " reached";
            adjustments.Add(waypointScore);
        }
    }

    // Reload the sim completely, including configs and reconnect
    public void ReloadSim()
    {
        State = GameState.INIT;
        SceneManager.LoadScene(0);
        adjustments = new List<ScoreAdjust>();
        AddBonusAdjustments();
    }

    // Restart the sim, basically just resetting the level
    public void RestartSim()
    {
        State = GameState.PLAYING;
        SceneManager.LoadScene(1);
        adjustments = new List<ScoreAdjust>();
        AddBonusAdjustments();
    }

    public void AddBonusAdjustments() {
        ScoreAdjust obstacles = new ScoreAdjust();
        obstacles.adjustment = ConfigLoader.competition.getObstacleScore();
        obstacles.reason = "Obstacles Bonus";
        adjustments.Add(obstacles);

        ScoreAdjust noise = new ScoreAdjust();
        noise.adjustment = ConfigLoader.competition.getNoiseScore();
        noise.reason = "Noise Bonus";
        adjustments.Add(noise);        
    }

    public void StopSim(string error)
    {
        float finalTime = Time.time - startTime;

        if (finalTime > maxTime) {
            finalTime = maxTime;
        }

        Debug.Log("Job's done!");

        StreamWriter writer = new StreamWriter("results.txt");

        if (error != null) {
            writer.WriteLine("Error: " + error);
            writer.Close();
            Application.Quit();
        }

        writer.WriteLine(finalTime);

        foreach (ScoreAdjust score in adjustments)
        {
            writer.WriteLine(score.adjustment + ": " + score.reason);
        }

        writer.Close();

        Application.Quit();
    }

    public void StartSim()
    {
        startTime = Time.time;
    }

    public class ScoreAdjust
    {
        public float adjustment;
        public string reason;
    }
}
