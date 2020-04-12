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
    public TMP_InputField rootField;
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
        Screen.fullScreen = false; // You win Unity

        if (instance)
            Destroy(instance.gameObject);

        instance = this;

        DontDestroyOnLoad(this);

        Initalize();
    }

    void Initalize()
    {
        if (!PlayerPrefs.HasKey("rootDir")) {
            PlayerPrefs.SetString("rootDir", "..");
            ConfigLoader.rootDir = "..";
            rootField.text = "..";
        } else {
            string rootDir = PlayerPrefs.GetString("rootDir");
            ConfigLoader.rootDir = rootDir;
            rootField.text = rootDir;
        }

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

        maxTime = ConfigLoader.simulator.MaxTime;

        waypoints.Add(new Vector2(-37, 0)); // Start Pos

        Random.InitState(ConfigLoader.simulator.Seed);

        waypoints.Add(new Vector2(Random.Range(-20, -5), Random.Range(-10, 10)));
        waypoints.Add(new Vector2(Random.Range(0, 15), Random.Range(-20, 20)));
        waypoints.Add(new Vector2(Random.Range(20, 35), Random.Range(-15, 15)));

        waypoints.Add(new Vector2(48, 15)); // Goal Pos

        loadingText.text = "Waiting for Ros Bridge at '" + rosConnector.RosBridgeServerUrl + "'...";

        State = GameState.WAITING_FOR_ROS;
    }
    private void FixedUpdate() {

        if (State == GameState.PLAYING && !rosConnector.Connected)
        {
            if (ConfigLoader.simulator.CompetitionMode) {
                StopSim("Connection to RosBridge lost!");
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

    public void UpdateRoot() {
        PlayerPrefs.SetString("rootDir", rootField.text);
    }

    public void StopSim(string error)
    {
        if (State == GameState.QUITTING) {
            return; // Already quitting
        }
        State = GameState.QUITTING;
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

        float scoreMultiplier = 1f;

        foreach (ScoreAdjust score in adjustments)
        {
            scoreMultiplier *= score.adjustment;
        }

        writer.WriteLine($"Team {ConfigLoader.competition.Name}");
        writer.WriteLine($"Date: {System.DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss zz")}");
        writer.WriteLine($"Seed: {ConfigLoader.simulator.Seed}");
        writer.WriteLine();
        writer.WriteLine($"Time: {finalTime:0.000}");
        writer.WriteLine($"Multiplier: x{scoreMultiplier:0.000}");
        writer.WriteLine($"Score: {(finalTime * scoreMultiplier):0.000}");
        writer.WriteLine();
        writer.WriteLine("-- Multipliers --");
        foreach (ScoreAdjust score in adjustments)
        {
            writer.WriteLine($"{score.reason}: x{score.adjustment:0.00}");
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
