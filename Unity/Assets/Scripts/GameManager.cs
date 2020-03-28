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

    private float startTime = 100000f;

    private List<ScoreAdjust> adjustments = new List<ScoreAdjust>();
    private List<Vector2> waypoints = new List<Vector2>();

    private RosConnector rosConnector;

    public enum GameState
    {
        INIT,
        WAITING_FOR_ROS,
        PLAYING
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
        waypoints.Add(new Vector2(15, 0));
        waypoints.Add(new Vector2(0, 37));

        loadingText.text = "Waiting for Ros Bridge at '" + rosConnector.RosBridgeServerUrl + "'...";

        State = GameState.WAITING_FOR_ROS;
    }

    // Update is called once per frame
    void Update()
    {
        
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
        if (index == waypoints.Count - 1 && ConfigLoader.simulator.CompetitionMode)
        {
            StopSim(null);
            return;
        }
    }

    // Reload the sim completely, including configs and reconnect
    public void ReloadSim()
    {
        State = GameState.INIT;
        SceneManager.LoadScene(0);
        adjustments = new List<ScoreAdjust>();
    }

    // Restart the sim, basically just resetting the level
    public void RestartSim()
    {
        State = GameState.PLAYING;
        SceneManager.LoadScene(1);
        adjustments = new List<ScoreAdjust>();
    }

    public void StopSim(string error)
    {
        float finalTime = Time.time - startTime;

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

    public class ScoreAdjust
    {
        public float adjustment;
        public string reason;
    }
}
