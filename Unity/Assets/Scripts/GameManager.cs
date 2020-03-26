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

        waypoints.Add(new Vector2(1, 0));

        loadingText.text = "Waiting for Ros Bridge at '" + rosConnector.RosBridgeServerUrl + "'...";

        State = GameState.WAITING_FOR_ROS;
    }

    // Update is called once per frame
    void Update()
    {
        
        if (State == GameState.PLAYING && !rosConnector.Connected)
        {
            RestartSim();
        }

        if (State == GameState.WAITING_FOR_ROS && rosConnector.Connected)
        {
            State = GameState.PLAYING;
            SceneManager.LoadScene(1);
        }

        // Time limit reached
        if (State == GameState.PLAYING && Time.time - startTime >= maxTime)
        {
            StopSim();
        }
    }

    public void ReachWaypoint(int index)
    {
        if (index == waypoints.Count - 1)
        {
            StopSim();
            return;
        }
    }

    public void RestartSim()
    {
        SceneManager.LoadScene(0);
        State = GameState.INIT;
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

    public class ScoreAdjust
    {
        public float adjustment;
        public string reason;
    }
}
