using RosSharp.RosBridgeClient;
using SharpConfig;
using System.IO;
using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;

[RequireComponent(typeof(RosConnector))]
public class ConfigLoader : MonoBehaviour
{
    public string configFile = "../simulator.cfg";
    public TextMeshProUGUI loadingText;

    private bool waitingForNextScene = false;
    private RosConnector rosConnector;

    public static Config configs { get; private set; } = new Config();

    public class Config
    {
        public bool ManualControl = false;
        public float ManualTopSpeed = 1.0f;
        public bool EnableCamera = false;
    }

    void Start()
    {
        DontDestroyOnLoad(this);

        rosConnector = GetComponent<RosConnector>();

        Configuration cfg;

        // Load simulator.cfg
        try
        {
            cfg = Configuration.LoadFromFile(configFile);
        } 
        catch
        {
            loadingText.text = "Could not find expected config file:\n" + Path.GetFullPath(configFile) + "\n\nPlease fix and restart.";
            return;
        }

        try
        {
            // Get configs
            configs.ManualControl = cfg["Simulator"]["ManualControl"].BoolValue;
            configs.EnableCamera = cfg["Simulator"]["EnableCamera"].BoolValue;
            configs.ManualTopSpeed = cfg["Simulator"]["ManualTopSpeed"].FloatValue;

            rosConnector.RosBridgeServerUrl = cfg["Simulator"]["RosBridgeUrl"].StringValue;
        }
        catch (SettingValueCastException e)
        {
            loadingText.text = "Error in simulator.cfg:\n" + e.Message + "\n\nPlease fix and restart.";
            return;
        }

        // Start connecting
        rosConnector.enabled = true;

        loadingText.text = "Waiting for Ros Bridge at '" + rosConnector.RosBridgeServerUrl + "'...";

        waitingForNextScene = true;
    }

    void Update()
    {
        if (waitingForNextScene && rosConnector.Connected) {
            waitingForNextScene = false;
            SceneManager.LoadScene(1);
        }
    }
}
