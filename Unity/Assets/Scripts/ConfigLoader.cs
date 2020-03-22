using RosSharp.RosBridgeClient;
using SharpConfig;
using System;
using System.IO;
using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;

public class ConfigLoader
{
    public static string configFile = "../simulator.cfg";
    public static Config configs { get; private set; } = new Config();
    public static bool initalized = false;

    public static void LoadConfig()
    {
        Configuration cfg;

        // Load simulator.cfg
        try
        {
            cfg = Configuration.LoadFromFile(configFile);
        } 
        catch
        {
            throw new ConfigLoadException("Error reading file:\n" + Path.GetFullPath(configFile) + "\n\nPlease fix and reload.");
        }

        // Get configs
        try
        {
            configs.ManualControl = cfg["Simulator"]["ManualControl"].BoolValue;
            configs.EnableCamera = cfg["Simulator"]["EnableCamera"].BoolValue;
            configs.ManualTopSpeed = cfg["Simulator"]["ManualTopSpeed"].FloatValue;
            configs.RosBridgeServerUrl = cfg["Simulator"]["RosBridgeUrl"].StringValue;
        }
        catch (SettingValueCastException e)
        {
            throw new ConfigLoadException("Error in simulator.cfg:\n" + e.Message + "\n\nPlease fix and reload.");
        }
    }
    public class Config
    {
        public bool ManualControl = false;
        public float ManualTopSpeed = 1.0f;
        public bool EnableCamera = false;
        public string RosBridgeServerUrl = "ws://localhost:9090";
    }

    public class ConfigLoadException : Exception
    {
        public ConfigLoadException(string message) : base(message)
        {
        }
    }
}
