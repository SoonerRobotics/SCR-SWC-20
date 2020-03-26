using RosSharp.RosBridgeClient;
using SharpConfig;
using System;
using System.IO;
using System.Linq;
using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;

public class ConfigLoader
{
    public static string simulatorConfig = "../simulator.cfg";
    public static string competitionConfig = "../competition.cfg";

    public static SimulatorConfig simulator { get; private set; } = new SimulatorConfig();
    public static CompetitionConfig competition { get; private set; } = new CompetitionConfig();
    public static bool initalized = false;

    public static void LoadConfig()
    {
        Configuration cfg;

        // Load simulator.cfg
        try
        {
            cfg = Configuration.LoadFromFile(simulatorConfig);
        } 
        catch
        {
            throw new ConfigLoadException("Error reading file:\n" + Path.GetFullPath(simulatorConfig) + "\n\nPlease fix and reload.");
        }

        // Get configs
        try
        {
            simulator.ManualControl = cfg["Simulator"]["ManualControl"].BoolValue;
            simulator.EnableCamera = cfg["Simulator"]["EnableCamera"].BoolValue;
            simulator.ManualTopSpeed = cfg["Simulator"]["ManualTopSpeed"].FloatValue;
            simulator.RosBridgeServerUrl = cfg["Simulator"]["RosBridgeUrl"].StringValue;
        }
        catch (Exception e)
        {
            throw new ConfigLoadException("Error in simulator.cfg:\n" + e.Message + "\n\nPlease fix and reload.");
        }

        // Load competition.cfg
        try
        {
            cfg = Configuration.LoadFromFile(competitionConfig);
        }
        catch
        {
            throw new ConfigLoadException("Error reading file:\n" + Path.GetFullPath(competitionConfig) + "\n\nPlease fix and reload.");
        }

        // Get configs
        try
        {
            competition.Name = cfg["Team"]["Name"].StringValue;
            competition.LaunchParams = cfg["Launch"]["LaunchParams"].StringValue;
            
            competition.Obstacles = EnumParse<CompetitionConfig.ObstacleTypes>(cfg["Bonuses"]["Obstacles"].StringValue, true);
            competition.NoiseLevel = EnumParse<CompetitionConfig.NoiseLevels>(cfg["Bonuses"]["NoiseLevel"].StringValue, true);
        }
        catch (Exception e)
        {
            throw new ConfigLoadException("Error in competition.cfg:\n" + e.Message + "\n\nPlease fix and reload.");
        }
    }

    /*
     * why is enum.parse dumb dumb
     */
    public static T EnumParse<T>(string value, bool ignoreCase)
    {
        T x = (T) Enum.Parse(typeof(T), value, ignoreCase);

        if (Enum.IsDefined(typeof(T), x)) {
            return x;
        }

        throw new Exception("BRO STOP");
    }

    public class SimulatorConfig
    {
        public bool ManualControl = true;
        public float ManualTopSpeed = 1.0f;
        public bool EnableCamera = false;
        public string RosBridgeServerUrl = "ws://localhost:9090";
    }

    public class CompetitionConfig
    {
        public string Name = "Scrubotics";
        public string LaunchParams = "swx_example swx_example.launch";
        public ObstacleTypes Obstacles = ObstacleTypes.normal;
        public NoiseLevels NoiseLevel = NoiseLevels.reduced;

        public enum ObstacleTypes { none, normal, hard };
        public enum NoiseLevels { none, reduced, realistic };
    }

    public class ConfigLoadException : Exception
    {
        public ConfigLoadException(string message) : base(message)
        {
        }
    }
}
