using SharpConfig;
using System;
using System.IO;
using TMPro;

public class ConfigLoader
{
    public static string rootDir = "..";
    public static SimulatorConfig simulator { get; private set; } = new SimulatorConfig();
    public static CompetitionConfig competition { get; private set; } = new CompetitionConfig();
    public static bool initalized = false;

    public static void LoadConfig()
    {
        Configuration cfg;

        string simulatorConfig = rootDir + "/simulator.cfg";
        string competitionConfig = rootDir + "/competition.cfg";

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
            simulator.Seed = cfg["Simulator"]["Seed"].IntValue;

            simulator.CompetitionMode = cfg["Competition"]["CompetitionMode"].BoolValue;
            simulator.MaxTime = cfg["Competition"]["MaxTime"].FloatValue;
        }
        catch (Exception e)
        {
            throw new ConfigLoadException("Error in simulator.cfg:\n" + e.Message + "\n\nPlease fix and reload.");
        }

        // Competition mode overwrites
        if (simulator.CompetitionMode) {
            simulator.ManualControl = false;
        }

        if (simulator.Seed == -1) {
            simulator.Seed = UnityEngine.Random.Range(0, int.MaxValue);
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
        public bool CompetitionMode = false;
        public int Seed = -1;

        public float MaxTime = 300f;
    }

    public class CompetitionConfig
    {
        public string Name = "Scrubotics";
        public string LaunchParams = "swx_example swx_example.launch";
        public ObstacleTypes Obstacles = ObstacleTypes.normal;
        public NoiseLevels NoiseLevel = NoiseLevels.reduced;

        public enum ObstacleTypes { none, normal, hard };
        public enum NoiseLevels { none, reduced, realistic };

        public float getObstacleScore() {
            switch (Obstacles) {
                case ObstacleTypes.none:
                    return 15.0f;
                case ObstacleTypes.normal:
                    return 1.0f;
                case ObstacleTypes.hard:
                    return 0.1f;
            }

            return 100f; // if error, assume they're cheating
        }

        public float getNoiseScore() {
            switch (NoiseLevel) {
                case NoiseLevels.none:
                    return 10.0f;
                case NoiseLevels.reduced:
                    return 1.0f;
                case NoiseLevels.realistic:
                    return 0.6f;
            }

            return 100f; // if error, assume they're cheating
        }
    }

    public class ConfigLoadException : Exception
    {
        public ConfigLoadException(string message) : base(message)
        {
        }
    }
}
