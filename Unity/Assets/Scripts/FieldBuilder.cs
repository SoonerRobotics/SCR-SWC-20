using System.Collections;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

public class FieldBuilder : MonoBehaviour
{
    public GameObject[] obstaclePrefabs;
    public GameObject waypointPrefab;
    public int height = 57;
    public int width = 117;

    public float spread = 2.5f;

    [Range(0.0f, 1.0f)]
    public float density = 0.35f;

    public int seed = 0;

    public int freeArea = 2;

    List<(int, int)> getNeighbors(bool[,] maze, int w, int h, bool pass) {
        List<(int, int)> list = new List<(int, int)>();
        if (w < width-2 && pass == maze[w+2, h])
            list.Add((w+2, h));
        if (w > 1 && pass == maze[w-2, h])
            list.Add((w-2, h));
        if (h < height-3 && pass == maze[w, h+2])
            list.Add((w, h+2));
        if (h > 2 && pass == maze[w, h-2])
            list.Add((w, h-2));
        return list;
    }

    (int, int) waypointToTuple(Vector2 waypoint) {
        int x = (int)(width / 2 + waypoint.x / spread);
        int y = (int)(height / 2 + waypoint.y / spread);
        return (x, y);
    }

    // Start is called before the first frame update
    void Start()
    {

        GameObject waypointHolder = new GameObject("Waypoints");;

        bool[,] maze;
        HashSet<(int, int)> frontier;

        if (ConfigLoader.simulator.Seed != -1)
            Random.InitState(ConfigLoader.simulator.Seed);

        Vector2[] waypoints = GameManager.instance.GetWaypoints();

        for (int i=0; i < waypoints.Count(); i++) {
            GameObject go = Instantiate(waypointPrefab, new Vector3(waypoints[i].x, 0, waypoints[i].y), Quaternion.identity, waypointHolder.transform);

            Waypoint wpt = go.GetComponent<Waypoint>();
            wpt.index = i;
        }

        if (ConfigLoader.competition.Obstacles == ConfigLoader.CompetitionConfig.ObstacleTypes.none) {
            return;
        }

        if (ConfigLoader.competition.Obstacles == ConfigLoader.CompetitionConfig.ObstacleTypes.normal) {
            density /= 3.0f;
        }

        width = (int)(width / spread);
        height = (int)(height / spread);

        maze = new bool[width, height];
        frontier = new HashSet<(int, int)>();
        
        (int, int) goal = waypointToTuple(waypoints[waypoints.Count() - 1]);
        maze[goal.Item1, goal.Item2] = true;
        frontier.Add(goal);

        while (frontier.Count > 0) {
            (int, int) cell = frontier.ToArray()[Random.Range(0, frontier.Count)];

            var nbrs = getNeighbors(maze, cell.Item1, cell.Item2, false);

            if (nbrs.Count > 0) {
                (int, int) toExplore = nbrs[Random.Range(0, nbrs.Count)];

                maze[toExplore.Item1, toExplore.Item2] = true;
                maze[(int)((toExplore.Item1 + cell.Item1)/ 2), (int)((toExplore.Item2 + cell.Item2)/ 2)] = true;
                
                // Instantiate(obstaclePrefabs[Random.Range(0, obstaclePrefabs.Length)], new Vector3(toExplore.Item1 - width/2, 0, toExplore.Item2 - height/2), Quaternion.identity, obstacleHolder.transform);
                // Instantiate(obstaclePrefabs[Random.Range(0, obstaclePrefabs.Length)], new Vector3((int)((toExplore.Item1 + cell.Item1)/ 2) - width/2, 0, (int)((toExplore.Item2 + cell.Item2)/ 2) - height/2), Quaternion.identity, obstacleHolder.transform);

                nbrs = getNeighbors(maze, cell.Item1, cell.Item2, true);
                foreach (var nbr in nbrs) {
                    frontier.Add(nbr);
                }
            }

            frontier.Remove(cell);
        }

        Debug.Log("width: " + width + ", height: " + height);

        foreach (Vector2 waypoint in waypoints) {
            (int, int) wpt = waypointToTuple(waypoint);
            Debug.Log(wpt);
            for (int i=-freeArea; i<=freeArea; i++) {
                for (int j=-freeArea; j<=freeArea; j++) {
                    if (wpt.Item1 + i >= 0 && wpt.Item1 + i < width && wpt.Item2 + j >= 0 && wpt.Item2 + j <= height)
                        maze[wpt.Item1 + i, wpt.Item2 + j] = true;
                }
            }
        }

        GameObject obstacleHolder = new GameObject("Obstacles");

        float counter = 0;

        for (int h = 2; h < height-2; h++) {
            for (int w = 2; w < width-2; w++) {
                counter += Random.Range(-1,2) * 0.5f + 1.0f;
                if (!maze[w, h] && counter % (1/density) < 1) {
                    Instantiate(obstaclePrefabs[Random.Range(0, obstaclePrefabs.Length)], new Vector3(w - width/2, 0, h - height/2) * spread, Quaternion.identity, obstacleHolder.transform);
                }
            }
        }
    }
}
