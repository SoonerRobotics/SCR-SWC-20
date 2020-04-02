using System.Collections;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

public class FieldBuilder : MonoBehaviour
{
    public GameObject[] obstaclePrefabs;
    public GameObject waypointPrefab;
    public GameObject waypointHolder;

    public int height = 117;
    public int width = 57;

    public float spread = 1.5f;

    [Range(0.0f, 1.0f)]
    public float density = 0.5f;

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

    bool[,] maze;
    HashSet<(int, int)> frontier;
    GameObject obstacleHolder;

    // Start is called before the first frame update
    void Start()
    {
        Random.InitState(seed);

        Vector2[] waypoints = GameManager.instance.GetWaypoints();

        width = (int)(width / spread);
        height = (int)(height / spread);

        maze = new bool[width, height];
        frontier = new HashSet<(int, int)>();
        
        (int, int) goal = waypointToTuple(waypoints[waypoints.Count() - 1]);
        maze[goal.Item1, goal.Item2] = true;
        frontier.Add(goal);

        obstacleHolder = new GameObject("Obstacles");

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

        foreach (Vector2 waypoint in waypoints) {
            for (int i=-freeArea; i<=freeArea; i++) {
                for (int j=-freeArea; j<=freeArea; j++) {
                    (int, int) wpt = waypointToTuple(waypoint);
                    maze[wpt.Item1 + i, wpt.Item2 + j] = true;
                }
            }
        }

        for (int h = 0; h < height; h++) {
            for (int w = 0; w < width; w++) {
                if (!maze[w, h] && Random.value < density)
                    Instantiate(obstaclePrefabs[Random.Range(0, obstaclePrefabs.Length)], new Vector3(w - width/2, 0, h - height/2) * spread, Quaternion.identity, obstacleHolder.transform);
            }
        }
    }
}
