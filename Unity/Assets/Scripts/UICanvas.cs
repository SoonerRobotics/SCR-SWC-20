using UnityEngine;

public class UICanvas : MonoBehaviour
{
    public GameObject PauseMenu;

    public bool paused = false;

    void Update() {
        if (Input.GetKeyDown(KeyCode.Escape)) {
            paused = !paused;
            PauseMenu.SetActive(paused);
        }
    }

    public void Restart() {
        GameManager.instance.RestartSim();
    }

    public void Reload() {
        GameManager.instance.ReloadSim();
    }
}
